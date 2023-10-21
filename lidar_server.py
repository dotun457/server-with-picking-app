# Velodyne LiDAR/PCAP server
# Code by Joachim Pouderoux, (c) Perception4D, 2023

from collections import namedtuple
from pylibpcap.pcap import rpcap
from velodyne_decoder_pylib import *
from copy import deepcopy

import argparse
import asyncio
import dpkt
import json
import numpy as np
import pynmea2
import time
import socket
import velodyne_decoder as vd  # Requires version 2.3.0
import websockets
from track import get_bdi_flight_track, get_nasa_flight_track
from tls_velodyne_decoder import point_cloud_dome, filter_points_by_pos,\
generate_transform_matrix, apply_affine_transformation

###############################################################################
# PUBLIC CONSTANTS TO MODIFY

gps_altitude = +123   # Should be define with default altitude if GPS does not provide it
authorized_origins = ["http://localhost:3000"]

settings = {
    '__header__': b'MATLAB 5.0 MAT-file, Platform: PCWIN64, Created on: Sun Aug  6 14:57:54 2023',
    '__version__': '1.0',
    '__globals__': [],
    'R': np.array([[0]], dtype=np.uint8),
    'alpha_1': np.array([[0]], dtype=np.uint8),
    'alpha_2': np.array([[0]], dtype=np.uint8),
    'angle': np.array([[360]], dtype=np.uint16),
    'first': np.array([[10]], dtype=np.uint8),
    'gridStep': np.array([[0.005]]),
    'input_file_name': np.array(['./pcap_files/5rpm 1min.pcap'], dtype='<U30'),
    'pos': np.array([[2]], dtype=np.uint8),
    'pos2': np.array([[0]], dtype=np.uint8),
    'puck': np.array([[2]], dtype=np.uint8),
    'times': np.array([[3600]], dtype=np.uint16),
    'totalScanFrames': np.array([[3652]], dtype=np.uint16),
    'usableFrames': np.array([[3642]], dtype=np.uint16)
}


times = settings['times'][0][0]
angle = settings['angle'][0][0]
if angle < 45 or angle > 360:
    print("Input for angle is invalid or outside the valid range (45-360). Setting angle to 360.")
    angle = 360

first = settings['first'][0][0]
puck = settings['puck'][0][0]
filesToMerge = []

# Export parameters
pos = settings['pos'][0][0]
pos2 = settings['pos2'][0][0]
gridStep = settings['gridStep'][0][0]

# Calibration parameters
alpha_1 = settings['alpha_1'][0][0]
alpha_2 = settings['alpha_2'][0][0]
R = settings['R'][0][0]
theta3 = 0

###############################################################################
sock = None
args = None
config = None
gps_fix = []

LIDAR_PACKET_SIZE = 1206
GPS_PACKET_SIZE = 512

###############################################################################
class NumpyFloatValuesEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.float32):
            return float(obj)
        return JSONEncoder.default(self, obj)

###############################################################################
# Convert Degree and Minutes coordinates to Decimal Degrees
def convertDMToDD(degrees_minutes):
    degrees = int(degrees_minutes)
    minutes = degrees_minutes - degrees
    decimal_degrees = degrees + minutes / 60 * 100
    return decimal_degrees

###############################################################################
# Decode a NMEA string and return an array with lon, lat & alt
def decode_gps_nmea(payload):
    data = bytearray(payload)
    nmea = str(data[206:334].decode('utf-8')).split('\r')[0]
    #print(nmea)
    if nmea.startswith("$GP"):
        try:
            msg = pynmea2.parse(nmea)
            if msg.status == 'A':
                lon = float(msg.lon) / 100.
                lat = float(msg.lat) / 100.
                if msg.lon_dir == 'W':
                    lon = -lon
                if msg.lat_dir == 'S':
                    lat = -lat
                lon = convertDMToDD(lon)
                lat = convertDMToDD(lat)
                alt = gps_altitude
                try:
                    alt = msg.altitude
                except:
                    pass
                #print("GPS fix found DM: {}, {}  Alt: {}".format(float(msg.lat)/100., -float(msg.lon)/100., alt))
                print("GPS fix found: https://www.google.com/maps/place/{},{}  Alt: {}".format(lat, lon, alt))
                return [ lon, lat, alt ]
        except Exception as e:
            pass
    return []

###############################################################################
# Get the first valid GPS fix in the GPS stream of a PCAP file
def get_gps_fix_pcap(pcap_file, config):
    with vd._fopen(pcap_file, "rb") as f:
        for stamp, buf in dpkt.pcap.Reader(f):
            data = dpkt.ethernet.Ethernet(buf).data.data.data
            if vd.is_py2:
                data = bytearray(data)
            if len(data) == GPS_PACKET_SIZE:
                fix = decode_gps_nmea(data)
                if len(fix) != 0:
                    return fix

###############################################################################
# Get the first valid GPS fix in the GPS stream of a socket
def get_gps_fix_socket(sockpos):
    while True:
        payload, addr = sockpos.recvfrom(GPS_PACKET_SIZE)
        payload_len = len(payload)
        if payload_len != GPS_PACKET_SIZE:
            continue
        fix = decode_gps_nmea(payload)
        if len(fix) != 0:
            return fix

###############################################################################
# Decode the lidar stream point cloud from a socket
def decode_lidar_stream(sock, config, as_pcl_structs=True):
    decoder = StreamDecoder(config)
    ResultTuple = namedtuple("StampCloudTuple", ("stamp", "points"))
    while True:
        payload, addr = sock.recvfrom(LIDAR_PACKET_SIZE)
        payload_len = len(payload)
        if payload_len == LIDAR_PACKET_SIZE:
            recv_stamp = time.time()
            result = decoder.decode(recv_stamp, payload, as_pcl_structs)
            if result is not None:
                yield ResultTuple(*result)
        #time.sleep(.01)

###############################################################################
# Print parse points information
def print_parsed_points(points, frame):
    print("Frame " + str(frame) + " with " + str(len(points.points)) + " points received at timestamp: " + str(points.stamp))
    #data = [(x[0], x[1], x[2], x[3]) for x in points.points]
    #print(data)

###############################################################################
# Parse and print the lidar stream from a socket
def parse_lidar_and_print(sock):
    frame = 0
    for points in decode_lidar_stream(sock, config):
        print_parsed_points(points, frame)
        frame = frame + 1

###############################################################################
# Parse and print the lidar stream from a PCAP file
def parse_pcap_and_print(pcap_file):
    frame = 0
    for points in vd.read_pcap(pcap_file, config):
        frame = frame + 1

###############################################################################
# Serve parse points information on the websocket
async def serve_parsed_points(points, websocket):
    pcl_data_json = json.dumps(points, cls=NumpyFloatValuesEncoder)
    await websocket.send(pcl_data_json)

###############################################################################
# Parse and serve lidar stream from a socket to a websocket connected client
async def handle_client_lidar(websocket):
    gps_data_json = json.dumps(gps_fix, cls=NumpyFloatValuesEncoder)
    await websocket.send(gps_data_json)

    frame = 0
    track_path = get_bdi_flight_track()
    for points in decode_lidar_stream(sock, config):
        await serve_parsed_points(points, frame, websocket, None)
        frame = frame + 1

###############################################################################
# Parse and serve lidar stream from a PCAP file to a websocket connected client
async def handle_client_pcap(websocket, pre_processed_pcd):
    gps_data_json = json.dumps(gps_fix, cls=NumpyFloatValuesEncoder)
    await websocket.send(gps_data_json)
    frame = 0
    # bdi_track_path = get_bdi_flight_track()
    # nasa_track_path = get_nasa_flight_track()

    # def convert_track(track):
    #     data = [(x[0], x[1], x[2], x[3]) for x in track]
    #     pcl_data = list(data)
    #     pcl_data_json = json.dumps(pcl_data, cls=NumpyFloatValuesEncoder)
    #     return pcl_data_json
    
    # pcl_bdi = convert_track(bdi_track_path)
    # pcl_nasa = convert_track(nasa_track_path)
    
    #await websocket.send(pcl_bdi)
    #await websocket.send(pcl_nasa)

    for points in pre_processed_pcd:
        n_points = [list(point) for point in points]
        await serve_parsed_points(n_points, websocket)
        frame = frame + 1
###############################################################################
# Start a websocket server to server lidar stream from a socket
async def start_server_lidar(host, port):
    async with websockets.serve(handle_client_lidar, host, port, origins=authorized_origins):
        print("WebSocket started on port {}{}\nWaiting for client...".format(host, port))
        await asyncio.Future()

###############################################################################
# Start a websocket server to server lidar stream from a PCAP file
async def start_server_pcap(host, port):
    global args
    def pre_process():
        frame = 0
        if args.option == 1:
            initial_read = vd.read_pcap(args.pcap.name, config)
            return initial_read
        elif args.option == 2:
            times = 0
            initial_read = vd.read_pcap(args.pcap.name, config)
            times = len(list(initial_read))
            initial_read = vd.read_pcap(args.pcap.name, config)
            angle_deg = np.linspace(0, angle, times + 1)
            angle_rad = np.radians(angle_deg)
            A1, A2, T, T4 = generate_transform_matrix(alpha_1, alpha_2, R, theta3)
        
            def filter_transform(points):
                points = points.points #points.points yields a numpy.ndarray object (n, 6)
                points = points[:, :4]
            
                angle_ = angle_rad[frame + 1]
                VM = np.array([
                    [np.cos(angle_), 0, np.sin(angle_), 0],
                    [0, 1, 0, 0],
                    [-np.sin(angle_), 0, np.cos(angle_), 0],
                    [0, 0, 0, 1]
                ])
            
                transformed_points = filter_points_by_pos(points, pos)
                for tform in [T, A1, A2, T4, VM]:
                    tmp = apply_affine_transformation(transformed_points, tform)
                    transformed_points = tmp
                return transformed_points
            
            pre_processed_pcd = list(map(filter_transform, initial_read))
            merged_pcd = point_cloud_dome(pre_processed_pcd, band_number=32)
            return merged_pcd

    point_cloud = pre_process()
    async with websockets.serve(lambda ws, path: handle_client_pcap(ws, point_cloud), host, port, origins=authorized_origins):
        print("WebSocket started on port {}:{}\nWaiting for client....".format(host, port))
        await asyncio.Future()

###############################################################################
###############################################################################
###############################################################################
def main():
    global args
    global config
    global gps_fix
    global sock

    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar_model", type=str, default='VLP-16', help="LiDAR Model (default: VLP-16)")
    parser.add_argument("--lidar_rpm", type=int, default=220, help="LiDAR RPM (default: 220)")
    parser.add_argument("--lidar_port", type=int, default=2368, help="LiDAR Server port (default: 2368)")
    parser.add_argument("--lidar_host", type=str, default='127.0.0.1', help="LiDAR Server host (default: 127.0.0.1)")
    parser.add_argument("--gps_port", type=int, default=8308, help="GPS Server port (default: 8308)")
    parser.add_argument("--gps_host", type=str, default='127.0.0.1', help="GPS Server host (default: 127.0.0.1)")
    parser.add_argument("--publish_port", type=int, default=8765, help="WebSocket Publishing port (default: 8765)")
    parser.add_argument("--publish_host", type=str, default='127.0.0.1', help="WebSocket Publishing host (default: 127.0.0.1)")
    parser.add_argument("--publish", action="store_true", help="Publish point cloud on a WebSocket")
    parser.add_argument("--pcap", type=argparse.FileType('r'), help="PCAP File Path")
    parser.add_argument("--option", type=int, default=1, help="option2 apply filter matrices for dome visualization")
    args = parser.parse_args()

    config = vd.Config(model=args.lidar_model, rpm=args.lidar_rpm, gps_time = True)  # LiDAR settings
    
    if args.pcap != None:
        print("Process PCAP file " + args.pcap.name + "...")
        gps_fix = get_gps_fix_pcap(args.pcap.name, config)
        print(gps_fix)
        if not args.publish: #parse pcap and print, let's parse the csv 
            parse_pcap_and_print(args.pcap.name)
        else:
            asyncio.run(start_server_pcap(args.publish_host, args.publish_port))
    else:
        print("Process LiDAR Server on {}:{}...".format(args.lidar_host, args.lidar_port))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = (args.lidar_host, args.lidar_port)
        sock.bind(server_address)

        sockpos = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address2 = (args.gps_host, args.gps_port)
        sockpos.bind(server_address2)
        gps_fix = get_gps_fix_socket(sockpos)

        if not args.publish:
            parse_lidar_and_print(sock)
        else:
            asyncio.run(start_server_lidar(args.publish_host, args.publish_port))

###############################################################################
if __name__ == '__main__':
    main()
