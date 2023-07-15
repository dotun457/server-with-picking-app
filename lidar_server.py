# Velodyne LiDAR/PCAP server
# Code by Joachim Pouderoux, (c) Perception4D, 2023

from collections import namedtuple
from pylibpcap.pcap import rpcap
from velodyne_decoder_pylib import *

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
from track import get_flight_track

###############################################################################
# PUBLIC CONSTANTS TO MODIFY

gps_altitude = +123   # Should be define with default altitude if GPS does not provide it
authorized_origins = ["http://localhost:3000"]

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
        #print_parsed_points(points, frame)
        frame = frame + 1

###############################################################################
# Serve parse points information on the websocket
async def serve_parsed_points(points, frame, websocket):
    # print("Frame " + str(frame) + " with " + str(len(points.points)) + " points received at timestamp: " + str(points.stamp))
    data = points.points
    data = [(x[0], x[1], x[2], x[3]) for x in data]
    pcl_data = list(data)
    pcl_data_json = json.dumps(pcl_data, cls=NumpyFloatValuesEncoder)
    await websocket.send(pcl_data_json)

###############################################################################
# Parse and serve lidar stream from a socket to a websocket connected client
async def handle_client_lidar(websocket):
    gps_data_json = json.dumps(gps_fix, cls=NumpyFloatValuesEncoder)
    await websocket.send(gps_data_json)

    frame = 0
    track_path = get_flight_track()
    for points in decode_lidar_stream(sock, config):
        await serve_parsed_points(points, frame, websocket)
        frame = frame + 1

###############################################################################
# Parse and serve lidar stream from a PCAP file to a websocket connected client
async def handle_client_pcap(websocket):
    global args
    gps_data_json = json.dumps(gps_fix, cls=NumpyFloatValuesEncoder)
    await websocket.send(gps_data_json)

    frame = 0
    print(args.pcap)

    track_path = get_flight_track()
    data = [(x[0], x[1], x[2], x[3]) for x in track_path]
    pcl_data = list(data)
    pcl_data_json = json.dumps(pcl_data, cls=NumpyFloatValuesEncoder)
    await websocket.send(pcl_data_json)

    # await serve_parsed_points(track_path, frame, websocket)

    for points in vd.read_pcap(args.pcap.name, config):
        await serve_parsed_points(points, frame, websocket)
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
    async with websockets.serve(handle_client_pcap, host, port, origins=authorized_origins):
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
