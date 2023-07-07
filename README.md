Velodyne LiDAR/PCAP server
Code by Joachim Pouderoux, (c) Perception4D, 2023

# Dependencies

Python 3 (tested with 3.10)  | I used python3.7 (--Dotun)
velodyne_decoder 2.3.0
pynmea2
pylibpcap
dpkt

# How to run

Show all options
$ python lidar_server.py --help

Listen LiDAR stream and publish the point cloud frames to a websocket:
$ python lidar_server.py --lidar_host=localhost --lidar_port=2368 --gps_port=8308 --publish

Read a PCAP file and publish the point cloud frames to a websocket:
$ python lidar_server.py --pcap FT7.pcap --publish

In publish mode, the server creates a websocket on localhost (default port 3001)
and wait for a client to connect. It then sends it the GPS fix first, then will
come the point cloud data as an array of (x,y,z) tuples.
