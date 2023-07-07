import glob
import csv

def get_flight_track():
    landing = "FT20 Nasa 13 June_ Landing Frames/*.csv"
    takeoff = "FT20 Nasa 13 June_Takeoff/*.csv"
    dirs = [landing, takeoff]
    track = []

    def convert_to_float(coord):
        try:
            float(coord)
        except ValueError:
            return False
        return True

    for dir in dirs:
        for filename in glob.glob(dir):
            with open(filename) as f:
                reader = csv.reader(f)
                for row in reader:
                    if convert_to_float(row[3]):
                        tmp = [float(row[3]), float(row[4]), float(row[5]), 255]
                        track.append(tmp)
    return track

