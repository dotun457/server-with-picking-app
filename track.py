import glob
import csv
import re

def get_bdi_flight_track():
    #index should be 3
    #landing_june15 = "FT20 Nasa 3  June 15 Horizontal/*.csv"
    #dirs = [landing_june15]
    
    #NASA 1 June 13
    # landing = "FT20 Nasa 13 June_ Landing Frames/*.csv"
    # takeoff = "FT20 Nasa 13 June_Takeoff/*.csv"

    #Nasa 2 June 13
    #takeoff = "FT20 NASA 2 June 13 Takeoff/*.csv"

    #NASA3 June 13
    # takeoff = "FT20 NASA3 June 13 Takeoff/*.csv"
    # landing = "FT20 NASA3 June 13 Landing/*.csv"

    #NASA june 14
    #takeoff = "FT20 Nasa 2 June 14/*.csv"
    # dirs = [takeoff_landing, ]

    #Nasa3 June 15 horizontal
    horizontal = "FT20 Nasa 3  June 15 Horizontal/*.csv"
    
    #dirs = [takeoff, landing]
    dirs = [horizontal]
    track = []

    track = _get_tracks(dirs, 3, track)

    #255 blue, 0 red
    cntr = 0
    intensity = 0
    for dir in dirs:
        if cntr:
            intensity = 200 #blue
        else:
            intensity = 191 #green
        for filename in glob.glob(dir):
            with open(filename) as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) > 6:
                        if convert_to_float(row[3]):
                            tmp = [float(row[3]),float(row[4]), -1 * float(row[5]), intensity]
                            track.append(tmp)

        cntr += 1
    return track

def _get_tracks(dirs, index, track):
    cntr = 0
    intensity = 0
    for csv_dir in dirs:
        if cntr:
            intensity = 255 #blue
        else:
            intensity = 128 #green
        for filename in glob.glob(csv_dir):
            with open(filename) as f:
                reader = csv.reader(f)
                true_filename = filename.split("/")[-1]
                if re.search(r"vehicle_local_position(?!_setpoint)", true_filename):
                    for row in reader:
                        if convert_to_float(row[index]):
                            x = float(row[index])  
                            y = float(row[index + 1]) * -1
                            z = float(row[index + 2]) * -1
                            tmp = [x, y, z, intensity]
                            track.append(tmp)
        cntr += 1
    return track



def get_nasa_flight_track():
    #indecx should be 4
    #NASA June_13_Run1
    # takeoff = "nasa/June_13_Run1/run1/*.csv"
    # landing = "nasa/June_13_Run1/run2/*.csv"

    #NASA June_13_Run2
    # takeoff = "nasa/June_13_Run2/run1/*.csv"
    # landing = "nasa/June_13_Run2/run2/*.csv"
   
    #NASA June_13_Run3
    #takeoff = "nasa/June_13_Run3/run1/*.csv"
    #landing = "nasa/June_13_Run3/run2/*.csv"

    #NASA June_14_run1
    # takeoff = "nasa/June_14_Run1/run1/*.csv"
    landing = "nasa/June_14_Run1/run2/*.csv"

    dirs = [landing]
    track = []
    track = _get_tracks(dirs, 4, track)
    return track
 
def convert_to_float(coord):
        try:
            float(coord)
        except ValueError:
            return False
        return True

get_nasa_flight_track()
get_bdi_flight_track()

