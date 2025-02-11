import sys
from rplidar import RPLidar


PORT_NAME = '/dev/ttyUSB0'



lidar = RPLidar(PORT_NAME)

try:
    print('Recording measurments... Press Crl+C to stop.')
    for measurment in lidar.iter_measures():
        print(measurement)
except KeyboardInterrupt:
    print('Stoping.')
lidar.stop()
lidar.disconnect()
outfile.close()
