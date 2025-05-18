import csv
import time
import numpy as np

theTime = []
theAngle = []
data = []

# Ouvrir le fichier CSV
with open('lidar_data.csv', mode='r', newline='') as file:
    reader = csv.reader(file)
    # Lire le fichier ligne par ligne
    for row in reader:
        epoch_time = float(row[0])

        struct_time = time.localtime(epoch_time)
        minutes = struct_time.tm_min
        seconds = struct_time.tm_sec
        milliseconds = int((epoch_time % 1) * 1000)
        seconds_with_millis = round(seconds + milliseconds / 1000.0, 3)
        theTime.append(seconds_with_millis)
        theAngle.append(float (row[1]))
        x = float(row[-2])
        y = float(row[-1])
        data.append([x, y])
    
    data_array = np.array(data)
    
def recupe_ms(theTime,n=0):
    timeReference = theTime[n]
    for i in range(1, len(theTime)):
        if(timeReference == theTime[i]):
            i+=1
        elif (theTime[i] - timeReference >0.500):
            return i
        else:
            i+=1

nb_echantillon = recupe_ms(theTime)
new_nb_echantillon = recupe_ms(theTime, nb_echantillon)

print(nb_echantillon)
print(new_nb_echantillon)

print(theTime[0])
print(theTime[nb_echantillon])
print(theTime[new_nb_echantillon])

    # print("time ", theTime)
    # print('\n')
    # print("angle ", theAngle)
    # print('\n')
    # print("x ", x)
    # print('\n')
    # print("y ", y)

