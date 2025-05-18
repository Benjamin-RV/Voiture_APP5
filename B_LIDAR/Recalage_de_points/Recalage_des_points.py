import numpy as np
import matplotlib.pyplot as plt
import csv
import math
import time
from icp import icp  # Assurez-vous que icp est correctement importé

# Chargement des données du fichier CSV
theTime = []
theAngle = []
data = []

with open('lidar_data2.csv', mode='r', newline='') as file:
    reader = csv.reader(file)
    for row in reader:
        epoch_time = float(row[0])
        struct_time = time.localtime(epoch_time)
        minutes = struct_time.tm_min
        seconds = struct_time.tm_sec
        milliseconds = int((epoch_time % 1) * 1000)
        seconds_with_millis = round(seconds + milliseconds / 1000.0, 3)
        theTime.append(seconds_with_millis)
        theAngle.append(float(row[1]))
        x = float(row[-2])
        y = float(row[-1])
        data.append([x, y])

reference_points = np.array(data)

def recupe_ms(theTime, n=0):
    timeReference = theTime[n]
    for i in range(1, len(theTime)):
        if(timeReference == theTime[i]):
            i += 1
        elif (theTime[i] - timeReference > 0.500):
            return i
        else:
            i += 1

def process_blocks(reference_points, block_size):
    # Process blocks of points
    num_points = len(reference_points)
    for i in range(0, num_points, block_size):
        # Définir le bloc de points de référence et le bloc à comparer
        ref_start = i
        ref_end = min(i + block_size, num_points)
        compare_start = ref_end
        compare_end = min(i + 2 * block_size, num_points)

        # S'assurer qu'il y a un deuxième bloc à comparer
        if compare_start < num_points:
            points_to_be_aligned = reference_points[compare_start:compare_end]
            points_reference = reference_points[ref_start:ref_end]

            # Appliquer ICP pour aligner les deux blocs
            transformation_history, aligned_points = icp(points_reference, points_to_be_aligned, verbose=True)

            # Affichage des résultats
            plt.title(f"start ref_points = {ref_start} | end ref_points = {ref_end}")
            plt.plot(points_reference[:, 0], points_reference[:, 1], 'rx', label='Reference points')
            plt.plot(points_to_be_aligned[:, 0], points_to_be_aligned[:, 1], 'b1', label='Points to be aligned')
            plt.plot(aligned_points[:, 0], aligned_points[:, 1], 'g+', label='Aligned points')
            plt.legend()
            plt.show()

# Taille des blocs à traiter
block_size = 250 # Par exemple, 500 points à chaque fois

# Lancer le traitement par blocs
process_blocks(reference_points, block_size)
