# -*- coding: utf-8 -*-


from flask import Flask, send_file
import matplotlib.pyplot as plt
import numpy as np
import time
from pyrplidar import PyRPlidar

app = Flask(__name__)

# Configuration du LIDAR
PORT = "/dev/ttyUSB0"
BAUDRATE = 256000

# Initialisation du LIDAR
lidar = PyRPlidar()
lidar.connect(port=PORT, baudrate=BAUDRATE, timeout=3)
lidar.set_motor_pwm(1000)
time.sleep(1)

scan_generator = lidar.start_scan_express(4)

def generate_scan_image():
    """Génère une image du scan LIDAR et la sauvegarde en PNG."""
    scan_data = []
    
    for count, scan in enumerate(scan_generator()):  # ? Correction ici
        angle = scan.angle
        distance = scan.distance
        quality = scan.quality

        if distance > 0:  # Filtrer les valeurs nulles
            x = distance * np.cos(np.radians(angle))
            y = distance * np.sin(np.radians(angle))
            scan_data.append((x, y))

        if count >= 300:  # Rafraîchissement tous les 300 points
            break

    if scan_data:
        x_data, y_data = zip(*scan_data)

        plt.figure(figsize=(6,6))
        plt.scatter(x_data, y_data, s=5, color='red')
        plt.xlim(-4000, 4000)
        plt.ylim(-4000, 4000)
        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.title("Scan LIDAR")
        plt.grid()
        plt.savefig("static/lidar_scan.png")  # Sauvegarde de l'image
        plt.close()

@app.route('/')
def display_scan():
    """Affiche le scan LIDAR sous forme d'image via Flask."""
    generate_scan_image()
    return send_file("static/lidar_scan.png", mimetype="image/png")

if __name__ == '__main__':
    try:
        app.run(host="0.0.0.0", port=5000)
    finally:
        # Arrêter proprement le LIDAR
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()


