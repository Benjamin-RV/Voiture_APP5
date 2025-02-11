import socket
import time
import threading
from collections import deque
from pyrplidar import PyRPlidar

# Configuration
LIDAR_PORT = "/dev/ttyUSB0"  # Modifier selon le port correct
UDP_IP = "192.168.97.164"  # Remplace par l'IP de ton PC
UDP_PORT = 5005  # Port UDP d'envoi

# Initialisation UDP avec un buffer plus grand
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # Augmente le buffer d'envoi UDP

# Variables partagées avec un buffer circulaire rapide
lidar_data = deque(maxlen=500)  # Buffer circulaire rapide
lock = threading.Lock()  # Verrou pour protéger l'accès aux données

def receive_lidar_data():
    """Thread qui récupère les données du LIDAR en continu"""
    global lidar_data
    lidar = PyRPlidar()
    lidar.connect(port=LIDAR_PORT, baudrate=256000, timeout=3)

    lidar.set_motor_pwm(900)  # Démarrer le moteur
    time.sleep(2)  # Laisser le temps au LIDAR de stabiliser

    print("Récupération des données LIDAR...")

    try:
        scan_generator = lidar.start_scan()

        for measurement in scan_generator():
            quality = measurement.quality
            angle = measurement.angle
            distance = measurement.distance

            if quality > 12:  # Filtrage qualité
                with lock:
                    lidar_data.append((angle, distance))  # Stockage rapide dans deque

    except KeyboardInterrupt:
        print("Arrêt du LIDAR.")
    
    finally:
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()

def send_udp_data():
    """Thread qui envoie les données du LIDAR en UDP"""
    global lidar_data

    print("Envoi des données UDP...")
    while True:
        time.sleep(0.001)  # Pause minimale pour éviter de surcharger le CPU

        with lock:
            if len(lidar_data) > 5:  # Vérifie s'il y a assez de données
                batch = []
                for _ in range(min(10, len(lidar_data))):  # Envoi par lots de 10 mesures
                    angle, distance = lidar_data.popleft()
                    batch.append(f"{angle:.2f},{distance:.2f}")

                message = "|".join(batch)  # Envoi groupé en une seule trame UDP
                sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

if __name__ == "__main__":
    # Création des threads
    thread_lidar = threading.Thread(target=receive_lidar_data, daemon=True)
    thread_udp = threading.Thread(target=send_udp_data, daemon=True)

    # Lancement des threads
    thread_lidar.start()
    thread_udp.start()

    try:
        while True:
            time.sleep(1)  # Boucle principale en veille
    except KeyboardInterrupt:
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()
        print("Arrêt du programme.")
