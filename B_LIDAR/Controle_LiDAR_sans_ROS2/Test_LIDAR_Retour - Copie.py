import time
import threading
import pandas as pd
import numpy as np
from collections import deque
from pyrplidar import PyRPlidar  # type: ignore
from rich.console import Console
from rich.table import Table
from rich.live import Live

# Configuration
LIDAR_PORT = "/dev/ttyUSB0"  # Modifier selon le port correct

# Console Rich
console = Console()

# Angles fixes pour l'affichage
FIXED_ANGLES = np.arange(0, 360, 10)  # Angles de 0° à 350° par pas de 10°

# Variables partagées
lidar_data = deque(maxlen=3600)  # Buffer circulaire rapide
lock = threading.Lock()  # Verrou pour protéger l'accès aux données
running = True  # Variable globale pour arrêter les threads

def receive_lidar_data():
    """Thread qui récupère les données du LIDAR en continu"""
    global lidar_data, running
    lidar = PyRPlidar()
    lidar.connect(port=LIDAR_PORT, baudrate=256000, timeout=3)

    lidar.set_motor_pwm(900)  # Démarrer le moteur
    time.sleep(2)  # Laisser le temps au LIDAR de stabiliser

    console.print("[bold green]Récupération des données LIDAR...[/bold green]")

    try:
        scan_generator = lidar.start_scan()

        for measurement in scan_generator():
            if not running:
                break  # Sortie si l'arrêt est demandé
            
            quality = measurement.quality
            angle = measurement.angle
            distance = measurement.distance

            if quality > 12 and angle < 360:  # Filtrage qualité
                with lock:
                    lidar_data.append((angle, distance))  # Stockage rapide dans deque

    except KeyboardInterrupt:
        console.print("[bold red]Arrêt du LIDAR.[/bold red]")
    
    finally:
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()

def generate_table():
    """Génère un tableau pour l'affichage des données LIDAR"""
    with lock:
        data = list(lidar_data)

    table = Table(title="Données LIDAR", show_lines=True)
    table.add_column("Angle (°)", justify="center", style="cyan", no_wrap=True)
    table.add_column("Distance (mm)", justify="center", style="magenta", no_wrap=True)
    table.add_column("Max Angle (°)", justify="center", style="green", no_wrap=True)
    table.add_column("Erreur (°)", justify="center", style="red", no_wrap=True)

    # Convertir les données en DataFrame pour analyse
    lidar_df = pd.DataFrame(data, columns=["Angle (°)", "Distance (mm)"]) if data else pd.DataFrame(columns=["Angle (°)", "Distance (mm)"])

    if not lidar_df.empty:
        # Identifier la rotation (en détectant les sauts d'angle)
        lidar_df['Rotation'] = (abs(lidar_df["Angle (°)"].diff()) > 355).cumsum()

        # Trouver l'angle avec la distance maximale pour chaque rotation complète
        max_distances = lidar_df.loc[lidar_df.groupby("Rotation")["Distance (mm)"].idxmax(), 
                                     ["Rotation", "Angle (°)", "Distance (mm)"]]

        # Calculer l'erreur pour chaque point
        lidar_df["Erreur (°)"] = np.where(
            lidar_df["Angle (°)"] > 270, lidar_df["Angle (°)"] - 360,
            np.where(lidar_df["Angle (°)"] < 90, lidar_df["Angle (°)"], 0)
        )

        # Associer l'erreur avec l'angle max
        max_distances = max_distances.merge(lidar_df[["Angle (°)", "Erreur (°)"]], on="Angle (°)", how="left")

    # Affichage des angles fixes
    for angle in FIXED_ANGLES:
        if not lidar_df.empty and angle in lidar_df["Angle (°)"].values:
            row = lidar_df[lidar_df["Angle (°)"] == angle].iloc[-1]
            distance = f"{row['Distance (mm)']:.2f}"
            error = f"{row['Erreur (°)']:.2f}"
        else:
            distance = "-"
            error = "-"

        max_angle = max_distances["Angle (°)"].max() if not max_distances.empty else "-"
        max_error = max_distances.loc[max_distances["Angle (°)"] == max_angle, "Erreur (°)"].values[0] if not max_distances.empty else "-"

        table.add_row(f"{angle:.2f}", distance, f"{max_angle:.2f}", f"{max_error:.2f}")

    return table

def process_data():
    """Thread qui traite les données du LIDAR pour un calcul de trajectoire"""
    global lidar_data, running

    console.print("[bold yellow]Process thread started...[/bold yellow]")

    with Live(generate_table(), refresh_per_second=2) as live:
        while running:
            live.update(generate_table())  # Mise à jour de l'affichage
            time.sleep(0.5)

if __name__ == "__main__":
    # Création des threads
    thread_lidar = threading.Thread(target=receive_lidar_data, daemon=True)
    thread_process = threading.Thread(target=process_data, daemon=True)

    # Lancement des threads
    thread_lidar.start()
    thread_process.start()

    try:
        while True:
            time.sleep(1)  # Boucle principale en veille
    except KeyboardInterrupt:
        running = False  # Demande d'arrêt des threads
        thread_lidar.join()
        thread_process.join()
        console.print("[bold red]Arrêt du programme.[/bold red]")

