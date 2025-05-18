import time
import threading
import pandas as pd
import numpy as np
from collections import deque
from pyrplidar import PyRPlidar
from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich.panel import Panel

# Configuration
LIDAR_PORT = "/dev/ttyUSB0"  # Modifier selon le port correct

# Paramètres de filtrage
MIN_DISTANCE = 300  # Distance minimale en mm (30 cm)
MAX_DISTANCE = 10000  # Distance maximale en mm (10 m)
QUALITY_THRESHOLD = 12  # Seuil de qualité minimum

# Console Rich
console = Console()

# Variables partagées
lidar_data = deque(maxlen=3600)  # Buffer circulaire rapide
lock = threading.Lock()  # Verrou pour protéger l'accès aux données
running = True  # Variable globale pour arrêter les threads

def receive_lidar_data():
    """Thread qui récupère les données du LIDAR en continu"""
    global lidar_data, running
    lidar = PyRPlidar()
    
    try:
        lidar.connect(port=LIDAR_PORT, baudrate=256000, timeout=3)
        lidar.set_motor_pwm(900)  # Démarrer le moteur
        time.sleep(2)  # Laisser le temps au LIDAR de stabiliser

        console.print("[bold green]Récupération des données LIDAR...[/bold green]")

        scan_generator = lidar.start_scan()

        for measurement in scan_generator():
            if not running:
                break  # Sortie si l'arrêt est demandé
            
            quality = measurement.quality
            angle = measurement.angle
            distance = measurement.distance

            # Filtrage par qualité et par distance
            if (quality > QUALITY_THRESHOLD and 
                MIN_DISTANCE <= distance <= MAX_DISTANCE):
                with lock:
                    lidar_data.append((angle, distance, quality))

    except KeyboardInterrupt:
        console.print("[bold red]Arrêt du LIDAR.[/bold red]")
    except Exception as e:
        console.print(f"[bold red]Erreur LIDAR: {e}[/bold red]")
    
    finally:
        try:
            lidar.stop()
            lidar.set_motor_pwm(0)
            lidar.disconnect()
        except:
            pass

def generate_table():
    """Génère un tableau pour l'affichage des données LIDAR"""
    # Structure du tableau
    table = Table(title="Données LIDAR pour Navigation", show_lines=True)
    table.add_column("Angle (°)", justify="center", style="cyan")
    table.add_column("Distance (m)", justify="center", style="magenta")
    table.add_column("Qualité", justify="center", style="yellow")
    table.add_column("Zone", justify="center", style="green")
    table.add_column("Erreur (°)", justify="center", style="red")
    table.add_column("Max?", justify="center", style="bold")

    with lock:
        data = list(lidar_data)

    # Convertir les données en DataFrame pour analyse
    if data:
        df = pd.DataFrame(data, columns=["Angle (°)", "Distance (mm)", "Qualité"])
        
        # Convertir mm en m pour l'affichage
        df["Distance (m)"] = df["Distance (mm)"] / 1000
        
        # Identifier la zone pour chaque angle
        df["Zone"] = np.where(
            ((df["Angle (°)"] >= 0) & (df["Angle (°)"] <= 90)) | 
            ((df["Angle (°)"] >= 270) & (df["Angle (°)"] <= 360)),
            "Intérêt", "Hors zone"
        )
        
        # Filtrer pour ne garder que les zones d'intérêt
        zone_df = df[df["Zone"] == "Intérêt"]
        
        # Chercher l'angle avec la distance maximale dans les zones d'intérêt
        if not zone_df.empty:
            max_idx = zone_df["Distance (m)"].idxmax()
            max_angle = zone_df.loc[max_idx, "Angle (°)"]
            
            # Calculer l'erreur par rapport à 0°
            zone_df["Erreur (°)"] = np.where(
                zone_df["Angle (°)"] <= 90, 
                zone_df["Angle (°)"], 
                zone_df["Angle (°)"] - 360
            )
            
            # Marquer l'angle maximum
            zone_df["Max"] = zone_df.index == max_idx
            
            # Calculer statistiques pour le résumé
            avg_dist = zone_df["Distance (m)"].mean()
            min_dist = zone_df["Distance (m)"].min()
            
            # Afficher les 8 dernières mesures dans la zone d'intérêt
            recent_data = zone_df.tail(8)
            
            for _, row in recent_data.iterrows():
                angle = row["Angle (°)"]
                distance = row["Distance (m)"]
                quality = row["Qualité"]
                zone = "0-90°" if angle <= 90 else "270-360°"
                error = row["Erreur (°)"]
                is_max = "✓" if row["Max"] else ""
                
                # Ajouter la ligne au tableau
                table.add_row(
                    f"{angle:.2f}", 
                    f"{distance:.2f}", 
                    f"{quality:.0f}",
                    zone,
                    f"{error:.2f}", 
                    is_max,
                    style="bold green" if row["Max"] else None
                )
            
            # Ajouter une ligne récapitulative
            max_distance = zone_df.loc[max_idx, "Distance (m)"]
            max_error = zone_df.loc[max_idx, "Erreur (°)"]
            
            table.add_row(
                "---", "---", "---", "---", "---", "---",
                style="dim"
            )
            
            table.add_row(
                f"[bold]MAX: {max_angle:.2f}[/bold]", 
                f"[bold]{max_distance:.2f}[/bold]", 
                "", 
                f"[bold]{('0-90°' if max_angle <= 90 else '270-360°')}[/bold]",
                f"[bold]{max_error:.2f}[/bold]", 
                "✓",
                style="bold yellow"
            )
            
            table.add_row(
                f"[blue]STATS[/blue]", 
                f"[blue]Min: {min_dist:.2f} | Moy: {avg_dist:.2f}[/blue]", 
                f"[blue]Points: {len(zone_df)}[/blue]", 
                "", "", ""
            )
        else:
            table.add_row("Aucune donnée dans les zones d'intérêt (0-90° et 270-360°)", "", "", "", "", "")
    else:
        table.add_row("En attente de données...", "", "", "", "", "")

    return table

def process_data():
    """Thread qui traite les données du LIDAR"""
    global lidar_data, running

    console.print("[bold yellow]Traitement des données démarré...[/bold yellow]")
    console.print("[bold cyan]Recherche de l'angle optimal dans les zones 0-90° et 270-360°[/bold cyan]")
    console.print(Panel(f"""[bold white]Paramètres de filtrage:
- Distance minimale: {MIN_DISTANCE/1000:.1f} m
- Distance maximale: {MAX_DISTANCE/1000:.1f} m
- L'erreur est calculée par rapport à 0°[/bold white]""", 
    title="Configuration", border_style="blue"))

    with Live(generate_table(), refresh_per_second=4) as live:
        while running:
            try:
                live.update(generate_table())
                time.sleep(0.25)  # Mise à jour toutes les 250ms
            except Exception as e:
                console.print(f"[bold red]Erreur de traitement: {e}[/bold red]")
                time.sleep(1)

if __name__ == "__main__":
    # Création des threads
    thread_lidar = threading.Thread(target=receive_lidar_data, daemon=True)
    thread_process = threading.Thread(target=process_data, daemon=True)

    # Lancement des threads
    thread_lidar.start()
    thread_process.start()

    try:
        console.print("[bold green]Programme démarré. Appuyez sur Ctrl+C pour quitter.[/bold green]")
        while True:
            time.sleep(1)  # Boucle principale en veille
    except KeyboardInterrupt:
        running = False  # Demande d'arrêt des threads
        console.print("[bold yellow]Arrêt en cours...[/bold yellow]")
        thread_lidar.join(timeout=2)
        thread_process.join(timeout=2)
        console.print("[bold red]Programme terminé.[/bold red]")
