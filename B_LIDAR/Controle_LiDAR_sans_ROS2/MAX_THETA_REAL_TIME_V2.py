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
from rich.progress import BarColumn, Progress, TextColumn

# Configuration
LIDAR_PORT = "/dev/ttyUSB0"  # Modifier selon le port correct

# Paramètres de filtrage
MIN_DISTANCE = 300  # Distance minimale en mm (30 cm)
MAX_DISTANCE = 10000  # Distance maximale en mm (10 m)
QUALITY_THRESHOLD = 12  # Seuil de qualité minimum

# Paramètres de contrôle
MAX_SERVO_ANGLE = 30  # Angle maximum du servo en degrés (dans chaque direction)
MAX_ACCELERATION = 50  # Accélération maximale réduite à 50%
SAFE_DISTANCE = 2000  # Distance en mm à partir de laquelle on réduit la vitesse (2m)
MIN_SAFE_DISTANCE = 500  # Distance minimale en mm pour l'arrêt complet (50cm)

# Paramètres PID pour le servo
PID_SERVO = {
    "Kp": 0.6,    # Gain proportionnel
    "Ki": 0.05,   # Gain intégral
    "Kd": 0.2,    # Gain dérivé
    "last_error": 0,
    "integral": 0,
    "last_time": 0
}

# Paramètres PID pour l'accélération
PID_ACCEL = {
    "Kp": 0.8,    # Gain proportionnel
    "Ki": 0.1,    # Gain intégral
    "Kd": 0.15,   # Gain dérivé
    "last_error": 0,
    "integral": 0,
    "last_time": 0,
    "target": 0   # Consigne d'accélération cible
}

# Console Rich
console = Console()

# Variables partagées
lidar_data = deque(maxlen=3600)  # Buffer circulaire rapide
lock = threading.Lock()  # Verrou pour protéger l'accès aux données
running = True  # Variable globale pour arrêter les threads
current_commands = {
    "angle_max": 0,
    "distance_max": 0,
    "erreur": 0,
    "rotation_servo": 0,
    "rotation_servo_raw": 0,  # Avant PID
    "acceleration": 0,
    "acceleration_raw": 0,    # Avant PID
    "timestamp": 0
}

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

def calculate_control_commands():
    """Calcule les commandes de contrôle à partir des données LIDAR"""
    global current_commands
    
    with lock:
        data = list(lidar_data)
    
    if not data:
        return
    
    # Convertir les données en DataFrame pour analyse
    df = pd.DataFrame(data, columns=["Angle (°)", "Distance (mm)", "Qualité"])
    
    # Filtrer les angles dans les zones d'intérêt (0-90° et 270-360°)
    mask = ((df["Angle (°)"] >= 0) & (df["Angle (°)"] <= 90)) | \
           ((df["Angle (°)"] >= 270) & (df["Angle (°)"] <= 360))
    zone_df = df[mask]
    
    if zone_df.empty:
        return
    
    # Trouver l'angle avec la distance maximale
    max_idx = zone_df["Distance (mm)"].idxmax()
    max_angle = zone_df.loc[max_idx, "Angle (°)"]
    max_distance = zone_df.loc[max_idx, "Distance (mm)"]
    
    # Calculer l'erreur par rapport à 0°
    if max_angle <= 90:
        error = max_angle  # Erreur positive (tournant à gauche)
    else:
        error = max_angle - 360  # Erreur négative (tournant à droite)
    
    # Calculer la rotation du servo brute (limitée à ±30°)
    # On inverse le signe car une erreur positive nécessite une rotation négative
    # pour corriger la trajectoire
    servo_rotation_raw = -error
    servo_rotation_raw = np.clip(servo_rotation_raw, -MAX_SERVO_ANGLE, MAX_SERVO_ANGLE)
    
    # Calculer l'accélération brute en fonction de la distance
    # Plus la distance est grande, plus on peut accélérer
    if max_distance <= MIN_SAFE_DISTANCE:
        acceleration_raw = 0  # Arrêt complet si trop proche
    elif max_distance >= SAFE_DISTANCE:
        acceleration_raw = MAX_ACCELERATION  # Accélération maximale si assez loin
    else:
        # Interpolation linéaire entre MIN_SAFE_DISTANCE et SAFE_DISTANCE
        acceleration_raw = ((max_distance - MIN_SAFE_DISTANCE) / 
                           (SAFE_DISTANCE - MIN_SAFE_DISTANCE)) * MAX_ACCELERATION
    
    # Appliquer le PID pour le servo et l'accélération
    servo_rotation = apply_pid_servo(servo_rotation_raw, error)
    acceleration = apply_pid_accel(acceleration_raw)
    
    with lock:
        current_commands = {
            "angle_max": max_angle,
            "distance_max": max_distance,
            "erreur": error,
            "rotation_servo": servo_rotation,
            "rotation_servo_raw": servo_rotation_raw,
            "acceleration": acceleration,
            "acceleration_raw": acceleration_raw,
            "timestamp": time.time()
        }

def apply_pid_servo(target_rotation, error):
    """Applique un contrôle PID à la rotation du servo"""
    global PID_SERVO
    
    current_time = time.time()
    if PID_SERVO["last_time"] == 0:
        PID_SERVO["last_time"] = current_time
        PID_SERVO["last_error"] = error
        return target_rotation
    
    # Calcul du delta de temps
    dt = current_time - PID_SERVO["last_time"]
    if dt <= 0:
        return target_rotation
    
    # Calcul des termes PID
    error_diff = error - PID_SERVO["last_error"]
    
    # Terme proportionnel
    p_term = PID_SERVO["Kp"] * error
    
    # Terme intégral (avec anti-windup)
    PID_SERVO["integral"] += error * dt
    PID_SERVO["integral"] = np.clip(PID_SERVO["integral"], -10, 10)  # Anti-windup
    i_term = PID_SERVO["Ki"] * PID_SERVO["integral"]
    
    # Terme dérivé
    d_term = 0
    if dt > 0:
        d_term = PID_SERVO["Kd"] * (error_diff / dt)
    
    # Calcul de la sortie PID
    output = -1 * (p_term + i_term + d_term)  # -1 pour inverser la relation erreur/sortie
    output = np.clip(output, -MAX_SERVO_ANGLE, MAX_SERVO_ANGLE)
    
    # Mise à jour des variables
    PID_SERVO["last_error"] = error
    PID_SERVO["last_time"] = current_time
    
    return output

def apply_pid_accel(target_accel):
    """Applique un contrôle PID à l'accélération"""
    global PID_ACCEL
    
    current_time = time.time()
    if PID_ACCEL["last_time"] == 0:
        PID_ACCEL["last_time"] = current_time
        PID_ACCEL["target"] = target_accel
        return target_accel
    
    # Mise à jour progressive de la consigne pour éviter les changements brusques
    alpha = 0.7  # Facteur de lissage (entre 0 et 1)
    PID_ACCEL["target"] = alpha * PID_ACCEL["target"] + (1 - alpha) * target_accel
    
    # Calcul de l'erreur (différence entre la consigne actuelle et la cible)
    error = PID_ACCEL["target"] - target_accel
    
    # Calcul du delta de temps
    dt = current_time - PID_ACCEL["last_time"]
    if dt <= 0:
        return PID_ACCEL["target"]
    
    # Calcul des termes PID
    error_diff = error - PID_ACCEL["last_error"]
    
    # Terme proportionnel
    p_term = PID_ACCEL["Kp"] * error
    
    # Terme intégral (avec anti-windup)
    PID_ACCEL["integral"] += error * dt
    PID_ACCEL["integral"] = np.clip(PID_ACCEL["integral"], -MAX_ACCELERATION/2, MAX_ACCELERATION/2)
    i_term = PID_ACCEL["Ki"] * PID_ACCEL["integral"]
    
    # Terme dérivé
    d_term = 0
    if dt > 0:
        d_term = PID_ACCEL["Kd"] * (error_diff / dt)
    
    # Calcul de la sortie PID et application
    output = PID_ACCEL["target"] - (p_term + i_term + d_term)
    output = np.clip(output, 0, MAX_ACCELERATION)
    
    # Mise à jour des variables
    PID_ACCEL["last_error"] = error
    PID_ACCEL["last_time"] = current_time
    
    return output

def generate_display():
    """Génère l'affichage pour les commandes de contrôle"""
    global current_commands
    
    # Vérifier la fraîcheur des données (pas plus vieilles que 1 seconde)
    if time.time() - current_commands["timestamp"] > 1:
        freshness = "[bold red]OBSOLÈTE[/bold red]"
    else:
        freshness = "[bold green]OK[/bold green]"
    
    # Créer un tableau pour les commandes
    table = Table(title="Commandes de Navigation avec PID", show_lines=True)
    table.add_column("Paramètre", style="cyan")
    table.add_column("Valeur", style="yellow")
    table.add_column("Brut → PID", style="magenta")
    table.add_column("État", style="green")
    
    # Ajouter les données au tableau
    angle_max = current_commands["angle_max"]
    distance_max = current_commands["distance_max"] / 1000  # Convertir en mètres
    erreur = current_commands["erreur"]
    rotation = current_commands["rotation_servo"]
    rotation_raw = current_commands["rotation_servo_raw"]
    acceleration = current_commands["acceleration"]
    acceleration_raw = current_commands["acceleration_raw"]
    
    # Définir la zone d'angle
    zone = "0-90°" if 0 <= angle_max <= 90 else "270-360°"
    
    table.add_row("Angle max détecté", f"{angle_max:.2f}° ({zone})", "", freshness)
    table.add_row("Distance max", f"{distance_max:.2f} m", "", "")
    table.add_row("Erreur d'angle", f"{erreur:.2f}°", "", "")
    
    # Ajouter les lignes pour les commandes de contrôle avec des barres de progression
    servo_progress = Progress(
        TextColumn("[bold blue]Servo[/bold blue]"), 
        BarColumn(bar_width=40),
        TextColumn("[bold]{task.percentage:.0f}%")
    )
    servo_task = servo_progress.add_task("", total=2*MAX_SERVO_ANGLE, 
                                         completed=rotation + MAX_SERVO_ANGLE)
    
    accel_progress = Progress(
        TextColumn("[bold green]Accélération[/bold green]"), 
        BarColumn(bar_width=40, complete_style="green"),
        TextColumn("[bold]{task.percentage:.0f}%")
    )
    accel_task = accel_progress.add_task("", total=MAX_ACCELERATION, completed=acceleration)
    
    # Ajouter les barres de progression au tableau
    table.add_row(
        "Rotation servo", 
        f"{rotation:.2f}° (max: ±{MAX_SERVO_ANGLE}°)", 
        f"{rotation_raw:.2f}° → {rotation:.2f}°",
        servo_progress
    )
    table.add_row(
        "Accélération", 
        f"{acceleration:.1f}% (max: {MAX_ACCELERATION}%)",
        f"{acceleration_raw:.1f}% → {acceleration:.1f}%",
        accel_progress
    )
    
    # Ajouter des informations sur le contrôleur PID
    pid_info = Table.grid()
    pid_info.add_column("PID Servo", style="blue")
    pid_info.add_column("PID Accel", style="green")
    
    servo_pid = f"P={PID_SERVO['Kp']}, I={PID_SERVO['Ki']}, D={PID_SERVO['Kd']}"
    accel_pid = f"P={PID_ACCEL['Kp']}, I={PID_ACCEL['Ki']}, D={PID_ACCEL['Kd']}"
    
    pid_info.add_row(servo_pid, accel_pid)
    
    table.add_row("Paramètres PID", "", "", pid_info)
    
    return Panel(table, border_style="yellow")

def process_data():
    """Thread qui traite les données du LIDAR et calcule les commandes"""
    global running
    
    console.print("[bold yellow]Traitement des données et calcul des commandes démarré...[/bold yellow]")
    
    with Live(generate_display(), refresh_per_second=5) as live:
        while running:
            try:
                calculate_control_commands()  # Calcul des commandes
                live.update(generate_display())  # Mise à jour de l'affichage
                time.sleep(0.2)  # 5 fois par seconde
            except Exception as e:
                console.print(f"[bold red]Erreur de traitement: {e}[/bold red]")
                time.sleep(1)

def output_control_commands():
    """Thread qui envoie les commandes à intervalle régulier (simulation)"""
    global current_commands, running
    
    console.print("[bold blue]Démarrage de la sortie des commandes...[/bold blue]")
    
    while running:
        with lock:
            cmd = current_commands.copy()
        
        # Ici, vous pourriez envoyer les commandes à votre matériel
        # Par exemple via GPIO, série, ou autre
        
        # Pour le moment, affichons simplement les commandes dans la console
        if cmd["timestamp"] > 0 and time.time() - cmd["timestamp"] < 1:
            console.print(f"[dim]CMD: Rotation={cmd['rotation_servo']:.1f}° | Accel={cmd['acceleration']:.1f}%[/dim]")
        
        time.sleep(0.5)  # Envoi des commandes 2 fois par seconde

if __name__ == "__main__":
    console.print(Panel.fit(
        "[bold cyan]SYSTÈME DE NAVIGATION LIDAR AVEC PID[/bold cyan]\n"
        f"[yellow]Contrôle de direction avec servo-moteur (±{MAX_SERVO_ANGLE}°)[/yellow]\n"
        f"[green]Accélération variable selon distance (max: {MAX_ACCELERATION}%)[/green]\n"
        "[magenta]Contrôleur PID pour stabiliser les commandes[/magenta]\n"
        "[white]Appuyez sur Ctrl+C pour quitter[/white]",
        border_style="green"
    ))
    
    # Création des threads
    thread_lidar = threading.Thread(target=receive_lidar_data, daemon=True)
    thread_process = threading.Thread(target=process_data, daemon=True)
    thread_output = threading.Thread(target=output_control_commands, daemon=True)

    # Lancement des threads
    thread_lidar.start()
    thread_process.start()
    thread_output.start()

    try:
        while True:
            time.sleep(1)  # Boucle principale en veille
    except KeyboardInterrupt:
        console.print("[bold yellow]Arrêt demandé par l'utilisateur...[/bold yellow]")
        running = False  # Demande d'arrêt des threads
        thread_lidar.join(timeout=2)
        thread_process.join(timeout=2)
        thread_output.join(timeout=2)
        console.print("[bold green]Programme terminé avec succès.[/bold green]")
