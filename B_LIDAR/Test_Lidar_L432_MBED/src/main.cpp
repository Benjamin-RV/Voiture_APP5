#include <Arduino.h>

#define LIDAR_TX PA2  // TX → LIDAR RX
#define LIDAR_RX PA3  // RX ← LIDAR TX
#define LIDAR_PWM PA8 // PWM pour moteur du LIDAR

HardwareSerial lidarSerial(2);  // UART2 pour la communication avec le LIDAR

// Commandes de démarrage LIDAR (à adapter avec la datasheet)
uint8_t startLidarCmd[] = {0xA5, 0x60, 0x01, 0x00, 0x00, 0x00};  // Exemple pour démarrer
uint8_t startMeasureCmd[] = {0xA5, 0x60, 0x01, 0x01, 0x00, 0x00}; // Démarrer la mesure

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    lidarSerial.begin(115200);  // Initialisation de l'UART pour le LIDAR
    Serial.println("Initialisation du LIDAR.");

    initLidar();  // Initialisation du LIDAR
    Serial.println("LIDAR initialisé.");
}

void loop() {
    // Exemple de lecture des données du LIDAR et affichage
    while (lidarSerial.available()) {
        char c = lidarSerial.read();
        Serial.print(c);  // Afficher les données reçues du LIDAR
    }

    delay(10);  // Petit délai
}

// Fonction pour initialiser le LIDAR
void initLidar() {
    // Envoyer la trame pour démarrer le LIDAR
    sendLidarCommand(startLidarCmd, sizeof(startLidarCmd));
    delay(100);  // Attendre un peu pour la réponse

    // Démarrer la prise de mesure
    sendLidarCommand(startMeasureCmd, sizeof(startMeasureCmd));
    delay(100);  // Attendre un peu pour la réponse

    // Configurer la vitesse du moteur du LIDAR via PWM (50% du cycle utile)
    pinMode(LIDAR_PWM, OUTPUT);
    Timer1.initialize(1000);  // 1000 µs = 1 ms, pour une fréquence de 1 kHz
    Timer1.pwm(LIDAR_PWM, 512);  // 50% du cycle utile

    Serial.println("Commande de démarrage et mesure envoyée.");
}

// Fonction pour envoyer une commande au LIDAR via UART
void sendLidarCommand(uint8_t* command, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        lidarSerial.write(command[i]);  // Envoyer chaque octet de la commande
    }
}
