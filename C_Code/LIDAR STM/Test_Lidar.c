#include "rplidar.h" // Inclure le header du SDK

int main() {
    // Créer une instance du driver RPLIDAR
    rp::standalone::rplidar::RPlidarDriver *drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        fprintf(stderr, "Erreur : Impossible de créer l'instance du driver\n");
        return -1;
    }

    // Connexion au LiDAR
    if (drv->connect("/dev/ttyUSB0", 115200) != rp::standalone::rplidar::RESULT_OK) {
        fprintf(stderr, "Erreur : Impossible de connecter au LiDAR\n");
        return -1;
    }

    // Vérification de l'état du LiDAR
    rplidar_response_device_health_t health;
    if (drv->getHealth(health) != rp::standalone::rplidar::RESULT_OK || health.status != RPLIDAR_STATUS_OK) {
        fprintf(stderr, "Erreur : Le LiDAR est en mauvais état\n");
        return -1;
    }

    // Démarrage du balayage
    drv->startScan();

    // Lecture des données
    while (true) {
        rplidar_response_measurement_node_t nodes[8192];
        size_t count = _countof(nodes);

        if (drv->grabScanData(nodes, count) == rp::standalone::rplidar::RESULT_OK) {
            drv->ascendScanData(nodes, count);
            for (size_t i = 0; i < count; ++i) {
                printf("Angle: %.2f Dist: %.2f\n",
                       nodes[i].angle_q6_checkbit / 64.0f,
                       nodes[i].distance_q2 / 4.0f);
            }
        }
    }

    // Arrêt et nettoyage
    drv->stop();
    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(drv);
    return 0;
}
