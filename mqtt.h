#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#include <WiFi.h>
#include <PubSubClient.h>

// ========== CONFIGURATION RÉSEAU ==========
const char* WIFI_SSID = "TUNISIETELECOM-5G-3GcA";
const char* WIFI_PASS = "EDxDPS4Zg";
const char* MQTT_SERVER = "192.168.100.19";

// ========== TOPIC D'ENVOI UNIQUE ==========
#define TOPIC_TELEMETRY "robot/telemetry"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ========== CONNEXION WiFi ==========
void connectWiFi() {
  Serial.print("Connexion WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK");
}

// ========== CONNEXION MQTT (sans abonnement) ==========
void mqttConnect() {
  mqttClient.setServer(MQTT_SERVER, 1883);
  while (!mqttClient.connected()) {
    Serial.print("Connexion MQTT...");
    // Nom client unique (peut être personnalisé)
    if (mqttClient.connect("RobotTelemetry")) {
      Serial.println(" OK");
    } else {
      Serial.print(" échec, code=");
      Serial.print(mqttClient.state());
      Serial.println(" -> nouvel essai dans 1s");
      delay(1000);
    }
  }
}

// ========== BOUCLE MQTT (maintien de connexion) ==========
void mqttLoop() {
  if (!mqttClient.connected()) mqttConnect();
  mqttClient.loop();   // nécessaire pour garder la connexion vivante
}

// ========== ENVOI ULTRA‑RAPIDE DES DONNÉES CRITIQUES ==========
// QoS 0, pas de retain, JSON compact
// Paramètres :
//   angle      : angle actuel (float)
//   distance   : distance parcourue en cm (float)
//   encL, encR : ticks encodeurs gauche/droit (long)
//   motorA, motorB : vitesses moteurs (int)
//   pidLineError   : erreur du PID suiveur de ligne (int)
//   pidMPUError    : erreur du PID MPU (float)
void envoyerTelemetrie(float angle, float distance,
                       long encL, long encR,
                       int motorA, int motorB,
                       int pidLineError, float pidMPUError) {
  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "{\"a\":%.2f,\"d\":%.2f,\"eL\":%ld,\"eR\":%ld,\"mA\":%d,\"mB\":%d,\"pLE\":%d,\"pME\":%.2f}",
           angle, distance, encL, encR, motorA, motorB, pidLineError, pidMPUError);
  mqttClient.publish(TOPIC_TELEMETRY, buffer, false); // false = QoS 0
}
void envoyerMessageSimple() {
  // La fonction publish prend deux paramètres : le "topic" et le "message"
  mqttClient.publish("robot/A", "Message de test !");
}

#endif