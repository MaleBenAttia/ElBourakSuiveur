#include "bourak.h"

// Définition des broches du multiplexeur (basé sur votre code)
#define S0 25
#define S1 26
#define S2 27
#define S3 14
#define SIG 36

void setup() {
  // Initialisation du moniteur série
  Serial.begin(115200);
  delay(1000);

  // Configuration des broches de sélection en SORTIE
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

   pinMode(MOT_R_PWM1, OUTPUT);
    pinMode(MOT_R_PWM2, OUTPUT);
     pinMode(MOT_L_PWM1, OUTPUT);
      pinMode(MOT_L_PWM2, OUTPUT);

  // Configuration de la broche de lecture en ENTRÉE (sans Pull-up !)
  pinMode(SIG, INPUT);

  Serial.println("=== TEST DES 16 CAPTEURS (SANS BIBLIOTHEQUE) ===");
}

void loop() {
   forward_brake_fast(100, 100);
  /*
  // Boucle pour lire les 16 canaux du multiplexeur (de 0 à 15)
  for (int channel = 0; channel < 16; channel++) {
    
    // 1. Sélectionner le canal sur le multiplexeur via les broches S0-S3
    // bitRead() permet d'extraire le bit 0, 1, 2 ou 3 du numéro du canal
    digitalWrite(S0, bitRead(channel, 0));
    digitalWrite(S1, bitRead(channel, 1));
    digitalWrite(S2, bitRead(channel, 2));
    digitalWrite(S3, bitRead(channel, 3));

    // Petite pause de 10 microsecondes pour laisser le signal se stabiliser
    delayMicroseconds(10);

    // 2. Lire la valeur analogique du canal sélectionné
    int valeurCapteur = analogRead(SIG);

    // 3. Afficher la valeur (entre 0 et 4095 sur un ESP32)
    Serial.print(valeurCapteur);
    Serial.print("\t"); // Ajoute une tabulation pour aligner les colonnes
  }
  
  // Retour à la ligne après avoir lu les 16 capteurs
  Serial.println();
  
  // Pause de 100 ms avant la prochaine lecture globale
  delay(500);
  */
}