// ===============================
// PID EN CASCADE (ENCODEURS + MPU)
// ===============================
// À AJOUTER DANS bourak.h AVANT la section "PID MPU (Optimisé)"
// 
// UTILISATION DANS LA BOUCLE :
// 
//   if (moving) {
//     cascadePID(speedBase);  // speedBase = 150 à 240 (que tu contrôles)
//     if (MASAFA >= distanceTarget) {
//       stopCascadePID();
//       moving = false;
//     }
//   }
//
// ===============================

// --- PID POSITION (feedback encodeurs) ---
struct PIDPosition {
  float Kp = 0.8;      // Contrôle proportionnel de position
  float Ki = 0.01;     // Contrôle intégral (anti-drift moteur)
  float Kd = 0.15;     // Contrôle dérivé (amortissement)
  
  float integral = 0;
  float lastError = 0;
  
  long savedTicksL = 0;
  long savedTicksR = 0;
  
  void reset() {
    integral = 0;
    lastError = 0;
    savedTicksL = readEncoderL();
    savedTicksR = readEncoderR();
  }
  
  // Retourne la correction de vitesse basée sur l'écart encodeurs L/R
  float compute() {
    long ticksL = readEncoderL() - savedTicksL;
    long ticksR = readEncoderR() - savedTicksR;
    
    // Écart entre les deux moteurs (si L > R, dévier à gauche)
    float error = (float)(ticksL - ticksR);
    
    float P = Kp * error;
    integral = constrain(integral + error, -500, 500);
    float I = Ki * integral;
    float D = Kd * (error - lastError);
    lastError = error;
    
    return P + I + D;
  }
} pidPos;

// --- PID ORIENTATION (feedback MPU) ---
struct PIDOrientation {
  float Kp = 6.5;     // Correction d'angle agressif
  float Ki = 0.05;    // Anti-drift gyro
  float Kd = 1.8;     // Amortissement des oscillations
  
  float integral = 0;
  float lastError = 0;
  float targetAngle = 0;
  
  void reset(float startAngle) {
    integral = 0;
    lastError = 0;
    targetAngle = startAngle;
  }
  
  // Retourne la correction basée sur la déviation d'angle MPU
  float compute() {
    calcANG();
    float error = angleZ - targetAngle;
    
    // Normaliser l'angle entre -180 et +180
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;
    
    float P = Kp * error;
    integral = constrain(integral + error, -200, 200);
    float I = Ki * integral;
    float D = Kd * (error - lastError);
    lastError = error;
    
    return P + I + D;
  }
} pidOri;

// --- FONCTION PRINCIPALE PID EN CASCADE ---
// Tu appelles cette fonction à chaque itération de loop()
// En passant la vitesse de base que TU CONTRÔLES
void cascadePID(float speedBase) {
  // ========== BOUCLE INTERNE 1 : FEEDBACK POSITION (encodeurs) ==========
  // Corrige si moteur L tourne plus vite que moteur R
  float posCorrection = pidPos.compute();
  
  // ========== BOUCLE INTERNE 2 : FEEDBACK ORIENTATION (MPU) ==========
  // Corrige si le robot dévie de son angle cible
  float oriCorrection = pidOri.compute();
  
  // ========== COMBINAISON DES CORRECTIONS ==========
  // Vitesses initiales = la vitesse de base que tu fixes
  int speedL = (int)speedBase;
  int speedR = (int)speedBase;
  
  // Correction POSITION : si L a plus de ticks, ralentir L et accélérer R
  // posCorrection > 0 → L trop rapide → ralentir L
  speedL -= posCorrection * 0.5f;
  speedR += posCorrection * 0.5f;
  
  // Correction ORIENTATION : si angle MPU dévie à droite, freiner R et accélérer L
  // oriCorrection > 0 → robot penche à droite → ralentir R
  speedR -= oriCorrection;
  speedL += oriCorrection;
  
  // ========== SATURATION & SÉCURITÉ ==========
  speedL = constrain(speedL, -240, 240);
  speedR = constrain(speedR, -240, 240);
  
  // Appliquer aux moteurs
  forward_brake_fast(speedR, speedL);
}

// ========== INITIALISATION ==========
// Appeler UNE FOIS avant de commencer le mouvement
void initCascadePID(float speedBase) {
  // Lire l'angle MPU actuel comme référence (le robot doit être droit)
  calcANG();
  pidPos.reset();         // Réinitialise encodeurs
  pidOri.reset(angleZ);   // Réinitialise gyro avec angle courant
}

// ========== ARRÊT ==========
void stopCascadePID() {
  forward_brake_fast(0, 0);
  pidPos.integral = 0;
  pidOri.integral = 0;
}

// ========== AJUSTER LES GAINS EN TEMPS RÉEL ==========
// Pour tuner le PID sans recompiler :
// setCascadePID(Kp_pos, Ki_pos, Kd_pos, Kp_ori, Ki_ori, Kd_ori);
void setCascadePID(float kpPos, float kiPos, float kdPos,
                   float kpOri, float kiOri, float kdOri) {
  pidPos.Kp = kpPos;
  pidPos.Ki = kiPos;
  pidPos.Kd = kdPos;
  
  pidOri.Kp = kpOri;
  pidOri.Ki = kiOri;
  pidOri.Kd = kdOri;
}

// ========== UTILITAIRES DEBUG ==========
void debugCascadePID(int delayMs = 500) {
  char buf[64];
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);
    
    // Afficher feedbacks
    snprintf(buf, sizeof(buf), "L-R: %.0f", pidPos.lastError);
    u8g2.drawStr(0, 10, buf);
    
    snprintf(buf, sizeof(buf), "Ang: %.1f", angleZ);
    u8g2.drawStr(0, 20, buf);
    
    snprintf(buf, sizeof(buf), "Dist: %.1f cm", MASAFA);
    u8g2.drawStr(0, 30, buf);
    
    snprintf(buf, sizeof(buf), "Pos_cor: %.0f", pidPos.lastError);
    u8g2.drawStr(0, 40, buf);
    
    snprintf(buf, sizeof(buf), "Ori_cor: %.0f", pidOri.lastError);
    u8g2.drawStr(0, 50, buf);
  } while (u8g2.nextPage());
  delay(delayMs);
}
