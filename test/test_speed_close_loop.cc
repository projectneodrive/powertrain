#include <Arduino.h>
#include <SimpleFOC.h>

// ============================================================================
// CONFIGURATION DE L'HORLOGE ODRIVE (Validée !)
// ============================================================================
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;             
  RCC_OscInitStruct.PLL.PLLN = 336;           
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  
  RCC_OscInitStruct.PLL.PLLQ = 7;             
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { while(1); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { while(1); }
}

// ============================================================================
// PINOUT ODRIVE V3.6
// ============================================================================
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15
#define EN_GATE  PB12
#define N_FAULT  PD2   

#define M0_CS    PC13  
#define M1_CS    PC14  

#define ENCODER_A PB6 
#define ENCODER_B PB7 

#define POLE_PAIRS 7 

// ============================================================================
// INSTANCES SIMPLEFOC
// ============================================================================
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
BLDCMotor motor = BLDCMotor(POLE_PAIRS); 
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, 600);

void doA(){ encoder.handleA(); }
void doB(){ encoder.handleB(); }

// --- CONFIGURATION DE LA VITESSE CIBLE ---
float vitesse_cible = 20.0f; // Vitesse en rad/s (Positif ou négatif)
unsigned long dernierMessage = 0;

void setup() {
  pinMode(M0_CS, OUTPUT); digitalWrite(M0_CS, HIGH);
  pinMode(M1_CS, OUTPUT); digitalWrite(M1_CS, HIGH);
  pinMode(N_FAULT, INPUT_PULLUP);

  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, LOW); delay(50);
  digitalWrite(EN_GATE, HIGH); delay(50); 

  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println("\n--- MODE : CONTRÔLE DE VITESSE BOUCLE FERMÉE ---");

  // 1. Initialisation Encodeur
  encoder.init();
  encoder.enableInterrupts(doA, doB); 
  motor.linkSensor(&encoder);

  // 2. Configuration Driver
  driver.voltage_power_supply = 24; 
  driver.pwm_frequency = 25000;
  driver.voltage_limit = 2.0; 
  driver.init();
  motor.linkDriver(&driver);
  
  // 3. Configuration du Mode de Contrôle (Vitesse via Tension)
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  
// --- CONFIGURATION DU REGULATEUR VITESSE (CORRIGÉE) ---
  
  // CRUCIAL : On ne calcule le PID que toutes les 50 boucles FOC (Fréquence idéale ~2kHz)
  motor.motion_downsample = 50; 

  // On augmente le gain P pour donner un vrai coup de collier au démarrage
  motor.PID_velocity.P = 0.04f;    // 4.0 rad/s d'erreur générera instantanément le max autorisé
  
  // On augmente l'intégrale pour qu'elle pousse de plus en plus fort si le moteur est coincé
  motor.PID_velocity.I = 0.80f;   
  
  motor.PID_velocity.D = 0.0f;    

  motor.voltage_limit = 2.0f;     // On garde la limite de sécurité à 2.0V
  motor.voltage_sensor_align = 0.8f; // On garde la tension de calibration à 2.0V

  // --- LE SECRET ANTI-VIBRATION : LE FILTRE PASSE-BAS ---
  // On dit à SimpleFOC de moyenner la vitesse sur 20 millisecondes (0.02s) 
  // pour gommer les vibrations de l'encodeur.
  //motor.LPF_velocity.Tf = 0.02f;

  motor.PID_velocity.output_ramp = 200.0f;

  // 5. Initialisation du moteur
  motor.init();
  driver.enable();
  motor.enable();

  // 6. Alignement et Calibration
  Serial.println("Lancement de la calibration initFOC()...");
  motor.initFOC();
  // VÉRIFICATION DU RÉSULTAT DE LA CALIBRATION
  Serial.print("Statut direction : ");
  Serial.println(motor.sensor_direction == Direction::CW ? "CW (Horaire)" : "CCW (Anti-horaire)");
  Serial.print("Offset electrique : ");
  Serial.print(motor.zero_electric_angle);
  Serial.println(" rad");
  
  if(motor.sensor_direction == Direction::UNKNOWN) {
    Serial.println("[!!!] ATTENTION : La calibration a échoué à cause des saccades.");
  } else {
    Serial.println("Calibration OK ! Régulation en cours...");
  }
}

void loop() {
  // SÉCURITÉ MATÉRIELLE INFRAFRANCHISSABLE
  if (digitalRead(N_FAULT) == LOW) {
    driver.disable(); motor.disable(); 
    if (millis() - dernierMessage > 1000) {
      Serial.println("[ERREUR MATÉRIELLE] Le DRV8301 a coupé l'alimentation !");
      dernierMessage = millis();
    }
    return; 
  }

  // 1. Boucle FOC (Gère la commutation des phases à haute vitesse)
  motor.loopFOC();

  // 2. Calcul du PID de vitesse (Prend la consigne et ajuste la tension)
  motor.move(vitesse_cible);

  // 3. Affichage des performances de régulation
  if (millis() - dernierMessage > 200) {
    Serial.print("Target: ");
    Serial.print(vitesse_cible);
    Serial.print(" rad/s | Réelle: ");
    Serial.print(motor.shaft_velocity); // Vitesse filtrée calculée par SimpleFOC
    Serial.print(" rad/s | Tension PID: ");
    Serial.print(motor.voltage.q);      // Tension instantanée injectée
    Serial.println(" V");
    dernierMessage = millis();
  }
}