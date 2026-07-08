#include <Arduino.h>
#include <SimpleFOC.h>

// ============================================================================
// CONFIGURATION DE L'HORLOGE ODRIVE
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

// Broches de l'encodeur (ODrive M0 par défaut)
#define ENCODER_A PB4 
#define ENCODER_B PB5 

#define POLE_PAIRS 7 

// ============================================================================
// INSTANCES SIMPLEFOC
// ============================================================================
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
BLDCMotor motor = BLDCMotor(POLE_PAIRS); 

// Initialisation de l'encodeur : 600 PPR * 4 = 2400 CPR
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, 600);

// Routines d'interruption matérielle pour l'encodeur
void doA(){ encoder.handleA(); }
void doB(){ encoder.handleB(); }

// Cible de couple (en Volts)
float target_voltage = 0.0f;
unsigned long dernierMessage = 0;

void setup() {
  pinMode(M0_CS, OUTPUT); digitalWrite(M0_CS, HIGH);
  pinMode(M1_CS, OUTPUT); digitalWrite(M1_CS, HIGH);
  pinMode(N_FAULT, INPUT_PULLUP);

  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, LOW);
  delay(50);
  digitalWrite(EN_GATE, HIGH); 
  delay(50); 

  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println("\n--- DEMARRAGE CLOSED-LOOP (ENCODEUR) ---");

  SimpleFOCDebug::enable(&Serial);

  // 1. Initialisation de l'encodeur
  encoder.init();
  encoder.enableInterrupts(doA, doB); 
  motor.linkSensor(&encoder);

  // 2. Configuration du Driver
  driver.voltage_power_supply = 24; 
  driver.pwm_frequency = 25000;
  driver.voltage_limit = 2.0; // Sécurité matérielle globale
  driver.init();
  motor.linkDriver(&driver);
  
  // 3. Configuration du Moteur
  motor.voltage_limit = 2.0;   // Limite de sécurité pour la calibration et l'utilisation
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_sensor_align = 0.8f; 

  motor.init();
  driver.enable();
  motor.enable();

  // 4. Calibration (Alignement Capteur / Moteur)
  // ATTENTION : Le moteur va faire des petits bruits et bouger tout seul.
  // C'est normal, il cherche le "Zéro" électrique par rapport à l'encodeur.
  Serial.println("Lancement de la calibration (Ne touche pas au moteur)...");
  motor.initFOC();
  Serial.println("Calibration terminée !");
  
  delay(1000);
  
  // On applique un très léger couple de 1.0V pour tester
  target_voltage = 0.8f; 
  Serial.println("Application d'un couple de 0.8V.");
}

void loop() {
  // SÉCURITÉ MATÉRIELLE
  if (digitalRead(N_FAULT) == LOW) {
    driver.disable(); motor.disable(); 
    if (millis() - dernierMessage > 1000) {
      Serial.println("[!!!] ERREUR DRV8301");
      dernierMessage = millis();
    }
    return; 
  }

  // 1. Boucle de contrôle FOC ultra-rapide
  // Lit l'encodeur et met à jour les phases instantanément
  motor.loopFOC();

  // 2. Commande de couple (Ici, on envoie 1.0V)
  // Si tu mets une valeur négative, le moteur forcera dans l'autre sens
  motor.move(target_voltage);

  // 3. Affichage pour vérifier que l'encodeur lit bien
  if (millis() - dernierMessage > 500) {
    Serial.print("Angle Encodeur : ");
    Serial.print(encoder.getAngle());
    Serial.print(" rad | Couple appliqué : ");
    Serial.print(target_voltage);
    Serial.println(" V");
    dernierMessage = millis();
  }
}