
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
  
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while(1);
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    while(1);
  }
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
#define N_FAULT  PD2   // Broche d'alerte erreur matérielle du DRV8301 (Partagée M0/M1)

#define M0_CS    PC4  
#define M1_CS    PC5  

// Un moteur de drone standard à 14 aimants possède 7 paires de pôles
#define POLE_PAIRS 7 

BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
BLDCMotor motor = BLDCMotor(POLE_PAIRS); 

void setup() {
  // 1. Désactiver le SPI pour éviter les interférences
  pinMode(M0_CS, OUTPUT);
  pinMode(M1_CS, OUTPUT);
  digitalWrite(M0_CS, HIGH);
  digitalWrite(M1_CS, HIGH);

  // 2. Configurer la broche d'alerte erreur du DRV
  pinMode(N_FAULT, INPUT_PULLUP);

  // 3. Reset matériel obligatoire du DRV8301
  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, LOW);
  delay(100);
  digitalWrite(EN_GATE, HIGH); // Allumage du driver
  delay(100); 

  // 4. Initialisation USB
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  SimpleFOCDebug::enable(&Serial);
  Serial.println("\n=====================================");
  Serial.println("   DIAGNOSTIC MOTEUR DE DRONE (M0)   ");
  Serial.println("=====================================");

  driver.voltage_power_supply = 24; // Ta tension d'alimentation principale
  driver.pwm_frequency = 25000;
  driver.dead_zone = 0.04f;
  
  // Démarrage volontairement conservateur pour éviter un pic de courant au lancement.
  driver.voltage_limit = 3.0; // Tension maximale appliquée au moteur 
  
  if (!driver.init()) {
    Serial.println("[-] Configuration du Driver : ÉCHEC");
    while(1);
  }
  Serial.println("[+] Configuration du Driver : OK");
  
  motor.linkDriver(&driver);
  motor.voltage_limit = 3.0; 
  motor.controller = MotionControlType::velocity_openloop;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
  if (!motor.init()) {
    Serial.println("[-] Configuration du Moteur : ÉCHEC");
    while(1);
  }
  Serial.println("[+] Configuration du Moteur : OK");

  // Démarrage des modules
  driver.enable();
  motor.enable();
  
  // Vérification immédiate de la puce de puissance
  if(digitalRead(N_FAULT) == LOW) {
    Serial.println("[!!!] ERREUR : Le DRV8301 s'est mis en sécurité dès le démarrage !");
  } else {
    Serial.println("[+] DRV8301 Sain : Prêt à envoyer du courant.");
  }
  Serial.println("=====================================");
}

unsigned long dernierMessage = 0;

const float couple_stationnaire = 3.0f;
const float angle_electrique_fixe = 0.0f;

void loop() {
  // SÉCURITÉ : Si la puce DRV8301 disjoncte en cours de route
  if (digitalRead(N_FAULT) == LOW) {
    if (millis() - dernierMessage > 1000) {
      Serial.println("[ERREUR MATÉRIELLE] Le DRV8301 a disjoncté (Overcurrent / Courant trop fort) !");
      dernierMessage = millis();
    }
    driver.disable();
    motor.disable();
    return; // On stoppe tout
  }

  // Champ électrique fixe: test simple de couple stationnaire.
  motor.setPhaseVoltage(couple_stationnaire, 0.0f, angle_electrique_fixe);

  if (millis() - dernierMessage > 1000) {
    Serial.print("[M0] Couple stationnaire. Uq : ");
    Serial.print(couple_stationnaire);
    Serial.print(" V, angle : ");
    Serial.print(angle_electrique_fixe);
    Serial.println(" V");
    dernierMessage = millis();
  }
}