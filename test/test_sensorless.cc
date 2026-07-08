#include <Arduino.h>
#include <SimpleFOC.h>

// ============================================================================
// CONFIGURATION DE L'HORLOGE ODRIVE (Parfaite, on ne change rien)
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
// PINOUT FONCTIONNEL
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

#define POLE_PAIRS 7 

BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
BLDCMotor motor = BLDCMotor(POLE_PAIRS); 

// --- VARIABLES POUR LA ROTATION MAISON ---
float electrical_angle = 0.0f;
float target_velocity = 100.0f;       // Vitesse en rad/s (Augmente ou baisse pour tester)
const float tension_moteur = 2.0f;  // La tension de 3V qui fonctionne chez toi
unsigned long last_micros = 0;
unsigned long dernierMessage = 0;

void setup() {
  // 1. Désactiver le SPI (Ta configuration fonctionnelle)
  pinMode(M0_CS, OUTPUT);
  pinMode(M1_CS, OUTPUT);
  digitalWrite(M0_CS, HIGH);
  digitalWrite(M1_CS, HIGH);

  pinMode(N_FAULT, INPUT_PULLUP);

  // 2. Reset matériel obligatoire du DRV8301
  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, LOW);
  delay(100);
  digitalWrite(EN_GATE, HIGH); 
  delay(100); 

  // 3. Initialisation USB
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  SimpleFOCDebug::enable(&Serial);
  Serial.println("\n=====================================");
  Serial.println("   ROTATION COMMANDE DIRECTE (M0)    ");
  Serial.println("=====================================");

  driver.voltage_power_supply = 24; 
  driver.pwm_frequency = 25000;
  driver.dead_zone = 0.04f;
  driver.voltage_limit = tension_moteur; 
  
  if (!driver.init()) {
    Serial.println("[-] Configuration du Driver : ÉCHEC");
    while(1);
  }
  
  motor.linkDriver(&driver);
  motor.voltage_limit = tension_moteur; 
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
  if (!motor.init()) {
    Serial.println("[-] Configuration du Moteur : ÉCHEC");
    while(1);
  }

  // Démarrage des modules
  driver.enable();
  motor.enable();
  
  if(digitalRead(N_FAULT) == LOW) {
    Serial.println("[!!!] ERREUR : Le DRV8301 est en sécurité !");
  } else {
    Serial.println("[+] Système Sain : Génération du mouvement fluide...");
  }
  Serial.println("=====================================");
  
  // Initialisation du chronomètre
  last_micros = micros();
}

void loop() {
  // SÉCURITÉ MATÉRIELLE
  if (digitalRead(N_FAULT) == LOW) {
    driver.disable();
    motor.disable();
    return; 
  }

  // 1. Calcul du temps précis écoulé depuis le dernier passage (en secondes)
  unsigned long current_micros = micros();
  float dt = (current_micros - last_micros) / 1000000.0f;
  last_micros = current_micros;

  // Anti-bug au premier démarrage ou en cas de saut de temps
  if (dt > 0.1f || dt <= 0.0f) dt = 0.001f;

  // 2. Calcul du nouvel angle en fonction de la vitesse demandée (Angle = Vitesse * Temps)
  electrical_angle += target_velocity * dt;

  // 3. Normalisation de l'angle entre 0 et 2*PI pour éviter que la variable ne déborde
  if (electrical_angle > _2PI) {
    electrical_angle -= _2PI;
  }

  // 4. Envoi DIRECT aux transistors via la fonction magique
  motor.setPhaseVoltage(tension_moteur, 0.0f, electrical_angle);

  // Debug toutes les secondes
  if (millis() - dernierMessage > 1000) {
    Serial.print("[M0] En mouvement. Angle Elec : ");
    Serial.print(electrical_angle);
    Serial.print(" rad | Vitesse : ");
    Serial.print(target_velocity);
    Serial.println(" rad/s");
    dernierMessage = millis();
  }
}