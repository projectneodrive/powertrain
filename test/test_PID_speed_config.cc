#include <Arduino.h>
#include <SimpleFOC.h>
#include <EEPROM.h> // <-- Nouvelle bibliothèque pour la mémoire

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

#define ENCODER_A PB6 
#define ENCODER_B PB7 
#define POLE_PAIRS 7 

BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
BLDCMotor motor = BLDCMotor(POLE_PAIRS); 
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, 600);

void doA(){ encoder.handleA(); }
void doB(){ encoder.handleB(); }
 
// ============================================================================
// STRUCTURE DE SAUVEGARDE EEPROM
// ============================================================================
// On regroupe toutes les variables qu'on veut rendre persistantes
struct ConfigurationMemoire {
  uint32_t code_magique; // Sert à savoir si la mémoire a déjà été formatée
  float pid_p;
  float pid_i;
  float pid_d;
};

ConfigurationMemoire config;
const uint32_t CODE_VALIDATION = 0xAABBCCDD; // Code arbitraire de sécurité

// Tes valeurs de calibration physique (à remplacer par les tiennes)
float saved_zero_angle = 0.00f; 
Direction saved_direction = Direction::UNKNOWN; 

float vitesse_cible = 5.0f; 
unsigned long dernierMessage = 0;
bool mode_calibration = false; 

void setup() {
  pinMode(M0_CS, OUTPUT); digitalWrite(M0_CS, HIGH);
  pinMode(M1_CS, OUTPUT); digitalWrite(M1_CS, HIGH);
  pinMode(N_FAULT, INPUT_PULLUP);

  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, LOW); delay(50);
  digitalWrite(EN_GATE, HIGH); delay(50); 

  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println("\n--- DEMARRAGE SYSTEME ---");

  // ============================================================================
  // LECTURE DE LA MÉMOIRE PERMANENTE
  // ============================================================================
  EEPROM.get(0, config);

  if (config.code_magique != CODE_VALIDATION) {
    // C'est la première fois qu'on lance la carte (ou mémoire corrompue)
    Serial.println("[!] Memoire vierge. Chargement des valeurs PID par defaut.");
    config.code_magique = CODE_VALIDATION;
    config.pid_p = 0.04f;
    config.pid_i = 0.80f;
    config.pid_d = 0.0001f;
    EEPROM.put(0, config); // On sauvegarde les valeurs par défaut
  } else {
    Serial.println("[+] Valeurs PID chargees depuis la memoire flash !");
  }

  Serial.println("Tapez 'init' dans les 5s pour calibrer, sinon demarrage auto...");

  unsigned long debut_attente = millis();
  while(millis() - debut_attente < 5000) {
    if(Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim(); 
      if(input.equalsIgnoreCase("init")) {
        mode_calibration = true;
        Serial.println("\n[!] Lancement calibration physique...");
        break; 
      }
    }
  }

  encoder.init();
  encoder.enableInterrupts(doA, doB); 
  motor.linkSensor(&encoder);

  driver.voltage_power_supply = 24; 
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);
  
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  
  // ============================================================================
  // APPLICATION DES GAINS PID DÉDUITS DE LA MÉMOIRE
  // ============================================================================
  motor.motion_downsample = 50; 
  motor.PID_velocity.P = config.pid_p;    
  motor.PID_velocity.I = config.pid_i;     
  motor.PID_velocity.D = config.pid_d;  

  motor.voltage_limit = 2.0f;          
  motor.voltage_sensor_align = 0.8f; 
  motor.PID_velocity.output_ramp = 200.0f; 

  motor.init();

  if(mode_calibration == false && saved_direction != Direction::UNKNOWN) {
    motor.zero_electric_angle = saved_zero_angle;
    motor.sensor_direction = saved_direction;
  } else {
    motor.zero_electric_angle = 0; 
    motor.sensor_direction = Direction::UNKNOWN;
  }

  driver.enable();
  motor.enable();
  motor.initFOC();

  Serial.println("\n--- MOTEUR PRET ---");
  Serial.println("Commandes dispo : P0.05, I6.0, D0.001, SAVE");
}

void loop() {
  if (digitalRead(N_FAULT) == LOW) {
    driver.disable(); motor.disable(); 
    return; 
  }

  motor.loopFOC();
  motor.move(vitesse_cible);

  // ============================================================================
  // MINI-TERMINAL POUR LE TUNING DU PID EN DIRECT
  // ============================================================================
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // Nettoie les espaces et retours chariot

    if (cmd.startsWith("P")) {
      config.pid_p = cmd.substring(1).toFloat();
      motor.PID_velocity.P = config.pid_p;
      Serial.print("Nouveau P applique : "); Serial.println(config.pid_p, 4);
    } 
    else if (cmd.startsWith("I")) {
      config.pid_i = cmd.substring(1).toFloat();
      motor.PID_velocity.I = config.pid_i;
      Serial.print("Nouveau I applique : "); Serial.println(config.pid_i, 4);
    } 
    else if (cmd.startsWith("D")) {
      config.pid_d = cmd.substring(1).toFloat();
      motor.PID_velocity.D = config.pid_d;
      Serial.print("Nouveau D applique : "); Serial.println(config.pid_d, 6);
    } 
    else if (cmd.equalsIgnoreCase("SAVE")) {
      EEPROM.put(0, config);
      Serial.println(">>> SUCCES : Valeurs PID sauvegardees en memoire permanente !");
    }
  }

  // Affichage périodique
  if (millis() - dernierMessage > 1000) {
    Serial.print("Vitesse Reelle: ");
    Serial.print(motor.shaft_velocity);
    Serial.print(" rad/s | PID actuel -> P: ");
    Serial.print(motor.PID_velocity.P, 3);
    Serial.print(" I: ");
    Serial.print(motor.PID_velocity.I, 2);
    Serial.print(" D: ");
    Serial.println(motor.PID_velocity.D, 5);
    dernierMessage = millis();
  }
}