#include <Arduino.h>
#include <SimpleFOC.h>
#include <STM32FreeRTOS.h> // Indispensable pour utiliser FreeRTOS sur STM32duino

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

#define ENCODER_A PB4 
#define ENCODER_B PB5 

#define POLE_PAIRS 7 

// ============================================================================
// INSTANCES SIMPLEFOC
// ============================================================================

BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
BLDCMotor motor = BLDCMotor(POLE_PAIRS); 
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, 600);

void doA(){ encoder.handleA(); }
void doB(){ encoder.handleB(); }

volatile float target_voltage = 0.8f;
volatile bool fault_detected = false;
volatile bool system_ready = false; // Nouveau flag pour synchroniser les tâches

void TaskMotor(void *pvParameters);
void TaskComms(void *pvParameters);

// ============================================================================
// SETUP : Démarrage basique uniquement
// ============================================================================
void setup() {
  pinMode(M0_CS, OUTPUT); digitalWrite(M0_CS, HIGH);
  pinMode(M1_CS, OUTPUT); digitalWrite(M1_CS, HIGH);
  pinMode(N_FAULT, INPUT_PULLUP);
  pinMode(EN_GATE, OUTPUT);

  Serial.begin(115200);
  while (!Serial) { delay(10); } 
  SimpleFOCDebug::enable(&Serial);
  Serial.println("\n--- DEMARRAGE FREERTOS ---");

  // On lance directement FreeRTOS. 
  // L'initialisation du moteur se fera dans TaskMotor.
  
  xTaskCreate(TaskMotor, "TaskMotor", 2048, NULL, 1, NULL); // Stack augmenté à 2048
  xTaskCreate(TaskComms, "TaskComms", 512, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop() {} // Toujours vide

// ============================================================================
// TÂCHE MOTEUR : Initialisation + Contrôle
// ============================================================================
void TaskMotor(void *pvParameters) {
  (void) pvParameters;

  // 1. Réveil du DRV8301 (avec les délais FreeRTOS sécurisés)
  digitalWrite(EN_GATE, LOW);
  vTaskDelay(pdMS_TO_TICKS(50));
  digitalWrite(EN_GATE, HIGH); 
  vTaskDelay(pdMS_TO_TICKS(50));

  // 2. Initialisation SimpleFOC complète (maintenant que l'horloge FreeRTOS tourne)
  encoder.init();
  encoder.enableInterrupts(doA, doB); 
  motor.linkSensor(&encoder);

  driver.voltage_power_supply = 24; 
  driver.pwm_frequency = 25000;
  driver.voltage_limit = 2.0; 
  driver.init();
  motor.linkDriver(&driver);
  
  motor.voltage_limit = 2.0;  
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_sensor_align = 0.8f; 

  motor.init();
  driver.enable();
  motor.enable();

  Serial.println("Lancement de la calibration SimpleFOC...");
  motor.initFOC();
  Serial.println("Calibration terminée ! Moteur prêt.");
  
  system_ready = true; // On indique à la tâche Comms qu'elle peut commencer

  // Boucle de contrôle FOC
  for (;;) {
    if (fault_detected) {
      driver.disable(); motor.disable();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue; 
    }

    motor.loopFOC();
    motor.move(target_voltage);
    taskYIELD(); 
  }
}

// ============================================================================
// TÂCHE COMMS
// ============================================================================
void TaskComms(void *pvParameters) {
  (void) pvParameters;

  // On attend sagement que la TaskMotor ait fini de calibrer SimpleFOC
  while (!system_ready) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  for (;;) {
    if (digitalRead(N_FAULT) == LOW) {
      fault_detected = true;
      Serial.println("[!!!] ERREUR DRV8301 (N_FAULT)");
    } else {
      fault_detected = false;
    }

    Serial.print("Angle : ");
    Serial.print(encoder.getAngle());
    Serial.print(" rad | Cmd: ");
    Serial.print(target_voltage);
    Serial.println(" V");

    vTaskDelay(pdMS_TO_TICKS(500)); 
  }
}