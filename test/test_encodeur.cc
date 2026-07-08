#include <Arduino.h>
#include <SimpleFOC.h>

// ============================================================================
// CONFIGURATION DE L'HORLOGE ODRIVE (Obligatoire pour le STM32F405)
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
// CONFIGURATION DU CAPTEUR
// ============================================================================
#define ENCODER_A PB4 
#define ENCODER_B PB5 

// Initialisation : 600 PPR * 4 = 2400 CPR
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, 600);

// Routines d'interruption
void doA(){ encoder.handleA(); }
void doB(){ encoder.handleB(); }

unsigned long dernierMessage = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("\n--- LECTURE BRUTE DE L'ENCODEUR ---");

  // Initialisation de l'encodeur et activation des interruptions
  encoder.init();
  encoder.enableInterrupts(doA, doB);

  Serial.println("Encodeur prêt ! Tourne le moteur à la main...");
}

void loop() {
  // Important : met à jour en continu les calculs internes de l'encodeur
  encoder.update();

  // Affichage toutes les 100 ms (10 fois par seconde) pour une lecture fluide
  if (millis() - dernierMessage > 100) {
    Serial.print("Position (Radians) : ");
    Serial.print(encoder.getAngle());
    
    Serial.print(" | Position (Degrés) : ");
    // Conversion simple : Radians * 180 / PI
    Serial.print(encoder.getAngle() * 57.295779513f); 
    
    Serial.print(" | Vitesse estimée : ");
    Serial.print(encoder.getVelocity());
    Serial.println(" rad/s");

    dernierMessage = millis();
  }
}
