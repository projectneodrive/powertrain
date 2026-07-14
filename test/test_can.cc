#include <Arduino.h>

// Handle global pour le périphérique CAN1
CAN_HandleTypeDef hcan1;

// La fonction SystemClock_Config est reprise de votre code
// pour garantir que l'APB1 tourne bien à 42 MHz.
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { while (1); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; // APB1 = 168 / 4 = 42 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { while (1); }
}

void setup_can() {
  // 1. Activer les horloges pour CAN1 et le port GPIO B
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // 2. Configurer les pins PB8 (RX) et PB9 (TX) - Standard sur F405/ODrive
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // 3. Configurer le baudrate à 100 kbps
  // Horloge APB1 = 42 MHz. 
  // Avec Prescaler = 21, le tick CAN est à 2 MHz (0.5 µs).
  // 1 bit = 10 µs (100 kbps) -> il faut 20 TQ (Time Quanta) par bit.
  // Sync (1 TQ) + TimeSeg1 (16 TQ) + TimeSeg2 (3 TQ) = 20 TQ.
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21; 
  hcan1.Init.Mode = CAN_MODE_NORMAL ; 
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE; // Réessaie si pas d'acquittement (ACK)
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Serial.println("Erreur Init CAN");
  } else {
    Serial.println("CAN Init OK");
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Serial.println("Erreur Start CAN");
  } else {
    Serial.println("CAN Démarré");
  }
}

void send_heartbeat() {
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  uint8_t TxData[8] = {0, 0, 0, 0, 8, 0, 0, 0}; // Payload factice : State = AXIS_CLOSED_LOOP (8)

  // Envoi sur l'ID 0x001 (Node 0 + CMD_HEARTBEAT) attendu par votre ESP32
  TxHeader.StdId = 0x001; 
  TxHeader.ExtId = 0x00;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8; // 8 octets envoyés
  TxHeader.TransmitGlobalTime = DISABLE;

  // On place le message dans une des boîtes aux lettres d'envoi
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
    Serial.println("[-] Échec de l'envoi CAN (Bus off ou buffers pleins)");
  } else {
    Serial.println("[+] Message Heartbeat 0x001 envoyé");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000); 
  Serial.println("\n--- STM32 CAN Sender Test ---");
  
  setup_can();
}

void loop() {
  send_heartbeat();
  delay(1000); // Envoie un message toutes les secondes
}