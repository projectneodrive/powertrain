#include <Arduino.h>
#include "driver/twai.h"

// Configuration des pins - Ajustez selon votre schéma
#define TWAI_TX_PIN GPIO_NUM_5
#define TWAI_RX_PIN GPIO_NUM_4

void setup() {
  Serial.begin(115200);

  // Configuration du driver TWAI (CAN)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS(); // Doit correspondre à votre bus
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Installation et démarrage
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installé.");
  } else {
    Serial.println("Erreur installation driver.");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("Bus CAN démarré, écoute en cours...");
  } else {
    Serial.println("Erreur démarrage bus.");
    return;
  }
}

void loop() {
  twai_message_t message;
  
  // Vérifie si un message est présent dans la file d'attente
  if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.print("ID: 0x");
    Serial.print(message.identifier, HEX);
    Serial.print(" | DLC: ");
    Serial.print(message.data_length_code);
    Serial.print(" | Data: ");
    
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.print(message.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}