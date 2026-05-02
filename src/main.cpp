#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "app_state.h"
#include "tasks/tasks.h"

void setup()
{
  Serial.begin(115200);

  pinMode(DUMP_PIN, OUTPUT);
  pinMode(FILL_PIN, OUTPUT);
  pinMode(O2_PIN, OUTPUT);
  pinMode(SEPARATE_PIN, OUTPUT);
  pinMode(IGNI_PIN, OUTPUT);

  digitalWrite(DUMP_PIN, LOW);
  digitalWrite(FILL_PIN, LOW);
  digitalWrite(O2_PIN, LOW);
  digitalWrite(SEPARATE_PIN, LOW);
  digitalWrite(IGNI_PIN, LOW);

  stateMutex = xSemaphoreCreateMutex();
  emergencySemaphore = xSemaphoreCreateBinary();

  if (CAN.begin(100E3, CAN_RX_PIN, CAN_TX_PIN))
  {
    Serial.println("CAN Init Failed");
    while (1)
      ;
  }
  delay(1000);

  xTaskCreateUniversal(canReceiveTask, "CAN_Rx", 4096, NULL, 3, NULL, 0);
  xTaskCreateUniversal(canTransmitTask, "CAN_Tx", 4096, NULL, 2, NULL, 0);
  xTaskCreateUniversal(stateControlTask, "State_Ctrl", 4096, NULL, 2, NULL, 1);
  xTaskCreateUniversal(solenoidValveTask, "Valve_Ctrl", 4096, NULL, 2, NULL, 1);
  xTaskCreateUniversal(executeIgnitionTask, "Ignition", 4096, NULL, 2, NULL, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
