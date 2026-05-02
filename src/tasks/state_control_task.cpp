#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "../app_state.h"

void stateControlTask(void *pvParameters)
{
  while (1)
  {
    bool canErrorActive = false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);

    unsigned long long now = millis();
    bool panelAlive = (now - lastPanelRxTime) < COMMUNICATION_TIMEOUT_MS;
    bool valveAlive = (now - lastValveRxTime) < COMMUNICATION_TIMEOUT_MS;

    if (panelAlive && valveAlive)
    {
      if (systemState == CANERROR)
      {
        systemState = hasTimedOut ? TIMEOUT : IDLE;
      }
    }
    else
    {
      systemState = CANERROR;
      openO2Flag = false;
      executeIgnitionFlag = false;
      canErrorActive = true;
    }

    xSemaphoreGive(stateMutex);

    if (canErrorActive)
    {
      digitalWrite(O2_PIN, LOW);
      digitalWrite(FILL_PIN, LOW);
      digitalWrite(IGNI_PIN, LOW);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
