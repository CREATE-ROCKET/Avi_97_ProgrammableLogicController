#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "../app_state.h"

namespace
{
bool shouldAbortIgnitionTask()
{
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  bool shouldAbort = (systemState == CANERROR) || !executeIgnitionFlag;
  xSemaphoreGive(stateMutex);

  if (shouldAbort)
  {
    digitalWrite(IGNI_PIN, LOW);
  }

  return shouldAbort;
}
}

void executeIgnitionTask(void *pvParameters)
{
  while (1)
  {
    bool startIgnition = false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    startIgnition = executeIgnitionFlag;
    xSemaphoreGive(stateMutex);

    if (startIgnition)
    {
      vTaskDelay(pdMS_TO_TICKS(IGNITION_WAIT_MS));
      if (shouldAbortIgnitionTask())
      {
        continue;
      }

      Serial.println("IGNI HIGH");
      digitalWrite(IGNI_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(MAIN_VALVE_OPEN_DELAY_MS));
      if (shouldAbortIgnitionTask())
      {
        continue;
      }

      digitalWrite(IGNI_PIN, LOW);
      Serial.println("IGNI LOW");

      xSemaphoreTake(stateMutex, portMAX_DELAY);
      mainValveAngleX10 = MAIN_VALVE_OPEN_ANGLE_X10;
      openO2Flag = false;
      xSemaphoreGive(stateMutex);
      vTaskDelay(pdMS_TO_TICKS(IGNITION_SEQUENCE_TIMEOUT_MS));
      if (shouldAbortIgnitionTask())
      {
        continue;
      }

      xSemaphoreTake(stateMutex, portMAX_DELAY);
      systemState = TIMEOUT;
      hasTimedOut = true;
      executeIgnitionFlag = false;
      xSemaphoreGive(stateMutex);

      digitalWrite(IGNI_PIN, LOW);
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
