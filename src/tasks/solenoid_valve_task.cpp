#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "../app_state.h"

void solenoidValveTask(void *pvParameters)
{
  bool prevO2TestFlag = false;
  bool manualO2Open = false;

  while (1)
  {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    uint8_t buttons = currentButtonState;
    STATE currentState = systemState;
    bool sequenceO2Open = openO2Flag;
    xSemaphoreGive(stateMutex);

    bool dumpFlag = (buttons & 1) == 1;
    bool fireFlag = ((buttons >> 1) & 1) == 1;
    bool fillFlag = ((buttons >> 2) & 1) == 1;
    bool separateFlag = ((buttons >> 3) & 1) == 1;
    bool valveSetFlag = ((buttons >> 4) & 1) == 1;
    bool o2TestFlag = ((buttons >> 5) & 1) == 1;
    bool mainValveOpenFlag = ((buttons >> 6) & 1) == 1;
    bool isIgnitionRunning = (currentState == IGNITION) || (fireFlag && currentState == IDLE);

    if (currentState == CANERROR)
    {
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      openO2Flag = false;
      executeIgnitionFlag = false;
      xSemaphoreGive(stateMutex);

      manualO2Open = false;
      prevO2TestFlag = o2TestFlag;

      digitalWrite(FILL_PIN, LOW);
      digitalWrite(O2_PIN, LOW);
      digitalWrite(IGNI_PIN, LOW);

      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    bool forceOutputsOff = false;
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    currentState = systemState;
    isIgnitionRunning = (currentState == IGNITION) || (fireFlag && currentState == IDLE);

    if (currentState == CANERROR)
    {
      openO2Flag = false;
      executeIgnitionFlag = false;
      sequenceO2Open = false;
      forceOutputsOff = true;
    }
    else if (fireFlag)
    {
      if (currentState == IDLE)
      {
        openO2Flag = true;
        systemState = IGNITION;
        executeIgnitionFlag = true;
        currentState = IGNITION;
        isIgnitionRunning = true;
      }
      else if (currentState == TIMEOUT)
      {
        mainValveAngleX10 = MAIN_VALVE_OPEN_ANGLE_X10;
      }
    }

    if (!forceOutputsOff && valveSetFlag && !isIgnitionRunning)
      mainValveAngleX10 = MAIN_VALVE_CLOSED_ANGLE_X10;

    if (!forceOutputsOff && mainValveOpenFlag)
      mainValveAngleX10 = MAIN_VALVE_OPEN_ANGLE_X10;

    sequenceO2Open = openO2Flag;
    xSemaphoreGive(stateMutex);

    if (forceOutputsOff)
    {
      manualO2Open = false;
      prevO2TestFlag = o2TestFlag;

      digitalWrite(FILL_PIN, LOW);
      digitalWrite(O2_PIN, LOW);
      digitalWrite(IGNI_PIN, LOW);

      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (!isIgnitionRunning)
    {
      digitalWrite(DUMP_PIN, dumpFlag);
      digitalWrite(FILL_PIN, fillFlag);
      digitalWrite(SEPARATE_PIN, separateFlag);
    }

    if (isIgnitionRunning)
    {
      manualO2Open = false;
    }
    else
    {
      if (o2TestFlag && !prevO2TestFlag)
      {
        manualO2Open = true;
      }
      else if (!o2TestFlag && prevO2TestFlag)
      {
        manualO2Open = false;
      }
    }
    prevO2TestFlag = o2TestFlag;

    bool isO2Open = sequenceO2Open || (!isIgnitionRunning && manualO2Open);
    digitalWrite(O2_PIN, isO2Open);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
