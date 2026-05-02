#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "../app_state.h"
#include "../main_valve.h"

void canReceiveTask(void *pvParameters)
{
  while (1)
  {
    if (CAN.available())
    {
      can_return_t message;
      if (!CAN.readWithDetail(&message))
      {
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        if (message.id == CAN_ID_FROM_CONTROL_PANEL)
        {
          lastPanelRxTime = millis();
          if (message.size > 0)
          {
            currentButtonState = message.data[0];
          }
        }
        else if (message.id == CAN_ID_FROM_MAIN_VAVLE)
        {
          lastValveRxTime = millis();
        }
        xSemaphoreGive(stateMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void canTransmitTask(void *pvParameters)
{
  while (1)
  {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    int16_t valveAngleX10 = mainValveAngleX10;
    uint8_t mainState = systemState;
    xSemaphoreGive(stateMutex);

    sendMainValveAngle(valveAngleX10);
    CAN.sendData(CAN_ID_TO_CTRL_MAIN_STATE, &mainState, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
