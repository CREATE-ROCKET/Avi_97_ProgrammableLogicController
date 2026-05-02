#include "main_valve.h"

#include "app_state.h"

void sendMainValveAngle(int16_t angleX10)
{
  const uint16_t rawAngle = static_cast<uint16_t>(angleX10);
  uint8_t angleData[2] = {
    static_cast<uint8_t>(rawAngle & 0xFF),
    static_cast<uint8_t>((rawAngle >> 8) & 0xFF),
  };
  CAN.sendData(CAN_ID_TO_MAIN_VALVE, angleData, sizeof(angleData));
}
