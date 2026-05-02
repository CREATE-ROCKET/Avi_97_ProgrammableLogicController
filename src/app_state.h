#pragma once

#include <Arduino.h>
#include "CANCREATE.h"
#include <freertos/semphr.h>

constexpr uint8_t EMERGENCY_SW_PIN = 4;
constexpr uint8_t DUMP_PIN = 14;
constexpr uint8_t SEPARATE_PIN = 25;
constexpr uint8_t FILL_PIN = 26;
constexpr uint8_t O2_PIN = 27;
constexpr uint8_t IGNI_PIN = 32;
constexpr uint8_t CAN_RX_PIN = 22;
constexpr uint8_t CAN_TX_PIN = 23;

constexpr uint32_t CAN_ID_FROM_CONTROL_PANEL = 0x101;
constexpr uint32_t CAN_ID_TO_CTRL_MAIN_STATE = 0x103;
constexpr uint32_t CAN_ID_TO_MAIN_VALVE = 0x105;
constexpr uint32_t CAN_ID_FROM_MAIN_VAVLE = 0x107;

constexpr int16_t MAIN_VALVE_CLOSED_ANGLE_X10 = -348; // -34.8 deg
constexpr int16_t MAIN_VALVE_OPEN_ANGLE_X10 = 552;    // 55.2 deg

constexpr unsigned long long IGNITION_WAIT_MS = 20000;
constexpr unsigned long long MAIN_VALVE_OPEN_DELAY_MS = 3000;
constexpr unsigned long long IGNITION_SEQUENCE_TIMEOUT_MS = 10000;
constexpr unsigned long long COMMUNICATION_TIMEOUT_MS = 3000;

enum STATE : uint8_t
{
  IDLE = 0,
  IGNITION = 1,
  TIMEOUT = 2,
  CANERROR = 3
};

extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t emergencySemaphore;

extern STATE systemState;
extern bool hasTimedOut;
extern int16_t mainValveAngleX10;
extern bool openO2Flag;
extern bool executeIgnitionFlag;

extern unsigned long long lastPanelRxTime;
extern unsigned long long lastValveRxTime;

extern uint8_t currentButtonState;

extern CAN_CREATE CAN;
