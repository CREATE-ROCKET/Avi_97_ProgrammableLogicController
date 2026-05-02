#include "app_state.h"

SemaphoreHandle_t stateMutex;
SemaphoreHandle_t emergencySemaphore;

STATE systemState = IDLE;
bool hasTimedOut = false;
int16_t mainValveAngleX10 = MAIN_VALVE_CLOSED_ANGLE_X10;
bool openO2Flag = false;
bool executeIgnitionFlag = false;

unsigned long long lastPanelRxTime = 0;
unsigned long long lastValveRxTime = 0;

uint8_t currentButtonState = 0;

CAN_CREATE CAN(true);
