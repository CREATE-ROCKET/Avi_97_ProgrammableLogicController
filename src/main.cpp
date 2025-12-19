#include <Arduino.h>
#include "CANCREATE.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// --- Pin Definitions --- IO22は使える
constexpr uint8_t DUMP_PIN = 23;
constexpr uint8_t FILL_PIN = 21;
constexpr uint8_t O2_PIN = 18;
constexpr uint8_t FD_PIN = 19;
constexpr uint8_t IGNI_PIN = 25;
constexpr uint8_t CAN_RX_PIN = 27;
constexpr uint8_t CAN_TX_PIN = 13;

// --- CAN ID Definitions ---
constexpr uint32_t CAN_ID_FROM_CONTROL_PANEL = 0x101;
constexpr uint32_t CAN_ID_TO_CTRL_ACK = 0x103;
constexpr uint32_t CAN_ID_FROM_CTRL_ACK = 0x104;
constexpr uint32_t CAN_ID_TO_MAIN_VALVE = 0x105;

// --- Timing and Threshold Constants ---
constexpr unsigned long long IGNITION_WAIT_MS = 4500;              // 点火ボタンを押し続けてからイグナイターをONにするまでの時間 (ms)
constexpr unsigned long long IGNITION_SEQUENCE_TIMEOUT_MS = 10000; // シーケンスのタイムアウト (ms)
constexpr unsigned long long MAIN_VALVE_OPEN_DELAY_MS = 6000;      // 点火シーケンス開始後,メインバルブを開くまでの時間 (ms)
constexpr unsigned long long CTRL_PANEL_TIMEOUT_MS = 3000;         // コントロールパネルの通信タイムアウト (ms)
constexpr unsigned long long IGNI_BTN_HOLD_DEBOUNCE = 20;          // 点火ボタンのチャタリング防止時間 (ms)
// --- Enums for State Management ---
enum IgnitionState
{
  IDLE,
  BUTTON_HELD,     // 点火ボタン押下中
  SEQUENCE_ACTIVE, // 点火シーケンス実行中
  TIMEOUT          // 終状態
};

// --- Global State Variables & Mutex ---
IgnitionState ignitionState = IDLE;
unsigned long long lastCtrlPanelTime = 0;
unsigned long long ignitionButtonHoldTimer = 0;
unsigned long long ignitionSequenceTimer = 0;

SemaphoreHandle_t stateMutex;
CAN_CREATE CAN(true);

// --- Task & Function Prototypes ---
void CANRecvTask(void *pvParameters);
void checkCtrlPanelConnectionTask(void *pvParameters);
void handleCANMessage(const can_return_t &message);
void executeIgnitionSequence();

void setup()
{
  Serial.begin(115200);
  Serial.println("PLC Initializing...");

  pinMode(DUMP_PIN, OUTPUT);
  pinMode(FILL_PIN, OUTPUT);
  pinMode(O2_PIN, OUTPUT);
  pinMode(FD_PIN, OUTPUT);
  pinMode(IGNI_PIN, OUTPUT);

  digitalWrite(IGNI_PIN, LOW);
  digitalWrite(DUMP_PIN, LOW);
  digitalWrite(FILL_PIN, LOW);
  digitalWrite(O2_PIN, LOW);
  digitalWrite(FD_PIN, LOW);
  digitalWrite(IGNI_PIN, LOW);

  stateMutex = xSemaphoreCreateMutex();

  if (CAN.begin(100E3, CAN_RX_PIN, CAN_TX_PIN))
  {
    while (1)
      ;
  }
  delay(1000);
  // switch (CAN.test())
  // {
  // case CAN_SUCCESS:
  //   Serial.println("Success!!!");
  //   break;
  // case CAN_UNKNOWN_ERROR:
  //   Serial.println("Unknown error occurred");
  //   break;
  // case CAN_NO_RESPONSE_ERROR:
  //   Serial.println("No response error");
  //   break;
  // case CAN_CONTROLLER_ERROR:
  //   Serial.println("CAN CONTROLLER ERROR");
  //   break;
  // default:
  //   break;
  // }
  xTaskCreateUniversal(CANRecvTask, "CANRecvTask", 4096, NULL, 3, NULL, 0);
  xTaskCreateUniversal(checkCtrlPanelConnectionTask, "checkCtrlPanelConnectionTask", 4096, NULL, 2, NULL, 0);
}

void loop()
{
  executeIgnitionSequence();
  delay(100);
}

void CANRecvTask(void *pvParameters)
{
  while (1)
  {
    if (CAN.available())
    {
      can_return_t message;
      if (!CAN.readWithDetail(&message))
      {
        handleCANMessage(message);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void handleCANMessage(const can_return_t &message)
{
  if (message.id == CAN_ID_FROM_CTRL_ACK)
  {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    lastCtrlPanelTime = millis();
    xSemaphoreGive(stateMutex);
  }

  if (message.id == CAN_ID_FROM_CONTROL_PANEL)
  {
    // --- Parse Control Panel Data ---
    bool dumpCmd = (message.data[0] >> 0) & 1;
    bool fillCmd = (message.data[0] >> 1) & 1;
    bool fireCmd = (message.data[0] >> 2) & 1;
    bool FDCmd = (message.data[0] >> 3) & 1;
    bool valveSetCmd = (message.data[0] >> 4) & 1;
    // Serial.print("DumpCmd: ");
    // Serial.println(dumpCmd);
    // Serial.print("FillCmd: ");
    // Serial.println(fillCmd);
    // Serial.print("FireCmd: ");
    // Serial.println(fireCmd);
    // Serial.print("FDCmd: ");
    // Serial.println(FDCmd);
    // Serial.print("ValveSetCmd: ");
    // Serial.println(valveSetCmd);

    // --- Ignition Logic  ---
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    if (fireCmd)
    {
      if (ignitionState == IDLE)
      {
        ignitionState = BUTTON_HELD;
        ignitionButtonHoldTimer = millis();
        Serial.println("BUTTON_HELD");
      }

      // 20ms以上押し続けられているかチェック
      if (ignitionState == BUTTON_HELD && (millis() - ignitionButtonHoldTimer > IGNI_BTN_HOLD_DEBOUNCE))
      {
        ignitionState = SEQUENCE_ACTIVE;
        ignitionSequenceTimer = millis(); // Start sequence timer
        Serial.println("SEQUENCE_ACTIVE");
      }
    }
    else if (ignitionState == BUTTON_HELD)
    {
      ignitionState = IDLE;
      Serial.println("IGNITION CANCELED");
    }

    xSemaphoreGive(stateMutex);

    // --- Other Commands (Direct hardware control, no mutex needed) ---
    digitalWrite(FD_PIN, FDCmd);
    digitalWrite(FILL_PIN, fillCmd);
    digitalWrite(DUMP_PIN, dumpCmd);

    if (valveSetCmd)
    {
      uint8_t main_valve_data[2] = {111, 0};                  // valveの角度をこれより開きたいなら2個目の引数で判定するようにする
      CAN.sendData(CAN_ID_TO_MAIN_VALVE, main_valve_data, 2); // valve基板側で-9になるように調整
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      if (ignitionState != TIMEOUT)
      {
        ignitionState = IDLE; // Stop ignition sequence on valve reset
      }
      xSemaphoreGive(stateMutex);
    }
  }
}

void executeIgnitionSequence()
{
  IgnitionState currentState;
  unsigned long long sequenceStartTime;

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  currentState = ignitionState;
  sequenceStartTime = ignitionSequenceTimer;
  xSemaphoreGive(stateMutex);

  if (currentState != SEQUENCE_ACTIVE)
  {
    digitalWrite(IGNI_PIN, LOW);
    return;
  }
  digitalWrite(O2_PIN, HIGH); // 酸素用電磁弁をON
  unsigned int elapsed = millis() - sequenceStartTime;
  if (elapsed > IGNITION_SEQUENCE_TIMEOUT_MS)
  {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    ignitionState = TIMEOUT;
    xSemaphoreGive(stateMutex);
    digitalWrite(IGNI_PIN, LOW);
    return;
  }
  if (elapsed > IGNITION_WAIT_MS)
  {
    digitalWrite(IGNI_PIN, HIGH); // イグナイターON
    Serial.println("IGNI ON");
  }
  if (elapsed > MAIN_VALVE_OPEN_DELAY_MS)
  {
    // Send open command to main valve
    uint8_t main_valve_data[2] = {255, 1};                  // valveの角度をこれより開きたいなら2個目の引数で判定するようにする
    CAN.sendData(CAN_ID_TO_MAIN_VALVE, main_valve_data, 2); // valve基板側で135になるように調整
    // O2電磁弁をOFF
    digitalWrite(O2_PIN, LOW);
    digitalWrite(IGNI_PIN, LOW);
  }
}

void checkCtrlPanelConnectionTask(void *pvParameters)
{
  while (1)
  {
    // Send Acknowledgment to Control Panel
    uint8_t ack_data = 0;
    CAN.sendData(CAN_ID_TO_CTRL_ACK, &ack_data, 1);

    unsigned long long lastTime;
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    lastTime = lastCtrlPanelTime;
    xSemaphoreGive(stateMutex);

    if (millis() - lastTime > CTRL_PANEL_TIMEOUT_MS)
    {
      digitalWrite(IGNI_PIN, LOW);
      digitalWrite(O2_PIN, LOW);

      xSemaphoreTake(stateMutex, portMAX_DELAY);
      // missing control panel connection, reset state
      ignitionState = TIMEOUT;
      xSemaphoreGive(stateMutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}