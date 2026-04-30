#include <Arduino.h>
#include "CANCREATE.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// --- Pin Definitions  ---
constexpr uint8_t EMERGENCY_SW_PIN = 4;
constexpr uint8_t DUMP_PIN = 14;
constexpr uint8_t SEPARATE_PIN = 25;
constexpr uint8_t FILL_PIN = 26;
constexpr uint8_t O2_PIN = 27;
constexpr uint8_t IGNI_PIN = 32;
constexpr uint8_t CAN_RX_PIN = 22;
constexpr uint8_t CAN_TX_PIN = 23;

// --- CAN ID Definitions  ---
constexpr uint32_t CAN_ID_FROM_CONTROL_PANEL = 0x101;
constexpr uint32_t CAN_ID_TO_CTRL_MAIN_STATE = 0x103;
constexpr uint32_t CAN_ID_TO_MAIN_VALVE = 0x105;
constexpr uint32_t CAN_ID_FROM_MAIN_VAVLE = 0x107;

// --- Time Definitions ---
constexpr unsigned long long IGNITION_WAIT_MS = 20000;             // 点火ボタンを押してからイグナイターON (ms)
constexpr unsigned long long MAIN_VALVE_OPEN_DELAY_MS = 2000;      // イグナイターONからメインバルブを開くまでの時間 (ms)
constexpr unsigned long long IGNITION_SEQUENCE_TIMEOUT_MS = 10000; // メインバルブ開放後、タイムアウトまでの時間 (ms)
constexpr unsigned long long COMMUNICATION_TIMEOUT_MS = 3000;      // 通信タイムアウト (ms)

// --- Enums for State Management ---
enum STATE : uint8_t
{
  IDLE = 0,
  IGNITION = 1,
  TIMEOUT = 2,
  CANERROR = 3
};

// --- Global State Variables & Mutex ---
SemaphoreHandle_t stateMutex;
SemaphoreHandle_t emergencySemaphore;

STATE systemState = IDLE;
bool hasTimedOut = false;
bool openValveFlag = false;
bool openO2Flag = false;
bool executeIgnitionFlag = false;

unsigned long long lastPanelRxTime = 0;
unsigned long long lastValveRxTime = 0;

uint8_t currentButtonState = 0;

CAN_CREATE CAN(true);

// --- Task & Function Prototypes ---
void canReceiveTask(void *pvParameters);
void canTransmitTask(void *pvParameters);
void stateControlTask(void *pvParameters);
void executeIgnitionTask(void *pvParameters);
void solenoidValveTask(void *pvParameters);

// void IRAM_ATTR emergencyISR()
// {
//   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//   // セマフォを与えて、待機中のタスクを起こす
//   xSemaphoreGiveFromISR(emergencySemaphore, &xHigherPriorityTaskWoken);
//   if (xHigherPriorityTaskWoken)
//   {
//     portYIELD_FROM_ISR();
//   }
// }

void setup()
{
  Serial.begin(115200);

  // GPIO Setup
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

  // pinMode(EMERGENCY_SW_PIN, INPUT_PULLDOWN);
  stateMutex = xSemaphoreCreateMutex();
  emergencySemaphore = xSemaphoreCreateBinary();
  // ピンの立ち上がりエッジ(RISING)で割り込みを発生させる
  // attachInterrupt(digitalPinToInterrupt(EMERGENCY_SW_PIN), emergencyISR, RISING);

  if (CAN.begin(100E3, CAN_RX_PIN, CAN_TX_PIN))
  { // Rust側は125kbaud
    Serial.println("CAN Init Failed");
    while (1)
      ;
  }
  delay(1000);

  // タスクの生成
  xTaskCreateUniversal(canReceiveTask, "CAN_Rx", 4096, NULL, 3, NULL, 0);
  xTaskCreateUniversal(canTransmitTask, "CAN_Tx", 4096, NULL, 2, NULL, 0);
  xTaskCreateUniversal(stateControlTask, "State_Ctrl", 4096, NULL, 2, NULL, 1);
  xTaskCreateUniversal(solenoidValveTask, "Valve_Ctrl", 4096, NULL, 2, NULL, 1);
  xTaskCreateUniversal(executeIgnitionTask, "Ignition", 4096, NULL, 2, NULL, 1);
}

void loop()
{
  // メインループはFreeRTOSのタスクに任せるため、ここでは待機のみ
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// パネルやバルブからのCAN受信処理
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

// メインバルブへの指令値と、現在のステートの定期送信
void canTransmitTask(void *pvParameters)
{
  bool lastValveAngle = false;

  while (1)
  {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    bool curAngle = openValveFlag;
    uint8_t mainState = systemState;
    xSemaphoreGive(stateMutex);

    if (curAngle != lastValveAngle)
    {
      uint8_t cmd = curAngle ? 1 : 0;
      CAN.sendData(CAN_ID_TO_MAIN_VALVE, &cmd, 1);
      lastValveAngle = curAngle;
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    CAN.sendData(CAN_ID_TO_CTRL_MAIN_STATE, &mainState, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// タイムアウト監視と状態遷移
void stateControlTask(void *pvParameters)
{
  while (1)
  {
    xSemaphoreTake(stateMutex, portMAX_DELAY);

    unsigned long long now = millis();
    bool panelAlive = (now - lastPanelRxTime) < COMMUNICATION_TIMEOUT_MS;
    bool valveAlive = (now - lastValveRxTime) < COMMUNICATION_TIMEOUT_MS;

    // 通信タイムアウト監視ロジック
    if (panelAlive && valveAlive)
    {
      if (systemState == CANERROR)
      {
        systemState = hasTimedOut ? TIMEOUT : IDLE;
      }
    }
    else
    {
      if (systemState != CANERROR)
      {
        systemState = CANERROR;
      }
    }

    xSemaphoreGive(stateMutex);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ソレノイドバルブとボタンの状態管理
void solenoidValveTask(void *pvParameters)
{
  bool prevO2TestFlag = false;
  unsigned long o2TestEndTime = 0;
  bool isO2TestActive = false;

  while (1)
  {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    uint8_t buttons = currentButtonState;
    STATE currentState = systemState; // 現在のステート取得
    bool currentOpenO2Flag = openO2Flag;

    // ビットアサイン
    bool dumpFlag = (buttons & 1) == 1;
    bool fireFlag = ((buttons >> 1) & 1) == 1;
    bool fillFlag = ((buttons >> 2) & 1) == 1;
    bool separateFlag = ((buttons >> 3) & 1) == 1;
    bool valveSetFlag = ((buttons >> 4) & 1) == 1;
    bool o2TestFlag = ((buttons >> 5) & 1) == 1;
    // bool mainResetFlag = ((buttons >> 6) & 1) == 1;
    bool mainValveOpenFlag = ((buttons >> 6) & 1) == 1;

    digitalWrite(DUMP_PIN, dumpFlag);
    digitalWrite(FILL_PIN, fillFlag);
    digitalWrite(SEPARATE_PIN, separateFlag);

    // 各種フラグの処理
    if (fireFlag)
    {
      if (currentState == IDLE)
      {
        openO2Flag = true;
        systemState = IGNITION;
        executeIgnitionFlag = true;
      }
      else if (currentState == TIMEOUT)
      {
        openValveFlag = true;
      }
    }

    if (valveSetFlag)
      openValveFlag = false;

    // if (mainResetFlag && currentState == TIMEOUT)
    // {
    //   systemState = IDLE;
    //   hasTimedOut = false;
    // }

    if (mainValveOpenFlag)
      openValveFlag = true;

    // --- O2 Test のタイマー＆エッジ検出ロジック ---
    bool isIgnitionRunning = (currentState == IGNITION);

    if (isIgnitionRunning)
    {
      isO2TestActive = false; // 点火中はテスト無効
    }
    else if (o2TestFlag && !prevO2TestFlag)
    {
      // ボタンが押された瞬間(立ち上がりエッジ)に3秒(3000ms)のタイマーセット
      isO2TestActive = true;
      o2TestEndTime = millis() + 3000;
    }
    prevO2TestFlag = o2TestFlag; // 前回の状態を保存

    // タイマー完了チェック
    if (isO2TestActive)
    {
      if (millis() >= o2TestEndTime)
      {
        isO2TestActive = false; // 3秒経過でオフ
      }
    }

    // 点火シーケンスによるO2開放、またはO2テストタイマーによる開放の論理和
    bool isO2Open = currentOpenO2Flag || isO2TestActive;
    digitalWrite(O2_PIN, isO2Open);

    xSemaphoreGive(stateMutex);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// 点火シーケンス
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
      // 念のため、過去の割り込みキュー（セマフォ）が残っていれば空にしておく
      // while (xSemaphoreTake(emergencySemaphore, 0) == pdTRUE)
      // {
      //   // 空読み
      // }
      // xSemaphoreTake(emergencySemaphore, 0);
      // 待機フェーズ
      // 緊急停止(Rust側のemergency_swの再現)はハードウェア割込みで処理するか、
      // 即座に pdTRUE が返る。時間切れまでセマフォが来なければ pdFALSE が返る。
      vTaskDelay(pdMS_TO_TICKS(IGNITION_WAIT_MS));

      // if (xSemaphoreTake(emergencySemaphore, pdMS_TO_TICKS(IGNITION_WAIT_MS)) == pdTRUE)
      // {
      //   goto ABORT; // 時間内に緊急割り込みが発生した
      // }

      // 点火フェーズ
      Serial.println("IGNI HIGH");
      digitalWrite(IGNI_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(MAIN_VALVE_OPEN_DELAY_MS));
      // if (xSemaphoreTake(emergencySemaphore, pdMS_TO_TICKS(MAIN_VALVE_OPEN_DELAY_MS)) == pdTRUE)
      // {
      //   goto ABORT; // 時間内に緊急割り込みが発生した
      // }

      digitalWrite(IGNI_PIN, LOW);
      Serial.println("IGNI LOW");

      // バルブ開放と最終タイムアウト
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      openValveFlag = true;
      openO2Flag = false;
      xSemaphoreGive(stateMutex);
      vTaskDelay(pdMS_TO_TICKS(IGNITION_SEQUENCE_TIMEOUT_MS));

      // if (xSemaphoreTake(emergencySemaphore, pdMS_TO_TICKS(IGNITION_SEQUENCE_TIMEOUT_MS)) == pdTRUE)
      // {
      //   goto ABORT;
      // }

      xSemaphoreTake(stateMutex, portMAX_DELAY);
      systemState = TIMEOUT;
      hasTimedOut = true;
      executeIgnitionFlag = false; // シーケンス終了
      xSemaphoreGive(stateMutex);

      digitalWrite(IGNI_PIN, LOW);
      continue;

      // ABORT:
      //   digitalWrite(IGNI_PIN, LOW);
      //   xSemaphoreTake(stateMutex, portMAX_DELAY);
      //   openO2Flag = false;
      //   systemState = TIMEOUT;
      //   hasTimedOut = true;
      //   executeIgnitionFlag = false;
      //   xSemaphoreGive(stateMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // フラグ監視のための待機
  }
}