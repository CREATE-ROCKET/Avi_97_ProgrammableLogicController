#include <Arduino.h>
#include "CANCREATE.h"
#define DUMP 23
#define FILL 21
#define O2 18
#define IGNI 25
#define CAN_RX 27
#define CAN_TX 13

constexpr short outpins[4] = {DUMP, FILL, O2, IGNI};
bool iginitionfireFlag = false;
unsigned long lastTime = 0; /*check if CANBUS is alive or not*/
unsigned long lastFireTime = 0;
bool ignbtnflag;
unsigned long lastigmbtntime = 0;
CAN_CREATE CAN(true);
/*CAN idは0x101*/
void setup()
{
  Serial.begin(115200);
  // tx,rxの順に、  32,33:コンパネ　　13,27:中継
  Serial.println("CAN Sender");
  // 100 kbpsでCANを動作させる
  if (CAN.begin(100E3, CAN_RX, CAN_TX))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
  for (short i = 0; i < 4; i++)
  {
    pinMode(outpins[i], OUTPUT);
    digitalWrite(outpins[i], LOW);
  }
}
/**/
void triggerIgnition()
{
  if (iginitionfireFlag)
  {
    // Serial.println("ignition");
    // Serial.println(millis() - lastFireTime);
    if ((25000 > (millis() - lastFireTime)) && ((millis() - lastFireTime) > 20000))
    {
      uint8_t data[4] = {186, 20, 0, 0};
      if (CAN.sendData(0x300, data, 4)) /*to main_valve*/
      {
        Serial.println("failed to send CAN data");
      }
      digitalWrite(IGNI, HIGH);
    }
    else if ((millis() - lastFireTime) > 25000)
    {
      digitalWrite(IGNI, LOW);
      if (ignbtnflag == false)
      {
        Serial.println("because of time and ignbtnflag is low");
        iginitionfireFlag = false;
      }
    }
  }
  else
  {
    digitalWrite(IGNI, LOW);
    Serial.println("because of flag");
  }
}
void missingCan() /*check if ctrl_panel is dead or not*/
{
  if (millis() - lastTime > 10000)
  {
    digitalWrite(IGNI, LOW);
    digitalWrite(O2, LOW);
    Serial.println("IGN is low bcaouse of missing time");
  }
}
void loop()
{
  missingCan();
  triggerIgnition();
  if (CAN.available())
  {
    can_return_t message; // 最大8文字+改行文字が送信される
    while (!CAN.readWithDetail(&message))
    {
      if (message.id == 0x101) /*plc's CAN_id*/
      {
        lastTime = millis();
        uint8_t data[4] = {0, 0, 0, 0};
        if (CAN.sendData(0x403, data, 4)) /*to control_panel*/
        {
        }
        if (((message.data[0] >> 3) & (uint8_t)0x1) == ((uint8_t)0x1))
        {
          uint8_t data[4] = {100, 0, 0, 0};
          if (CAN.sendData(0x200, data, 4))
            ;
        }
        else
        {
          uint8_t data[4] = {10, 0, 0, 0};
          if (CAN.sendData(0x200, data, 4))
            ;
        }
        if (((message.data[0] >> 2) & (uint8_t)0x1) == ((uint8_t)0x1))
        {
          if (!ignbtnflag)
          {
            lastigmbtntime = millis();
            lastFireTime = millis();
          }
          ignbtnflag = true;
          digitalWrite(O2, HIGH);
          if (iginitionfireFlag == false && millis() - lastigmbtntime > 1000) // 1秒以上押し続けたら点火
          {
            iginitionfireFlag = true;
          }
          else
          {
            Serial.println("fireFragisTrue");
          }
        }
        else
        {
          // iginitionfireFlag=false;
          ignbtnflag = false;
          digitalWrite(O2, LOW);
          if (iginitionfireFlag == false)
          {
            digitalWrite(IGNI, LOW);
            Serial.println("IGN is low bcaouse of sw2 and flag");
          }
        }
        if (((message.data[0] >> 1) & (uint8_t)0x1) == ((uint8_t)0x1))
        {
          digitalWrite(FILL, HIGH);
        }
        else
        {
          digitalWrite(FILL, LOW);
        }
        if (((message.data[0] >> 0) & (uint8_t)0x1) == ((uint8_t)0x1))
        {
          digitalWrite(DUMP, HIGH);
        }
        else
        {
          digitalWrite(DUMP, LOW);
        }
        if (((message.data[0] >> 4) & (uint8_t)0x1) == ((uint8_t)0x1))
        {
          uint8_t data[4] = {51, 0, 0, 0};
          if (CAN.sendData(0x300, data, 4))
            ;
          Serial.println("valve set");
          iginitionfireFlag = false; // バルブリセットで点火カウントダウンも停止
        }
      }
    }
  }
  delay(10);
}