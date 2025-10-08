#include <Arduino.h>
#include "CANCREATE.h"
#define DUMP 23
#define FILL 21
#define NOD 18 /*NOD経路のサブバルブ（初期燃焼実験だけ電磁弁）*/
#define IGNI 25
#define CAN_RX 27
#define CAN_TX 13
#define NICHROME 26

constexpr short outpins[4] = {DUMP, FILL, NOD, IGNI};
constexpr short waitTime = 3000; /*点火ボタンを押してから、nichrome線が通電するまでの時間(ms)*/
constexpr short TimeOut = 5000;

bool iginitionfireFlag = false;
unsigned long lastTime = 0; /*check if CANBUS is alive or not*/
unsigned long lastFireTime = 0;
unsigned long fireAfterNodTime = 0;   /*NOD電磁弁(本当はサブバルブ)に通電した数秒後にignitorに通電*/
unsigned long NodTime = 0;            /*Nichrome線に通電した数秒後にNOD電磁弁を通電*/
unsigned long valveAfterfireTime = 0; /*ignitorに通電*/
bool NodOpenFlag = false;
bool ignbtnflag;
unsigned long lastigmbtntime = 0;
CAN_CREATE CAN(true);
/*PLCのCAN idは0x101*/
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

void triggerIgnition()
{
  if (iginitionfireFlag)
  {
    // Serial.println("ignition");
    // Serial.println(millis() - lastFireTime);
    if ((TimeOut > (millis() - lastFireTime)) && ((millis() - lastFireTime) > waitTime))
    {
      uint8_t data[4] = {186, 20, 0, 0};
      if (CAN.sendData(0x300, data, 4)) /*to main_valve*/
      {
        Serial.println("failed to send CAN data");
      }
      if (CAN.sendData(0x10a, data, 4)) /*to nichrome controller*/
      {
        Serial.println("failed to send CAN data");
      }
      if (millis() - NodTime > 5000 && NodOpenFlag)
      {
        digitalWrite(NOD, HIGH);
        fireAfterNodTime = millis();
      }
      if (millis() - fireAfterNodTime > 8000)
      {
        digitalWrite(IGNI, HIGH);
        valveAfterfireTime = millis();
      }
      if (millis() - valveAfterfireTime > 1000) /*NichromeとignitorだけLOWにする*/
      {
        uint8_t message[4] = {};
        if (CAN.sendData(0x10b, message, 4))
        {
        }
      }
    }
    else if ((millis() - lastFireTime) > TimeOut)
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
    digitalWrite(NICHROME, LOW);
    NodOpenFlag = false;
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
          ;
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
            NodTime = millis();
          }
          ignbtnflag = true;
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
          digitalWrite(NOD, LOW);
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
          digitalWrite(NOD, LOW);
        }
        if (((message.data[0] >> 4) & (uint8_t)0x1) == ((uint8_t)0x1))
        {
          uint8_t data[4] = {51, 0, 0, 0};
          if (CAN.sendData(0x300, data, 4))
            ;
          // Serial.println("valve set");
          iginitionfireFlag = false; // バルブリセットで点火カウントダウンも停止
        }
      }
    }
  }
  delay(10);
}