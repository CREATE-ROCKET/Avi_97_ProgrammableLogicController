#pragma once

void canReceiveTask(void *pvParameters);
void canTransmitTask(void *pvParameters);
void stateControlTask(void *pvParameters);
void executeIgnitionTask(void *pvParameters);
void solenoidValveTask(void *pvParameters);
