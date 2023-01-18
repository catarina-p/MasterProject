#include "pyCommsLib.h"
#include "HX711.h"

// Communication with python
String msgName[] = {"lc3", "pressure1"};

// An array of strings which will be the payload
String Data[2];

#define sensor1Pin 0
HX711 lc3;

uint8_t data_lc3 = 3;
uint8_t clock_lc3 = 2;

float raw_lc3;

void setup() {
  Serial.begin(115200);

  lc3.begin(data_lc3, clock_lc3);
  lc3.set_scale(770);
  lc3.tare();

  init_python_communication();
}

float avSensor1 = 0;

int N = 10;

void loop() {
  raw_lc3 = lc3.get_units();
  avSensor1 = 0;

  for(int i = 0; i < N; i++) { 
    avSensor1 += analogRead(sensor1Pin);
  }

  Data[0] = String(raw_lc3);
  Data[1] = String(avSensor1/N);

  load_msg_to_python(msgName, Data, size_of_array(msgName));
  sync();
}
