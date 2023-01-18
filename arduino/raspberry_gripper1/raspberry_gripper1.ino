#include "pyCommsLib.h"
#include "HX711.h"
#include <Wire.h>
#include "Adafruit_VL6180X.h"

// Communication with python
String msgName[] = {"lc1", "lc2", "d"};

// An array of strings which will be the payload
String Data[3];

Adafruit_VL6180X vl = Adafruit_VL6180X();

HX711 lc1;
HX711 lc2;

uint8_t data_lc1 = 2;
uint8_t clock_lc1 = 3;

uint8_t data_lc2 = 11;
uint8_t clock_lc2 = 12;

float raw_lc1;
float raw_lc2;

void setup() {
  Serial.begin(115200);

  lc1.begin(data_lc1, clock_lc1);
  lc1.set_scale(3090);
  lc1.tare();

  lc2.begin(data_lc2, clock_lc2);
  lc2.set_scale(3090);
  lc2.tare();

  vl.begin();
  init_python_communication();
}


void loop() {

  String msg = latest_received_msg();

  raw_lc1 = lc1.get_units();
  raw_lc2 = lc2.get_units()*(-1);

  Data[0] = String(raw_lc1);
  Data[1] = String(raw_lc2);

  String dist = "null";
  if(msg == "read_d") {
    dist = String(vl.readRange());
  }

  Data[2] = dist;

  load_msg_to_python(msgName, Data, size_of_array(msgName));
  sync();
}
