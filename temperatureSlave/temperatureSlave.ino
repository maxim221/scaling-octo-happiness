#include <Wire.h>
#include <DallasTemperature.h>

#include <I2C_Anything.h>

#define TEMPERATURE_DEVICE 8

#define ONE_WIRE_BUS_1 6
#define ONE_WIRE_BUS_2 7

OneWire oneWire_in(ONE_WIRE_BUS_1);
OneWire oneWire_out(ONE_WIRE_BUS_2);

DallasTemperature sensor_one(&oneWire_in);
DallasTemperature sensor_two(&oneWire_out);

int tempDelay = 1500;

unsigned long lastTempTime;
 
void setup() {
  Wire.begin(TEMPERATURE_DEVICE);
  Wire.onRequest(requestEvent);
  sensor_one.setResolution(12);
  sensor_two.setResolution(12);
}

volatile float temp1;
volatile float temp2;

void requestEvent()
{  
 I2C_singleWriteAnything(temp1);
 I2C_singleWriteAnything(temp2);
}

void loop() {
if(millis() - lastTempTime >= tempDelay){
  sensor_one.requestTemperatures();
  sensor_two.requestTemperatures();
  temp1 = sensor_one.getTempCByIndex(0);
  temp2 = sensor_two.getTempCByIndex(0);
  lastTempTime = millis();
}
}
