/*
  Water Conductivity Monitor
  Sketch for an Arduino gadget that measures the electrical
  conductivity of water.
  This example code is based on example code that is in the public domain.
*/
#include <SoftwareSerial.h>

const float ArduinoVoltage = 5.00; // CHANGE THIS FOR 3.3v Arduinos
const float ArduinoResolution = ArduinoVoltage / 1024;
const float resistorValue = 10000.0;
SoftwareSerial Serial7Segment(5, 6); //RX pin, TX pin
int threshold = 3;
int inputPin = A5;
int ouputPin = A0;
char tempString[10];
void setup()
{
  Serial.begin(9600);
  pinMode(ouputPin, OUTPUT);
  pinMode(inputPin, INPUT);
  Serial7Segment.begin(9600); //Talk to the Serial7Segment at 9600 bps
  Serial7Segment.write('v');
}
void loop()
{
  int analogValue = 0;
  int oldAnalogValue = 1000;
  float returnVoltage = 0.0;
  float resistance = 0.0;
  double Siemens;
  float TDS = 00.00;
  while (((oldAnalogValue - analogValue) > threshold) || (oldAnalogValue < 50))
  {
    oldAnalogValue = analogValue;
    digitalWrite( ouputPin, HIGH );
    delay(1000); // allow ringing to stop
    analogValue = analogRead( inputPin );
    digitalWrite( ouputPin, LOW );
  }
  
  
  returnVoltage = analogValue * ArduinoResolution;
  
  resistance = ((5.00 * resistorValue) / returnVoltage) - resistorValue;
  
  
  Siemens = 1.0 / (resistance / 1000000);
  
  
  Serial.print("Total Dissolved Solids are on the order of ");
  TDS = 500 * (Siemens / 1000);
  Serial.print(TDS);
  sprintf(tempString, "%4d", TDS);
  Serial7Segment.print(tempString);
  setDecimals(0b0000010);
  
  delay(5000);
}
void setDecimals(byte decimals)
{
  Serial7Segment.write(0x77);
  Serial7Segment.write(decimals);
}

