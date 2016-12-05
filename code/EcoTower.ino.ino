#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BLE_Firmata.h>
#include "Adafruit_BLE_UART.h"
#include <SoftwareSerial.h>   

#define FIRMATADEBUG    Serial
#define WAITFORSERIAL   true
#define VERBOSE_MODE    false
#define AUTO_INPUT_PULLUPS true

uint8_t boards_digitaliopins[] = {1, 3, 4, 5, 6, 7, 8, A0, A1, A2, A3, A4, A5}; 

#if defined(__AVR_ATmega328P__) 
  uint8_t boards_analogiopins[] = {A0, A1, A2, A3, A4, A5};  // A0 == digital 14, etc
  uint8_t boards_pwmpins[] = {3, 5, 6, 9, 10, 11};
  uint8_t boards_servopins[] = {9, 10};
  uint8_t boards_i2cpins[] = {SDA, SCL};
#elif defined(__AVR_ATmega32U4__)
  uint8_t boards_analogiopins[] = {A0, A1, A2, A3, A4, A5};  // A0 == digital 14, etc
  uint8_t boards_pwmpins[] = {3, 5, 6, 9, 10, 11, 13};
  uint8_t boards_servopins[] = {9, 10};
  uint8_t boards_i2cpins[] = {SDA, SCL};
#elif defined(__SAMD21G18A__)
  #define SDA PIN_WIRE_SDA
  #define SCL PIN_WIRE_SCL
  uint8_t boards_analogiopins[] = {PIN_A0, PIN_A1, PIN_A2, PIN_A3, PIN_A4, PIN_A5,PIN_A6, PIN_A7};  // A0 == digital 14, etc
  uint8_t boards_pwmpins[] = {3,4,5,6,8,10,11,12,A0,A1,A2,A3,A4,A5};
  uint8_t boards_servopins[] = {9, 10};
  uint8_t boards_i2cpins[] = {SDA, SCL};
  #define NUM_DIGITAL_PINS 26
#endif


#define TOTAL_PINS     NUM_DIGITAL_PINS 
#define TOTAL_PORTS    ((TOTAL_PINS + 7) / 8)

#include "Adafruit_BLE_Firmata_Boards.h"
#include "BluefruitConfig.h"
//-------------------------------------------------------
const int pumpPin = 8;
int pumpState = LOW;
const int lightPin = 7;
int lightState = LOW;
unsigned long previousMillis = 0;
const long interval = 30000;

/*
 //PH level
#define rx 1                                        
#define tx 3   

SoftwareSerial myserial(rx, tx);                     

String PHinputstring = "";                            
String PHsensorstring = "";                            
boolean PHinput_stringcomplete = false;                
boolean PHsensor_stringcomplete = false;                
float ph;                                            

*/
/*
 //TDS Levels
const float TDSarduinoVoltage = 5.00; 
const float TDSarduinoResolution = TDSarduinoVoltage / 1024;
const float TDSresistorValue = 10000.0;
int TDSthreshold = 3;
int TDSinputPin = A0;
int TDSouputPin = A5;
*/
//--------------------------------------------------------
Adafruit_BLE_UART bluefruit = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

aci_evt_opcode_t lastBTLEstatus, BTLEstatus;

Adafruit_BLE_FirmataClass BLE_Firmata = Adafruit_BLE_FirmataClass(bluefruit);

void error(const __FlashStringHelper*err) {
  FIRMATADEBUG.println(err);
  while (1);
}

int analogInputsToReport = 0; // bitwise array to store pin reporting
int lastAnalogReads[NUM_ANALOG_INPUTS];

byte reportPINs[TOTAL_PORTS];       
byte previousPINs[TOTAL_PORTS];     

byte pinConfig[TOTAL_PINS];         
byte portConfigInputs[TOTAL_PORTS]; 
int pinState[TOTAL_PINS];           

unsigned long currentMillis;       
unsigned long PpreviousMillis;       
int samplingInterval = 200;          
#define MINIMUM_SAMPLE_DELAY 150
#define ANALOG_SAMPLE_DELAY 50


struct i2c_device_info {
  byte addr;
  byte reg;
  byte bytes;
};

i2c_device_info query[MAX_QUERIES];

byte i2cRxData[32];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
unsigned int i2cReadDelayTime = 0;

Servo servos[MAX_SERVOS];

void readAndReportData(byte address, int theRegister, byte numBytes) {
  if (theRegister != REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
    #if ARDUINO >= 100
    Wire.write((byte)theRegister);
    #else
    Wire.send((byte)theRegister);
    #endif
    Wire.endTransmission();
    delayMicroseconds(i2cReadDelayTime); 
  } else {
    theRegister = 0; 
  }

  Wire.requestFrom(address, numBytes);  

  if(numBytes == Wire.available()) {
    i2cRxData[0] = address;
    i2cRxData[1] = theRegister;
    for (int i = 0; i < numBytes; i++) {
      #if ARDUINO >= 100
      i2cRxData[2 + i] = Wire.read();
      #else
      i2cRxData[2 + i] = Wire.receive();
      #endif
    }
  }
  else {
    if(numBytes > Wire.available()) {
      BLE_Firmata.sendString("I2C Read Error: Too many bytes received");
    } else {
      BLE_Firmata.sendString("I2C Read Error: Too few bytes received"); 
    }
  }

  BLE_Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  portValue = portValue & portConfigInputs[portNumber];
  if(forceSend || previousPINs[portNumber] != portValue) {
    BLE_Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

void checkDigitalInputs(boolean forceSend = false)
{
  for (uint8_t i=0; i<TOTAL_PORTS; i++) {
    if (reportPINs[i]) {
      uint8_t x = BLE_Firmata.readPort(i, portConfigInputs[i]);
      outputPort(i, x, forceSend);
    }
  }
}

void setPinModeCallback(byte pin, int mode)
{
  if ((pinConfig[pin] == I2C) && (isI2CEnabled) && (mode != I2C)) {
    disableI2CPins();
  }
  if (BLE_Firmata.IS_PIN_SERVO(pin) && mode != SERVO && servos[BLE_Firmata.PIN_TO_SERVO(pin)].attached()) {
    servos[BLE_Firmata.PIN_TO_SERVO(pin)].detach();
  }
  if (BLE_Firmata.IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(BLE_Firmata.PIN_TO_ANALOG(pin), mode == ANALOG ? 1 : 0);
  }
  if (BLE_Firmata.IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT) {
      portConfigInputs[pin/8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin/8] &= ~(1 << (pin & 7));
    }
  }
  pinState[pin] = 0;
  switch(mode) {
  case ANALOG:
    if (BLE_Firmata.IS_PIN_ANALOG(pin)) {
      if (BLE_Firmata.IS_PIN_DIGITAL(pin)) {
        pinMode(BLE_Firmata.PIN_TO_DIGITAL(pin), INPUT); 
      }
      pinConfig[pin] = ANALOG;
      lastAnalogReads[BLE_Firmata.PIN_TO_ANALOG(pin)] = -1;
    }
    break;
  case INPUT:
    if (BLE_Firmata.IS_PIN_DIGITAL(pin)) {      
      if (AUTO_INPUT_PULLUPS) {
        pinMode(BLE_Firmata.PIN_TO_DIGITAL(pin), INPUT_PULLUP); 
      } else {
        pinMode(BLE_Firmata.PIN_TO_DIGITAL(pin), INPUT); 
      }
      pinConfig[pin] = INPUT;
    }
    break;
  case OUTPUT:
    if (BLE_Firmata.IS_PIN_DIGITAL(pin)) {
      digitalWrite(BLE_Firmata.PIN_TO_DIGITAL(pin), LOW);
      pinMode(BLE_Firmata.PIN_TO_DIGITAL(pin), OUTPUT);
      pinConfig[pin] = OUTPUT;
    }
    break;
  case PWM:
    if (BLE_Firmata.IS_PIN_PWM(pin)) {
      pinMode(BLE_Firmata.PIN_TO_PWM(pin), OUTPUT);
      analogWrite(BLE_Firmata.PIN_TO_PWM(pin), 0);
      pinConfig[pin] = PWM;
    }
    break;
  case SERVO:
    if (BLE_Firmata.IS_PIN_SERVO(pin)) {
      pinConfig[pin] = SERVO;
      if (!servos[BLE_Firmata.PIN_TO_SERVO(pin)].attached()) {
          servos[BLE_Firmata.PIN_TO_SERVO(pin)].attach(BLE_Firmata.PIN_TO_DIGITAL(pin));
      }
    }
    break;
  case I2C:
    if (BLE_Firmata.IS_PIN_I2C(pin)) {
      pinConfig[pin] = I2C;
    }
    break;
  default:
    FIRMATADEBUG.print(F("Unknown pin mode")); 
  }
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch(pinConfig[pin]) {
    case SERVO:
      if (BLE_Firmata.IS_PIN_SERVO(pin))
        servos[BLE_Firmata.PIN_TO_SERVO(pin)].write(value);
        pinState[pin] = value;
      break;
    case PWM:
      if (BLE_Firmata.IS_PIN_PWM(pin))
        analogWrite(BLE_Firmata.PIN_TO_PWM(pin), value);
        pinState[pin] = value;
      break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask=1, pinWriteMask=0;

  if (port < TOTAL_PORTS) {
    lastPin = port*8+8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin=port*8; pin < lastPin; pin++) {
      if (BLE_Firmata.IS_PIN_DIGITAL(pin)) {
        if (pinConfig[pin] == OUTPUT) {
          pinWriteMask |= mask;
          pinState[pin] = ((byte)value & mask) ? 1 : 0;
        }
      }
      mask = mask << 1;
    }
    BLE_Firmata.writePort(port, (byte)value, pinWriteMask);
  }
}

void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < BLE_Firmata._num_analogiopins) {
    if(value == 0) {
      analogInputsToReport = analogInputsToReport &~ (1 << analogPin);
    } else {
      analogInputsToReport |= (1 << analogPin);
    }
  }
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
  }
}

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte slaveAddress;
  byte slaveRegister;
  byte data;
  unsigned int delayTime; 
  
  FIRMATADEBUG.println("********** Sysex callback");

  switch(command) {
  case I2C_REQUEST:
    mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
    if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
      return;
    }
    else {
      slaveAddress = argv[0];
    }

    switch(mode) {
    case I2C_WRITE:
      Wire.beginTransmission(slaveAddress);
      for (byte i = 2; i < argc; i += 2) {
        data = argv[i] + (argv[i + 1] << 7);
        #if ARDUINO >= 100
        Wire.write(data);
        #else
        Wire.send(data);
        #endif
      }
      Wire.endTransmission();
      delayMicroseconds(70);
      break;
    case I2C_READ:
      if (argc == 6) {
        slaveRegister = argv[2] + (argv[3] << 7);
        data = argv[4] + (argv[5] << 7); 
        readAndReportData(slaveAddress, (int)slaveRegister, data);
      }
      else {
        data = argv[2] + (argv[3] << 7); 
        readAndReportData(slaveAddress, (int)REGISTER_NOT_SPECIFIED, data);
      }
      break;
    case I2C_READ_CONTINUOUSLY:
      if ((queryIndex + 1) >= MAX_QUERIES) {
        // too many queries, just ignore
        BLE_Firmata.sendString("too many queries");
        break;
      }
      queryIndex++;
      query[queryIndex].addr = slaveAddress;
      query[queryIndex].reg = argv[2] + (argv[3] << 7);
      query[queryIndex].bytes = argv[4] + (argv[5] << 7);
      break;
    case I2C_STOP_READING:
	  byte queryIndexToSkip;      
      if (queryIndex <= 0) {
        queryIndex = -1;        
      } else {
        for (byte i = 0; i < queryIndex + 1; i++) {
          if (query[i].addr = slaveAddress) {
            queryIndexToSkip = i;
            break;
          }
        }
        
        for (byte i = queryIndexToSkip; i<queryIndex + 1; i++) {
          if (i < MAX_QUERIES) {
            query[i].addr = query[i+1].addr;
            query[i].reg = query[i+1].addr;
            query[i].bytes = query[i+1].bytes; 
          }
        }
        queryIndex--;
      }
      break;
    default:
      break;
    }
    break;
  case I2C_CONFIG:
    delayTime = (argv[0] + (argv[1] << 7));

    if(delayTime > 0) {
      i2cReadDelayTime = delayTime;
    }

    if (!isI2CEnabled) {
      enableI2CPins();
    }
    
    break;
  case SERVO_CONFIG:
    if(argc > 4) {
      byte pin = argv[0];
      int minPulse = argv[1] + (argv[2] << 7);
      int maxPulse = argv[3] + (argv[4] << 7);

      if (BLE_Firmata.IS_PIN_SERVO(pin)) {
        if (servos[BLE_Firmata.PIN_TO_SERVO(pin)].attached())
          servos[BLE_Firmata.PIN_TO_SERVO(pin)].detach();
        servos[BLE_Firmata.PIN_TO_SERVO(pin)].attach(BLE_Firmata.PIN_TO_DIGITAL(pin), minPulse, maxPulse);
        setPinModeCallback(pin, SERVO);
      }
    }
    break;
  case SAMPLING_INTERVAL:
    if (argc > 1) {
      samplingInterval = argv[0] + (argv[1] << 7);
      if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
        samplingInterval = MINIMUM_SAMPLING_INTERVAL;
      }      
    } else {
    }
    break;
  case EXTENDED_ANALOG:
    if (argc > 1) {
      int val = argv[1];
      if (argc > 2) val |= (argv[2] << 7);
      if (argc > 3) val |= (argv[3] << 14);
      analogWriteCallback(argv[0], val);
    }
    break;
  case CAPABILITY_QUERY:
    bluefruit.write(START_SYSEX);
    bluefruit.write(CAPABILITY_RESPONSE);

    delay(10);
    for (byte pin=0; pin < TOTAL_PINS; pin++) {
      if (BLE_Firmata.IS_PIN_DIGITAL(pin)) {
        bluefruit.write((byte)INPUT);
        bluefruit.write(1);
        bluefruit.write((byte)OUTPUT);
        bluefruit.write(1);

        delay(20);
      } else {
        bluefruit.write(127);
        delay(20);
        continue;
      }
      if (BLE_Firmata.IS_PIN_ANALOG(pin)) {
        bluefruit.write(ANALOG);
        bluefruit.write(10);
        
        delay(20);
      }
      if (BLE_Firmata.IS_PIN_PWM(pin)) {
        bluefruit.write(PWM);
        bluefruit.write(8);

        delay(20);
      }
      if (BLE_Firmata.IS_PIN_SERVO(pin)) {
        bluefruit.write(SERVO);
        bluefruit.write(14);

        delay(20);
      }
      if (BLE_Firmata.IS_PIN_I2C(pin)) {
        bluefruit.write(I2C);
        bluefruit.write(1);
        delay(20);
      }
      bluefruit.write(127);
    }
    bluefruit.write(END_SYSEX);
    break;
  case PIN_STATE_QUERY:
    if (argc > 0) {
      byte pin=argv[0];
      bluefruit.write(START_SYSEX);
      bluefruit.write(PIN_STATE_RESPONSE);
      bluefruit.write(pin);
      if (pin < TOTAL_PINS) {
        bluefruit.write((byte)pinConfig[pin]);
	bluefruit.write((byte)pinState[pin] & 0x7F);
	if (pinState[pin] & 0xFF80) bluefruit.write((byte)(pinState[pin] >> 7) & 0x7F);
	if (pinState[pin] & 0xC000) bluefruit.write((byte)(pinState[pin] >> 14) & 0x7F);
      }
      bluefruit.write(END_SYSEX);
    }
    break;
  case ANALOG_MAPPING_QUERY:
    bluefruit.write(START_SYSEX);
    bluefruit.write(ANALOG_MAPPING_RESPONSE);
    for (byte pin=0; pin < TOTAL_PINS; pin++) {
      bluefruit.write(BLE_Firmata.IS_PIN_ANALOG(pin) ? BLE_Firmata.PIN_TO_ANALOG(pin) : 127);
    }
    bluefruit.write(END_SYSEX);
    break;
  }
}

void enableI2CPins()
{
  byte i;
  for (i=0; i < TOTAL_PINS; i++) {
    if(BLE_Firmata.IS_PIN_I2C(i)) {
      setPinModeCallback(i, I2C);
    } 
  }
   
  isI2CEnabled = true; 
  
  Wire.begin();
}

void disableI2CPins() {
    isI2CEnabled = false;
    queryIndex = -1;
}

void systemResetCallback()
{
  FIRMATADEBUG.println(F("***RESET***"));
  if (isI2CEnabled) {
  	disableI2CPins();
  }
  for (byte i=0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;      
    portConfigInputs[i] = 0;	
    previousPINs[i] = 0;
  }

  for (byte i=0; i < TOTAL_PINS; i++) {
    if (BLE_Firmata.IS_PIN_ANALOG(i)) {
      setPinModeCallback(i, ANALOG);
    } else {
      setPinModeCallback(i, INPUT);
    }
  }
  analogInputsToReport = 0;
}

void setup() 
{
  if (WAITFORSERIAL) {
    while (!FIRMATADEBUG) delay(1);
  }
  
  FIRMATADEBUG.begin(9600);
  FIRMATADEBUG.println(F("Adafruit Bluefruit nRF8001 Firmata test"));
  
  FIRMATADEBUG.print("Total pins: "); FIRMATADEBUG.println(NUM_DIGITAL_PINS);
  FIRMATADEBUG.print("Analog pins: "); FIRMATADEBUG.println(sizeof(boards_analogiopins));
  
  BLE_Firmata.setUsablePins(boards_digitaliopins, sizeof(boards_digitaliopins), 
    boards_analogiopins, sizeof(boards_analogiopins),
    boards_pwmpins, sizeof(boards_pwmpins),
    boards_servopins, sizeof(boards_servopins), SDA, SCL);

  FIRMATADEBUG.print(F("Init nRF8001: "));
  if (! bluefruit.begin()) {
    error(F("Failed"));
  }
  bluefruit.setDeviceName("ADA_BLE");
  FIRMATADEBUG.println(F("Done"));
  BTLEstatus = lastBTLEstatus = ACI_EVT_DISCONNECTED;
  //-------------------------------------------------------------------------------
  pinMode(pumpPin, OUTPUT);
  pinMode(lightPin, OUTPUT);

  /*
  Serial.begin(9600);                                
  myserial.begin(9600);                             
  PHinputstring.reserve(10);                            
  PHsensorstring.reserve(30); 
  */
 /*
  Serial.begin(9600);
  pinMode(TDSouputPin, OUTPUT);
  pinMode(TDSinputPin, INPUT);
 */
}

void firmataInit() {
  FIRMATADEBUG.println(F("Init firmata"));
  BLE_Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  BLE_Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  BLE_Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  BLE_Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  BLE_Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  BLE_Firmata.attach(START_SYSEX, sysexCallback);
  BLE_Firmata.attach(SYSTEM_RESET, systemResetCallback);

  FIRMATADEBUG.println(F("Begin firmata"));
  BLE_Firmata.begin();
  systemResetCallback(); 
}
/*
void serialEvent() {                                 
  char inchar = (char)Serial.read();                 
  PHinputstring += inchar;                             
  if (inchar == '\r') {                               
    PHinput_stringcomplete = true;                     
  }
}
*/
void loop() 
{
  bluefruit.pollACI();
  BTLEstatus = bluefruit.getState();

  if (BTLEstatus != lastBTLEstatus) {
    if (BTLEstatus == ACI_EVT_DEVICE_STARTED) {
        FIRMATADEBUG.println(F("* Advertising"));
    }
    if (BTLEstatus == ACI_EVT_CONNECTED) {
        FIRMATADEBUG.println(F("* Connected!"));
         firmataInit();
    }
    if (BTLEstatus == ACI_EVT_DISCONNECTED) {
        FIRMATADEBUG.println(F("* Disconnected"));
    }
    lastBTLEstatus = BTLEstatus;
  }
  if (BTLEstatus != ACI_EVT_CONNECTED) {
    delay(100);
    
    return;
  }
  
  if (FIRMATADEBUG.available()) {
    bluefruit.write(FIRMATADEBUG.read());
  }
    
  byte pin, analogPin;
  checkDigitalInputs();  
  
  while(BLE_Firmata.available()) {
    BLE_Firmata.processInput();
  }

  uint8_t analogreportnums = 0;
  for(uint8_t a=0; a<8; a++) {
    if (analogInputsToReport & (1 << a)) {
      analogreportnums++;
    }
  }

  samplingInterval = (uint16_t)MINIMUM_SAMPLE_DELAY  + (uint16_t)ANALOG_SAMPLE_DELAY * (1+analogreportnums); 
  
  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;

    for(pin=0; pin<TOTAL_PINS; pin++) {
      if (BLE_Firmata.IS_PIN_ANALOG(pin) && (pinConfig[pin] == ANALOG)) {
        analogPin = BLE_Firmata.PIN_TO_ANALOG(pin);

        if (analogInputsToReport & (1 << analogPin)) {
          int currentRead = analogRead(analogPin);
          
          if ((lastAnalogReads[analogPin] == -1) || (lastAnalogReads[analogPin] != currentRead)) {
            BLE_Firmata.sendAnalog(analogPin, currentRead);
            lastAnalogReads[analogPin] = currentRead;
          }
        }
      }
    }
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes);
      }
    }
  }
//--------------------------------------------------------------------------
 unsigned long PcurrentMillis = millis();
  if (PcurrentMillis - PpreviousMillis >= interval){
    PpreviousMillis = PcurrentMillis;
    if (pumpState == LOW){
      pumpState = HIGH;
      delay(6000);
    }
    else{
      pumpState = LOW;
      delay(36000);
    }
    digitalWrite(pumpPin, pumpState);
  }
/*
   unsigned long LcurrentMillis = millis();
  if (LcurrentMillis - LpreviousMillis >= interval){
    LpreviousMillis = LcurrentMillis;
    if (lightState == LOW){
      lightState = HIGH;
      delay(6000);
    }
    else{
      lightState = LOW;
      delay(36000);
    }
    digitalWrite(lightPin, lightState);
  }
  */
/*
  if (PHinput_stringcomplete) {                         
    myserial.print(PHinputstring);                     
    PHinputstring = "";                                 
    PHinput_stringcomplete = false;                    
  }

  if (myserial.available() > 0) {                    
    char inchar = (char)myserial.read();             
    PHsensorstring += inchar;
    if (inchar == '\r') {
      PHsensor_stringcomplete = true;                   
    }
  }


  if (PHsensor_stringcomplete) {                        
    Serial.println(PHsensorstring);                    
    ph = PHsensorstring.toFloat();                      

    if (ph >= 5.5 && ph<=6.2) {                        
      Serial.println("Ideal");                       
    }
    if (ph <= 5.499) {                                
      Serial.println("Too Acidic");                      
    }
    if(ph >= 6.201){
      Serial.println("Too Basic");
    }
    PHsensorstring = "";                                
    PHsensor_stringcomplete = false;                   
  }
  */
/*
  int TDSanalogValue=0;
  int TDSoldAnalogValue=1000;
  float TDSreturnVoltage=0.0;
  float TDSresistance=0.0;
  double TDSsiemens;
  float TDS=0.0;
  while(((TDSoldAnalogValue - TDSanalogValue) > TDSthreshold) || (TDSoldAnalogValue < 50)){
    TDSoldAnalogValue = TDSanalogValue;
    digitalWrite( TDSouputPin, HIGH );
    delay(10); 
    TDSanalogValue = analogRead( TDSinputPin );
    digitalWrite( TDSouputPin, LOW );
  }

  TDSreturnVoltage = TDSanalogValue * TDSarduinoResolution;
  TDSresistance = ((5.00 * TDSresistorValue) / TDSreturnVoltage) - TDSresistorValue;
  TDSsiemens = 1.0/(TDSresistance/1000000);

  Serial.print("Total Dissolved Solids are on the order of ");
  TDS = 500 * (TDSsiemens/1000);
  delay(5000);
*/
  }
