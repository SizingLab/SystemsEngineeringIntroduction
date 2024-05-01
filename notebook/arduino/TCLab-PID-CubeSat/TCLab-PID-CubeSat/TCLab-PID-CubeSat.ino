/*
  TCLab Temperature Control Lab Firmware with PI controller
  Marc Budinger
  March 2024 


  This firmware provides a high level interface to the CubeSat and a Temperature PI controller for  Q1. 
  The firmware scans the serial port for commands. Commands are case-insensitive. Any
  unrecognized command results in sleep model. Each command returns a result string.

  PID part
  ------------
  PIDON   Output of the PID is active
  PIDOFF  Output of the PID is inactive
  T1ORDER float set Temperature T1 order. Returns value of T1 in °C.
  PID_KP float  set Kp coefficient value
  PID_KI float  set Ki coefficient value
  PID_KD float  set Kd coefficient value
  PID_TS float  set timeSample value
  ECHOON  Active echo PID status
  ECHOOFF Desactive echo PID status

  Open Loop part
  ------------
  A         software restart. Returns "Start".
  P1 float  set pwm limit on heater 2, range 0 to 255. Default 100. Returns P2.
  Q1 float  set Heater 1, range 0 to 100. Returns value of Q2.
  R1        get value of Heater 1, range 0 to 100
  SCAN      get values T1 Q1 in line delimited values
  T1        get Temperature T1. Returns value of T1 in °C.
  T1B       get Temperature T1. Returns value of T1 in °C binary format.
  VER       get firmware version string
  X         stop, enter sleep mode. Returns "Stop"



  Limits on the heater power can be configured with the constants below.

  Status is indicated by LED1 on the Temperature Control Lab. Status conditions are:

      LED1        LED1
      Brightness  State
      ----------  -----
      dim         steady     Normal operation, heaters off
      bright      steady     Normal operation, heaters on
      dim         blinking   High temperature alarm on, heaters off
      bright      blinking   High temperature alarm on, heaters on

  The Temperature Control Lab shuts down the heaters if it receives no host commands
  during a timeout period (configure below), receives an "X" command, or receives
  an unrecognized command from the host.

*/

#include "Arduino.h"

// determine board type
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  String boardType = "Arduino Uno";
#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
  String boardType = "Arduino Leonardo/Micro";
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  String boardType = "Arduino Mega";
#else 
  String boardType = "Unknown board";
#endif

// For DS18B20 sensor temperature
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into following ports 
#define ONE_WIRE_T1 5 // temperature sensor

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire1(ONE_WIRE_T1);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors1(&oneWire1);

// Enable debugging output
const bool DEBUG = false;

// constants
const String vers = "2.0.1";   // version of this firmware
const long baud = 9600;      // serial baud rate
const char sp = ' ';           // command separator
const char nl = '\n';          // command terminator

// pin numbers corresponding to signals on the TC Lab Shield
const int pinT1   = 0;         // T1
const int pinQ1   = 3;         // Q1
const int pinLED1 = 9;         // LED1

// temperature alarm limits
const int limT1   = 50;       // T1 high alarm (°C)

// LED1 levels
const int hiLED   =  60;       // hi LED
const int loLED   = hiLED/16;  // lo LED

// global variables
char Buffer[64];               // buffer for parsing serial input
int buffer_index = 0;          // index for Buffer
String cmd;                    // command
float val;                     // command value
int ledStatus;                 // 1: loLED
                               // 2: hiLED
                               // 3: loLED blink
                               // 4: hiLED blink
long ledTimeout = 0;           // when to return LED to normal operation
float LED = 100;               // LED override brightness
float P1 = 200;                // heater 1 power limit in units of pwm. Range 0 to 255
float Q1 = 0;                  // last value written to heater 1 in units of percent
int alarmStatus;               // hi temperature alarm status
boolean newData = false;       // boolean flag indicating new command
int n =  10;                   // number of samples for each temperature measurement

// PID variables
unsigned long lastTime, sampleTime;         // pour calcul du pas de temps (en ms)
double Input, Output, Setpoint; // differentes variables pour calculer l'erreur et la consigne
double o_k_1, e_k_1, e_k_2;       // variables pour l'equation de recurrence du PID
double kp, ki, kd;            // parametres du correcteur PID sous la forme output = kd.erreur + ki. int erreur + kd. derivee de l'erreur
boolean PIDOn = false;        // Activation ou non du PID
boolean PIDEcho = false;      // Echo ou pas des resultats du PID

void readCommand() {
  while (Serial && (Serial.available() > 0) && (newData == false)) {
    int byte = Serial.read();
    if ((byte != '\r') && (byte != nl) && (buffer_index < 64)) {
      Buffer[buffer_index] = byte;
      buffer_index++;
    }
    else {
      newData = true;
    }
  }   
}

// for debugging with the serial monitor in Arduino IDE
void echoCommand() {
  if (newData) {
    Serial.write("Received Command: ");
    Serial.write(Buffer, buffer_index);
    Serial.write(nl);
    Serial.flush();
  }
}

// return average  of n reads of thermister temperature in °C
inline float readTemperature() {
    // call sensors1.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus
    sensors1.requestTemperatures(); // Send the command to get temperatures
    float tempC = sensors1.getTempCByIndex(0);
    // Check if reading was successful
    if(tempC == DEVICE_DISCONNECTED_C) 
      {
        Serial.println("Error: Could not read temperature data");
      }
  return tempC;
}

void parseCommand(void) {
  if (newData) {
    String read_ = String(Buffer);

    // separate command from associated data
    int idx = read_.indexOf(sp);
    cmd = read_.substring(0, idx);
    cmd.trim();
    cmd.toUpperCase();

    // extract data. toFloat() returns 0 on error
    String data = read_.substring(idx + 1);
    data.trim();
    val = data.toFloat();

    // reset parameter for next command
    memset(Buffer, 0, sizeof(Buffer));
    buffer_index = 0;
    newData = false;
  }
}

void sendResponse(String msg) {
  Serial.println(msg);
}

void sendFloatResponse(float val) {
  Serial.println(String(val, 3));
}

void sendBinaryResponse(float val) {
  byte *b = (byte*)&val;
  Serial.write(b, 4);  
}

void dispatchCommand(void) {
  if (cmd == "A") {
    setHeater1(0);
    sendResponse("Start");
  }
  else if (cmd == "P1") {
    P1 = max(0, min(255, val));
    sendResponse(String(P1));
  }
  else if (cmd == "Q1") {
    setHeater1(val);
    sendFloatResponse(Q1);
  }
  else if (cmd == "Q1B") {
    setHeater1(val);
    sendBinaryResponse(Q1);
  }
  else if (cmd == "R1") {
    sendFloatResponse(Q1);
  }
  else if (cmd == "SCAN") {
    sendFloatResponse(readTemperature());
    sendFloatResponse(Q1);
  }
  else if (cmd == "T1") {
    sendFloatResponse(readTemperature());
  }
  else if (cmd == "T1B") {
    sendBinaryResponse(readTemperature());
  }
  else if (cmd == "VER") {
    sendResponse("TCLab Firmware " + vers + " " + boardType);
  }
  else if (cmd == "X") {
    setHeater1(0);
    PIDOn = false;
    sendResponse("Stop");
      }
  else if (cmd == "T1ORDER") {
    if ( (val < 55) && (val > 0) ) {
      Setpoint = val;
      sendFloatResponse(val);  
    }
    else sendResponse("Too hot or too cold");
  }
  else if (cmd == "SETKP") {
    kp=val;
    sendResponse("Kp =" + String(kp));
  }
  else if (cmd == "SETKI") {
    ki=val;
    sendResponse("Ki =" + String(ki));
  }
  else if (cmd == "SETKD") {
    kd=val;
    sendResponse("Kd =" + String(kd));
  }
  else if (cmd == "PIDON") {
    PIDOn = true;
    o_k_1=0; // init de la sortie
    e_k_1=0;   // pas d'erreur pour le moment
    e_k_2=0; // pas d'erreur pour le moment
    lastTime=millis(); // temps
    sendResponse("PID is ON");
    sendResponse(" Kp = "+ String(kp) + " Ki = "+ String(ki) + " Kd = "+ String(kd));
  }
  else if (cmd == "PIDOFF") {
    PIDOn = false;
    sendResponse("PID is OFF");
  }
    else if (cmd == "PID_TS") {
    PIDOn = false;
    sendResponse("PID is OFF");
  }
    else if (cmd == "ECHOON") {
    PIDEcho = true;
    sendResponse("PID echo is On = Time ; Setpoint ; Input ; Output");
  }
    else if (cmd == "ECHOOFF") {
    PIDEcho = false;
    sendResponse("PID echo is OFF");
  }
  else if (cmd.length() > 0) {
    setHeater1(0);
    sendResponse(cmd);
  }
  
  Serial.flush();
  cmd = "";
}

// set Heater 1
void setHeater1(float qval) {
  Q1 = max(0., min(qval, 100.));
  analogWrite(pinQ1, (Q1*P1)/100);
}

// arduino startup
void setup() {
  // Init card
  analogReference(EXTERNAL);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.begin(baud);
  Serial.flush();
  
  setHeater1(0);

  // Initialisation du PID
  kp=10.336; // Gain Kp Pour TCLab
  ki=0.061; // Gain Ki Pour TCLab
  kd=0; // Ici PI uniquement
  Setpoint=35; // Temperature objective
  o_k_1=0; // init de la sortie
  e_k_1=0;   // pas d'erreur pour le moment
  e_k_2=0; // pas d'erreur pour le moment
  PIDOn = false; // PID starts off
  sampleTime=2000; // 2000 ms = 2s
}

void ComputePID()
{
  
  /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime)/1000;
  
   /*Compute all the working error variables*/
   double e_k = Setpoint - Input;
  
   /*Compute PID Output*/
   Output = o_k_1 +  ki*timeChange*e_k + kp*(e_k-e_k_1) + kd/timeChange*(e_k+ e_k_2-2*e_k_1);
    
   /*Remember some variables for next time*/
   e_k_2 = e_k_1;
   e_k_1 = e_k;
   o_k_1 = max(0., min(Output, 100.));
   Output = o_k_1;
   lastTime = now;

   /*Print data for serial plotter*/ 
   if (PIDEcho == true) {    
      Serial.print("Setpoint:");
      Serial.print(Setpoint);
      Serial.print(",");
      Serial.print("Input:");
      Serial.print(Input);
      Serial.print(",");
      Serial.print("Output:");
      Serial.println(Output);
   }  
}


// arduino main event loop
void loop() {

  // Gestion des commande sur port serie
  readCommand();
  if (DEBUG) echoCommand();
  parseCommand();
  dispatchCommand();

  // execution du PID
  if (PIDOn == true && ((millis()-lastTime) >= sampleTime) ) {
    Input=readTemperature();    //on mesure temperature T1 ==> Input
    ComputePID(); // on lance le correcteur
    setHeater1(Output);   // on implemente la sortie du correcteur
  }
  //else if (PIDOn == false) {
  //  setHeater1(0);
  //}
  
}
