/* Flywheel Control with PD controller 
VAILLON Mélanie and ROUILLARD Clement
BUDINGER Marc
*/
// libraries
#include <Arduino.h>
#include <Wire.h>
#include "bmm150.h"
#include "bmm150_defs.h"

//pin Motor control
int dirPin = 13;
int speedPin = 11;
int brakePin = 8;
//boolean to switch direction
bool directionState;

//Sensor
BMM150 bmm = BMM150();
bmm150_mag_data value_offset;

// PID variables
unsigned long lastTime, sampleTime;         // pour calcul du pas de temps (en ms)
double Input, Output, Setpoint; // differentes variables pour calculer l'erreur et la consigne
double o_k_1, e_k_1, e_k_2;       // variables pour l'equation de recurrence du PID
double kp, ki, kd;            // parametres du correcteur PID sous la forme output = kd.erreur + ki. int erreur + kd. derivee de l'erreur
boolean PIDOn = false;        // Activation ou non du PID
boolean PIDEcho = true;      // Echo ou pas des resultats du PID

void setup() {
  // Init card
  Serial.begin(9600);

  // Init sensor
  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
      Serial.println("Chip ID can not read!");
      while (1);
  } else {
      Serial.println("Initialize done!");
  }

  Serial.println("Start Calibration after 3 seconds.");
    delay(3000);
    calibrate(3000);
    Serial.print("\n\rCalibrate done..");

  //define pins
  pinMode(dirPin, OUTPUT);
  pinMode(speedPin, OUTPUT);
  pinMode(brakePin, OUTPUT);

  // Initialisation du PID
  kp=-0.75;// Gain Kp Pour CubeSat
  ki=-0.0; // Gain Ki Pour CubeSat Ici PD uniquement
  kd=-0.5; // Gain Kd Pour CubeSat
  Setpoint=90.0; // Angle objective
  o_k_1=0; // init de la sortie
  e_k_1=0;   // pas d'erreur pour le moment
  e_k_2=0; // pas d'erreur pour le moment
  PIDOn = true; // PID starts off
  sampleTime=100; // 2000 ms = 2s
  lastTime=millis(); // temps
}

/**
    @brief Do figure-8 calibration for a limited time to get offset values of x/y/z axis.
    @param timeout - seconds of calibration period.
*/
void calibrate(uint32_t timeout) {
    int16_t value_x_min = 0;
    int16_t value_x_max = 0;
    int16_t value_y_min = 0;
    int16_t value_y_max = 0;
    int16_t value_z_min = 0;
    int16_t value_z_max = 0;
    uint32_t timeStart = 0;

    bmm.read_mag_data();
    value_x_min = bmm.raw_mag_data.raw_datax;
    value_x_max = bmm.raw_mag_data.raw_datax;
    value_y_min = bmm.raw_mag_data.raw_datay;
    value_y_max = bmm.raw_mag_data.raw_datay;
    value_z_min = bmm.raw_mag_data.raw_dataz;
    value_z_max = bmm.raw_mag_data.raw_dataz;
    delay(100);

    timeStart = millis();

    while ((millis() - timeStart) < timeout) {
        bmm.read_mag_data();

        /* Update x-Axis max/min value */
        if (value_x_min > bmm.raw_mag_data.raw_datax) {
            value_x_min = bmm.raw_mag_data.raw_datax;
            // Serial.print("Update value_x_min: ");
            // Serial.println(value_x_min);

        } else if (value_x_max < bmm.raw_mag_data.raw_datax) {
            value_x_max = bmm.raw_mag_data.raw_datax;
            // Serial.print("update value_x_max: ");
            // Serial.println(value_x_max);
        }

        /* Update y-Axis max/min value */
        if (value_y_min > bmm.raw_mag_data.raw_datay) {
            value_y_min = bmm.raw_mag_data.raw_datay;
            // Serial.print("Update value_y_min: ");
            // Serial.println(value_y_min);

        } else if (value_y_max < bmm.raw_mag_data.raw_datay) {
            value_y_max = bmm.raw_mag_data.raw_datay;
            // Serial.print("update value_y_max: ");
            // Serial.println(value_y_max);
        }

        /* Update z-Axis max/min value */
        if (value_z_min > bmm.raw_mag_data.raw_dataz) {
            value_z_min = bmm.raw_mag_data.raw_dataz;
            // Serial.print("Update value_z_min: ");
            // Serial.println(value_z_min);

        } else if (value_z_max < bmm.raw_mag_data.raw_dataz) {
            value_z_max = bmm.raw_mag_data.raw_dataz;
            // Serial.print("update value_z_max: ");
            // Serial.println(value_z_max);
        }

        Serial.print(".");
        delay(100);

    }

    value_offset.x = value_x_min + (value_x_max - value_x_min) / 2;
    value_offset.y = value_y_min + (value_y_max - value_y_min) / 2;
    value_offset.z = value_z_min + (value_z_max - value_z_min) / 2;
}




inline float readPosition() {
  bmm150_mag_data value;           // Declare a variable to store magnetometer data
  bmm.read_mag_data();             // Read magnetometer data
  value.x = bmm.raw_mag_data.raw_datax;  // Store the X-axis data from the sensor
  value.y = bmm.raw_mag_data.raw_datay;  // Store the Y-axis data from the sensor
  
  // Calculate the heading in radians based on the X and Y magnetometer values
  float xyHeading = atan2(value.x, value.y);


  // Ensure the heading is within the 0 to 2*PI range
  if (xyHeading < 2*PI) {
        xyHeading +=  2*PI;
  }
  if (xyHeading >  2*PI) {
        xyHeading -=  2*PI;
  }
  // Convert the heading from radians to degrees
  float xyHeadingDegrees = xyHeading * 180 / M_PI;
  // Between 180 and -180 Degrees
  if (xyHeadingDegrees >  180.0) {
        xyHeadingDegrees=xyHeadingDegrees - 360.0;
  }
   // Return the heading in degrees
  Serial.print("Heading: ");
  Serial.println(xyHeadingDegrees);
  return xyHeadingDegrees;
}

void setCommand(float qval){
  // Braking the engine at start A vérifier
  //Define rotation direction
  if (qval < 0.0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }
  
  //Convert the degree into PWM
  
  qval=min(255,int(abs(qval)*255/180));
  
  //Give the command to the motor
  // We define some condition, like if PWM < 20 no command and if PWM between 20 and 60, we need to by pass the mortor friction
  if (qval < 20) {
    analogWrite(speedPin, 0); 
  } else if (qval <60) {
    analogWrite(speedPin, 150);    
    delay(20);
    analogWrite(speedPin, qval);    //Define PWM command
  
  } else {
    analogWrite(speedPin, qval);    //Define PWM command
  }
  Serial.print("Speed PWM:");
  Serial.println(qval);
}


inline float ComputePID(float Input)
{
  
  /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime)/1000;
  
   /*Compute all the working error variables*/
   float e_k = Setpoint - Input;
   
  
   /*Compute PID Output*/
   Output = o_k_1 +  ki*timeChange*e_k + kp*(e_k-e_k_1) + kd/timeChange*(e_k+ e_k_2-2*e_k_1);
   
   
   /*Remember some variables for next time*/
   e_k_2 = e_k_1;
   e_k_1 = e_k;
   o_k_1 = Output;
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
      Serial.print(Output);
      Serial.print(",");
      Serial.print("Time change:");
      Serial.print(timeChange);
      Serial.print(",");
      Serial.print("e_k_1:");
      Serial.println(e_k_1);
   } 
   return Output; 
}

void loop() {
  
  // execution du PID
  if (PIDOn == true && ((millis()-lastTime) >= sampleTime) ) {
    Serial.print("time:");
    Serial.println(millis()-lastTime);
    Input=readPosition();    //on mesure la différence de degrés
    Output=ComputePID(Input); // on lance le correcteur
    setCommand(Output);   // on implemente la sortie du correcteur
  }
  else if (PIDOn == false) {
    setCommand(0);
  }
  
}
