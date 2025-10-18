#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
bool calibrationstate = 0;

int LineGoal = 3500;
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.0;

int LastError = 0;
int LastIntegral = 0;
bool lineMode = 0;

const int maxSpeed = 255;
const int baseSpeed = 150;
const int minSpeed = 110;


void setup() 
{
  pinMode(53, INPUT_PULLUP);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  
  Serial.begin(9600);
}


void calibrateQTR() 
{
  for (uint16_t i = 0; i < 400; i++) 
  {
    qtr.calibrate();
    delay(5);
  }
  calibrationstate = 1;
}

void Pista2() 
{
  int HighCount = 0;
  int LowCount = 0;
  int Threshold = 600;

  for (int i = 0; i < SensorCount; i++) 
  {
    if (sensorValues[i] > Threshold) 
    {
      HighCount++;
    } 
    
    else 
    {
      LowCount++;
    }
  }

  if (LowCount < 2) 
  {
    lineMode = 0;
  } else {
    lineMode = 1;
  }

  uint16_t LinePosition;
  if (lineMode == 0) 
  {
    LinePosition = qtr.readLineBlack(sensorValues);
  } 
  
  else 
  {
    LinePosition = qtr.readLineWhite(sensorValues);
  }

  int error = LineGoal - LinePosition;
  int P = error;
  int I = LastIntegral + error;
  int D = error - LastError;
  
  if (I > 1000) 
  {
    I = 1000;
  }
  else if (I < -1000) 
  {
    I = -1000;
  }
  
  int AdjustSpeed = P * Kp + I * Ki + D * Kd;
  
  int RightAdjustSpeed = baseSpeed + AdjustSpeed;
  int LeftAdjustSpeed = baseSpeed - AdjustSpeed;

  /*
  setRightMotors(RightAdjustSpeed);
  setLeftMotors(LeftAdjustSpeed);
  */

  LastIntegral = I;
  
  LastError = error;

}

void loop() 
{
    if (calibrationstate == 0)
    {  
    calibrateQTR();
    //stopAllMotors();
    }

    else 
    {
      qtr.read(sensorValues);
      Pista2();
      delay(50);
    
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
      }
      Serial.println();
 
      delay(250);
    
    }
}


