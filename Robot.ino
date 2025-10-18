
const int leftFront_ENA = 4;
const int leftFront_IN1 = 3;
const int leftFront_IN2 = 2;

const int leftRear_ENB = 8;
const int leftRear_IN3 = 10;
const int leftRear_IN4 = 9;

int ColorDetected = 0;

const int rightFront_ENA = 5;
const int rightFront_IN1 = 6;
const int rightFront_IN2 = 7;

const int rightRear_ENB = 46;
const int rightRear_IN3 = 45;
const int rightRear_IN4 = 44;

const int maxSpeed = 255;
const int baseSpeed = 170;

const int delayTimeU = 1350; 
const int delayTimeT = 1500;

#include <Servo.h>

Servo Garra;   

const int ServoPin = 11;   
const int openPosition = 0;
const int closePosition = 60;

unsigned int LastMove;

int stepcount = 0;

float LastErrorMoves = 0;

const float WALL_CM = 5.0; 
const float WALL_THRESHOLD = 15.0;

float KpF = 0.05;
float KdF = 0.02;


#include <ShiftRegister74HC595.h>

const int numberOfShiftRegisters  = 1;
int serialDataPin = 30;
int clockPin = 28;
int latchPin = 29;

ShiftRegister74HC595<numberOfShiftRegisters > sr(serialDataPin, clockPin, latchPin);

const int TcS0 = 40;
const int TcS1 = 38;
const int TcS2 = 36;
const int TcS3 = 34;
const int signal = 12;

unsigned long red;
unsigned long blue;
unsigned long green;

unsigned long redSum = 0;
unsigned long greenSum = 0;
unsigned long blueSum = 0;

int readingCount = 0;

unsigned long previousTime = 0;
const unsigned long IntervaloL = 5000;

const unsigned long yellowRedMin = 700;
const unsigned long yellowRedMax = 1100;
const unsigned long yellowGreenMin = 750;
const unsigned long yellowGreenMax = 1150;
const unsigned long yellowBlueMax = 500;

const unsigned long cyanRedMax = 500;
const unsigned long cyanGreenMin = 800;
const unsigned long cyanGreenMax = 1200;
const unsigned long cyanBlueMin = 850;
const unsigned long cyanBlueMax = 1250;

const unsigned long blueBlueMin = 900;
const unsigned long blueBlueMax = 1300;
const unsigned long blueRedMax = 600;
const unsigned long blueGreenMax = 650;

const unsigned long pinkRedMin = 800;
const unsigned long pinkRedMax = 1200;
const unsigned long pinkGreenMin = 400;
const unsigned long pinkGreenMax = 800;
const unsigned long pinkBlueMin = 700;
const unsigned long pinkBlueMax = 1100;

const unsigned long greenGreenMin = 850;
const unsigned long greenGreenMax = 1250;
const unsigned long greenRedMax = 600;
const unsigned long greenBlueMax = 650;

const unsigned long redRedMin = 500;
const unsigned long redRedMax = 700;
const unsigned long redGreenMax = 800;
const unsigned long redBlueMax =  1000;

const unsigned long blackRedMax = 350;
const unsigned long blackGreenMax = 350;
const unsigned long blackBlueMax = 350;

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


struct USonic 
{
  int trig;
  int echo;
  float lastValidReading;
};

USonic US_FRONT  = {53, 52, 400.0};
USonic US_RIGHT  = {27, 26, 400.0};
USonic US_LEFT   = {24, 25, 400.0};

const int US_SAMPLES = 3;

float readDistanceCM(USonic& u);
float readDistanceWithRetry(USonic& u);
float medianOf3(float a, float b, float c);
bool isValidReading(float reading, float lastReading);

bool isPathFree(float d_cm);
bool isBlackTile();


void setup() 
{
  pinMode(serialDataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  digitalWrite(serialDataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(latchPin, LOW);

  sr.setAllLow();

  pinMode(leftFront_ENA, OUTPUT);
  pinMode(leftFront_IN1, OUTPUT);
  pinMode(leftFront_IN2, OUTPUT);
  
  pinMode(leftRear_ENB, OUTPUT);
  pinMode(leftRear_IN3, OUTPUT);
  pinMode(leftRear_IN4, OUTPUT);
  
  pinMode(rightFront_ENA, OUTPUT);
  pinMode(rightFront_IN1, OUTPUT);
  pinMode(rightFront_IN2, OUTPUT);
  
  pinMode(rightRear_ENB, OUTPUT);
  pinMode(rightRear_IN3, OUTPUT);
  pinMode(rightRear_IN4, OUTPUT);

  stopAllMotors();
  
  pinMode(US_FRONT.trig, OUTPUT);
  pinMode(US_FRONT.echo, INPUT);
  pinMode(US_RIGHT.trig, OUTPUT);
  pinMode(US_RIGHT.echo, INPUT);
  pinMode(US_LEFT.trig, OUTPUT);
  pinMode(US_LEFT.echo, INPUT);

  Garra.attach(ServoPin);   
  Garra.write(openPosition);

  pinMode(TcS0, OUTPUT);
  pinMode(TcS1, OUTPUT);
  pinMode(TcS2, OUTPUT);
  pinMode(TcS3, OUTPUT);
  pinMode(signal, INPUT);
  
  digitalWrite(TcS0, HIGH);
  digitalWrite(TcS1, LOW);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  stopAllMotors();

  pinMode(53, INPUT_PULLUP);

  delay(1000);
  
  Serial.begin(9600);
}

void moveForward(float Units)
{
  setRightMotors(baseSpeed);
  setLeftMotors(baseSpeed - 25);
  delay(Units * delayTimeU);
  stopAllMotors();
  setRightMotors(130); //110
  delay(1800 * Units / 10); //1500
  stopAllMotors();
}

void moveBackward(float Units)
{
  setRightMotors(-baseSpeed);
  setLeftMotors(-baseSpeed + 25);
  delay(Units * delayTimeU);
  stopAllMotors();
  setRightMotors(-130);
  delay(2000 * Units / 10);
  stopAllMotors();
}

void turnRight() 
{
  setLeftMotors(baseSpeed+90);
  setRightMotors(-baseSpeed-15);
  delay(delayTimeT);
  stopAllMotors();
}

void turnLeft() 
{
  setLeftMotors(-baseSpeed-35);
  setRightMotors(baseSpeed+35);
  delay(delayTimeT);
  stopAllMotors();
}

void turn180()
{
  setLeftMotors(-baseSpeed);
  setRightMotors(baseSpeed);
  delay(2 * delayTimeT);
  stopAllMotors();
}


void dropball()
{
  Garra.write(openPosition);
  delay(1000);
  moveBackward(0.5);
  delay(5000);
  moveForward(1);
  Garra.write(closePosition);
}

void kickball()
{
  Garra.write(openPosition);
  delay(300);
  setLeftMotors(255);
  setRightMotors(255); 
  delay (500);
  stopAllMotors();
  setLeftMotors(-baseSpeed);
  setRightMotors(-baseSpeed); 
  delay(1000);
  stopAllMotors();
  delay(5000);
  setLeftMotors(baseSpeed); 
  setRightMotors(baseSpeed); 
  delay(1000);
  Garra.write(closePosition);
}

void setLeftMotors(float speed) 
{
  speed = constrain(speed, -maxSpeed, maxSpeed);
  
  
  if (speed > 0) 
  {
    digitalWrite(leftFront_IN1, LOW);
    digitalWrite(leftFront_IN2, HIGH);
  } 
  else if (speed < 0) 
  {
    digitalWrite(leftFront_IN1, HIGH);
    digitalWrite(leftFront_IN2, LOW);
  } 
  else 
  {
    digitalWrite(leftFront_IN1, LOW);
    digitalWrite(leftFront_IN2, LOW);
  }
  analogWrite(leftFront_ENA, abs(speed));
  
  if (speed > 0) 
  {
    digitalWrite(leftRear_IN3, LOW);
    digitalWrite(leftRear_IN4, HIGH);
  } 
  else if (speed < 0) 
  {
    digitalWrite(leftRear_IN3, HIGH);
    digitalWrite(leftRear_IN4, LOW);
  } 
  else 
  {
    digitalWrite(leftRear_IN3, LOW);
    digitalWrite(leftRear_IN4, LOW);
  }
  analogWrite(leftRear_ENB, abs(speed));
}

void setRightMotors(float speed) 
{
  speed = constrain(speed, -maxSpeed, maxSpeed);
  
  if (speed > 0) 
  {
    digitalWrite(rightFront_IN1, HIGH);
    digitalWrite(rightFront_IN2, LOW);
  } 
  else if (speed < 0) 
  {
    digitalWrite(rightFront_IN1, LOW);
    digitalWrite(rightFront_IN2, HIGH);
  } 
  else 
  {
    digitalWrite(rightFront_IN1, LOW);
    digitalWrite(rightFront_IN2, LOW);
  }
  analogWrite(rightFront_ENA, abs(speed));
  
  if (speed > 0) 
  {
    digitalWrite(rightRear_IN3, HIGH);
    digitalWrite(rightRear_IN4, LOW);
  } 
  else if (speed < 0) 
  {
    digitalWrite(rightRear_IN3, LOW);
    digitalWrite(rightRear_IN4, HIGH);
  } 
  else 
  {
    digitalWrite(rightRear_IN3, LOW);
    digitalWrite(rightRear_IN4, LOW);
  }
  analogWrite(rightRear_ENB, abs(speed));
}

void stopAllMotors() 
{
  setLeftMotors(0);
  setRightMotors(0);
  delay(1000);
}

float readDistanceCM(const USonic& u) 
{
  float r1, r2, r3;
  // 1
  digitalWrite(u.trig, LOW); delayMicroseconds(2);
  digitalWrite(u.trig, HIGH); delayMicroseconds(10);
  digitalWrite(u.trig, LOW);
  r1 = pulseIn(u.echo, HIGH, 30000UL) * 0.0343 / 2.0;  
  delay(10);
  // 2
  digitalWrite(u.trig, LOW); delayMicroseconds(2);
  digitalWrite(u.trig, HIGH); delayMicroseconds(10);
  digitalWrite(u.trig, LOW);
  r2 = pulseIn(u.echo, HIGH, 30000UL) * 0.0343 / 2.0;
  delay(10);
  // 3
  digitalWrite(u.trig, LOW); delayMicroseconds(2);
  digitalWrite(u.trig, HIGH); delayMicroseconds(10);
  digitalWrite(u.trig, LOW);
  r3 = pulseIn(u.echo, HIGH, 30000UL) * 0.0343 / 2.0;
  delay(5);

  float m = medianOf3(r1, r2, r3);
  if (m <= 0 || m > 400) m = 400; 
  return m;
}

float medianOf3(float a, float b, float c) {
  float x = a, y = b, z = c;
  if (x > y) { float t = x; x = y; y = t; }
  if (y > z) { float t = y; y = z; z = t; }
  if (x > y) { float t = x; x = y; y = t; }
  return y; 
}

bool isPathFree(float d_cm) {
  return d_cm > WALL_CM;
}

bool isBlackTile() 
{
  int ColorN = ColorDetectation();

  if (ColorN != 7)
  {
    moveForward(0.5);
  }

  else
  {
    turn180();
    moveForward(0.5);
  }
}


int ColorDetectation()
{
  digitalWrite(TcS2, LOW);
  digitalWrite(TcS3, LOW);
  red = pulseIn(signal, HIGH);
  
  digitalWrite(TcS2, HIGH);
  digitalWrite(TcS3, HIGH);
  green = pulseIn(signal, HIGH);
  
  digitalWrite(TcS2, LOW);
  digitalWrite(TcS3, HIGH);
  blue = pulseIn(signal, HIGH);
  
  redSum += red;
  greenSum += green;
  blueSum += blue;
  readingCount++;
  
  unsigned long currentTime = millis();
  
  if (currentTime - previousTime >= IntervaloL) 
  {
    unsigned long redAvg = redSum / readingCount;
    unsigned long greenAvg = greenSum / readingCount;
    unsigned long blueAvg = blueSum / readingCount;
    
    Serial.println();
    Serial.println("PROMEDIO DE 5 SEGUNDOS");
    Serial.print("Lecturas tomadas: ");
    Serial.println(readingCount);
    Serial.print("Promedio RED: ");
    Serial.println(redAvg);
    Serial.print("Promedio GREEN: ");
    Serial.println(greenAvg);
    Serial.print("Promedio BLUE: ");
    Serial.println(blueAvg);
    Serial.println();
    Serial.println();
    
    redSum = 0;
    greenSum = 0;
    blueSum = 0;
    readingCount = 0;
    previousTime = currentTime;

  if (redAvg >= yellowRedMin && redAvg <= yellowRedMax &&
           greenAvg >= yellowGreenMin && greenAvg <= yellowGreenMax &&
           blueAvg <= yellowBlueMax) 
  {
    sr.set(1, HIGH);
    ColorDetected = 1;
    delay(1000);
    sr.setAllLow();
    return ColorDetected;
  }

  else if (redAvg <= cyanRedMax &&
           greenAvg >= cyanGreenMin && greenAvg <= cyanGreenMax &&
           blueAvg >= cyanBlueMin && blueAvg <= cyanBlueMax) 
  {
    ColorDetected = 2;
    return ColorDetected;
  }
  
  else if (blueAvg >= blueBlueMin && blueAvg <= blueBlueMax &&
           redAvg <= blueRedMax &&
           greenAvg <= blueGreenMax) 
  {
    sr.set(2, HIGH);
    ColorDetected = 3;
    delay(1000);
    sr.setAllLow();
    return ColorDetected;
  }
  
  else if (redAvg >= pinkRedMin && redAvg <= pinkRedMax &&
           greenAvg >= pinkGreenMin && greenAvg <= pinkGreenMax &&
           blueAvg >= pinkBlueMin && blueAvg <= pinkBlueMax) 
  {
    sr.set(3, HIGH);
    ColorDetected = 4;
    delay(1000);
    sr.setAllLow();
    return ColorDetected;
  }
  
  else if (greenAvg >= greenGreenMin && greenAvg <= greenGreenMax &&
           redAvg <= greenRedMax &&
           blueAvg <= greenBlueMax) 
  {
    sr.set(4, HIGH);
    ColorDetected = 5;
    delay(1000);
    sr.setAllLow();
    return ColorDetected;
  }

  else if (redAvg >= redRedMin && redAvg <= redRedMax &&
          greenAvg <= redGreenMax &&
          blueAvg <= redBlueMax)
  {
    ColorDetected = 6;
    return ColorDetected;
  }
  
  else if (blackRedMax <= redAvg &&
          blackGreenMax <= redAvg &&
          blackBlueMax <= redAvg)
  {
    ColorDetected = 7;
    Serial.println(ColorDetected);
    return ColorDetected;
  }

  else 
  {
    ColorDetected = 7;
    return ColorDetected;
  }
  }
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

void PistaA() 
{  
  Garra.write(openPosition);
  delay(1000);

  while (stepcount < 6)
  {

    unsigned int Color = ColorDetectation();

    stepcount ++;

    if (stepcount == 1)
      {
        moveForward(0.5);

        if (Color==1)
        {
        kickball();
        LastMove = 1;
        }
        else if (Color==3) 
        {
        dropball();
        LastMove = 0;
        }
        else if (red)
        {
          if (LastMove == 0)
          {
            dropball();
          }
          else if (LastMove == 1)
          {
            kickball();
          }
        }
      }

    if (stepcount == 2)
      {
        turnRight();
        moveForward(1);

        if (Color==1)
        {
        kickball();
        LastMove = 1;
        }
        else if (Color==3) 
        {
        dropball();
        LastMove = 0;
        }
      }

    if (stepcount == 3)
      {
        turnLeft();
        moveForward(1);
        turnLeft();

        if (Color==1)
        {
        kickball();
        LastMove = 1;
        }
        else if (Color==3) 
        {
        dropball();
        LastMove = 0;
        }
      }

    if (stepcount == 4)
      {
        
        turnRight();
        moveForward(1);

        
        if (Color==1)
        {
        kickball();
        LastMove = 1;
        }
        else if (Color==3) 
        {
        dropball();
        LastMove = 0;
        }

      }
      
    if (stepcount == 5)
      {
        if(LastMove == 0)
        {
          dropball();
        }
        else 
        {
          kickball();
        }
      }
    }
  }

void Pista3() 
{
  int Color = ColorDetectation();

  float dF = readDistanceCM(US_FRONT);
  float dR = readDistanceCM(US_RIGHT);
  float dL = readDistanceCM(US_LEFT);

  bool frontFree  = isPathFree(dF);
  bool rightFree  = isPathFree(dR);
  bool leftFree   = isPathFree(dL);

  delay(50);

  if (Color == 6)
  {
    stopAllMotors();
  }
  // 4) Tres tapados o casilla negra -> 180°
  else if ((!frontFree && !rightFree && !leftFree)) 
  {
    Serial.println(F("Regla 4: deadEnd -> giro 180°"));
    turn180();
    moveForward(1);
  }

  // 1) Si derecha libre: girar derecha y avanzar
  else if (rightFree && leftFree && frontFree)
  {
    Serial.println("Adelante");
    moveForward(0.5);
    isBlackTile();
  }

  else if (rightFree && leftFree && !frontFree)
  {
    turnRight();
    moveForward(0.5);
    isBlackTile();
  }

  else if (rightFree && frontFree && !leftFree) 
  {
    Serial.println(F("Regla 1: derecha libre -> giro derecha + avanzar"));
    turnRight();
    moveForward(0.5);
    isBlackTile();
  }

  // 2) Frente libre con pared a la derecha: avanzar
  else if (frontFree && leftFree && !rightFree) 
  {
    Serial.println(F("Regla 2: frente libre con pared a la derecha -> avanzar"));
    moveForward(0.5);
    isBlackTile();
  }

  else if (!frontFree && rightFree && !leftFree)
  {
    turnRight();
    moveForward(0.5);
    isBlackTile();
  }

  // 3) Pared enfrente: girar izquierda y avanzar
  else if (!frontFree && !rightFree && leftFree) 
  {
    Serial.println(F("Regla 3: pared enfrente -> giro izquierda + avanzar"));
    turnLeft();
    moveForward(0.5);
    isBlackTile();
  }
}

void loop()
{
  //turnRight();
  
  setLeftMotors(baseSpeed);
  delay(1000);
  stopAllMotors();
  delay(1000);
  setRightMotors(baseSpeed);
  delay(1000);
  stopAllMotors();
  delay(1000);
  
}


