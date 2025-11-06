#include <Servo.h>
#include <ShiftRegister74HC595.h>
#include <QTRSensors.h>

// ================= MOTOR PINS ==================
const int leftFront_ENA = 4;
const int leftFront_IN1 = 3;
const int leftFront_IN2 = 2;

const int leftRear_ENB = 8;
const int leftRear_IN3 = 10;
const int leftRear_IN4 = 9;

const int rightFront_ENA = 5;
const int rightFront_IN1 = 51;
const int rightFront_IN2 = 50;

const int rightRear_ENB = 7;
const int rightRear_IN3 = 48;
const int rightRear_IN4 = 49;

const int maxSpeed = 255;
const int baseSpeed = 170;

const int delayTimeU = 1800; 
const int delayTimeT = 1800;

// ================= VARIABLES GLOBALES ==================
int Option = 3;
int ColorDetected = 0;
unsigned int LastMove = 0;

// ================= SERVO ==================
Servo Garra;
const int ServoPin = 13;   
const int openPosition = 0;
const int closePosition = 60;

// ================= SENSOR DE COLOR ==================
const int TcS0 = 40;
const int TcS1 = 38;
const int TcS2 = 36;
const int TcS3 = 34;
const int signal = 12;

unsigned long red, green, blue;

// Límites de colores
const unsigned long yellowRedMin = 10;
const unsigned long yellowRedMax = 30;
const unsigned long yellowGreenMin = 20;
const unsigned long yellowGreenMax = 40;
const unsigned long yellowBlueMax = 60;

const unsigned long cyanRedMax = 500;
const unsigned long cyanGreenMin = 800;
const unsigned long cyanGreenMax = 1200;
const unsigned long cyanBlueMin = 850;
const unsigned long cyanBlueMax = 1250;

const unsigned long blueBlueMin = 15;
const unsigned long blueBlueMax = 40;
const unsigned long blueRedMax = 140;
const unsigned long blueGreenMax = 70;

const unsigned long pinkRedMin = 100;
const unsigned long pinkRedMax = 150;
const unsigned long pinkGreenMin = 50;
const unsigned long pinkGreenMax = 80;
const unsigned long pinkBlueMin = 20;
const unsigned long pinkBlueMax = 50;

const unsigned long greenGreenMin = 70;
const unsigned long greenGreenMax = 90;
const unsigned long greenRedMax = 110;
const unsigned long greenBlueMax = 125;

const unsigned long redRedMin = 30;
const unsigned long redRedMax = 60;
const unsigned long redGreenMax = 250;
const unsigned long redBlueMax = 190;

const unsigned long blackRedMax = 600;
const unsigned long blackGreenMax = 600;
const unsigned long blackBlueMax = 600;

// ================= SHIFT REGISTER ==================
const int numberOfShiftRegisters = 1;
int serialDataPin = 30;
int clockPin = 28;
int latchPin = 29;
ShiftRegister74HC595<numberOfShiftRegisters> sr(serialDataPin, clockPin, latchPin);

// ================= SENSORES QTR ==================
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
bool calibrationstate = 0;

int LineGoal = 3500;
float Kp = 0.05;
float Ki = 0.01;
float Kd = 0.02;
int LastError = 0;
int LastIntegral = 0;
bool lineMode = 0;

// ================= ULTRASONICOS ==================
struct USonic {
  int trig;
  int echo;
  float lastValidReading;
};

USonic US_FRONT  = {53, 52, 400.0};
USonic US_RIGHT  = {27, 26, 400.0};
USonic US_LEFT   = {24, 25, 400.0};

const float WALL_CM = 5.0;

// ================= FUNCIONES ==================

void stopAllMotors();
void setLeftMotors(float speed);
void setRightMotors(float speed);
void moveForward(float Units);
void moveBackward(float Units);
void turnRight();
void turnLeft();
void turn180();
void dropball();
void kickball();
float readDistanceCM(USonic& u);
float medianOf3(float a, float b, float c);
bool isPathFree(float d_cm);
bool isBlackTile();
int ColorDetectation();
void calibrateQTR();
void PistaA();
void Pista2();
void Pista3();

// ================================================================

void setup() {
  sr.setAllLow();

  pinMode(42, INPUT_PULLUP);
  pinMode(43, INPUT_PULLUP);
  pinMode(44, INPUT_PULLUP);

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

  delay(1000);
  
  Serial.begin(9600);
}

// ================================================================
// ========== FUNCIONES DE MOVIMIENTO ==============================
void moveForward(float Units) {
  setRightMotors(baseSpeed);
  setLeftMotors(baseSpeed - 30);
  delay(Units * delayTimeU);
  stopAllMotors();
}

void moveBackward(float Units) {
  setRightMotors(-baseSpeed);
  setLeftMotors(-baseSpeed + 25);
  delay(Units * delayTimeU);
  stopAllMotors();
}

void turnRight() {
  setLeftMotors(255);
  setRightMotors(-255);
  delay(delayTimeT);
  stopAllMotors();
}

void turnLeft() {
  setLeftMotors(-255);
  setRightMotors(255);
  delay(delayTimeT + 100);
  stopAllMotors();
}

void turn180() {
  setLeftMotors(-255);
  setRightMotors(255);
  delay(2 * delayTimeT);
  stopAllMotors();
}

void dropball() {
  Garra.write(openPosition);
  delay(1000);
  moveBackward(0.3);
  delay(5000);
  moveForward(0.3);
  Garra.write(closePosition);
  moveForward(0.3);
}

void kickball() {
  Garra.write(openPosition);
  delay(300);
  setLeftMotors(255);
  setRightMotors(255); 
  delay(500);
  stopAllMotors();
  delay(5000);
}

// ================================================================
// ========== CONTROL DE MOTORES ==================================
void setLeftMotors(float speed) {
  speed = constrain(speed, -maxSpeed, maxSpeed);
  if (speed > 0) {
    digitalWrite(leftFront_IN1, LOW);
    digitalWrite(leftFront_IN2, HIGH);
  } else if (speed < 0) {
    digitalWrite(leftFront_IN1, HIGH);
    digitalWrite(leftFront_IN2, LOW);
  } else {
    digitalWrite(leftFront_IN1, LOW);
    digitalWrite(leftFront_IN2, LOW);
  }
  analogWrite(leftFront_ENA, abs(speed));
  
  if (speed > 0) {
    digitalWrite(leftRear_IN3, LOW);
    digitalWrite(leftRear_IN4, HIGH);
  } else if (speed < 0) {
    digitalWrite(leftRear_IN3, HIGH);
    digitalWrite(leftRear_IN4, LOW);
  } else {
    digitalWrite(leftRear_IN3, LOW);
    digitalWrite(leftRear_IN4, LOW);
  }
  analogWrite(leftRear_ENB, abs(speed));
}

void setRightMotors(float speed) {
  speed = constrain(speed, -maxSpeed, maxSpeed);
  if (speed > 0) {
    digitalWrite(rightFront_IN1, LOW);
    digitalWrite(rightFront_IN2, HIGH);
  } else if (speed < 0) {
    digitalWrite(rightFront_IN1, HIGH);
    digitalWrite(rightFront_IN2, LOW);
  } else {
    digitalWrite(rightFront_IN1, LOW);
    digitalWrite(rightFront_IN2, LOW);
  }
  analogWrite(rightFront_ENA, abs(speed));
  
  if (speed > 0) {
    digitalWrite(rightRear_IN3, HIGH);
    digitalWrite(rightRear_IN4, LOW);
  } else if (speed < 0) {
    digitalWrite(rightRear_IN3, LOW);
    digitalWrite(rightRear_IN4, HIGH);
  } else {
    digitalWrite(rightRear_IN3, LOW);
    digitalWrite(rightRear_IN4, LOW);
  }
  analogWrite(rightRear_ENB, abs(speed));
}

void stopAllMotors() {
  setLeftMotors(0);
  setRightMotors(0);
  delay(1000);
}

// ================================================================
// ========== COLOR DETECTION (instantánea) ========================
int ColorDetectation() {
  // Leer rojo
  digitalWrite(TcS2, LOW);
  digitalWrite(TcS3, LOW);
  red = pulseIn(signal, HIGH);

  // Leer verde
  digitalWrite(TcS2, HIGH);
  digitalWrite(TcS3, HIGH);
  green = pulseIn(signal, HIGH);

  // Leer azul
  digitalWrite(TcS2, LOW);
  digitalWrite(TcS3, HIGH);
  blue = pulseIn(signal, HIGH);

  Serial.print("R: "); Serial.print(red);
  Serial.print(" G: "); Serial.print(green);
  Serial.print(" B: "); Serial.println(blue);

  // Evaluar color
  if (red >= yellowRedMin && red <= yellowRedMax &&
      green >= yellowGreenMin && green <= yellowGreenMax &&
      blue <= yellowBlueMax) {
    ColorDetected = 1;  // amarillo
  }
  else if (red <= cyanRedMax &&
           green >= cyanGreenMin && green <= cyanGreenMax &&
           blue >= cyanBlueMin && blue <= cyanBlueMax) {
    ColorDetected = 2;  // cian
  }
  else if (blue >= blueBlueMin && blue <= blueBlueMax &&
           red <= blueRedMax &&
           green <= blueGreenMax) {
    ColorDetected = 3;  // azul
  }
  else if (red >= pinkRedMin && red <= pinkRedMax &&
           green >= pinkGreenMin && green <= pinkGreenMax &&
           blue >= pinkBlueMin && blue <= pinkBlueMax) {
    ColorDetected = 4;  // rosa
  }
  else if (green >= greenGreenMin && green <= greenGreenMax &&
           red <= greenRedMax &&
           blue <= greenBlueMax) {
    ColorDetected = 5;  // verde
  }
  else if (red >= redRedMin && red <= redRedMax &&
           green <= redGreenMax &&
           blue <= redBlueMax) {
    ColorDetected = 6;  // rojo
  }
  else if (red <= blackRedMax &&
           green <= blackGreenMax &&
           blue <= blackBlueMax) {
    ColorDetected = 7;  // negro
  }
  else {
    ColorDetected = 7;  // valor por defecto (negro/desconocido)
  }

  return ColorDetected;
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
  // Reiniciamos el contador localmente para que la rutina sea repetible
  int localStep = 0;

  // Asegurarnos que la garra está abierta al iniciar
  Garra.write(openPosition);
  delay(1000);

  while (localStep < 6) {
    // Leer color justo antes de decidir

    if (localStep == 0) {
      moveForward(0.3);
      moveForward(0.3);

      Garra.write(closePosition);
      delay(300); // pequeño tiempo para que la garra cierre
    }

    else if (localStep == 1) {
      moveForward(0.6);
      unsigned int Color = ColorDetectation();
      delay(1000);

      if (Color == 1) {
        kickball();
        LastMove = 1;
      } 
      else if (Color == 3) {
        dropball();
        LastMove = 0;
      } 
    }
    else if (localStep == 2) {
      moveForward(0.6);

      turnRight();
      moveForward(1.5);
      unsigned int Color = ColorDetectation();
      if (Color == 1) {
        kickball();
        LastMove = 1;
      } 
      else if (Color == 3) {
        dropball();
        LastMove = 0;
      }
    }
    else if (localStep == 3) {
      moveForward(0.4);
      turnLeft();
      moveForward(1);
      turnLeft();
      unsigned int Color = ColorDetectation();
      if (Color == 1) {
        kickball();
        LastMove = 1;
      } 
      else if (Color == 3) {
        dropball();
        LastMove = 0;
      }
    }
    else if (localStep == 4) {
      turnRight();
      moveForward(1);
      unsigned int Color = ColorDetectation();
      if (Color == 1) {
        kickball();
        LastMove = 1;
      } 
      else if (Color == 3) {
        dropball();
        LastMove = 0;
      }
    }
    else if (localStep == 5) {
      if (LastMove == 0) dropball();
      else kickball();
    }

    // avanzar al siguiente paso
    localStep++;

    // pequeño delay para evitar lecturas repetidas/ruido
    delay(200);
  } // fin while

  // Al salir, asegurar que quede en estado seguro
  stopAllMotors();
}

void Pista3() 
{
  int Color = ColorDetectation();

  float dF = readDistanceCM(US_FRONT);
  float dR = readDistanceCM(US_RIGHT);
  float dL = readDistanceCM(US_LEFT);

  Serial.println((dF));
  Serial.println((dR));
  Serial.println((dL));

  

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
    Serial.println(F("Regla GIRO DERECHA"));
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
    Serial.println(F("Regla GIRO DERECHA"));
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

void Pista2() 
{
  qtr.read(sensorValues);

  int HighCount = 0;
  int LowCount = 0;
  int Threshold = 880;

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

  float error = LineGoal - LinePosition;
  float P = error;
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
  AdjustSpeed = constrain(AdjustSpeed, -100.0, 100.0);

  int RightAdjustSpeed = 180 + AdjustSpeed;
  int LeftAdjustSpeed = 180 - AdjustSpeed;

  setRightMotors(RightAdjustSpeed);
  setLeftMotors(LeftAdjustSpeed);

  LastIntegral = I;
  
  LastError = error;

  for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
      }

  Serial.print('\t');
  Serial.print(RightAdjustSpeed);
  Serial.print(LeftAdjustSpeed);

}


void loop()
{
  
  if (digitalRead(42) == LOW)
  {
    Option = 1;
  }
  
  if (digitalRead(43) == LOW)
  {
    Option = 2;
  }
  
  if (digitalRead(44) == LOW)
  {
    Option = 3;
  }

  if (Option == 1)
  {
    //turnRight();
    PistaA();
    delay(50);
  }

  if (Option == 2)
  {

    if (calibrationstate == 0)
    {  
    calibrateQTR();
    stopAllMotors();
    }

    else 
    {
      Pista2();    
    }

  }
  
  if (Option == 3)
  {
   Pista3();
  }
}
