
#include <ShiftRegister74HC595.h>

const int numberOfShiftRegisters  = 1;
int serialDataPin = 30;
int clockPin = 28;
int latchPin = 29;

ShiftRegister74HC595<numberOfShiftRegisters > sr(serialDataPin, clockPin, latchPin);

const int TcS0 = 3;
const int TcS1 = 40;
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

const unsigned long yellowRedMin = 200;
const unsigned long yellowRedMax = 300;
const unsigned long yellowGreenMin = 300;
const unsigned long yellowGreenMax = 450;
const unsigned long yellowBlueMax = 700;

const unsigned long cyanRedMax = 500;
const unsigned long cyanGreenMin = 800;
const unsigned long cyanGreenMax = 1200;
const unsigned long cyanBlueMin = 850;
const unsigned long cyanBlueMax = 1250;

const unsigned long blueBlueMin = 300;
const unsigned long blueBlueMax = 450;
const unsigned long blueRedMax = 1600;
const unsigned long blueGreenMax = 880;

const unsigned long pinkRedMin = 300;
const unsigned long pinkRedMax = 370;
const unsigned long pinkGreenMin = 707;
const unsigned long pinkGreenMax = 810;
const unsigned long pinkBlueMin = 445;
const unsigned long pinkBlueMax = 500;

const unsigned long greenGreenMin = 900;
const unsigned long greenGreenMax = 1100;
const unsigned long greenRedMax = 1600;
const unsigned long greenBlueMax = 1100;

const unsigned long redRedMin = 500;
const unsigned long redRedMax = 700;
const unsigned long redGreenMax = 800;
const unsigned long redBlueMax =  1000;

const unsigned long blackRedMax = 1800;
const unsigned long blackGreenMax = 1800;
const unsigned long blackBlueMax = 1300;

int ColorDetected = 0;

/*
yellow = 1
cyan = 2
blue = 3
rosa = 4
verde = 5
rojo = 6
negro = 7
*/

void setup() 
{
  pinMode(TcS0, OUTPUT);
  pinMode(TcS1, OUTPUT);
  pinMode(TcS2, OUTPUT);
  pinMode(TcS3, OUTPUT);
  pinMode(signal, INPUT);
  
  digitalWrite(TcS0, HIGH);
  digitalWrite(TcS1, LOW);
  
  sr.setAllLow();
  delay(500); 
  
  Serial.begin(9600);
}

void loop() 
{
  delay(1000);
  ColorDetectation();
}

void ColorDetectation()
{
  /**
  S2 S3
  L L Red
  L H Blue
  H L Clear (no filter)
  H H Green
  */
  
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
