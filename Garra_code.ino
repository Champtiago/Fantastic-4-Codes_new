#include <Servo.h>

Servo Garra;   

const int ServoPin = 13;   
const int openPosition = 0;
const int closePosition = 60;
unsigned int LastMove;
int stepcount = 0;

void setup() 
{
  Garra.attach(ServoPin);   
  Garra.write(openPosition);

  delay(1000); 
}

void dropball()
{
  Garra.write(openPosition);
  delay(1000);
  //moveBackwards();
  delay(5000);
  //setLeftMotors(baseSpeed);
  //setRightMotors(baseSpeed);
  //delay(?);
  Garra.write(closePosition);
  delay(1000);
  //moveForward();
  //LastMove = 0;
}

/*
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
  delay(3000);
  stopAllMotors();
  setLeftMotors(baseSpeed); 
  setRightMotors(baseSpeed); 
  delay(?);
  Garra.write(closePosition);
  LastMove = 1;
}

*/

void loop() 
{  
  Garra.write(openPosition);
  delay(1000);
  Garra.write(closePosition);
  delay(1000);
  /*
  while (digitalRead(52) == HIGH)
  {
    stepcount ++;

    if stepcount == 1
    {
      setLeftMotors(baseSpeed) 
      setRightMotors(baseSpeed) 
      delay(3000)
    }

    else if stepcount > 1
    {
      if (yellow)
      {
      kickball()
      LastMove = 1;
      }
      else if (cyan) 
      {
      dropball()
      LastMove = 0;
      }
      else if (red)
      {
        if LastMove == 0
        {
          dropball()
        }
        else if LastMove == 1
        {
          kickball()
        }
      }
    }

    if stepcount == 2
    {
      turnRight()
      moveForward()
    }

    if stepcount == 3
    {
      turnLeft()
      moveForward()
      turnLeft()
    }

    if stepcount == 4
    {
      turnRight()
      moveForward()
    }


  }
  */
}
