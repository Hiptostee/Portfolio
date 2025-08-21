#include <Servo.h>
#include <LiquidCrystal.h>

Servo bottomServo; // horizontal
Servo topServo;    // vertical

const int cameraWidth = 1920;
const int cameraHeight = 1080;

const int servoMinAngle = 20;
const int servoMaxAngle = 160;

const int maxStepBottom = 6; 
const int maxStepTop = 6;    

int angleBottomInit = 95; 
int angleTopInit = 90;    

int correctionOffsetX = 5; 
int correctionOffsetY = 0; 

int angleBottom = angleBottomInit;
int angleTop = angleTopInit;

bool useScaling = true;

float scaleInput(float norm)
{
  float inverted = 1.0 - norm;
  float shifted = 2 * inverted - 1;
  float y = 0.6 * (0.7 * shifted + 0.3 * shifted * shifted * shifted);
  return (y + 1) / 2;
}

void setup()
{
  Serial.begin(115200);
  bottomServo.attach(A2);
  topServo.attach(A3);

  bottomServo.write(angleBottom);
  topServo.write(angleTop);

  delay(1000);

}

void loop()
{
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');

    int commaIndex = input.indexOf(',');
    if (commaIndex > 0)
    {
      int faceX = input.substring(0, commaIndex).toInt();
      int faceY = input.substring(commaIndex + 1).toInt();

      float normX = (float)faceX / cameraWidth;
      float normY = (float)faceY / cameraHeight;

      float scaledX = useScaling ? scaleInput(normX) : normX;
      float scaledY = useScaling ? scaleInput(normY) : normY;

      int targetAngleBottom = map((int)(scaledX * 1000), 0, 1000, servoMinAngle, servoMaxAngle);
      int targetAngleTop = map((int)(scaledY * 1000), 0, 1000, servoMinAngle, servoMaxAngle);

      targetAngleBottom += correctionOffsetX;
      targetAngleTop += correctionOffsetY;

      int errorBottom = targetAngleBottom - angleBottom;
      int errorTop = targetAngleTop - angleTop;

      if (abs(errorBottom) > 1)
      {
        int stepB = constrain(errorBottom, -maxStepBottom, maxStepBottom);
        angleBottom += stepB;
        angleBottom = constrain(angleBottom, servoMinAngle, servoMaxAngle);
        bottomServo.write(angleBottom);
      }

      if (abs(errorTop) > 1)
      {
        int stepT = constrain(errorTop, -maxStepTop, maxStepTop);
        angleTop += stepT;
        angleTop = constrain(angleTop, servoMinAngle, servoMaxAngle);
        topServo.write(angleTop);
      }

      Serial.print("FaceX: ");
      Serial.print(faceX);
      Serial.print(" FaceY: ");
      Serial.print(faceY);
      Serial.print(" -> ServoB: ");
      Serial.print(angleBottom);
      Serial.print(" ServoT: ");
      Serial.println(angleTop);
    }
  }

  delay(15);
}
