// Define pin connections & motor's steps per revolution
const int dirPin = 23;
const int stepPin = 22;
#define MODE0 26
#define MODE1 25
#define MODE2 24
const int stepsPerRevolution = 200;

void setup()
{
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(MODE0, OUTPUT);
  pinMode(MODE1, OUTPUT);
  pinMode(MODE2, OUTPUT);
  
  // Set motor direction clockwise
  digitalWrite(MODE0, LOW);
  digitalWrite(MODE1, HIGH);
  digitalWrite(MODE2, HIGH);
}
void loop()
{
  int r = analogRead(A0);
  
  if(r <= 1020){
    float sensorValue = (float)r/1023;
    int d = sensorValue*1000;
    if(d > 16000){
      d = d/1000;
      digitalWrite(stepPin, HIGH);
      delay(d);
      digitalWrite(stepPin, LOW);
      delay(d);
    } else {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(d);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(d);
    }
  }
  
  
}
