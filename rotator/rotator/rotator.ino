// Define pin connections & motor's steps per revolution
#define DIR_PIN 23
#define STEP_PIN 22
#define MODE0 26
#define MODE1 25
#define MODE2 24
#define STEPS_PER_REVOLUTION 1600
#define DELAY_1U_PER_SEC 1000000/STEPS_PER_REVOLUTION



// Communication
#define BEGIN_FRAME Serial.write('A');Serial.write('B');
#define END_FRAME Serial.write('C');Serial.write('D');
#define INTERVAL_STATUS_SEND 100000
char inputBuffer[20] = {0};
char inputBytes = 0;
char lastChar = 0;
bool start = false;

unsigned long time;

// absolute position in steps (from start of Programm)
long absoluteSteps = 0;
// scheduler timer
unsigned long timeToWait = 0;
unsigned long timeStepper = 0;
unsigned long time_statusSend = 0;
// rotation management
unsigned int frameCount = 0;
struct rotation {
  long steps;
  int speed;
  bool done;
};
struct rotation activeRotation = {0, 0, true};
struct rotation nextRotation = {0, 0, true};



void setup()
{
  // Declare pins as Outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MODE0, OUTPUT);
  pinMode(MODE1, OUTPUT);
  pinMode(MODE2, OUTPUT);

  // 8
  digitalWrite(MODE0, HIGH);
  digitalWrite(MODE1, HIGH);
  digitalWrite(MODE2, LOW);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  Serial.begin(115200);
}

void resetRotations(){
  activeRotation.steps = 0;
  activeRotation.speed = 0;
  activeRotation.done = true;
  nextRotation.steps = 0;
  nextRotation.speed = 0;
  nextRotation.done = true;
}
// Main Loop
void loop()
{
  // rotation done load next rotation if possible
  if (activeRotation.done && !nextRotation.done) {
    activeRotation.steps = nextRotation.steps;
    activeRotation.speed = nextRotation.speed;
    activeRotation.done = false;
    // set direction off Rotation
    absoluteSteps > activeRotation.steps ? digitalWrite(DIR_PIN, HIGH) : digitalWrite(DIR_PIN, LOW);
    // calculate timeToWait from Rotation per Minutes
    if (activeRotation.speed > 0) {
      timeToWait = (60 / ((float)activeRotation.speed)) * DELAY_1U_PER_SEC;
    } else { // speed 0 means wait timer
      timeToWait = activeRotation.steps*1000;
      activeRotation.steps = absoluteSteps;
      activeRotation.speed = 0;
      activeRotation.done = false;
    }
    // clear next rotation buffer
    nextRotation.steps = 0;
    nextRotation.speed = 0;
    nextRotation.done = true;
  }


  if (!activeRotation.done && timeToWait > 0 && (micros() - timeStepper) >  timeToWait ) {
    timeStepper = micros();
    if (activeRotation.speed > 0) {
      digitalWrite(STEP_PIN, HIGH);
      digitalWrite(STEP_PIN, LOW);
      absoluteSteps > activeRotation.steps ?  absoluteSteps-- : absoluteSteps++;
    }
    if (activeRotation.steps == absoluteSteps) {
      activeRotation.done = true;
      BEGIN_FRAME
      Serial.write('S');
      Serial.write((uint8_t*)&activeRotation.steps, 4);
      Serial.write((uint8_t*)&absoluteSteps, 4);
      END_FRAME
    }
  }



  if (micros() - time_statusSend >  INTERVAL_STATUS_SEND) {
    time_statusSend = micros();
    printStatus();
    readData();
  }

}

// communication helper
void readData() {
  char data = 0;
  while (Serial.available()) { // read all available data
    data = Serial.read();
    if (start) {
      if (lastChar == 'C') {
        if (data == 'D') {
          start = false;
          if (inputBytes == 6) {
            memcpy(&nextRotation.steps, inputBuffer, 4);
            memcpy(&nextRotation.speed, &inputBuffer[4], 2);
            nextRotation.done = false;
            frameCount++;
            /*BEGIN_FRAME
              Serial.write((uint8_t*)&nextRotation.steps,4);
              Serial.write((uint8_t*)&nextRotation.speed,2);
              END_FRAME*/
          } else if(inputBytes == 1){
            if(inputBuffer[0] == 'R'){
              Serial.write("Reset");
              frameCount=0;
              absoluteSteps = 0;
              resetRotations();
            }
          } else {
            BEGIN_FRAME
            Serial.write("FrameERROR");
            END_FRAME
          }
        } else {
          addData(lastChar);
          addData(data);
        }
      } else if (data != 'C') {
        addData(data);
      }
    } else if ( data == 'B' && lastChar == 'A') {
      start = true;
      memset(inputBuffer, 0, inputBytes);
      inputBytes = 0;
    }
    lastChar = data;
  }
}
void addData( char data)
{
  if (inputBytes < 20) {
    inputBuffer[inputBytes++] = data;
  }
}
void printStatus() {
  BEGIN_FRAME
  Serial.write('S');
  Serial.write((uint8_t*)&activeRotation.steps, 4);
  Serial.write((uint8_t*)&absoluteSteps, 4);
  END_FRAME
  if (nextRotation.done) {
    BEGIN_FRAME
    Serial.write('R');
    Serial.write((uint8_t*)&frameCount, 2);
    END_FRAME
  }
}
