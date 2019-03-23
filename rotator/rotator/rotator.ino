// Define pin connections & motor's steps per revolution
#define DIR_PIN 23
#define STEP_PIN 22
#define MODE0 26
#define MODE1 25
#define MODE2 24
#define STEPS_PER_REVOLUTION 1600
#define DELAY_1U_PER_SEC 1000000/STEPS_PER_REVOLUTION


// LTM Protokoll
char inputBuffer[20] = {0};
char inputBytes = 0;
char lastChar = 0;

unsigned int uPerMinute = 60;



unsigned long time;
#define INTERVAL_STATUS_SEND 100000

// absolute position in steps (from start of Programm)
long absoluteSteps = 0;

unsigned long timeToWait = 0;
unsigned long timeStepper = 0;
unsigned long time_statusSend = 0;



struct rotation {
  long steps;
  unsigned int speed;
};

struct rotation activeRotation = {600, 10};
struct rotation nextRotation = {200, 5};



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


  //----------------REMOVE ME --------------------------------
  timeToWait = (60 / ((float)activeRotation.speed)) * DELAY_1U_PER_SEC;
  //----------------REMOVE ME --------------------------------
}
char buffer[50];


inline void printStatus(){
   if(nextRotation.speed == 0){
      sprintf(buffer, "%lu;READY", time_statusSend);
      Serial.write(buffer);
      Serial.write(13);
    } 
    //if(activeRotation.speed != 0){
      sprintf(buffer, "%lu;STATUS;%ld;%ld;%d", time_statusSend, activeRotation.steps,absoluteSteps,activeRotation.speed);
      Serial.write(buffer);
      Serial.write(13);
    //}
}




void loop()
{
  // rotation finished start next rotation and clear next rotation buffer
  if (activeRotation.steps == absoluteSteps) {
    printStatus();// print Rotation finished
    
    activeRotation.steps = nextRotation.steps;
    activeRotation.speed = nextRotation.speed;
    // set direction off Rotation
    absoluteSteps > activeRotation.steps ? digitalWrite(DIR_PIN, HIGH) : digitalWrite(DIR_PIN, LOW);
    // calculate timeToWait from Rotation per Minutes
    if (activeRotation.speed > 0) {
      timeToWait = (60 / ((float)activeRotation.speed)) * DELAY_1U_PER_SEC;
    } else { // invalid speed skip this rotation
      timeToWait = 0;
      activeRotation.steps = absoluteSteps;
    }
    // clear next rotation buffer
    nextRotation.steps = 0;
    nextRotation.speed = 0;
  }

  if (timeToWait > 0 && (micros() - timeStepper) >  timeToWait) {
    timeStepper = micros();
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    absoluteSteps > activeRotation.steps ?  absoluteSteps-- : absoluteSteps++;
  }

  if (micros() - time_statusSend >  INTERVAL_STATUS_SEND) {
    time_statusSend = micros();
    printStatus();
    readData();
  }

}

void readData(){
 
  char data = 0;
  while(Serial.available()){ // read all available data
      data = Serial.read();
       //Serial.print(data);
      if (data == 'T' && lastChar == '$'){ // new Frame
        if(inputBytes==4 && nextRotation.steps == 0 && nextRotation.speed == 0){
          memcpy(&nextRotation.steps, inputBuffer, 4);
          //memcpy(&uPerMinute, &inputBuffer[4], 2);
          nextRotation.speed = 60;
          sprintf(buffer, "NEU = %ld",nextRotation.steps);
          Serial.write(buffer);
          Serial.write(13);
        }
        memset(inputBuffer,0,inputBytes);
        inputBytes=0;
      } else if(lastChar == '$'){
          addData(lastChar);
          addData(data);
      } else if(data != '$'){
          addData(data);
      }
      lastChar = data;
  }
}

/*
void get_LTM_data()
{
  char data = 0;
  while(Serial.available()){ // read all available data
  
      data = Serial.read();
      //Serial.print(data);
      if (data == 'T' && lastChar == '$'){ // new Frame
          long s;
          memcpy(&s, inputBuffer, 4);
          //memcpy(&uPerMinute, &inputBuffer[4], 2);
          memset(inputBuffer,0,6);
          inputBytes=0;
      } else if(lastChar == '$'){
          addData(lastChar);
          addData(data);
      } else if(data != '$'){
          addData(data);
      }
      lastChar = data;
  }
}*/

void addData( char data)
{
  if(inputBytes<20){
      inputBuffer[inputBytes]= data;
      inputBytes++;
  }
}
