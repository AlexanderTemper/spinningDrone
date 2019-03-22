// Define pin connections & motor's steps per revolution
const int dirPin = 23;
const int stepPin = 22;
#define MODE0 26
#define MODE1 25
#define MODE2 24
#define STEPS_PER_REVOLUTION 1600
#define DELAY_1U_PER_SEC 1000000/STEPS_PER_REVOLUTION


// LTM Protokoll
char inputBuffer[20]={0};
char inputBytes = 0;
char lastChar = 0;


long steps = 0;
unsigned int uPerMinute = 60;



void setup()
{
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(MODE0, OUTPUT);
  pinMode(MODE1, OUTPUT);
  pinMode(MODE2, OUTPUT);
  
  // 8
  digitalWrite(MODE0, HIGH);
  digitalWrite(MODE1, HIGH);
  digitalWrite(MODE2, LOW);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  Serial.begin(115200);
}

void loop()
{
  get_LTM_data();
  step(steps,uPerMinute);


  steps = 0;
}


/* 
 * Look if there are new LTM data available
 */
void get_LTM_data() 
{
    char data = 0;
    while(Serial.available()){ // read all available data
        
        data = Serial.read();
        //Serial.print(data);
        if (data == 'T' && lastChar == '$'){ // new Frame
            memcpy(&steps, inputBuffer, 4);
            memcpy(&uPerMinute, &inputBuffer[4], 2); 
            memset(inputBuffer,0,6);
            if(uPerMinute>350){// safty
              uPerMinute = 350;
            }
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
 * add LTM data to the inputBuffer buffer max 20 Bytes
 */
void addData( char data)
{
    if(inputBytes<20){
        inputBuffer[inputBytes]= data;
        inputBytes++;
    }
}


void step(long s,unsigned int rotationsPerMinute){
  if(s<0){
    s = -s;
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }
  
  if(rotationsPerMinute>0){
    unsigned int d = (60/((float)rotationsPerMinute)) * DELAY_1U_PER_SEC;
    if(d > 16000){
      d= d/1000;
      for(long i =0; i< s; i++){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(stepPin, LOW);
        delay(d);
      }
    } else {
      for(long i =0; i< s; i++){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(d-10);
      }
    }
    
  }
}
