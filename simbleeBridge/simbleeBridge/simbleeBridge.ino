#include <SimbleeBLE.h>

char serialbuffer[20] = "";        
uint8_t top;


void setup() {
  // initialize serial:
  Serial.begin(9600);

  SimbleeBLE.deviceName = "Drone";
  SimbleeBLE.begin();

  top = 0;
}

void loop() {

  while (Serial.available()) { 
    char inChar = (char)Serial.read();
    serialbuffer[top] = inChar;
    top++;
    if(top == 20){
      SimbleeBLE.send(serialbuffer,20);
      top = 0;
    }
  }
  
}


