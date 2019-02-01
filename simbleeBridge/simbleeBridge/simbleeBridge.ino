#include <SimbleeBLE.h>

#define INPUT_BUFFER 200


char serialbuffer[INPUT_BUFFER] = "";        
boolean stringComplete = false;  
char *serialbufferP = &serialbuffer[0];

void setup() {
  // initialize serial:
  Serial.begin(9600);

  SimbleeBLE.deviceName = "Drone";
  SimbleeBLE.begin();
}

void loop() {

  // print the string when a newline arrives:
  if (stringComplete) {
    serialbufferP = &serialbuffer[0];
    SimbleeBLE.send(serialbufferP,20);
    memset(serialbuffer,0,INPUT_BUFFER);
    stringComplete = false;
  }

  
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    *serialbufferP = inChar;
    serialbufferP++;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


