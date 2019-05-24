#include <SimbleeBLE.h>
#include <LoopbackStream.h>

LoopbackStream inBuffer(256);

unsigned long t;

uint8_t bleBuffer[20];
void setup() {
  // initialize serial:
  Serial.begin(19200);//,3,2);
  override_uart_limit = true;
  SimbleeBLE.deviceName = "Drone";
  SimbleeBLE.begin();
  t = millis();
}

void loop() {
  

  // Copy Serial Buffer
  while (Serial.available()) {
    inBuffer.write((char)Serial.read());
    t = millis();
  }
  // Send inBuffer
  if (inBuffer.available() >= 20 || (inBuffer.available() > 0  && ((millis()-t) > 5) )) {
    sendBle();
  }
  
}

void SimbleeBLE_onReceive(char *dataBLE, int len) {
  Serial.write((uint8_t *)dataBLE,len);
}


void sendBle() {
  int i = 0;
  while(inBuffer.available()){
    bleBuffer[i] = inBuffer.read();
    i++;
    if(i==20){
      break;
    }
  }
  SimbleeBLE.send((char *)&bleBuffer[0], i);
}


