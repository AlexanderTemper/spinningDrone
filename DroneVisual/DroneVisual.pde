// import libraries
import processing.serial.*;
import java.util.ArrayList;
// Serial port to connect to
boolean SerialEnable = false;

String serialPortName = "/dev/ttyACM0";
Serial serialPort; // Serial port object
SimbleeDaten simblee;


// Communication
ArrayList<Integer> inputBuffer = new ArrayList<Integer>(); 
int inputBytes = 0;
int lastChar = 0;
boolean start = false;

class Sensor {
  float[] rawValues= {0,0,0};
  Graph g;
  float min,max;
  int[] visible = {1,1,1};
  int[] scale = {1,1,1};
  float[][] diagrammValues = new float[3][100];
  int posX,posY = 0;
  color[] colors = new color[3];
  float[] xLabels;
  
  
  Sensor(int posX,int posY,int w,int h,float min,float max,String label,color[] colors,float[] xLabels){
    this.posX = posX;
    this.posY = posY;
    this.min = min;
    this.max = max;
    this.g = new Graph(this.posX+90, this.posY+20, w, h, color (20, 20, 200));
    this.g.xLabel=" Samples ";
    this.g.yLabel=label;
    this.g.Title="";  
    this.g.xDiv=20;  
    this.g.xMax=0; 
    this.g.xMin=-100;  
    this.g.yMax=this.max; 
    this.g.yMin=this.min;
    this.colors[0] = colors[0];
    this.colors[1] = colors[1];
    this.colors[2] = colors[2];
    this.xLabels = xLabels;
  }
  
  void updateDiagramm(float[] raw){
    for (int i=0; i<3; i++) {
      this.rawValues[i] = raw[i];
      if (i<this.diagrammValues.length) {
        for (int k=0; k<this.diagrammValues[i].length-1; k++) {
          this.diagrammValues[i][k] = this.diagrammValues[i][k+1];
        }
        this.diagrammValues[i][this.diagrammValues[i].length-1] = raw[i] * this.scale[i];
      }      
    }
  }
  void draw(){
    fill(this.colors[0]);
    text(nf(this.rawValues[0],1,3),this.posX+45,this.posY+20); 
    fill(this.colors[1]);
    text(nf(this.rawValues[1],1,3),this.posX+45,this.posY+32); 
    fill(this.colors[2]);
    text(nf(this.rawValues[2],1,3),this.posX+45,this.posY+44); 
    this.g.DrawAxis();
    for (int i=0;i<this.diagrammValues.length; i++) {
      this.g.GraphColor = this.colors[i];
      if (this.visible[i]==1)
        this.g.LineGraph(xLabels, this.diagrammValues[i]);
    }
  }
}

// State
//0= wait , 1 = booted, 2=starttart
int timeToWait = 5000; // 5 sec
int state = 0;

// Data from Socket 
BufferedReader in;
Sensor gyro,acc,mag,att,tof,stats;
float[] gyroRaw= {0,0,0};
float[] accRaw = {0,0,0};
float[] magRaw = {0,0,0};
float[] attRaw = {0,0,0};
float[] tofRaw = {0,0,0};
float[] statsRaw = {0,0,0};
PFont f;

int imuLoopTime = 0;
int totalTime = 0;
int totalTimebetweenFrames = 0;

public void settings() {
  System.setProperty("jogl.disable.openglcore", "true");
  size(1680, 860,P3D);
}


void setup() {
  
  ///set line graph colors
  color[] graphColors = new color[3];
  graphColors[0] = color(0, 0, 200);
  graphColors[1] = color(232, 158, 12);
  graphColors[2] = color(255, 0, 0);
  
  // build x axis values for the line graph
  float[] xLabel = new float[100];
  for (int k=0; k<xLabel.length; k++) {
      xLabel[k] = k;
  }
    
  gyro = new Sensor(0,0,(width/3)-90,height/3-40,-10,10,"Gyro",graphColors,xLabel);
  acc = new Sensor(0,height/3,(width/3)-90,height/3-40,-1.5,1.5,"Acc",graphColors,xLabel);
  mag = new Sensor(0,(2*height)/3,(width/3)-90,height/3-40,-50,50,"Mag",graphColors,xLabel);
  
  att = new Sensor((2*width/3)-80,0,(width/3)-90,height/3-40,-180,360,"Att",graphColors,xLabel);
  tof = new Sensor((2*width/3)-80,height/3,(width/3)-90,height/3-40,0,200,"Tof",graphColors,xLabel);
  stats = new Sensor((2*width/3)-80,(2*height)/3,(width/3)-90,height/3-40,0,360,"Stats",graphColors,xLabel);

  simblee = new SimbleeDaten();
  surface.setTitle("BMF055");
  
  if(SerialEnable){
    serialPort = new Serial(this, serialPortName, 115200);
    serialPort.clear();
  }
}

void draw() {
  if(SerialEnable){
    processSerial();
  }
  
  background(255);
  text("TotalTime: "+float(totalTime/1000),(width/2)-100,20); 
  text("TbF: "+float(totalTimebetweenFrames),(width/2)-100,30); 
  text("State: "+state,(width/2)-100,40); 
  text("Frame: "+framecount,(width/2)-100,50); 
  gyro.draw();
  acc.draw();
  mag.draw();
  att.draw();
  tof.draw();
  stats.draw();
  drawOrientation();
}


void drawOrientation(){
    lights();
    pushMatrix(); // begin object
    translate(width/2, height/2); // set position to centre
  
    float c1 = cos(radians(attRaw[2]));
    float s1 = sin(radians(attRaw[2]));
    float c2 = cos(radians(-attRaw[1]));
    float s2 = sin(radians(-attRaw[1]));
    float c3 = cos(radians(360-attRaw[0]-90));
    float s3 = sin(radians(360-attRaw[0]-90));
    applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
                 -s2, c1*c2, c2*s1, 0,
                 c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
                 0, 0, 0, 1);
    drawDrone();
    popMatrix(); // end of object
}

float drehung = 0.0;
int stepperRotation[][] = {
{16000, 60},
{2000, 0},
{0, 60},
{2000, 0},
{16000, 20},
{2000, 0},
{0, 20},
{2000, 0},
{0, 60},
{2000, 0},
};

public static byte[] intToBytes(int l) {
    byte[] result = new byte[4];
    for (int i = 3; i >= 0; i--) {
        result[i] = (byte)(l & 0xFF);
        l >>= 8;
    }
    return result;
}
int framecount = -1;

void sendData(int rotation, int speed){
  serialPort.write((int)'A');
  serialPort.write((int)'B');
  byte[] a = intToBytes(rotation);
  serialPort.write(a[3]);
  serialPort.write(a[2]);
  serialPort.write(a[1]);
  serialPort.write(a[0]);
  byte[] b = intToBytes(speed);
  serialPort.write(b[3]);
  serialPort.write(b[2]);
  serialPort.write((int)'C');
  serialPort.write((int)'D');
}
void sendResetRotation(){
  serialPort.write((int)'A');
  serialPort.write((int)'B');
  serialPort.write((int)'R');
  serialPort.write((int)'C');
  serialPort.write((int)'D');
}
void processSerial(){
  while (serialPort.available() > 0) {
    int data = serialPort.read();
    if (start) {
      if (lastChar == 'C') {
        if (data == 'D') {
          start = false;
          if (inputBuffer.size() == 3 && inputBuffer.get(0) == (int)'R') {
            int reqFrame = (inputBuffer.get(2)<<8)+inputBuffer.get(1);
            //println("Request Frame: "+reqFrame+" is " + );
            if(framecount < reqFrame && reqFrame < stepperRotation.length && state == 1){
              framecount = reqFrame;
              println("send Frame:" + framecount);
              sendData(stepperRotation[framecount][0],stepperRotation[framecount][1]);
            }
            
          } else if(inputBuffer.size() == 9 && inputBuffer.get(0) == (int)'S'){
            int aSteps = (inputBuffer.get(4)<<24)+(inputBuffer.get(3)<<16)+(inputBuffer.get(2)<<8)+inputBuffer.get(1);
            int absSteps = (inputBuffer.get(8)<<24)+(inputBuffer.get(7)<<16)+(inputBuffer.get(6)<<8)+inputBuffer.get(5);
            println("Status:"+aSteps +" "+ absSteps);
            drehung = (1-(float(absSteps)%1600)/1600)*360;
          } else {
            print("FrameError: ");
            println(inputBuffer);
          }
        } else {
          inputBuffer.add(lastChar);
          inputBuffer.add(data);
        }
      } else if (data != 'C') {
        inputBuffer.add(data);
      }
    } else if ( data == 'B' && lastChar == 'A') {
      start = true;
      inputBuffer.clear();
      inputBytes = 0;
    }
    lastChar = data;
  }
}

int visualR = 0;
int visualRhelp = 0;
void drawRotor(int dir){
  
   pushMatrix();
   translate(0,-5,0);
   rotateY(dir*visualR);
   if(visualRhelp == 20){
    visualR++;
     visualRhelp = 0;
   } else {
     visualRhelp++;
   }
    box( 6, 6, 100);
    box( 100, 6, 6);
  popMatrix(); 
  
}
void drawDrone()
{
  noStroke();
  // PCB
  fill(0, 128, 0); 
  box(80, 8, 80); 
  
  // Arrow
  pushMatrix();
    fill(0, 0, 128); 
    translate(50, 0, 0);
    box( 100, 6, 10); 
    pushMatrix();
      translate(36, 0, -12);
      rotateY(PI/4.0);
      box( 10, 6, 45); 
    popMatrix(); 
    pushMatrix();
      translate(36, 0, +12);
      rotateY(-PI/4.0);
      box( 10, 6, 45); 
    popMatrix(); 
  popMatrix(); 
  
  // rotator
  fill(128, 0, 0); 
  pushMatrix();
  rotateY(PI/4.0);
  box( 6, 6, 300);
  box( 300, 6, 6);
  translate(0, 0, 150);
  fill(0, 239, 15); 
    drawRotor(-1);
  translate(0, 0, -300);
  fill(255, 0, 255);  
    drawRotor(-1);
  translate(150, 0, 150);
  fill(0, 239, 15); 
    drawRotor(1);
  translate(-300, 0, 0);
  fill(255, 0, 255); 
    drawRotor(1);
  popMatrix(); 
}
