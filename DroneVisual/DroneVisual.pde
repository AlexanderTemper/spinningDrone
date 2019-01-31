// import libraries
import controlP5.*; // http://www.sojamo.de/libraries/controlP5/
import processing.serial.*;

Orientation orientation;
// Serial port to connect to
String serialPortName = "/dev/ttyUSB0";
Serial serialPort; // Serial port object

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
float[] acc = {0,0,0};
float[] gyro = {0,0,0};
float[] mag = {0,0,0};

// interface stuff
ControlP5 cp5;

int GyroGraphX = 10;
int GyroGraphY = 80;
int GyroGraphMin = -10;
int GyroGraphMax = 10;
int vGyro[] = {1,1,1};
float sGyro[] = {1,1,1};
Graph GyroGraph = new Graph(GyroGraphX+220, GyroGraphY, 550, 120, color (20, 20, 200));

int AccGraphX = 10;
int AccGraphY = 350;
int AccGraphMin = -1;
int AccGraphMax = 1;
int vAcc[] = {1,1,1};
float sAcc[] = {1,1,1};
Graph AccGraph = new Graph(AccGraphX+220, AccGraphY, 550, 120, color (20, 20, 200));

int MagGraphX = 10;
int MagGraphY = 620;
int MagGraphMin = -50;
int MagGraphMax = 50;
int vMag[] = {1,1,1};
float sMag[] = {0.0625,0.0625,0.0625};
Graph MagGraph = new Graph(MagGraphX+220, MagGraphY, 550, 120, color (20, 20, 200));

// plots
float[][] GyroGraphValues = new float[3][100];
float[][] AccGraphValues = new float[3][100];
float[][] MagGraphValues = new float[3][100];
float[] lineGraphSampleNumbers = new float[100];
color[] graphColors = new color[3];

PFont f;

public void settings() {
  size(800, 860);
}


void setup() {
  orientation = new Orientation();
  surface.setTitle("BMF055");

  // set line graph colors
  graphColors[0] = color(131, 255, 20);
  graphColors[1] = color(232, 158, 12);
  graphColors[2] = color(255, 0, 0);

  // gui
  cp5 = new ControlP5(this);
  
  // init charts
  setChartSettings();
  // build x axis values for the line graph
  for (int i=0; i<GyroGraphValues.length; i++) {
    for (int k=0; k<GyroGraphValues[0].length; k++) {
      GyroGraphValues[i][k] = 0;
      if (i==0)
        lineGraphSampleNumbers[k] = k;
    }
  }

  serialPort = new Serial(this, serialPortName, 115200);


  // build the gui
  int x = GyroGraphX;
  int y = GyroGraphY;
  
  cp5.addTextfield("GyroMax").setPosition(x+200, y-40).setText(GyroGraphMax+"").setWidth(40).setAutoClear(false);
  cp5.addTextfield("GyroMin").setPosition(x+200, y+140).setText(GyroGraphMin+"").setWidth(40).setAutoClear(false);
  cp5.addTextlabel("on/off/gyro").setText("on/off").setPosition(GyroGraphX + 13, GyroGraphY -20).setColor(0);
  cp5.addTextlabel("multipliers/gyro").setText("multipliers").setPosition(GyroGraphX + 55, GyroGraphY-20).setColor(0);
  cp5.addTextfield("sGyrox").setPosition(x=GyroGraphX+50, y=GyroGraphY).setText(sGyro[0]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("sGyroy").setPosition(x, y=y+40).setText(sGyro[1]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("sGyroz").setPosition(x, y=y+40).setText(sGyro[2]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addToggle("vGyrox").setPosition(x=x-50, y=GyroGraphY).setValue(vGyro[0]).setMode(ControlP5.SWITCH).setColorActive(graphColors[0]);
  cp5.addToggle("vGyroy").setPosition(x, y=y+40).setValue(vGyro[1]).setMode(ControlP5.SWITCH).setColorActive(graphColors[1]);
  cp5.addToggle("vGyroz").setPosition(x, y=y+40).setValue(vGyro[2]).setMode(ControlP5.SWITCH).setColorActive(graphColors[2]);
  
  x = AccGraphX;
  y = AccGraphY;
  
  cp5.addTextfield("AccMax").setPosition(x+200, y-40).setText(AccGraphMax+"").setWidth(40).setAutoClear(false);
  cp5.addTextfield("AccMin").setPosition(x+200, y+140).setText(AccGraphMin+"").setWidth(40).setAutoClear(false);
  cp5.addTextlabel("on/off/acc").setText("on/off").setPosition(AccGraphX +13, AccGraphY -20).setColor(0);
  cp5.addTextlabel("multipliers/acc").setText("multipliers").setPosition( AccGraphX + 55, AccGraphY-20).setColor(0);
  cp5.addTextfield("sAccx").setPosition(x=AccGraphX+50, y=AccGraphY).setText(sAcc[0]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("sAccy").setPosition(x, y=y+40).setText(sAcc[1]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("sAccz").setPosition(x, y=y+40).setText(sAcc[2]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addToggle("vAccx").setPosition(x=x-50, y=AccGraphY).setValue(vAcc[0]).setMode(ControlP5.SWITCH).setColorActive(graphColors[0]);
  cp5.addToggle("vAccy").setPosition(x, y=y+40).setValue(vAcc[1]).setMode(ControlP5.SWITCH).setColorActive(graphColors[1]);
  cp5.addToggle("vAccz").setPosition(x, y=y+40).setValue(vAcc[2]).setMode(ControlP5.SWITCH).setColorActive(graphColors[2]);
  
  x = MagGraphX;
  y = MagGraphY;
  
  cp5.addTextfield("MagMax").setPosition(x+200, y-40).setText(MagGraphMax+"").setWidth(40).setAutoClear(false);
  cp5.addTextfield("MagMin").setPosition(x+200, y+140).setText(MagGraphMin+"").setWidth(40).setAutoClear(false);
  cp5.addTextlabel("on/off/Mag").setText("on/off").setPosition(MagGraphX +13, MagGraphY -20).setColor(0);
  cp5.addTextlabel("multipliers/Mag").setText("multipliers").setPosition( MagGraphX + 55, MagGraphY-20).setColor(0);
  cp5.addTextfield("sMagx").setPosition(x=MagGraphX+50, y=MagGraphY).setText(sMag[0]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("sMagy").setPosition(x, y=y+40).setText(sMag[1]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("sMagz").setPosition(x, y=y+40).setText(sMag[2]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addToggle("vMagx").setPosition(x=x-50, y=MagGraphY).setValue(vMag[0]).setMode(ControlP5.SWITCH).setColorActive(graphColors[0]);
  cp5.addToggle("vMagy").setPosition(x, y=y+40).setValue(vMag[1]).setMode(ControlP5.SWITCH).setColorActive(graphColors[1]);
  cp5.addToggle("vMagz").setPosition(x, y=y+40).setValue(vMag[2]).setMode(ControlP5.SWITCH).setColorActive(graphColors[2]);
}

void updateGraphs(){
  for (i=0; i<3; i++) {
      // update Gyro
        if (i<GyroGraphValues.length) {
          for (int k=0; k<GyroGraphValues[i].length-1; k++) {
            GyroGraphValues[i][k] = GyroGraphValues[i][k+1];
          }
          GyroGraphValues[i][GyroGraphValues[i].length-1] = gyro[i] * sGyro[i];// Muilltiply
        }      
      // update Acc
        if (i<AccGraphValues.length) {
          for (int k=0; k<AccGraphValues[i].length-1; k++) {
            AccGraphValues[i][k] = AccGraphValues[i][k+1];
          }
          AccGraphValues[i][AccGraphValues[i].length-1] = acc[i]* sAcc[i];
        }
      // update Mag
        if (i<MagGraphValues.length) {
          for (int k=0; k<MagGraphValues[i].length-1; k++) {
            MagGraphValues[i][k] = MagGraphValues[i][k+1];
          }
          MagGraphValues[i][MagGraphValues[i].length-1] = mag[i] * sMag[i];
      }
    }
}

int i = 0; // loop variable
void draw() {
  int newLine = 13; // new line character in ASCII
  String message;
  // pars serial
  do {
    message = serialPort.readStringUntil(newLine); // read from port until new line
    if (message != null) {
      String[] list = split(trim(message), " ");
      if (list.length >= 4 && list[0].equals("Orientation:")) {
        yaw = float(list[1]); // convert to float yaw
        pitch = float(list[2]); // convert to float pitch
        roll = float(list[3]); // convert to float roll
      } else if (list.length >= 10 && list[0].equals("DATA:")){
        acc[0]=float(list[1]);
        acc[1]=float(list[2]);
        acc[2]=float(list[3]);
        
        gyro[0]=float(list[4]);
        gyro[1]=float(list[5]);
        gyro[2]=float(list[6]);
        
        mag[0]=float(list[7]);
        mag[1]=float(list[8]);
        mag[2]=float(list[9]);
        updateGraphs();
      }
      else {
        println(message);
      }
    }
  } while (message != null);
  
  background(255);
  // gyro
 GyroGraph.DrawAxis();
  for (int i=0;i<GyroGraphValues.length; i++) {
    GyroGraph.GraphColor = graphColors[i];
    if (vGyro[i]==1)
      GyroGraph.LineGraph(lineGraphSampleNumbers, GyroGraphValues[i]);
  }
  
  // acc
  AccGraph.DrawAxis();
  for (int i=0;i<AccGraphValues.length; i++) {
    AccGraph.GraphColor = graphColors[i];
    if (vAcc[i]==1)
      AccGraph.LineGraph(lineGraphSampleNumbers, AccGraphValues[i]);
  }
  
  // Mag
  MagGraph.DrawAxis();
  for (int i=0;i<MagGraphValues.length; i++) {
    MagGraph.GraphColor = graphColors[i];
    if (vMag[i]==1)
      MagGraph.LineGraph(lineGraphSampleNumbers, MagGraphValues[i]);
  }
  
}

// called each time the chart settings are changed by the user 
void setChartSettings() {
  GyroGraph.xLabel=" Samples ";
  GyroGraph.yLabel="Gyro";
  GyroGraph.Title="";  
  GyroGraph.xDiv=20;  
  GyroGraph.xMax=0; 
  GyroGraph.xMin=-100;  
  GyroGraph.yMax=GyroGraphMax; 
  GyroGraph.yMin=GyroGraphMin;
  
  AccGraph.xLabel=" Samples ";
  AccGraph.yLabel="Acc";
  AccGraph.Title="";  
  AccGraph.xDiv=20;  
  AccGraph.xMax=0; 
  AccGraph.xMin=-100;  
  AccGraph.yMax=AccGraphMax; 
  AccGraph.yMin=AccGraphMin;
  
  MagGraph.xLabel=" Samples ";
  MagGraph.yLabel="Mag";
  MagGraph.Title="";  
  MagGraph.xDiv=20;  
  MagGraph.xMax=0; 
  MagGraph.xMin=-100;  
  MagGraph.yMax=MagGraphMax; 
  MagGraph.yMin=MagGraphMin;
 
}

// handle gui actions
void controlEvent(ControlEvent theEvent) {
  if (theEvent.isAssignableFrom(Textfield.class) || theEvent.isAssignableFrom(Toggle.class) || theEvent.isAssignableFrom(Button.class)) {
    String parameter = theEvent.getName();
    String value = "";
    if (theEvent.isAssignableFrom(Textfield.class))
      value = theEvent.getStringValue();
    else if (theEvent.isAssignableFrom(Toggle.class) || theEvent.isAssignableFrom(Button.class))
      value = theEvent.getValue()+"";

    if(parameter == "vGyrox"){
      vGyro[0]=int(value);
    } else if(parameter == "vGyroy"){
      vGyro[1]=int(value);
    } else if(parameter == "vGyroz"){
      vGyro[2]=int(value);
    } else if(parameter == "vAccx"){
      vAcc[0]=int(value);
    } else if(parameter == "vAccy"){
      vAcc[1]=int(value);
    } else if(parameter == "vAccz"){
      vAcc[2]=int(value);
    } else if(parameter == "vMagx"){
      vMag[0]=int(value);
    } else if(parameter == "vMagy"){
      vMag[1]=int(value);
    } else if(parameter == "vMagz"){
      vMag[2]=int(value);
    }
    
    if(parameter == "GyroMax"){
      GyroGraphMax=int(value);
    } else if(parameter == "GyroMin"){
      GyroGraphMin=int(value);
    } else if(parameter == "AccMax"){
      AccGraphMax=int(value);
    } else if(parameter == "AccMin"){
      AccGraphMin=int(value);
    } else if(parameter == "MagMax"){
      MagGraphMax=int(value);
    } else if(parameter == "MagMin"){
      MagGraphMin=int(value);
    }
    
    
    if(parameter == "sGyrox"){
      sGyro[0]=float(value);
    } else if(parameter == "sGyroy"){
      sGyro[1]=float(value);
    } else if(parameter == "sGyroz"){
      sGyro[2]=float(value);
    } else if(parameter == "sAccx"){
      sAcc[0]=float(value);
    } else if(parameter == "sAccy"){
      sAcc[1]=float(value);
    } else if(parameter == "sAccz"){
      sAcc[2]=float(value);
    } else if(parameter == "sMagx"){
      sMag[0]=float(value);
    } else if(parameter == "sMagy"){
      sMag[1]=float(value);
    } else if(parameter == "sMagz"){
      sMag[2]=float(value);
    }
    
  }
  setChartSettings();
}

void drawPropShield()
{
  // 3D art by Benjamin Rheinland
  stroke(0); // black outline
  fill(0, 128, 0); // fill color PCB green
  box(190, 6, 70); // PCB base shape

  fill(255, 215, 0); // gold color
  noStroke();

  //draw 14 contacts on Y- side
  translate(65, 0, 30);
  for (int i=0; i<14; i++) {
    sphere(4.5); // draw gold contacts
    translate(-10, 0, 0); // set new position
  }

  //draw 14 contacts on Y+ side
  translate(10, 0, -60);
  for (int i=0; i<14; i++) {
    sphere(4.5); // draw gold contacts
    translate(10, 0, 0); // set position
  }

  //draw 5 contacts on X+ side (DAC, 3v3, gnd)
  translate(-10,0,10);
  for (int i=0; i<5; i++) {
    sphere(4.5);
    translate(0,0,10);
  }

  //draw 4 contacts on X+ side (G C D 5)
  translate(25,0,-15);
  for (int i=0; i<4; i++) {
    sphere(4.5);
    translate(0,0,-10);
  }

  //draw 4 contacts on X- side (5V - + GND)
  translate(-180,0,10);
  for (int i=0; i<4; i++) {
    sphere(4.5);
    translate(0,0,10);
  }

  //draw audio amp IC
  stroke(128);
  fill(24);    //Epoxy color
  translate(30,-6,-25);
  box(13,6,13);

  //draw pressure sensor IC
  stroke(64);
  translate(32,0,0);
  fill(192);
  box(10,6,18);

  //draw gyroscope IC
  stroke(128);
  translate(27,0,0);
  fill(24);
  box(16,6,16);

  //draw flash memory IC
  translate(40,0,-15);
  box(20,6,20);

  //draw accelerometer/magnetometer IC
  translate(-5,0,25);
  box(12,6,12);

  //draw 5V level shifter ICs
  translate(42.5,2,0);
  box(6,4,8);
  translate(0,0,-20);
  box(6,4,8);
}
