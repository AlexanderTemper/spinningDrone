// import libraries
import controlP5.*; // http://www.sojamo.de/libraries/controlP5/
import processing.serial.*;

Orientation orientation;
// Serial port to connect to
String serialPortName = "/dev/ttyUSB0";
Serial serialPort; // Serial port object

// Serial Protokoll Parse
int readtoBuffer = 0;
byte[] inBuffer = new byte[200];
int nextBuffer = 0;
int lastread = 0;


float[] acc = {0,0,0};
float[] gyro = {0,0,0};
float[] mag = {0,0,0};
float[] att = {0,0,0}; // {yaw,pitch,roll}

// interface stuff
ControlP5 cp5;

int GyroGraphX = 10;
int GyroGraphY = 80;
int GyroGraphMin = -10;
int GyroGraphMax = 10;
int vGyro[] = {1,1,1};
float sGyro[] = {1,1,1};
Graph GyroGraph = new Graph(GyroGraphX+220, GyroGraphY, 450, 120, color (20, 20, 200));

int AccGraphX = 10;
int AccGraphY = 350;
int AccGraphMin = -1;
int AccGraphMax = 1;
int vAcc[] = {1,1,1};
float sAcc[] = {1,1,1};
Graph AccGraph = new Graph(AccGraphX+220, AccGraphY, 450, 120, color (20, 20, 200));

int MagGraphX = 10;
int MagGraphY = 620;
int MagGraphMin = -50;
int MagGraphMax = 50;
int vMag[] = {1,1,1};
float sMag[] = {0.0625,0.0625,0.0625};
Graph MagGraph = new Graph(MagGraphX+220, MagGraphY, 450, 120, color (20, 20, 200));

int AttGraphX = 760;
int AttGraphY = 80;
int AttGraphMin = -180;
int AttGraphMax = 360;
int vAtt[] = {1,1,1};
float sAtt[] = {1,1,1};
Graph AttGraph = new Graph(AttGraphX+220, AttGraphY, 550, 600 ,color (20, 20, 200));

// plots
float[][] GyroGraphValues = new float[3][100];
float[][] AccGraphValues = new float[3][100];
float[][] MagGraphValues = new float[3][100];
float[][] AttGraphValues = new float[3][100];
float[] lineGraphSampleNumbers = new float[100];
color[] graphColors = new color[3];

PFont f;

public void settings() {
  size(1600, 860);
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

  serialPort = new Serial(this, serialPortName, 9600);


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
  
  x = AttGraphX;
  y = AttGraphY;
  
  cp5.addTextfield("AttMax").setPosition(x+200, y-40).setText(AttGraphMax+"").setWidth(40).setAutoClear(false);
  cp5.addTextfield("AttMin").setPosition(x+200, y+640).setText(AttGraphMin+"").setWidth(40).setAutoClear(false);
  cp5.addTextlabel("on/off/Att").setText("on/off").setPosition(AttGraphX +13, AttGraphY -20).setColor(0);
  cp5.addTextlabel("multipliers/Att").setText("multipliers").setPosition( AttGraphX + 55, AttGraphY-20).setColor(0);
  cp5.addTextfield("sAttx").setPosition(x=AttGraphX+50, y=AttGraphY).setText(sAtt[0]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("sAtty").setPosition(x, y=y+40).setText(sAtt[1]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("sAttz").setPosition(x, y=y+40).setText(sAtt[2]+"").setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addToggle("vAttx").setPosition(x=x-50, y=AttGraphY).setValue(vAtt[0]).setMode(ControlP5.SWITCH).setColorActive(graphColors[0]);
  cp5.addToggle("vAtty").setPosition(x, y=y+40).setValue(vAtt[1]).setMode(ControlP5.SWITCH).setColorActive(graphColors[1]);
  cp5.addToggle("vAttz").setPosition(x, y=y+40).setValue(vAtt[2]).setMode(ControlP5.SWITCH).setColorActive(graphColors[2]);
}

void updateGyroGraph(){
  for (int i=0; i<3; i++) {
    // update Gyro
    if (i<GyroGraphValues.length) {
      for (int k=0; k<GyroGraphValues[i].length-1; k++) {
        GyroGraphValues[i][k] = GyroGraphValues[i][k+1];
      }
      GyroGraphValues[i][GyroGraphValues[i].length-1] = gyro[i] * sGyro[i];// Muilltiply
    }      
  }
}
void updateAccGraph(){
  for (int i=0; i<3; i++) {
    // update Acc
    if (i<AccGraphValues.length) {
      for (int k=0; k<AccGraphValues[i].length-1; k++) {
        AccGraphValues[i][k] = AccGraphValues[i][k+1];
      }
      AccGraphValues[i][AccGraphValues[i].length-1] = acc[i]* sAcc[i];
    }
  }
}
void updateMagGraph(){
  for (int i=0; i<3; i++) {
    /// update Mag
    if (i<MagGraphValues.length) {
      for (int k=0; k<MagGraphValues[i].length-1; k++) {
        MagGraphValues[i][k] = MagGraphValues[i][k+1];
      }
      MagGraphValues[i][MagGraphValues[i].length-1] = mag[i] * sMag[i];
    }  
  }
}
void updateAttGraph(){
  for (int i=0; i<3; i++) {
    // update Att
    if (i<AttGraphValues.length) {
      for (int k=0; k<AttGraphValues[i].length-1; k++) {
        AttGraphValues[i][k] = AttGraphValues[i][k+1];
      }
      AttGraphValues[i][AttGraphValues[i].length-1] = att[i] * sAtt[i];
    }   
  }
}

void draw() {

  processSerial();
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
  
  // mag
  MagGraph.DrawAxis();
  for (int i=0;i<MagGraphValues.length; i++) {
    MagGraph.GraphColor = graphColors[i];
    if (vMag[i]==1)
      MagGraph.LineGraph(lineGraphSampleNumbers, MagGraphValues[i]);
  }
  // att
  AttGraph.DrawAxis();
  for (int i=0;i<AttGraphValues.length; i++) {
    AttGraph.GraphColor = graphColors[i];
    if (vAtt[i]==1)
      AttGraph.LineGraph(lineGraphSampleNumbers, AttGraphValues[i]);
  }
  
}

void processSerial(){
  int r = 0;
  while(serialPort.available()>0){
    r = serialPort.read(); 
   // byte[]  t = {(byte)r};
   // print(new String(t));
    if(lastread == 'A' && r == 'B'){ // next Frame Starts 
      if(nextBuffer == 13){
        int i=1;
        if(inBuffer[0] == 'O'){
          att[2] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++])/100;
          att[1] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++])/100;
          att[0] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++])/100;
          updateAttGraph();
        } else if(inBuffer[0] == 'G'){
          gyro[0]= getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++])/1000;
          gyro[1] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++])/1000;
          gyro[2] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++])/1000;
          updateGyroGraph();
        } else if(inBuffer[0] == 'R'){
          acc[0] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++])/1024;
          acc[1] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++])/1024;
          acc[2] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++])/1024;
          updateAccGraph();
        } else if(inBuffer[0] == 'M'){
          mag[0] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++]);
          mag[1] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++]);
          mag[2] = getfloat(inBuffer[i++],inBuffer[i++],inBuffer[i++],inBuffer[i++]);
          updateMagGraph();
        }
      }
      nextBuffer = 0;
    } else if(lastread == 'A'){ // false Alarm 
      inBuffer[nextBuffer++] = (byte)lastread;
      inBuffer[nextBuffer++] = (byte)r;
    }else if(r != 'A'){
      inBuffer[nextBuffer++] = (byte)r;
    }
    lastread = r;
  }
}

float getfloat(byte a,byte b,byte c,byte d){
  long value = a & 0xFF;
    value |= (b << 8) & 0xFFFF;
    value |= (c << 16) & 0xFFFFFF;
    value |= (d << 24) & 0xFFFFFFFF;
    return (float)value;
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
 
  AttGraph.xLabel=" Samples ";
  AttGraph.yLabel="Att";
  AttGraph.Title="";  
  AttGraph.xDiv=20;  
  AttGraph.xMax=0; 
  AttGraph.xMin=-100;  
  AttGraph.yMax=AttGraphMax; 
  AttGraph.yMin=AttGraphMin;
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
    } else if(parameter == "vAttx"){
      vAtt[0]=int(value);
    } else if(parameter == "vAtty"){
      vAtt[1]=int(value);
    } else if(parameter == "vAttz"){
      vAtt[2]=int(value);
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
    } else if(parameter == "AttMax"){
      AttGraphMax=int(value);
    } else if(parameter == "AttMin"){
      AttGraphMin=int(value);
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
    } else if(parameter == "sAttx"){
      sAtt[0]=float(value);
    } else if(parameter == "sAtty"){
      sAtt[1]=float(value);
    } else if(parameter == "sAttz"){
      sAtt[2]=float(value);
    }
    
  }
  setChartSettings();
}
