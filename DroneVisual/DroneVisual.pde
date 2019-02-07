// import libraries
SimbleeDaten simblee;
// Serial port to connect to
String serialPortName = "/dev/ttyUSB0";

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
// Data from Socket 
BufferedReader in;
Sensor gyro,acc,mag,att,tof;
float[] gyroRaw= {0,0,0};
float[] accRaw = {0,0,0};
float[] magRaw = {0,0,0};
float[] attRaw = {0,0,0};
float[] tofRaw = {0,0,0};
PFont f;

public void settings() {
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
  tof = new Sensor((2*width/3)-80,height/2,(width/3)-90,height/3-40,-360,360,"Tof",graphColors,xLabel);

  simblee = new SimbleeDaten();
  surface.setTitle("BMF055");
}

void draw() {
  background(255);
  gyro.draw();
  acc.draw();
  mag.draw();
  att.draw();
  tof.draw();
  drawOrientation();
}


void drawOrientation(){
    lights();
    translate(width/2, height/2); // set position to centre
    pushMatrix(); // begin object
  
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
    drawPropShield();
    popMatrix(); // end of object
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
