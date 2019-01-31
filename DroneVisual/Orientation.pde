class Orientation extends PApplet {
  Orientation() {
    super();
    PApplet.runSketch(new String[] {this.getClass().getSimpleName()}, this);
  }

  void settings() {
    size(600, 500, P3D);
  }

  void setup() {
    textSize(16); // set text size
    textMode(SHAPE); // set text mode to shape
    f = createFont("Arial",12,true);
    textFont(f,12);
  }

  void draw() {
    background(255); // set background to white
    lights();
  
    translate(width/2, height/2); // set position to centre
  
    pushMatrix(); // begin object
  
    float c1 = cos(radians(roll));
    float s1 = sin(radians(roll));
    float c2 = cos(radians(-pitch));
    float s2 = sin(radians(-pitch));
    float c3 = cos(radians(360-yaw-90));
    float s3 = sin(radians(360-yaw-90));
    applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
                 -s2, c1*c2, c2*s1, 0,
                 c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
                 0, 0, 0, 1);
  
    drawPropShield();
    
    popMatrix(); // end of object
    text("roll: "+roll,-260,-200);   
    text("pitch:"+pitch,-260,-180);   
    text("yaw:  "+yaw,-260,-160);  
    
    text("ACC",-260,140);   
    text("x: "+acc[0],-260,160);   
    text("y: "+acc[1],-260,180);   
    text("z: "+acc[2],-260,200);  
    text("Gyro",220,-220);   
    text("x: "+gyro[0],220,-200);   
    text("y: "+gyro[1],220,-180);   
    text("z: "+gyro[2],220,-160);  
    text("Mag",220,140);   
    text("x: "+nf(mag[0]/16,2,3),220,160);   
    text("y: "+nf(mag[1]/16,2,3),220,180);   
    text("z: "+nf(mag[2]/16,2,3),220,200);  
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

}
