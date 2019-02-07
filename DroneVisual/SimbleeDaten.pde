import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;

class SimbleeDaten extends PApplet {
  private ServerSocket server;
  SimbleeDaten() {
    super();
    PApplet.runSketch(new String[] {this.getClass().getSimpleName()}, this);
  }

  void settings() {
  
  }

  void setup() {
    try{
      this.server = new ServerSocket(65432, 1, InetAddress.getByName("127.0.0.1"));
      System.out.println("\r\nRunning Server:");
      this.listen();
    }catch( Exception e){
      println("Server geht ned");
    }
  }

  void draw() {
   this.processTCP();
  }
  
  private void listen() throws Exception {
    Socket client = this.server.accept();
    String clientAddress = client.getInetAddress().getHostAddress();
    System.out.println("\r\nNew connection from " + clientAddress);
    
    in = new BufferedReader(new InputStreamReader(client.getInputStream(),"UTF-8"));   
  }
  
  void processTCP(){
    try
    {
      String message = null;
      if ((message = in.readLine()) != null) {
        String[] list = split(trim(message), " ");
        if (list.length >= 4 && list[0].equals("Att")) {
            attRaw[2] = float(list[1])/100;
            attRaw[1] = float(list[2])/100; 
            attRaw[0] = float(list[3])/100;
            att.updateDiagramm(attRaw);
            tofRaw[2] = float(list[1])/100;
            tofRaw[1] = float(list[2])/100; 
            tofRaw[0] = float(list[3])/100;
            tof.updateDiagramm(tofRaw);
         } else if (list.length >= 4 && list[0].equals("Gyro")) {
            gyroRaw[0] = float(list[1])/1000;
            gyroRaw[1] = float(list[2])/1000; 
            gyroRaw[2] = float(list[3])/1000;
            gyro.updateDiagramm(gyroRaw);
         } else if (list.length >= 4 && list[0].equals("Acc")) {
            accRaw[0] = float(list[1])/1024;
            accRaw[1] = float(list[2])/1024; 
            accRaw[2] = float(list[3])/1024;
            acc.updateDiagramm(accRaw);
         } else if (list.length >= 4 && list[0].equals("Mag")) {
            magRaw[0] = float(list[1])/16;
            magRaw[1] = float(list[2])/16; 
            magRaw[2] = float(list[3])/16;
            mag.updateDiagramm(magRaw);
         }
      }
    }
    catch (Exception e){}
  }

}
