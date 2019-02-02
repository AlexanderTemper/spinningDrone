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
            att[2] = float(list[1])/100;
            att[1] = float(list[2])/100; 
            att[0] = float(list[3])/100;
            updateAttGraph();
         } else if (list.length >= 4 && list[0].equals("Gyro")) {
            gyro[0] = float(list[1])/1000;
            gyro[1] = float(list[2])/1000; 
            gyro[2] = float(list[3])/1000;
            updateGyroGraph();
         } else if (list.length >= 4 && list[0].equals("Acc")) {
            acc[0] = float(list[1])/1024;
            acc[1] = float(list[2])/1024; 
            acc[2] = float(list[3])/1024;
            updateAccGraph();
         } else if (list.length >= 4 && list[0].equals("Mag")) {
            mag[0] = float(list[1])/16;
            mag[1] = float(list[2])/16; 
            mag[2] = float(list[3])/16;
            updateMagGraph();
         }
      }
    }
    catch (Exception e){}
  }

}
