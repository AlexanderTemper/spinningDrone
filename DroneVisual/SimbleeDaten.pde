import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.awt.Frame;

class SimbleeDaten extends PApplet {
  private ServerSocket server;
  private boolean closed = true;
  SimbleeDaten() {
    super();
    PApplet.runSketch(new String[] {this.getClass().getSimpleName()}, this);
  }
  
  void setup() {
    try{
      this.server = new ServerSocket(65432, 1, InetAddress.getByName("127.0.0.1"));
      System.out.println("\rServer is running");
    }catch( Exception e){
      println("server creation error");
    }
    this.surface.setVisible(false);
  }
  
  void draw() {
    if(closed){
      try{
        this.listen();
        closed = false;
      }catch( Exception e){
        println("client connection error");
      }
    }
    this.processTCP();
  }
  
  private void listen() throws Exception {
    Socket client = this.server.accept();
    String clientAddress = client.getInetAddress().getHostAddress();
    System.out.println("\r\nNew connection from " + clientAddress);
    
    in = new BufferedReader(new InputStreamReader(client.getInputStream(),"UTF-8"));   
  }
  
  private void processTCP(){
    try
    {
      String message = null;
      if ((message = in.readLine()) != null) {
        String[] list = split(trim(message), " ");
        if (list.length >= 4 && list[0].equals("Att")) {
            attRaw[2] = float(list[1])/10;
            attRaw[1] = float(list[2])/10; 
            attRaw[0] = float(list[3])/10;
            att.updateDiagramm(attRaw);
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
         }else if (list.length >= 4 && list[0].equals("Tof")) {
            tofRaw[0] = float(list[1]);
            tofRaw[1] = float(list[2]); 
            tofRaw[2] = float(list[3])/1000;//clock in ms
            tof.updateDiagramm(tofRaw);
         } else if(list[0].equals("end")){
           println("Client closed connection");
           closed=true;
         } else if(list[0].equals("DebugD")){
           println(String.join(" ", list));
         }
      }
    }
    catch (Exception e){}
  }
}
