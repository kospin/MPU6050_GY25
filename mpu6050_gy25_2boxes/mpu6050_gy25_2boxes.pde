
import processing.serial.*;

Serial myPort;        
float angleX = 0;
float angleY = 0; 
float angleZ = 0;
float pitch=0,roll=0,yaw=0;

PGraphics cubeA;
PGraphics cubeB;

void setup() {
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
  
  size(600, 400, P3D); 
  cubeA = createGraphics(300, 300, P3D);
  cubeB = createGraphics(300, 300, P3D);
} 

void draw() {
  background(#ffffff);
  readSerial();  
  
  drawCube(cubeA, angleX,angleY,angleZ);
  tint(0xff0000ff);
  image(cubeA, 0, 0);
 
  drawCube(cubeB, pitch,roll,yaw);
  tint(0xffff0000);
  image(cubeB, 300, 0);
}

void drawCube(PGraphics cube, float rx,float ry,float rz) {
  cube.beginDraw();
  cube.lights();
  cube.clear();
  cube.strokeWeight(10);
  //cube.noStroke();
  cube.translate(cube.width/2, cube.height/2 );
  cube.rotateZ(-rx/180*3.14);
  cube.rotateX(ry/180*3.14);
  cube.rotateY(rz/180*3.14);
  cube.box(150);
  cube.endDraw(); 
} 

boolean readSerial(){
  
  if (myPort.available() > 0) {
    String inString = myPort.readStringUntil('\n');
    if (inString != null) {
      inString = trim(inString);
      String[] angles = split(inString,',');
      if(angles.length == 6){
        angleX = float(angles[0]);
        angleY = float(angles[1]);
        angleZ = float(angles[2]);
        pitch = float(angles[3]);
        roll = float(angles[4]);
        yaw = float(angles[5]);
        //return true;
      }  
    }
  }
  return true;
}
