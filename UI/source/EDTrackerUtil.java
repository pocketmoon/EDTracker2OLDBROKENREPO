import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import toxi.geom.*; 
import toxi.geom.mesh.*; 
import toxi.geom.*; 
import toxi.processing.*; 
import processing.serial.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class EDTrackerUtil extends PApplet {

// 28/05/2014 Scale head movement to mimic in-game scaling
//            Add toggle for this behaviour





 



/*
Commands
 M send toggle monitor
 C Full Calibrate
 I Request Info
 D Debug Info
 */


TriangleMesh mesh;
ToxiclibsSupport gfx;

float rad2deg = 57.29578f;
float rad2FSR = 10430.06f;

float maxGX, maxGY, maxGZ;
float maxAX, maxAY, maxAZ;

String info = "Unknown Device";

String []messages = {
  "", "", "", "", "", "", "", "", "", ""
};


float headScale = 1.0f;
Serial  arduinoPort; // Usually the last port 
int     portNumber = 2;
String  portName ;
String  buffer;      //String for testing serial communication

float DMPRoll, DMPPitch, DMPYaw;

int  rawGyroX, rawGyroY, rawGyroZ;
int  rawAccelX, rawAccelY, rawAccelZ;

float driftScale = 10.0f;
float yawDrift =0.0f;
float pitchDrift =0.0f;
boolean monitoring = false;

float  yawDriftComp=0.0f;

boolean driftComp = false;



float[] yawHist = new float [400];
float[] pitchHist = new float [400];
float[] yawDriftHist = new float [400];

long lastPress=0;
PFont mono;

iLowPass lpX, lpY, lpZ;


public void setup() {
  size(800, 600, P3D);
  // The font "AndaleMono-48.vlw"" must be located in the 
  // current sketch's "data" directory to load successfully

  //mono =  createFont("Courier New", 64, false);
  //mono = loadFont("Eureka-48.vlw");
  mono = loadFont("CourierNewPSMT-24.vlw");

  background(0);
  textFont(mono);
  
  maxGX= maxGY= maxGZ =0.0f;
  maxAX= maxAY= maxAZ =0.0f;

  frameRate(60);
  mesh=(TriangleMesh)new STLReader().loadBinary(sketchPath("head3.stl"), STLReader.TRIANGLEMESH);
  //mesh=(TriangleMesh)new STLReader().loadBinary(sketchPath("mesh-flipped.stl"),STLReader.TRIANGLEMESH).flipYAxis();

  mesh.computeFaceNormals();
  mesh.computeVertexNormals();
  Vec3D forward = new Vec3D (0.0f, 1.0f, 0.0f); 
  mesh.scale(2.0f);
  mesh.pointTowards(forward);
  mesh.rotateZ(3.1415926f);
  gfx=new ToxiclibsSupport(this);


  int lastPort = Serial.list().length -1;
  println(Serial.list());


  while (lastPort >=0)
  {
    portName = Serial.list()[lastPort];
    println(" Connecting to -> " + portName);

    try {
      arduinoPort = new Serial(this, portName, 115200);
      arduinoPort.clear();
      arduinoPort.bufferUntil(10);  
      lastPort =0;
      arduinoPort.write('U');
    } 
    catch (Exception e) {
      println("Unable to connect to port.");
      println(e);
    }

    lastPort--;
  }

  delay(1000);
  arduinoPort.write('V');

  ellipseMode(CENTER);

  lpX = new iLowPass(100);  //The argument is the FIFO queue length
  lpY = new iLowPass(100);  //The argument is the FIFO queue length
  lpZ = new iLowPass(100);  //The argument is the FIFO queue length
}


public void serialEvent(Serial p) {

  String dataIn = (arduinoPort.readString());
  char c = dataIn.charAt(0);

  //println(dataIn);
  monitoring = true;// if we're getting data

  try 
  {
    if (c == 'S')
    {
      monitoring = false;
    }
    else if (c == 'V')
    {
      monitoring = true;
    }
    else
    {
      String data[] = split(dataIn, '\t');

      if (data[0].equals ("M"))
      {
        for (int i = 0;i<9;i++)
          messages[i] = messages [i+1];
        messages[9] = data[1];
      }
      else  if (data[0].equals("I"))
      {
       // println("Info");
        info =  new String(data[1].substring(0, data[1].length()-2));
      }
      else  if (data[0].equals("D"))
      {
        //println("Info");
        yawDrift = PApplet.parseFloat(data[1]);
        yawDriftComp = PApplet.parseFloat(data[2]);
        for (int i=0; i<399;i++)
        {
          yawDriftHist[i] = yawDriftHist[i+1];
        }

        //yawDriftHist[399]=(yawDrift/(abs(yawDrift)+0.000001))*(sqrt(abs(yawDrift)))*10.0;
        yawDriftHist[399]=yawDrift;
        
        //println("A " + maxAX + " " + maxAY + " " + maxAZ + "    G " + maxGX + " " + maxGY + " " + maxGZ); 
        
      }
      else  if (data[0].equals("R"))
      {
        println("Recentered");
        yawDrift=0.0f;
        pitchDrift=0.0f;
      }
      else if (data[0].equals("S"))
      {
        println("Silent");
        monitoring = false;
      }
      else if (data[0].equals("V"))
      {
        println("Verbose");
        monitoring = true;
      }
      else {
        //println("YPR");
        DMPYaw = PApplet.parseFloat(data[0])/10430.06f;
        DMPPitch = PApplet.parseFloat(data[1])/10430.06f;
        DMPRoll  = -PApplet.parseFloat(data[2])/10430.06f;

        //        rawAccelX = int(data[3]);
        //        rawAccelY = int(data[4]);
        //        rawAccelZ = int(data[5]);
        lpX.input (PApplet.parseInt(data[3]));
        lpY.input (PApplet.parseInt(data[4]));
        lpZ.input (PApplet.parseInt(data[5])-16383);

        rawAccelX = lpX.output;
        rawAccelY = lpY.output;
        rawAccelZ = lpZ.output;

        rawGyroX = PApplet.parseInt(data[6]);
        rawGyroY = PApplet.parseInt(data[7]);
        rawGyroZ = PApplet.parseInt(data[8]);
        
        if (abs(rawAccelX) > maxAX)    maxAX = abs(rawAccelX) ;
        if (abs(rawAccelY) > maxAY)    maxAY = abs(rawAccelY) ;
        if (abs(rawAccelZ) > maxAZ)    maxAZ = abs(rawAccelZ) ;
        
        if (abs(rawGyroX) > maxGX)    maxGX = abs(rawGyroX) ;
        if (abs(rawGyroY) > maxGY)    maxGY = abs(rawGyroY) ;
        if (abs(rawGyroZ) > maxGZ)    maxGZ = abs(rawGyroZ) ;

        //println("A "+maxAX+" "+maxAY+" "+maxAZ+"   G "+maxGX+" "+maxGY+" "+maxGZ); 

          
      }
    }
  } 
  catch (Exception e) {
//    println("Caught Exception");
//    println(dataIn);
  //  println(e);
  }
}

int debounce=0;

public void draw() {

  long now = millis();

  if (debounce>0)
    debounce--;

  if (keyPressed && debounce <=0) 
  {
    debounce=20;

    lastPress = now + 800;

    //    if (key == 'r')  // reset Arduino!
    //    {
    //      arduinoPort.clear();
    //      arduinoPort.stop();
    //      delay(100);
    //      arduinoPort = new Serial(this, portName, 1200);
    //      arduinoPort.stop();
    //      
    //      arduinoPort = new Serial(this, portName,115200);
    //      arduinoPort.clear();
    //      delay(100);
    //
    //    }

    if (key == 'i' || key == 'I')  // into
    {
      //yawOffset = -DMPYaw;
      //pitchOffset = -DMPPitch;
      arduinoPort.write('I');
    }

    if (key == '1') 
    {
      if (monitoring)
        arduinoPort.write('S');
      else
      {
        arduinoPort.write('V');
        arduinoPort.write('I');
      }
    }

    if (monitoring)
    {
      if (key == '2') 
      {
        arduinoPort.write('I');
      }

      if (key == '3') 
      {
        arduinoPort.write('R');
      }

      if (key=='5')
      {
        arduinoPort.write('P');
      }

      if (key=='6')
      {
        if (headScale>0)  headScale =0.0f;
        else headScale = 1.0f;
      }

      if (key=='7')
      {
        driftScale *= 10.0f;
        if (driftScale > 1001.0f)
          driftScale = 1.0f;
      }

      if (key=='8')
      {
        arduinoPort.write('D');
      }

      if (key=='9')
      {
        arduinoPort.write('B');
      }
    }

    //      
    if (key=='q' || key =='Q')
    {
      arduinoPort.write('S');
        
      delay(200);  
      arduinoPort.clear();
      arduinoPort.stop();
      exit();
    }
  }

  background(30);
  fill(0, 0, 0);
  rect(0, 500, 800, 600);


  fill(249, 250, 150);
  textSize(26); 
  text("ED Tracker Configuration and Calibration Utility", 10, 30);


  if (monitoring)
  {
    text(info + " - Monitoring", 10, 60);
  }
  else
  {
    fill(250, 50, 50);
    text(info +  " - Not Monitoring", 10, 60);
  }

  fill(249, 250, 150);
  textSize(18); 
  text("1 Toggle Monitoring", 10, 100);
  text("2 Get Info", 10, 120);


  if (!info.equals("Unknown Device"))
  {
    if (info.indexOf("Calib")<0)
    {
      text("3 Reset View/Drift Tracking", 10, 140);
      text("4 ", 10, 160);
      text("5 Rotate Mounting Axis", 10, 180);
        text("6 Toggle Scaling", 10, 200);

      text("7 Adjust Drift Graph Scale", 10, 220);
      text("8 Save Drift Compensation", 10, 240);
    }
    else
    {
      text("9 Recalc Bias Values", 10, 280);
    }
  }
  text("Q Quit", 10, 300);

  fill(255, 200, 150);


if (info.indexOf("Calib")<0)
    {
  text("DMP Yaw", (int)width-240, 120);
  text (nfp(DMPYaw*rad2deg, 0, 2), (int)width-100, 120);
  text("DMP Pitch", (int)width-240, 100); 
  text (nfp( DMPPitch*rad2deg, 0, 2), (int)width-100, 100);
  text("DMP Roll", (int)width-240, 140);
  text (nfp(DMPRoll*rad2deg, 0, 2), (int)width-100, 140);
    }
    else
    {
  text("Raw X Accel", (int)width-240, 100);
  text (rawAccelX, (int)width-100, 100); 
  
    text("Raw Y Accel", (int)width-240, 120);
  text (rawAccelY, (int)width-100, 120);
  
 text("Raw Z Accel", (int)width-240, 140);
  text (rawAccelZ, (int)width-100, 140);  
    }

  

  //text("Yaw Offset", (int)width-240, 440);
  //text (nfp(yawOffset*rad2deg, 0, 2), (int)width-100, 440);
  //text("Pitch Offset", (int)width-240, 460);
  //text (nfp(pitchOffset*rad2deg, 0, 2), (int)width-100, 460);


  text("Yaw Drift", (int)width-240, 400);
  text (nfp(yawDrift, 1, 2), (int)width-100, 400);

  text("Drift Comp", (int)width-240, 420); 
  text (nfp(yawDriftComp, 0, 2), width -100, 420);

  text("x" + driftScale, 10, 520);

  //draw the message
  stroke(255, 255, 255);
  fill(255, 255, 255);

  textSize(14); 
  for (int i=0;i<10; i++)
    text (messages[i], 10, 360+i*14);

  // text(gyrStr, (int) (width/6.0) - 40, 50);

  stroke(255, 255, 255);
  line(0, 550, 799, 550);

  stroke(10, 255, 10);
  for (int i=0; i<399;i++)
  {
    line(i*2, 550-yawHist[i], (i*2)+1, 550-yawHist[i+1]); 
    yawHist[i] = yawHist[i+1];
  }
  yawHist[399]=DMPYaw *rad2deg*0.55f;


  stroke(250, 10, 10);
  for (int i=0; i<399;i++)
  {
    line(i*2, 550-pitchHist[i], (i*2)+1, 550-pitchHist[i+1]); 
    pitchHist[i] = pitchHist[i+1];
  }
  pitchHist[399]=(DMPPitch)*rad2deg*0.55f;    

  stroke(255, 255, 0);
  for (int i=0; i<399;i++)
  {
    line(i*2, 550.0f-constrain(yawDriftHist[i]*driftScale, -45, 45), (i*2)+1, 
    550.0f-constrain(yawDriftHist[i+1]*driftScale, -45, 45));
  }

  // sprit level
  fill(0, 0, 0);
  stroke(255, 255, 255);

  ellipse(670, 280, 180, 180);
  ellipse(670, 280, 45, 45);
  ellipse(670, 280, 90, 90);
  ellipse(670, 280, 45, 45);

  line(580, 280, 760, 280);
  line(670, 190, 670, 370);

  fill(255, 255, 0);
  stroke(255, 40, 40);

  ellipse(670 + constrain (rawAccelX/10, -90, 90), 280 -constrain(rawAccelY/20, -90, 90), 5, 5);
  //  ellipse(670 , 280 , 10, 10);
  
   fill(0, 255, 255);
  stroke(25, 255, 40);
  ellipse(670 + constrain (rawGyroX, -90, 90), 280 -constrain(rawGyroY/2, -90, 90), 5, 5);



  fill(255, 255, 255);
  ambientLight(80, 80, 100);
  pointLight(255, 255, 255, 1000, -2000, 1000 );
  translate(width/2, height/2-20, 0);

  rotateY(DMPYaw * (1.0f + headScale*2.0f));// + yawOffset);
  rotateX(DMPPitch * (1.0f + headScale*2.0f));// + pitchOffset);

  gfx.origin(new Vec3D(), 200);
  noStroke();
  noSmooth();
  gfx.mesh(mesh, false);
}

class iLowPass {
    IntList  buffer;
    int len;
    int output;

    iLowPass(int len) {
        this.len = len;
        buffer = new IntList();
        for(int i = 0; i < len; i++) {
            buffer.append(0);
        }
    }

    public void input(int v) {
        buffer.append(v);
        buffer.remove(0);

        int  sum = 0;
        for(int i=0; i<buffer.size(); i++) {
                int fv = buffer.get(i);
                sum += fv;
        }
        output = sum / buffer.size();
    }
}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "EDTrackerUtil" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
