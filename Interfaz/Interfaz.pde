import processing.serial.*;
Serial myPort;
import controlP5.*;

boolean newData = false;
int xPos1 = 1;         // horizontal position of the graph 
int xPos2 = 1; 
int xPos3 = 1; 

int xPos4 = 1;         // horizontal position of the graph 
int xPos5 = 1; 
int xPos6 = 1; 
//Variales to draw a continuous line.
int lastxPos1=1;
float lastheight1=0;
int lastxPos2=1;
float lastheight2=0;
int lastxPos3=1;
float lastheight3=0;

int lastxPos4=1;
float lastheight4=0;
int lastxPos5=1;
float lastheight5=0;
int lastxPos6=1;
float lastheight6=0;

float offset = 0;

ControlP5 jControl;
ControlP5 jControl1;
ControlP5 jControl2;
ControlP5 jControl3;
float setPoint, KP, KD, KI,  set, Control, ajuste, sK, PID_p, PID_i, PID_d, prev_sp, prev_kp, prev_kd, prev_ki;
float velocidad, temp, sP;
float distancia = 0;
//*******************************
String val;
boolean firstContact = false;
float[] vals = new float[7]; //La informacion del puerto serial
//*******************************



void setup () {
  // set the window size:
  size(1300, 700);
  
//*******************************
  myPort = new Serial(this, "COM8", 115200);
  myPort.bufferUntil('%');
//*******************************
  jControl = new ControlP5(this);
  Slider s = jControl.addSlider("setPoint", 0,20, 0, 10,40, 400, 20);
  jControl1 = new ControlP5(this);
  Slider skp = jControl1.addSlider("KP",0, 50, 0, 500, 40, 200, 20);
  jControl2 = new ControlP5(this);
  Slider ski = jControl2.addSlider("KI",0, 1, 0, 750, 40, 200, 20);
  jControl3 = new ControlP5(this);
  Slider skd = jControl3.addSlider("KD",0, 500, 0, 1000, 40, 200, 20);
   background(255);      // set inital background:
   for(int i = 5;i>=0;i--){
    fill(100);
    text(i*5, 5,(height-((i*10)*7))-3);
    stroke(100,100,100);     //stroke color
    strokeWeight(1);        //stroke wider
    line(0, height-((i*10)*7), 1300, height -((i*10)*7)); 
   }
   for(int i = 15;i>=0;i--){
    fill(100);
    text((i-5)*10, 5,(250-(i*10)+0));
    stroke(100,100,100);     //stroke color
    strokeWeight(1);        //stroke wider
    line(0, 250-(i*10), 1300, 250 -(i*10)); 
   }
}
void draw () {
  if((set != setPoint) || (sP != KP)){
  set = setPoint;
  sK = KP;
  }
    strokeWeight(3);        //stroke wider
    temp = vals[0]-offset;
    stroke(127,34,255);     //stroke color
    fill(153);
    rect(10,5,130,30);
    fill(0);
    text("Velocidad: ", 20,23);
    text(velocidad, 90,23);
    
    stroke(255,0,0);     //stroke color
    fill(153);
    rect(170,5,130,30);
    fill(0);
    text("Potencia: ", 180,23);
    text(vals[2], 240,23); 
    
    stroke(255,255,255);     //stroke color
    fill(153);
    rect(320,5,140,30);
    fill(0);
    text("Distancia: ", 330,23);
    text(distancia, 395,23); 
    
    int yup = 25;
    int xkp = 525;
    int xkd = 775;
    int xki = 1005;
    
    stroke(16,110,44);     //stroke color
    fill(153);
    rect(xkp,yup-20,150,30);
    fill(0);
    text("KP: ", xkp+10,yup);
    text(KP, xkp+30,yup); 
    text("P: ", xkp+80,yup);
    text(PID_p/10, xkp+90,yup); 
    
    stroke(0,50,255);     //stroke color
    fill(153);
    rect(xkd,yup-20,150,30);
    fill(0);
    text("KI: ", xkd+10,yup);
    text(KI, xkd+30,yup); 
    text("I: ", xkd+80,yup);
    text(PID_i/10, xkd+90,yup); 
    
    stroke(255,100,3);     //stroke color
    fill(153);
    rect(xki,yup-20,190,30);
    fill(0);
    text("KD: ", xki+10,yup);
    text(KD, xki+30,yup); 
    text("D: ", xki+100,yup);
    text(PID_d/10, xki+120,yup); 
  if (newData) {
    //Drawing a line from Last inByte to the new one.
    stroke(127,34,255);     //stroke color
    strokeWeight(3);        //stroke wider
    line(lastxPos1, lastheight1, xPos1, height - velocidad*14); 
    lastxPos1= xPos1;
    lastheight1= int(height-velocidad*14);
    stroke(0,0,255);     //stroke color
    strokeWeight(4);        //stroke wider
    line(lastxPos2, lastheight2, xPos2, height - sP*14); 
    lastxPos2= xPos2;
    lastheight2= int(height-sP*14);
    
     stroke(255,0,0);     //stroke color
    strokeWeight(2);        //stroke wider
    line(lastxPos3, lastheight3, xPos3, (height-500) - vals[2]); 
    lastxPos3= xPos3;
    lastheight3= int((height-500)-vals[2]);
    
    
    stroke(16,110,44);     //stroke color
    strokeWeight(1);        //stroke wider
    line(lastxPos4, lastheight4, xPos4, (height-500) - PID_p/10); 
    lastxPos4= xPos4;
    lastheight4= int((height-500)-PID_p/10);
    
    
     stroke(0,50,255);     //stroke color
    strokeWeight(1);        //stroke wider
    line(lastxPos5, lastheight5, xPos5, (height-500) - PID_i/10); 
    lastxPos5= xPos5;
    lastheight5= int((height-500)-PID_i/10);
    
    
     stroke(255,100,3);     //stroke color
    strokeWeight(1);        //stroke wider
    line(lastxPos6, lastheight6, xPos6, (height-500) - PID_d/10); 
    lastxPos6= xPos6;
    lastheight6= int((height-500)-PID_d/10);
    
    // at the edge of the window, go back to the beginning:
    if (xPos1 >= width) {
      xPos1 = 0;
      lastxPos1= 0;
      xPos2 = 0;
      lastxPos2= 0;
      xPos3 = 0;
      lastxPos3= 0;
      
      xPos4 = 0;
      lastxPos4= 0;
      xPos5 = 0;
      lastxPos5= 0;
      xPos6 = 0;
      lastxPos6= 0;
      background(255);  //Clear the screen.
      for(int i = 5;i>=0;i--){
        fill(100);
        text(i*5, 5,(height-((i*10)*7))-3);
        stroke(100,100,100);     //stroke color
        strokeWeight(1);        //stroke wider
        line(0, height-((i*10)*7), 1300, height -((i*10)*7)); 
       }
       for(int i = 15;i>=0;i--){
        fill(100);
        text((i-5)*10, 5,(250-(i*10)-0));
        stroke(100,100,100);     //stroke color
        strokeWeight(1);        //stroke wider
        line(0, 250-(i*10), 1300, 250 -(i*10)); 
       }
      fill(153);
    rect(10,5,110,30);
    fill(0);
    text("Velocidad: ", 20,20);
    text(velocidad, 80,20); 
    } 
    else {
      // increment the horizontal position:
      xPos1++;
      xPos2++;
      xPos3++;
      xPos4++;
      xPos5++;
      xPos6++;
    }
   newData =false;
 }
}

void serialEvent(Serial myPort) {
  val = myPort.readStringUntil('%');
  print("ESTO VIENE ---------");
  println(val);
  if (val != null) {
    if (firstContact == false) {
      String[] realval = split(val, '$');
      val = realval[1];
      print("EL VALOR DE HANDSHAKE---------");
      print(firstContact);
      println(val);
      if (val.equals("A")) {
      myPort.clear();
      firstContact = true;
      myPort.write("A");
      println("contact");
    }
  }else {
    String[] nums = split(val, '$');
    if(nums.length >= 2){
      vals[0] = float(nums[1]);
      vals[1] = float(nums[2]);
      vals[2] = float(nums[3]);
      vals[3] = float(nums[4]);
      vals[4] = float(nums[5]);
      vals[5] = float(nums[6]);
      vals[6] = float(nums[7]);
    }
    print(vals[0]);
    print(">");
    print(vals[1]);
    print(">");
    print(vals[2]);
    print(">");
    println(val);
    if(prev_sp != setPoint || prev_kp != KP || prev_kd != KD || prev_ki != KI){
      myPort.write("$"+(setPoint+offset)+"$"+KP+"$"+KD+"$"+KI+"$%");
    }
    prev_sp = setPoint;
    prev_kp = KP;
    prev_kd = KD;
    prev_ki = KI;
    //
    }
    newData = true;
    velocidad = vals[0]-offset;
    sP = vals[1]-offset;
    PID_p = vals[3];
    PID_d = vals[4];
    PID_i = vals[5];
    distancia = vals[6];
  }
}
