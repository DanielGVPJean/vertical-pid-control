#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

//*******************************
String val; // Data recibida desde el Puerto serial
int index_sep_1,index_sep_2,index_sep_3,index_sep_4,index_sep_5;
//*******************************

// Instantiate an object for the sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// the number of the LED pin
#define SERVO_PIN_1 14  // 14 corresponds to GPIO14
#define SERVO_PIN_2 15 // 15 corresponds to GPIO15

// setting PWM properties
#define HZ 50
#define PWM_CHANNEL 0
#define RESOLUTION 16
#define PULSE_MIN 1200
#define PULSE_MAX 1700

const int pulse_Min = map(PULSE_MIN,0,20000,0,65535);
const int pulse_Max = map(PULSE_MAX,0,20000,0,65535);

int multiplicador = 1;
int pulse = (pulse_Max)/2;

////////////////////////Variables///////////////////////
int Read = 0;
int direction_y = 1;
float distance = 0.0;
float velocity = 0.0;
float distanceSet = 0.0;
float rupture = 4200;
float min_val = pulse_Min;
float max_val = pulse_Max;
float lim_B = -1*(((rupture-min_val)/(max_val-min_val))*3800);
int lim_Bellow = 0;
float lim_A = (1-((rupture-min_val)/(max_val-min_val)))*3800;
int lim_Above = 1000;
float elapsedTime, timi, timePrev;        //Variables for time control
float distance_previous_error, distance_error, dist_diference;
int period = 20;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////

///////////////////PID constants///////////////////////
float kp=0; //Mine was 48
float ki=0; //Mine was 0.25
float kd=0; //Mine was 2100

//float kp=0; //Mine was 8
//float ki=0; //Mine was 0.2
//float kd=0; //Mine was 3100

float distance_setpoint = 8.9;
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////

int mapeo_de_potencia = 0; //Es la potencia que se envia a la interfaz

void setup(){
  // configure LED PWM functionalitites
  Serial.begin(115200);
  while (! Serial)
  {
  delay(1);
  }
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin())
  {
  Serial.println(F("Failed to boot VL53L0X"));
  while(1);
  }
  lox.setMeasurementTimingBudgetMicroSeconds(20000);
  
  Serial.println(pulse_Max);
  ledcSetup(PWM_CHANNEL, HZ, RESOLUTION);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(SERVO_PIN_1, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 3276);
  delay(2000);
  for(int i = 1; i <= 10; i++){
    distanceSet += get_dist(100);
  }
  distanceSet = distanceSet/10;
  establishContact();
  timi = millis();
}
 
void loop(){
  if(Serial.available() > 0) {
    ledcWrite(PWM_CHANNEL, pulse_Min);
    
    val = Serial.readStringUntil('%');
    index_sep_1 = val.indexOf("$");
    index_sep_2 = val.indexOf("$",(index_sep_1+1));
    index_sep_3 = val.indexOf("$",(index_sep_2+1));
    index_sep_4 = val.indexOf("$",(index_sep_3+1));
    index_sep_5 = val.indexOf("$",(index_sep_4+1));
    if(index_sep_1 == 0){
      distance_setpoint = (val.substring((index_sep_1+1), index_sep_2)).toFloat();
      kp = (val.substring((index_sep_2+1), index_sep_3)).toFloat();
      kd = (val.substring((index_sep_3+1), index_sep_4)).toFloat();
      ki = (val.substring((index_sep_4+1), index_sep_5)).toFloat();
      }
    //Reiniciar los valores del PID y poner las condiciones iniciales a 0
      PID_p = 0;
      PID_d = 0;
      PID_i = 0;
      distance_previous_error =  get_dist(100);
      distance_error = distance_previous_error;
      timi = millis();
   }
    if (millis() > timi+period)
    {
     //Obtener los valores de entrada para el PID
      timi = millis();
      distance = get_dist(100);
      distance_error = distance_setpoint - distance;  


     //Obtener los valores de "p", "i" y "d"
      PID_p = kp * distance_error;
      PID_p = constrain(PID_p,lim_Bellow,lim_Above);
      
      dist_diference = distance_error - distance_previous_error;     
      PID_d = kd*((dist_diference)/period);
        
      PID_i = PID_i + (ki * distance_error);
      PID_i = constrain(PID_i,-500,400);
    
      PID_total = PID_p + PID_i + PID_d;
      
      PID_total = constrain(PID_total,lim_Bellow,lim_Above);
      PID_total = map(PID_total, lim_Bellow, lim_Above, pulse_Min, pulse_Max);

     //---------PROTECCION* Si el sensor deja de mandar lecturas poner el motor en iddle, de lo contrario se manda la variable de control
      if(distance_error > 0 || distance_error < 0){
        ledcWrite(PWM_CHANNEL, PID_total);  
      }else{
        ledcWrite(PWM_CHANNEL, pulse_Min);  
      }

      mapeo_de_potencia = map(PID_total, pulse_Min, pulse_Max, 0, 100);
      
      Serial.print('$');
      Serial.print(distance);
      Serial.print('$');
      Serial.print(distance_setpoint);
      Serial.print('$');
      Serial.print(mapeo_de_potencia);
      Serial.print('$');
      Serial.print(PID_p);
      Serial.print('$');
      Serial.print(PID_d);
      Serial.print('$');
      Serial.print(PID_i);
      Serial.println("$%");
      
      distance_previous_error = distance_error;
    }
}

float get_dist(int n)
{
  float adc=0;
  // Declare variables for storing the sensor data
  VL53L0X_RangingMeasurementData_t value;
  // Get the sensor data
  lox.rangingTest(&value, false);
  // If we get a valid measurement, send it to the screen
  if(value.RangeStatus != 4) {
    adc = value.RangeMilliMeter;
  }else {
    adc = 500;
  }
  if(adc > 500){
    adc = 500;
  }
  float distance_cm = (0.1)*adc;
  return(distance_cm);
}

//*******************************
void establishContact(){
  while (Serial.available() <= 0) {
  Serial.println("$A$%");   // envía una A mayúscula
  delay(10);
  }
}
//*******************************
