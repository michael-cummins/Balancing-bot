#include <Arduino_LSM6DS3.h>
#include <math.h>

//pid co-efficients
double kp = 700;
double ki = 300;
double kd = 300;

//variables used in computePID function
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input;
double setPoint;
double cumError, rateError;
int pid;

const int hard_limit = 255;

//motors
const int l_backward = 12;
const int l_forward = 11;
const int r_forward = 3;
const int r_backward = 2;

//variables used for accelerometer and angle of orientation
float aX,aY,aZ;
double alpha;

void setup() {
  //initialise imu and serial
  Serial.begin(9600);
  IMU.begin();
  Serial.println("IMU initialised");

  //initialise pins
  pinMode( r_forward, OUTPUT );
  pinMode( r_backward, OUTPUT );
  pinMode( l_backward, OUTPUT );
  pinMode( l_forward, OUTPUT );
  
  //make sure buggy is stationary when moved on
  stop_();
  
  //centre of gravity for the buggy
  setPoint = -0.05;
}

void loop() {

  //read in acceleration to find angle of orientation
  IMU.readAcceleration(aX,aY,aZ);
  alpha = atan(aZ/aY);

  //compute pid as an integer so we can use it as a speed
  pid = computePID(alpha);

  //movement algorithm
  //If pid is positive, we need to reverse to get back to centre of gravity 
  //Then if the pid is negatvie, we need to move forward
  if(pid < 0){
    //branchless technique; if pid is greater than 255, set it to 255
    pid = hard_limit*(pid <= -hard_limit) - pid*(pid > -hard_limit); 
    reverse(pid);
  }
  else if (pid >= 0){
    pid = hard_limit*(pid > hard_limit) + pid*(pid < hard_limit);
    move_forward(pid);
  }  
  
}

//pid function
int computePID(float input){
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);
  
  error = input - setPoint ;
  cumError += error*elapsedTime;
  rateError = (error - lastError)/elapsedTime;
    
  double out = kp*error + ki*cumError + kd*rateError;

  previousTime = currentTime;
  lastError = error;
  
  if(previousTime >= 100){
    cumError = 0;
  }
  
  //accept input as a floating point
  //round to integer at the end so it can be used as a speed
  return (int)(out);
}

//movement functions
void move_forward(int x){
  //analogwrite only accepts integers between 255 and 0
  if(x <= 255 && x >= 0){
    analogWrite(r_forward, x);
    analogWrite(r_backward, 0);  
    analogWrite(l_forward, x);
    analogWrite(l_backward, 0);  
  }
  else{
    stop_();
  }
}

void reverse(int x){
  //analogwrite only accepts integers between 255 and 0
  if(x <= 255 && x >= 0){
    analogWrite(r_forward, 0);
    analogWrite(r_backward, x);  
    analogWrite(l_forward, 0);
    analogWrite(l_backward, x);  
  }
  else{
    stop_();
  }
}

void stop_(){
  digitalWrite(r_forward, LOW);
  digitalWrite(r_backward, LOW);  
  digitalWrite(l_forward, LOW);
  digitalWrite(l_backward, LOW);
}
