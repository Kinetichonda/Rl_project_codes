#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

MPU6050 mpu;
#define LED_PIN 13


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2],startTime=0;
float Gyro_angle[2];
float Total_angle[2];
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float roll1=0,roll2=0,roll3=0,roll4=0,roll5=0,pitch1=0,pitch2=0,count=1;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight,pwLeft, pwRight, error;
float previous_error=0;
float pid_p=0;
float pid_i=0;
float pid_d=0;

int enA = 3;
int in1 = 4;
int in2 = 5;
// motor two
int enB = 9;
int in3 = 10;
int in4 = 11;
/////////////////PID CONSTANTS/////////////////
double kp=0.081;//3.55
double ki=0.00098;//0.003
double kd=0.650;//2.05
///////////////////////////////////////////////

double throttle=122; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady


void setup() {
  startTime= millis();
  
// join I2C bus (I2Cdev library doesn't do this automatically)
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
       Wire.begin();
       TWBR = 12; // 400kHz I2C clock (200kHz if CPU is 8MHz)
   #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
   Fastwire::setup(400, true);
   #endif
   Serial.begin(115200);
  // while (!Serial);
   //Serial.println(F("Initializing I2C devices..."));
   mpu.initialize();

   // verify connection
   Serial.println(F("Testing device connections..."));
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

   // wait for ready
   Serial.println(F("\nSend any character to begin DMP programming and demo: "));
   while (Serial.available() && Serial.read()); // empty buffer
   while (!Serial.available());                 // wait for data
   while (Serial.available() && Serial.read()); // empty buffer again
   delay(1000);
   // load and configure the DMP
   //Serial.println(F("Initializing DMP..."));
   devStatus = mpu.dmpInitialize();
   
   // supply your own gyro offsets here, scaled for min sensitivity
   mpu.setXGyroOffset(220);
   mpu.setYGyroOffset(76);
   mpu.setZGyroOffset(-85);
   mpu.setZAccelOffset(1788); // 1688 factory

   if (devStatus == 0) {
   // turn on the DMP, now that it's ready
   //Serial.println(F("Enabling DMP..."));
   mpu.setDMPEnabled(true);

   // enable Arduino interrupt detection
   //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
   attachInterrupt(0, dmpDataReady, RISING);
   mpuIntStatus = mpu.getIntStatus();

   // set our DMP Ready flag so the main loop() function knows it's okay to use it
   //Serial.println(F("DMP ready! Waiting for first interrupt..."));
   dmpReady = true;

   // get expected DMP packet size for later comparison
   packetSize = mpu.dmpGetFIFOPacketSize();
   } 
   else {
   // ERROR!
   // 1 = initial memory load failed
   // 2 = DMP configuration updates failed
   // (if it's going to break, usually the code will be 1)
   Serial.print(F("DMP Initialization failed (code "));
   Serial.print(devStatus);
   Serial.println(F(")"));
   }

    pinMode(LED_PIN, OUTPUT);
   if(!mag.begin()){
   /* There was a problem detecting the HMC5883 ... check your connections */
   Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
   while(1);
   }

  
  time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/

}//end of setup void

void loop() {

/////////////////////////////I M U/////////////////////////////////////
 // if programming failed, don't try to do anything
   if (!dmpReady) return;
   
   // wait for MPU interrupt or extra packet(s) available
   while (!mpuInterrupt && fifoCount < packetSize) {   }

   // reset interrupt flag and get INT_STATUS byte
   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();
    
   // get current FIFO count
   fifoCount = mpu.getFIFOCount();
//   Serial.print(fifoCount);

   // check for overflow (this should never happen unless our code is too inefficient)
   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
   // reset so we can continue cleanly
   mpu.resetFIFO();
   //Serial.println(F("FIFO overflow!"));

   // otherwise, check for DMP data ready interrupt (this should happen frequently)
   } 
   
   else if (mpuIntStatus & 0x02) {
        
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

            
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Total_angle[1] = ypr[2]* 180/M_PI;;           
        Serial.println(Total_angle[1]);}

   
  
/*///////////////////////////P I D///////////////////////////////////*/
/*Remember that for the balance we will use just one axis. I've choose the x angle
to implement the PID with. That means that the x axis of the IMU has to be paralel to
the balance*/

/*First calculate the error between the desired angle and 
*the real measured angle*/
error = Total_angle[1] - desired_angle;
    
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pid_p = kp*error;
elapsedTime = millis()-startTime;
//if(-3 <error <3)
//{
if(millis()>10000)
{
pid_i = pid_i+(ki*error);  
}
//}

pid_d = kd*((error - previous_error)/elapsedTime);

/*The final PID values is the sum of each of this 3 parts*/
PID = pid_p + pid_i + pid_d;
PID = PID*0.5;
/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
if(PID < -122)
{
  PID=-122;
}
if(PID > 122)
{
  PID=122;
}

//PID = (PID/255)*122;
/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
//pwmLeft= throttle - PID;
//pwmRight = throttle + PID;
if(millis()>10000)
{
if(error<0)
{
  pwmLeft= throttle - abs(PID);
  pwmRight = throttle + abs(PID);
}
if(error>=0)
{
  pwmLeft= throttle + abs(PID);
  pwmRight = throttle - abs(PID);
}

/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right

 //Serial.print(pwLeft);Serial.print("\t");
 //Serial.print(pwRight);Serial.print("\n");

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, pwmLeft);/*Left*/
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB,pwmRight);
}
  delay(30);
  Serial.print("\t");Serial.print(pwmLeft);
  Serial.print("\t");Serial.print(pwmRight);  
  Serial.print("\t");Serial.print(pid_p/kp);
  Serial.print("\t");Serial.print(pid_d/kd);
  Serial.print("\t");Serial.print(pid_i/ki);
  Serial.print("\t");Serial.print(PID);
  
  Serial.print("\n");
  //Serial.print("\t");Serial.print(pwmLeft);
  //Serial.print("\t");Serial.println(pwmRight);

previous_error = error; //Remember to store the previous error.
startTime = millis();
}//end of loop void
