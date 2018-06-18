/* Updated till June 18,2018*/

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>


#include<Servo.h>

#include <SoftwareSerial.h>

SoftwareSerial mySerial(11,12); // RX, TX

Servo myServo;
Servo myServo1;
Servo myServo2;
Servo myServo3;

// sensor interfacing
int shrpsensor;
int sensorread;

MPU6050 mpu;    //Variable to read Gyro Sensor Value
int btRead;
int loopCount;
int runningMode;
int angle1;
int angleMax=120;
int angleMin=60; 
int stepMove=1;
int numberCount;
int pos;
int delayTime1=15;
int delayTime2=5000;
int directionMove;
int printLine=0;
long val1, val2, val3;
int pitch =0;
int roll=0;
int yaw=0;
void getGryo();
int Sharp();
void Move();
float Xaxis;
float Yaxis;
float Zaxis;
void checkMPUSettings();
void moveContinuous(int pos,int maxAngle,int minAngle,int stepSize,int delayTime);
void moveStep(int pos,int maxAngle,int minAngle,int stepSize,int loopNumber, int delayTime);

void setup() 
{
myServo.attach(6);
myServo1.attach(3);
myServo2.attach(4);
myServo3.attach(5);

Serial.begin(9600);
mySerial.begin(9600);

Wire.begin();  //Accelerometer checking
while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
 {
  Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
  delay(500);
  }
  //checkMPUSettings();
  //Accelerometer checking
loopCount=0;
//pos=angleMin;
directionMove=0;
myServo.write(pos);
myServo1.write(90);
myServo2.write(90);
myServo3.write(90);

Serial.println("Start Program");
//loopCount=8;
mySerial.println("");
if(printLine==0)
{
  printLine=1;
  mySerial.println("");
  mySerial.println(":Set Running Mode:");
}
}
void loop() 
{
//  delay(100);
////////////-READ Bluetooth-////////////////////////////////////
if(mySerial.available() > 0)
{
  btRead = mySerial.read();
  loopCount++;    
///////////////////////////////////////////////////////////////////
  
if(btRead=='z')
{
  loopCount=0;
  Serial.println("///////////////// Reset /////////////////////");
  if(printLine==1)
  { //printLine=0;
  mySerial.println("Reset");
  mySerial.println("");
  mySerial.println(":Set Running Mode:");
  }
}
//if(loopCount==1)////first iretaion 
//{
//delay(1000);
  if(btRead=='g')
  {
     loopCount=1;
     delay(1000);
  while(1)
  {
    getGryo();
      //////SENSOR VALUE
     Sharp();
     mySerial.print("");
     checkMPUSettings(); //Accelerometer checking
     delay(4000);
     mySerial.println();
  if(mySerial.available() > 0)
      { 
        btRead = mySerial.read();
      }
      if(btRead=='z')
       {
        loopCount=0;
        break;
        } 
  }
}
/////////-Countinus Mode Setting////////////////////////////////////////////
if(btRead=='a')
{    runningMode=1;///continues  
if( printLine==1)
{   printLine=0; 
    goto autoMode;
}
}
////////////////-Step Mode Setting-/////////////////////////////////////////////////////
if(btRead=='b'){runningMode=0;////Step Mode  
if(printLine==1){printLine=0;mySerial.println("");mySerial.println(":Set Max Angle :");}
}
////////////////////////////////////////////////////////////
if(loopCount==2)
{
switch(btRead)//////set Motor Degree
{
case '1':angleMax=90;break;  
case '2':angleMax=95;break;  
case '3':angleMax=100;break;  
case '4':angleMax=105;break;  
case '5':angleMax=110;break;  
case '6':angleMax=115;break;  
case '7':angleMax=120;break;  
case '8':angleMax=125;break;  
case '9':angleMax=130;break;  
}
if(printLine==0){printLine=1;  
mySerial.println("");
mySerial.println(":Set Min Angle :");}
}
if(loopCount==3)
{
switch(btRead)//////set Motor Degree
{
case '1':angleMin=50;break;  
case '2':angleMin=55;break;  
case '3':angleMin=60;break;  
case '4':angleMin=65;break;  
case '5':angleMin=70;break;  
case '6':angleMin=75;break;  
case '7':angleMin=80;break;  
case '8':angleMin=85;break;  
case '9':angleMin=90;break;  
}
if(printLine==1){printLine=0;   
mySerial.println("");
 mySerial.println(":Set step Size :");}

} 
if(loopCount==4)
{
switch(btRead)//////set Motor Degree
{
case '1':stepMove=1;break;  
case '2':stepMove=2;break;  
case '3':stepMove=3;break;  
case '4':stepMove=4;break;  
case '5':stepMove=5;break;  
case '6':stepMove=6;break;  
case '7':stepMove=7;break;  
case '8':stepMove=8;break;  
case '9':stepMove=9;break;  
}
if(printLine==0){printLine=1;
  mySerial.println("");
mySerial.println(":Set delay per move step");}

} 
if(loopCount==5)//mS
{
switch(btRead)//////set Motor Degree
{
case '1':delayTime1=8;break;  
case '2':delayTime1=9;break;  
case '3':delayTime1=10;break;  
case '4':delayTime1=11;break;  
case '5':delayTime1=12;break;  
case '6':delayTime1=13;break;  
case '7':delayTime1=14;break;  
case '8':delayTime1=15;break;  
case '9':delayTime1=16;break;  
}
if(printLine==1){printLine=0;  
  mySerial.println("");
mySerial.println(":Set delay per action");}

}
if(loopCount==6)//after delay
{
switch(btRead)//////set Motor Degree
{
case '1':delayTime2=1;break;  
case '2':delayTime2=3;break;  
case '3':delayTime2=4;break;  
case '4':delayTime2=5;break;  
case '5':delayTime2=6;break;  
case '6':delayTime2=7;break;  
case '7':delayTime2=8;break;  
case '8':delayTime2=9;break;  
case '9':delayTime2=10;break;  
}

if(printLine==0){printLine=1;
  mySerial.println("-----Your Setting-----");

if(runningMode==1)
{
  Serial.println("Mode: Countinus");
  mySerial.println("Mode: Countinus");
  }  
if(runningMode==0)
{
  Serial.println("Mode: Step");
  mySerial.println("Mode: Step");
  }  

Serial.print("Max Angle: ");
Serial.println(angleMax);
mySerial.print("Max Angle: ");
mySerial.println(angleMax);
Serial.print("Min Angle: ");Serial.println(angleMin);mySerial.print("Min Angle: ");mySerial.println(angleMin);
Serial.print("Step     : ");Serial.println(stepMove);mySerial.print("Step     : ");mySerial.println(stepMove);
Serial.print("Delay1     : ");Serial.println(delayTime1);mySerial.print("Delay1     : ");mySerial.println(delayTime1);
Serial.print("Loop Number     : ");Serial.println(delayTime2);mySerial.print("loop Number     : ");mySerial.println(delayTime2);
  mySerial.println("");
  mySerial.println("Press 'c' for countine and press 'z' for Reset ");
  pos=angleMin;
}
}

if(loopCount==7)
{
if(btRead=='c')
{
loopCount=8;
}
}
////////////////////////////////////////////////////////
if(loopCount==8)
{
  while(1)
  {
////////////-READ Bluetooth-////////////////////////////////////
    if(mySerial.available() > 0){btRead = mySerial.read();loopCount++;  }  
///////////////////////////////////////////////////////////////////
if(btRead=='z')
{
  loopCount=0;
  runningMode=1;////Step Mode  
  btRead=0;
  Serial.println("///////////////// Reset /////////////////////");
  if(printLine==1)
  {
    //printLine=0;
  mySerial.println("Reset");
  mySerial.println("");
  mySerial.println(":Set Running Mode:");
  break;
  }
}
if(runningMode==1)
{
  autoMode:
  moveContinuous(angleMax,angleMin,stepMove,delayTime1);
}

if(runningMode==0)
{
  moveStep(angleMax,angleMin,stepMove,delayTime2,delayTime1);  
}

}

}
}
}

void getGryo()
{
  // Read normalized values 
Vector normAccel = mpu.readNormalizeAccel();
  // Calculate Pitch & Roll
 
  pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  yaw=(atan2(normAccel.ZAxis,normAccel.YAxis)*180.0)/M_PI;

// Serial.print("Pitch: ");
// Serial.print(pitch);
// Serial.print(" Roll: ");
// Serial.print(roll);
 /////////////////////////////
 mySerial.print("Pitch: ");
 mySerial.print(pitch);
 mySerial.print(" Roll: ");
 mySerial.print(roll);
 mySerial.println();
 mySerial.print("Yaw:");
 mySerial.print(yaw);
 mySerial.println();
 
 
}
//////////////////////////////////////////////////////////////////
void moveContinuous(int maxAngle,int minAngle,int stepSize,int delayTime)
{
   
 if(pos>=maxAngle && directionMove==0)
 {
  directionMove=1;
  pos=maxAngle;
 }       // if close to max Limit than change the direction one side
else
 if(pos<=minAngle && directionMove==1)
 { 
    directionMove=0;
    pos=minAngle;
 }     // if close to min Limit than change the direction other Side

if(directionMove==0){pos+=stepSize;}
if(directionMove==1){pos-=stepSize;}

////////////////////-Move Motors-/////////////////////////////////////////////////////////////////////////////////////////////////
myServo.write(pos+20);
//myServo1.write(pos);
myServo2.write(pos);
myServo3.write(pos);// goes from 0 degrees to 180 degrees// in steps of 1 degree
delay(delayTime);// waits 15ms for the servo to reach the position

     sensorread = analogRead(A0);  // read the input on analog pin 0:         
     shrpsensor= (6787/(sensorread-4)-8);
     if(shrpsensor<=13){ myServo1.write(50);}
     else
          myServo1.write(pos);

     if(mySerial.available() > 0)
     {
      btRead = mySerial.read();
     }
     if (btRead == 'z')
     {
      mySerial.println("Reset");
      softReset();
     }
  }

void moveStep(int maxAngle,int minAngle,int stepSize,int loopNumber, int delayTime)
{  
if(numberCount>=loopNumber)
{
numberCount=0; 
delay(5000);
}
 if(pos>=maxAngle && directionMove==0)
 {  
     directionMove=1;
     pos=maxAngle;
     numberCount++;}//// if close to max Limit than change the direction one side
else
 if(pos<=minAngle && directionMove==1)
 { 
    directionMove=0;
    pos=minAngle;
    }//// if close to min Limit than change the direction other Side

if(directionMove==0)
{ 
  pos+=stepSize;
  }
if(directionMove==1)
{
   pos-=stepSize;
   }
   
 
  
////////////////////-Move Motors-/////////////////////////////////////////////////////////////////////////////////////////////////
myServo.write(pos+20);
//myServo1.write(pos);
myServo2.write(pos);
myServo3.write(pos);// goes from 0 degrees to 180 degrees// in steps of 1 degree
delay(delayTime); // waits 15ms for the servo to reach the position

sensorread = analogRead(A0);  // read the input on analog pin 0:         
shrpsensor= (6787/(sensorread-4)-8);
if(shrpsensor<=13){ myServo1.write(pos+40);}
else
myServo1.write(pos);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Serial.print("Servo Motor Angle: ");
//Serial.println(pos);
//Serial.print("Loop:                   ");
//Serial.println(numberCount);
//mySerial.print("Servo Motor Angle: ");
//mySerial.println(pos);
//mySerial.print("Loop:                   ");
//mySerial.println(numberCount);
}

int Sharp(){

     // Sharp infared sensor
     sensorread = analogRead(A0);  // read the input on analog pin 0:         
     shrpsensor= (6787/(sensorread-4)-8); // conversion formula
     mySerial.print("Hurdle Is at: ");
     mySerial.print(shrpsensor);
     mySerial.println();
     delay(10);
  
}


void checkMPUSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print("Accelerometer:");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  
    val1 = map(mpu.getAccelOffsetX(),-2000,2000,-360,360);
    val2 = map(mpu.getAccelOffsetY(),-2000,2000,-360,360); 
    val3 =map( mpu.getAccelOffsetZ(),-2000,2000,-360,360);
    
  
  mySerial.print("* Accelerometer: ");
  mySerial.println();
  mySerial.print("X-axis:");
  mySerial.print(val1);
  mySerial.println();
  mySerial.print("Y-axis:");
  mySerial.print(val2);
  mySerial.println();
  mySerial.print("Z-axis:");
  mySerial.print(val3);
  mySerial.println();
 

  /*
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  Serial.println();
  */
}
void reset_all(){
  Serial.print("Reset");
}
void Reset()
{
  myServo.write(90);
  myServo1.write(90);
  myServo2.write(90);
  myServo3.write(90);
}

void softReset(){
asm volatile ("  jmp 0");
}
