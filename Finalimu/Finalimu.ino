#include <MPU9250.h>
#include <U8g2lib.h>
#include <math.h>

#include <SPI.h>
#include <Wire.h>
void moveMotor(unsigned char c, unsigned int value){
  if(value <=10)
    Serial1.write('#');//STOP THE MOTOR
  else if(value <= 20)
    Serial1.write(c);//MOVE THE MOTOR AT SLOW SPEED
  else if(value <= 30)
    Serial1.write(c+1);//MOVE THE MOTOR AT MEDIUM SPEED
  else if(value <= 40)
    Serial1.write(c+2);//MOVE THE MOTOR AT HIGH SPEED
  else
    Serial1.write(c+3);//MOVE THR MOTOR AT FULL SPEED
}

void convert(float roll, float pitch){
  int absRoll = round(abs(roll));
  int absPitch = round(abs(pitch));
  int greater;
  bool flag;
  
  //Determine the general direction i.e front/back or left/right
  
  if(absRoll > absPitch){
    greater = absRoll;
    //determine particular direction i.e front or back
    //Set the flag according to the direction. For negative values of roll
    //i.e. for front direction, flag is 1 and for back direction the flag is 0
    flag = (roll < 0)?0:1;
    if(flag)//If the hand is tilted forward
    {
      moveMotor('a', greater);
    }else moveMotor('e', greater);//If hand is tilted back
  }else{
    greater = absPitch;
    flag = (pitch < 0)?1:0;
    if(flag)//If the hand is tilted left
    {
      moveMotor('i', greater);
    }else moveMotor('m', greater);//If hand is tilted right
  }
}

void convert1(float roll, float pitch){
  if((roll >=0 and roll <= 20) and (pitch >= 20 and pitch <= 30)){
    Serial1.write('#');
  }else if((roll >=20 and roll <= 50) and (pitch >= 30 and pitch <= 36){
    Serial1.write('f');
  }else if((roll < 0) and (pitch >= 20){
    Serial1.write('b');
  }else if((roll >=10 and roll <= 20) and (pitch <= 15){
    Serial1.write('l');
  }else if((roll >0 and roll < 10) and (pitch > 30))
    Serial1.write('r');
  else Serial1.write('*');
}
#define SCREEN U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI
#define SPI_CLK 14
#define ARRAY_SIZE 100
#define HWSERIAL Serial1

MPU9250 imu;
const int UPDATE_INTERVAL = 500;    // how often to update display (ms)
float accel_data[ARRAY_SIZE] = {0};   // initialize array holding accel data

int time_since_update=0;
float avg[5]={0, 0, 0, 0, 0};
float sum[5]={0, 0, 0, 0, 0};
float roll,pitch;

void setup() {
  Serial.begin(9600);

  HWSERIAL.begin(9600);
  Wire.begin();
//  SPI.setSCK(14);
  
  

  byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.println("MPU9250 is online...");
  delay(5000);
  
  // Calibrate gyro and accelerometers, load biases in bias registers
  imu.initMPU9250();
  imu.MPU9250SelfTest(imu.selfTest);
  imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);
  imu.initAK8963(imu.factoryMagCalibration);
  imu.getAres();
  imu.getGres();
  imu.getMres();
  // put your setup code here, to run once:

}

void loop() {
  imu.readAccelData(imu.accelCount);
  

  // calculate acceleration in g's and store in ax, ay, and az
  imu.ax = (float)imu.accelCount[0]*imu.aRes;
  imu.ay = (float)imu.accelCount[1]*imu.aRes;
  imu.az = (float)imu.accelCount[2]*imu.aRes;
 

  if (time_since_update < UPDATE_INTERVAL) {
      avg[0] += imu.ax;
      avg[1]+=imu.ay;
      avg[2]+= imu.az;
      time_since_update++;
    
    }
    
    else
    {
      //Serial.println("I am here okay");
      sum[0]= avg[0] /time_since_update;
      sum[1]= avg[1]/time_since_update;
      sum[2]= avg[2]/time_since_update;
      roll= 57.29*atan2(sum[1],sum[2]); 
      pitch = 57.29*atan2(sum[0],sum[2]);

      convert1(roll, pitch); //Convert roll and pitch and transmit via Serial1
      
      Serial.println("roll" + String(roll));
      Serial.println("pitch" + String(pitch));
    
  
      time_since_update = 0;
      avg[0] = 0;
      avg[1]=0;
      avg[2]= 0;
  }
  }
  


void print_accel()
{

  Serial.println(String(sum[0],4) + "  " +
     String(sum[1],4) +"  " + String(sum[2],4));

  
}
