//KOD AÇIKLAMASI:
//  Kod pegasus_multi_driver_model4 ün devamıdır. Eklenen tek şey kara sürüşünde geriye doğru harekettir.
#include<Servo.h>
#define INTERRUPT_PIN 2
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define echo 39
#define triq 41
#define sagtek_pwm 5
#define soltek_pwm 6
#define enA  7
#define enB  8
#define KP_yaw 1.3
#define KI_yaw 0.00
#define KD_yaw 200.00
#define IMU_COMMUNICATION_TIMEOUT 1000
MPU6050 mpu;
Servo servo1, servo2, servo3, servo4;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float sicalik_gerilim = 0;
int okunan_degerlm35 = 0;
float hiz ;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
unsigned long lastmillis1=0, lastmillis2=0, lastmillis3=0, lastmillis4=0;
int16_t * final_sets;
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t * first_sets;
int16_t * second_sets;
bool stabil=true;
float doluluk;
int okunan_deger =0;
int receiver_pins[] = {A0, A1, A2};
long int receiver_values[] = {0, 0, 0};
int servo_val1=0, servo_val2= 180, servo_val3= 0, servo_val4= 180;
long int  throttle=0, prevyaw=0, prevpitch=0;
float mesafe, sure, sicalik = 0;
double  mesError=0, rollError=0, yawError=0, pitchError=0, roll_pid_i, roll_last_error, roll_control_signal, pitch_pid_i, pitch_last_error, pitch_control_signal, yaw_pid_i, yaw_last_error, yaw_control_signal, mes_pid_i, mes_last_error, mes_control_signal;
int motpower1=0, motpower2=0, motpower3=0, motpower4=0, motpower5=0, motpower6=0;  
int sagtek_val=0, soltek_val=0;
void mot_calib_setup() {
  Serial.begin(9600);
  pinMode(triq,OUTPUT);
  pinMode(echo,INPUT);

  pinMode(sagtek_pwm, OUTPUT);
  pinMode(soltek_pwm, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  servo1.attach(3,1000,2000);
  servo2.attach(9,1000,2000);
  servo3.attach(10,1000,2000);
  servo4.attach(11,1000,2000);


  

 
  //Serial.println("esc kalibre bitti");
}

long int rec_ver_transform() {
    for(int i=0; i<3; i++) {
      receiver_values[i]=pulseIn (receiver_pins[i], HIGH, 40000);
      /*Serial.print(" ");
      Serial.print(receiver_values[i]);
      Serial.print(" , "); */
    }
    //Serial.println(" ");
    long int rec[]= { receiver_values[0], receiver_values[1], receiver_values[2], receiver_values[3]};    
    prevpitch=map(rec[1],1000,2000,-20,20);
    if(prevpitch<5 && prevpitch>-5) prevpitch=0;

    //prevyaw=map(rec[2],1000,2000,-20,20);
    prevyaw=0;
    if(prevyaw<5 && prevyaw>-5) prevyaw=0; 
    if(prevpitch>5){
      
     throttle=(prevpitch*17)-85;
     if(throttle<0) throttle=0; if(throttle>255) throttle=255;
    }
    else {
      
        throttle=-1*((prevpitch*17)+85);
        if(throttle<0) throttle=0; if(throttle>255) throttle=255;
       
      
      
    }  
     
    /*Serial.print("                                         ");
    Serial.print(prevyaw);
    Serial.print(" , ");
    Serial.print(prevpitch);
    Serial.print(" , ");
    Serial.print(throttle);    
    Serial.println(" ");  
  return prevyaw, prevpitch, throttle;*/
}

void resetPidVariables() {
  roll_pid_i = 0;
  roll_last_error = 0;
  yaw_pid_i = 0;
  yaw_last_error = 0;
  pitch_pid_i = 0;
  pitch_last_error = 0;
}

void dmpDataReady() {
  mpuInterrupt = true;
}
unsigned long last_time=0; 
struct Orientation {
  double YawAngle;
  double PitchAngle;
  double RollAngle;
};
struct IMU_Values {
  bool Error;
  bool NewDataAvailable;
  unsigned long DeltaTime;
  struct Orientation CurrentOrientation;
};

void initializeIMU() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  if(!mpu.testConnection()){
    Serial.println("*imu test connection failed!");
  }
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    first_sets=mpu.GetActiveOffsets();
    mpu.PrintActiveOffsets();
    delay(1000);
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    second_sets=mpu.GetActiveOffsets();
    mpu.PrintActiveOffsets();
    if(abs(first_sets[0]-second_sets[0])<10&&abs(first_sets[1]-second_sets[1])<10&&abs(first_sets[2]-second_sets[2])<10
     &&abs(first_sets[3]-second_sets[3])<10&&abs(first_sets[4]-second_sets[4])<10&&abs(first_sets[5]-second_sets[5])<10){
       final_sets=first_sets;
    }
    else{
       while(stabil){
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        first_sets=mpu.GetActiveOffsets();
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        second_sets=mpu.GetActiveOffsets();
        if(abs(first_sets[0]-second_sets[0])<10&&abs(first_sets[1]-second_sets[1])<10&&abs(first_sets[2]-second_sets[2])<10
         &&abs(first_sets[3]-second_sets[3])<10&&abs(first_sets[4]-second_sets[4])<10&&abs(first_sets[5]-second_sets[5])<10){
          final_sets=first_sets;
          stabil=false;
         }
       }
     }
  }

  mpu.setXAccelOffset(final_sets[0]);
  mpu.setYAccelOffset(final_sets[1]);
  mpu.setZAccelOffset(final_sets[2]);
  mpu.setXGyroOffset(final_sets[3]);
  mpu.setYGyroOffset(final_sets[4]);
  mpu.setZGyroOffset(final_sets[5]);


  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
}

struct IMU_Values GetIMU_Values() {
  struct IMU_Values o;
  o.NewDataAvailable = false;
  if (!dmpReady) 
    return o;

  unsigned long current_time = millis();
  unsigned long delta_time = current_time - last_time;
    
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    o.CurrentOrientation.YawAngle = ypr[0] * 180 / M_PI;
    o.CurrentOrientation.RollAngle = ypr[1] * 180 / M_PI * -1; //-1 for changing rotation
    o.CurrentOrientation.PitchAngle = ypr[2] * 180 / M_PI;
    o.NewDataAvailable = true;
    o.DeltaTime = delta_time;
    
    if (last_time == 0){
      last_time = current_time;
     
      o.Error = true;
      return o;
    }
    last_time = current_time;
  }
  
  if(delta_time > IMU_COMMUNICATION_TIMEOUT){
      
      o.Error = true;

  }else{
      o.Error = false;
  }

  return o;
}



double getControlSignal(double error, double kp, double ki, double kd, double& pid_i, double& last_error, unsigned long delta_time) {
  double pid_p = error;
  double pid_d = (error - last_error) / delta_time;
  pid_i += error * delta_time;
  //Serial.println(delta_time);
  double control_signal = (kp*pid_p) + (ki*pid_i) + (kd*pid_d);
  last_error = error;
  return control_signal;
}


void errorstate(bool imu_error, double pitchError, double rollError) {
  if(throttle>10) {
    if( imu_error || pitchError>40 || rollError>40 || pitchError<-40 || rollError<-40) {

        resetPidVariables();
        //Serial.println(imu_error);
        while (1) {
          digitalWrite(enA, LOW);
          digitalWrite(enB, LOW);
        } 
        return 0;
    }
  }
}
void rightleftturning(uint32_t yawturn) {
	if(receiver_values[0]>1500) {
		int incdelay;
		if(yawturn>=3){
			if(yawturn>=3 || yawturn<6) incdelay=400;
			else if(yawturn>=6 || yawturn<10) incdelay=300;
			else if(yawturn>=10 || yawturn<13) incdelay=200;
			else if(yawturn>=13 || yawturn<17) incdelay=100;
			else incdelay=40;
			uint32_t now= millis();
			if(now-lastmillis1>incdelay) {
				lastmillis1=now;
				soltek_val=throttle+3;
				sagtek_val=throttle-3;
				
       /* Serial.print("mot5:");
        Serial.print(soltek_val);
        Serial.println(" ");
        Serial.print("mot5:");
        Serial.print(sagtek_val);
        Serial.println(" "); */
			}
		}
		else if(yawturn<=-3){
			if(yawturn<=-3 || yawturn>-6) incdelay=400;
			else if(yawturn<=-6 || yawturn>-10) incdelay=300;
			else if(yawturn<=-10 || yawturn>-13) incdelay=200;
			else if(yawturn<=-13 || yawturn>-17) incdelay=100;
			else incdelay=40;
			uint32_t now= millis();
			if(now-lastmillis2>incdelay) {
				lastmillis2=now;
				sagtek_val=throttle+3;
				soltek_val=throttle-3;
				
         /*Serial.print("mot5:");
        Serial.print(soltek_val);
        Serial.println(" ");
        Serial.print("mot5:");
        Serial.print(sagtek_val);
        Serial.println(" "); */
			}
		}
	}
}

void landdrivecontrol(float prYaw, struct IMU_Values imu_values){

	if(receiver_values[0]>1500) {
		double yawError = prYaw - imu_values.CurrentOrientation.YawAngle;
		yaw_control_signal =  getControlSignal(yawError, KP_yaw, KI_yaw, KD_yaw, yaw_pid_i, yaw_last_error, imu_values.DeltaTime);
		yaw_last_error= yawError;

		sagtek_val= throttle-yaw_control_signal;
		
		soltek_val= throttle+yaw_control_signal;
		
    /*Serial.print("mot1:");
    Serial.print(motpower1);
    Serial.println(" ");
    Serial.print("mot2:");
    Serial.print(motpower2);
    Serial.println(" "); */
		
	}
}




float mesafe_yer() {
  
  digitalWrite(triq,LOW);
  delayMicroseconds(2);
  digitalWrite(triq,HIGH);
  delayMicroseconds(10);
  digitalWrite(triq,LOW);
  float x;
  x= sqrt(1 + ( 27 / 273));
  hiz = 10000 / (331 * x);
  sure = pulseIn(echo,HIGH);
  mesafe = sure / hiz  / 2;
  if (mesafe>500)
    mesafe=500;
  /*Serial.print("mesafe:");
  Serial.print(mesafe);
  Serial.println(" "); */
  
  return mesafe;
}

void setup(){
  initializeIMU();
  delay(2000);
  mot_calib_setup();

}

void printServoValues() {
  Serial.print("servo_val1: ");
  Serial.println(servo_val1);
  /*Serial.print("servo_val2: ");
  Serial.println(servo_val2);
  Serial.print("servo_val3: ");
  Serial.println(servo_val3);
  Serial.print("servo_val4: ");
  Serial.println(servo_val4);*/
}

void loop() {
  struct IMU_Values imuValues = GetIMU_Values();
   /*Serial.print("Yaw:");
  Serial.print(imuValues.CurrentOrientation.YawAngle);
  Serial.print(",");

  Serial.print("Pitch:");
  Serial.print(imuValues.CurrentOrientation.PitchAngle);
  Serial.print(",");

  Serial.print("Roll:");
  Serial.print(imuValues.CurrentOrientation.RollAngle);
  Serial.println(); */

  rec_ver_transform();

  if(receiver_values[0] < 1500) {
    Serial.println("Uçuşa geçildi");
    digitalWrite(enA, LOW);
    digitalWrite(enB, LOW);
    if(servo_val1 == 0 && servo_val2 == 180) {
      servo1.write(0);
      servo2.write(180);
      servo3.write(0);
      servo4.write(180);
    } else {
      uint32_t now = millis();
      if(now - lastmillis3 > 25) {
        lastmillis3 = now;
        servo_val1--;
        servo_val2++;
        servo_val3--;
        servo_val4++;
        if(servo_val1 < 0) servo_val1 = 0;
        if(servo_val2 > 180) servo_val2 = 180;
        if(servo_val3 < 0) servo_val3 = 0;
        if(servo_val4 > 180) servo_val4 = 180;
        printServoValues();
        servo1.write(servo_val1);
        servo2.write(servo_val2);
        servo3.write(servo_val3);
        servo4.write(servo_val4);
      }
    }
    
  } else {
    Serial.println("kara sürüşünde");
    if(servo_val1 == 180 && servo_val2 == 0) {
      servo1.write(180);
      servo2.write(0);
      servo3.write(180);
      servo4.write(0);
    } else {
      uint32_t now = millis();
      if(now - lastmillis4 > 25) {
        lastmillis4 = now;
        servo_val1++;
        servo_val2--;
        servo_val3++;
        servo_val4--;
        if(servo_val1 > 180) servo_val1 = 180;
        if(servo_val2 < 0) servo_val2 = 0;
        if(servo_val3 > 180) servo_val3 = 180;
        if(servo_val4 < 0) servo_val4 = 0;
        printServoValues();
        servo1.write(servo_val1);
        servo2.write(servo_val2);
        servo3.write(servo_val3);
        servo4.write(servo_val4);
      }
      
    }

    if(prevyaw < 3 && prevyaw > -3) {
      if(prevpitch > 3) {
        prevyaw = imuValues.CurrentOrientation.YawAngle;
        landdrivecontrol(prevyaw, imuValues);
        analogWrite(sagtek_pwm, sagtek_val);
        analogWrite(soltek_pwm, soltek_val);
        digitalWrite(enA, HIGH);
        digitalWrite(enB, LOW);
      } else if(prevpitch < -3) {
        prevyaw = imuValues.CurrentOrientation.YawAngle;
        landdrivecontrol(prevyaw, imuValues);
        analogWrite(sagtek_pwm, sagtek_val);
        analogWrite(soltek_pwm, soltek_val);
        digitalWrite(enA, LOW);
        digitalWrite(enB, HIGH);
      } else {
        digitalWrite(enA, LOW);
        digitalWrite(enB, LOW);
      }
    } else {
      if(prevpitch > 3) {
        rightleftturning(-1 * prevyaw);
        analogWrite(sagtek_pwm, sagtek_val);
        analogWrite(soltek_pwm, soltek_val);
        digitalWrite(enA, LOW);
        digitalWrite(enB, HIGH);
      } else if(prevpitch < -3) {
        rightleftturning(prevyaw);
        analogWrite(sagtek_pwm, sagtek_val);
        analogWrite(soltek_pwm, soltek_val);
        digitalWrite(enA, HIGH);
        digitalWrite(enB, LOW);
      } else {
        digitalWrite(enA, LOW);
        digitalWrite(enB, LOW);
      }
    }
   
  }
}
