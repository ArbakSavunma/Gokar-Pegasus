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
#define KP_mes 0.80
#define KI_mes 0.0
#define KD_mes 150.00
int KP_roll =6, KI_roll = 0, KD_roll = 4000;
int KP_pitch = 3, KI_pitch = 0, KD_pitch = 2000;
int KP_yaw = 8, KI_yaw = 0.01, KD_yaw = 2500;
#define IMU_COMMUNICATION_TIMEOUT 1000
MPU6050 mpu;
Servo ESC1, ESC2, ESC3, ESC4;
Servo servo1, servo2, servo3, servo4;
Servo ontek1, ontek2, arkatek;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float hiz ;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
unsigned long lastmillis=0;
int16_t * final_sets;
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t * first_sets;
int16_t * second_sets;
bool stabil=true;
int receiver_pins[] = {A0, A1, A2, A3, A4, A5};
long int receiver_values[] = {0, 0, 0, 0, 0, 0};
long int throttle=0, prevyaw=0, prevpitch=0, prevroll=0;
float mesafe, sure;
double  mesError=0, rollError=0, yawError=0, pitchError=0, roll_pid_i, roll_last_error, roll_control_signal, pitch_pid_i, pitch_last_error, pitch_control_signal, yaw_pid_i, yaw_last_error, yaw_control_signal, mes_pid_i, mes_last_error, mes_control_signal;
int motpower1=0, motpower2=0, motpower3=0, motpower4=0, motpower5=0, motpower6=0;  

void mot_calib_setup() {
  Serial.begin(9600);
  pinMode(triq,OUTPUT);
  pinMode(echo,INPUT);
//  pinMode(thermalError,INPUT);
//  pinMode(chargeError,INPUT);
  servo1.attach(2,1000,2000);
  servo2.attach(3,1000,2000);
  servo3.attach(4,1000,2000);
  servo4.attach(5,1000,2000);
  ESC1.attach(6,1000,2000);
  ESC2.attach(7,1000,2000);
  ESC3.attach(8,1000,2000);
  ESC4.attach(9,1000,2000);
  ontek1.attach(11,1000,2000);
  ontek2.attach(12,1000,2000);
  arkatek.attach(13,1000,2000);
  Serial.println("esc kalibre basladı...");

  servo1.write(0);
  servo2.write(180);
  servo3.write(0);
  servo4.write(180);
  ESC1.write(180);
  ESC2.write(180);
  ESC3.write(180);
  ESC4.write(180);
  ontek1.write(180);
  ontek2.write(180);
  arkatek.write(180);
  delay(2000);
  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0); 
  ontek1.write(0);
  ontek2.write(0);
  arkatek.write(0);
  delay(2000);
  
  ontek1.write(85);
  ontek2.write(85);
  arkatek.write(85);
  delay(3000);
  Serial.println("esc kalibre bitti");
}

long int rec_ver_transform() {
    for(int i=0; i<6; i++) {
      receiver_values[i]=pulseIn (receiver_pins[i], HIGH, 40000);
      /*Serial.print(" ");
      Serial.print(receiver_values[i]);
      Serial.print(" , "); */
    }
    //Serial.println(" ");
    long int rec[]= { receiver_values[0], receiver_values[1], receiver_values[2], receiver_values[3]};    
    
    
    
    prevroll=map(rec[0],1091,1852,-20,20);
    if(prevroll<5 && prevroll>-5) prevroll=0;
    //else if(prevroll>15) prevroll=15;
    //else if(prevroll<-15) prevroll=-15;
    prevpitch=map(rec[1],984,1980,-20,20);
    if(prevpitch<5 && prevpitch>-5) prevpitch=0;
    //else if(prevpitch>15) prevpitch=15;
    //else if(prevpitch<-15) prevpitch=-15;
    prevyaw=map(rec[3],1022,1955,-20,20);
    if(prevyaw<5 && prevyaw>-5) prevyaw=0; 
    //else if(prevyaw>15) prevyaw=15;
    //else if(prevyaw<-15) prevyaw=-15;
    if(receiver_values[4]<1500){
      throttle=map(rec[2],986,1977,0,180);
      if(throttle>180) throttle=180;
      else if(throttle<0) throttle= 0;
    }
    else {
      if(prevpitch>3){
        throttle=map(rec[2],986,1977,85,180);
        if(throttle<85) throttle=85; if(throttle>180) throttle=180;
       
      }
      else if (prevpitch<-3) {
        throttle=map(rec[2],986,1977,0,85);
        if(throttle<0) throttle=0; if(throttle>85) throttle=85;
        
      }
      else {
        ;
      }
    }  
     
    /*Serial.print("                                                                                                           ");
    Serial.print(prevyaw);
    Serial.print(" , ");
    Serial.print(prevpitch);
    Serial.print(" , ");
    Serial.print(prevroll);
    Serial.print(" , ");
    Serial.print(throttle);    
    Serial.println(" ");  */
  //return prevyaw, prevpitch, prevroll, throttle;
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

int calculateMotorPowers(int irtifa,int uzaklik,int PrRoll, int PrPitch, int PrYaw, double current_Roll, double current_Yaw, double current_Pitch, unsigned long deltatime) {
  mesError= irtifa - uzaklik;
  Serial.print("                                 mesaferror:");
  Serial.print(mesError);
  Serial.print(","); 
  rollError = PrRoll - current_Roll;
  Serial.print("Rollerror:");
  Serial.print(rollError);
  Serial.print(",");
  pitchError = PrPitch - current_Pitch;
  Serial.print("Pitcherror:");
  Serial.print(pitchError);
  Serial.print(","); 
  //if(rollError==0 && pitchError==0) {
  //  PrYaw=current_Yaw;
  //}
  yawError = PrYaw - current_Yaw;
  Serial.print("Yawerror:");
  Serial.print(yawError);
  Serial.println(" "); 
  if(yawError<1 && yawError>-1)  yawError=0;
  if(rollError<1 && rollError>-1)  rollError=0;
  if(pitchError<1 && pitchError>-1)  pitchError=0;
  
  mes_control_signal=  getControlSignal(mesError, KP_mes, KI_mes, KD_mes, mes_pid_i, mes_last_error, sure);
  roll_control_signal = getControlSignal(rollError, KP_roll, KI_roll, KD_roll, roll_pid_i, roll_last_error, deltatime);
  pitch_control_signal = getControlSignal(pitchError, KP_pitch, KI_pitch, KD_pitch, pitch_pid_i, pitch_last_error, deltatime);
  yaw_control_signal = getControlSignal(yawError, KP_yaw, KI_yaw, KD_yaw, yaw_pid_i, yaw_last_error, deltatime);
 
 
  motpower1 = throttle + mes_control_signal - roll_control_signal - yaw_control_signal - pitch_control_signal;
  if(motpower1<20) motpower1=20;
  if(motpower1>180) motpower1=180;
  //motpower1 = map(motpower1,0,180,85,180);
  //if(motpower1<85) motpower1=85;
  //if(motpower1>180) motpower1=180;
  motpower2 = throttle + mes_control_signal + roll_control_signal + yaw_control_signal - pitch_control_signal;
  if(motpower2<20) motpower2=20;
  if(motpower2>180) motpower2=180;
  motpower3 = throttle + mes_control_signal + roll_control_signal - yaw_control_signal + pitch_control_signal;
  if(motpower3<20) motpower3=20;
  if(motpower3>180) motpower3=180;
  motpower4 = throttle + mes_control_signal - roll_control_signal + yaw_control_signal + pitch_control_signal;
  if(motpower4<20) motpower4=20;
  if(motpower4>180) motpower4=180;
  if(throttle<10){
    motpower1=0; motpower2=0; motpower3=0; motpower4=0;
  }
  //if(motpower1>100) motpower1=100; if(motpower2>100) motpower2=100;  if(motpower3>100) motpower3=100; if(motpower4>100) motpower4=100;

  Serial.print("mot1:");
  Serial.print(motpower1);
  Serial.print(" ");
  Serial.print("mot2:");
  Serial.print(motpower2);
  Serial.print(" "); 
  Serial.print("mot3:");
  Serial.print(motpower3);
  Serial.print(" ");
  Serial.print("mot4:");
  Serial.print(motpower4);
  Serial.println(" "); 
  ESC1.write(motpower1);
  ESC2.write(motpower2);
  ESC3.write(motpower3);
  ESC4.write(motpower4);
}
void errorstate(bool imu_error, double pitchError, double rollError) {
  if(throttle>10) {
    if(prevroll!=0) {
      if( imu_error || rollError>60 || rollError<-60) {

          resetPidVariables();
          Serial.println(imu_error);
          while (1) {
            ESC1.write(0);
            ESC2.write(0);
            ESC3.write(0);
            ESC4.write(0);
            
          }    
      }
    }
    else {
      if( imu_error || rollError>40 || rollError<-40) {

          resetPidVariables();
          Serial.println(imu_error);
          while (1) {
            ESC1.write(0);
            ESC2.write(0);
            ESC3.write(0);
            ESC4.write(0);
            
          }    
      }
    }
    if(prevpitch!=0) {
      if( imu_error || pitchError>60 || pitchError<-60) {

          resetPidVariables();
          Serial.println(imu_error);
          while (1) {
            ESC1.write(0);
            ESC2.write(0);
            ESC3.write(0);
            ESC4.write(0);
            
          }    
      }
    }
    else {
      if( imu_error || pitchError>40 || rollError<-40) {

          resetPidVariables();
          Serial.println(imu_error);
          while (1) {
            ESC1.write(0);
            ESC2.write(0);
            ESC3.write(0);
            ESC4.write(0);
            
          }    
      }
    }
  }
}
void rightleftturning(uint32_t yawturn) {
	if(receiver_values[5]>1500) {
		int incdelay;
		if(yawturn>=3){
			if(yawturn>=3 || yawturn<6) incdelay=400;
			else if(yawturn>=6 || yawturn<10) incdelay=300;
			else if(yawturn>=10 || yawturn<13) incdelay=200;
			else if(yawturn>=13 || yawturn<17) incdelay=100;
			else incdelay=40;
			uint32_t now= millis();
			if(now-lastmillis>incdelay) {
				lastmillis=now;
				motpower5=throttle+3;
				motpower6=throttle-3;
				
        /*Serial.print("mot5:");
        Serial.print(motpower5);
        Serial.println(" ");
        Serial.print("mot5:");
        Serial.print(motpower5);
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
			if(now-lastmillis>incdelay) {
				lastmillis=now;
				motpower5=throttle+3;
				motpower6=throttle-3;
				
        /*Serial.print("mot5:");
        Serial.print(motpower5);
        Serial.println(" ");
        Serial.print("mot6:");
        Serial.print(motpower6);
        Serial.println(" "); */
			}
		}
	}
}

void landdrivecontrol(float prYaw, struct IMU_Values imu_values){

	if(receiver_values[5]>1500) {
		double yawError = prYaw - imu_values.CurrentOrientation.YawAngle;
		yaw_control_signal =  getControlSignal(yawError, KP_yaw, KI_yaw, KD_yaw, yaw_pid_i, yaw_last_error, imu_values.DeltaTime);
		yaw_last_error= yawError;
		motpower5= throttle-yaw_control_signal;
		
		motpower6= throttle+yaw_control_signal;
		
    /*Serial.print("mot1:");
    Serial.print(motpower1);
    Serial.println(" ");
    Serial.print("mot2:");
    Serial.print(motpower2);
    Serial.println(" "); */
		
	}
}

void mods(long int kademe, struct IMU_Values imu_values) {

  rec_ver_transform();
  
  mesafe_yer();
  
  if (kademe>1700) {
    
    Serial.println("*******MOD1********");
    
    calculateMotorPowers(100,mesafe,0,0,0, imu_values.CurrentOrientation.RollAngle,imu_values.CurrentOrientation.YawAngle,imu_values.CurrentOrientation.PitchAngle,imu_values.DeltaTime);
    errorstate(imu_values.Error, pitchError, rollError);
  }
  else if(kademe<1700 && kademe>1200) {
    Serial.println("*******MOD2********");
    calculateMotorPowers(100,mesafe,-1*prevroll,prevpitch, prevyaw, imu_values.CurrentOrientation.RollAngle,imu_values.CurrentOrientation.YawAngle,imu_values.CurrentOrientation.PitchAngle,imu_values.DeltaTime);
    errorstate(imu_values.Error, pitchError, rollError);
  }
  else if(kademe<1200) {
    Serial.println("*******MOD3********");
    calculateMotorPowers(0,0,-1*prevroll, prevpitch, prevyaw, imu_values.CurrentOrientation.RollAngle,imu_values.CurrentOrientation.YawAngle,imu_values.CurrentOrientation.PitchAngle,imu_values.DeltaTime);
    
    errorstate(imu_values.Error, pitchError, rollError);
  }
  else {
    errorstate(imu_values.Error, pitchError, rollError);
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
  /*if(digitalRead(thermalError)==1 || digitalRead(chargeError)==1) {
    ESC1.write(85);
    ESC2.write(0);
    ESC3.write(0);
    ESC4.write(0);
    ontek1.write(85);
    ontek2.write(85);
    arkatek.write(85);
    if(digitalRead(thermalError)==1) {
      Serial.println("Batarya cok isinmis sistemi kapatin!");
    }
    else if(digitalRead(chargeError)==1) {
      Serial.println("Batarya da enerji yok kapatin!");
    }
    else 
      ;
  } */
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

  Serial.println(imuValues.DeltaTime);
  /*if(digitalRead(thermalError)==1 || digitalRead(chargeError)==1) {
    
    while (mesafe>2) {
      motpower1-=5;
      motpower2-=5;
      motpower3-=5;
      motpower4-=5;
      delay(400);
    }
    if(digitalRead(thermalError)==1) {
      Serial.println("Batarya cok isinmis sistemi kapatin!");
    }
    if(digitalRead(chargeError)==1) {
      Serial.println("Batarya da enerji yok kapatin!");
    }
  }
  else {*/
    if( receiver_values[4]<1500) {
        //Serial.println("Uçuşa geçildi");
        
          ontek1.write(85);
          ontek2.write(85);
          arkatek.write(85);
          servo1.write(0);
          servo2.write(165);
          servo3.write(5);
          servo4.write(160); 
          
           
        mods(receiver_values[5],imuValues);
    }
    else {
      Serial.println("kara sürüşünde");

          
          ESC1.write(0);
          ESC2.write(0);
          ESC3.write(0);
          ESC4.write(0);
          servo1.write(180);
          servo2.write(0);
          servo3.write(180);
          servo4.write(0);
          
        
      
      
      if(-1 * prevyaw<3 && -1 * prevyaw>-3) {
        if(prevpitch>3){
          
          prevyaw=imuValues.CurrentOrientation.YawAngle;
          landdrivecontrol(prevyaw,imuValues);
          ESC1.write(0);
          ESC2.write(0);
          ESC3.write(0);
          ESC4.write(0);
          
          ontek1.write(motpower5);
          ontek2.write(motpower6);
          arkatek.write(throttle);
        }
        else if(prevpitch<-3){ 
          
          prevyaw=imuValues.CurrentOrientation.YawAngle;
          landdrivecontrol(prevyaw,imuValues);
          ESC1.write(0);
          ESC2.write(0);
          ESC3.write(0);
          ESC4.write(0);
          /*Serial.print("mot5:");
          Serial.print(85 - motpower5);
          Serial.println(" ");
          Serial.print("mot6:");
          Serial.print(85 - motpower6);
          Serial.println(" "); */
          ontek1.write(85 - motpower5);
          ontek2.write(85 - motpower6);
          arkatek.write(85 - throttle);
        }
        else {
          ontek1.write(85);
          ontek2.write(85);
          arkatek.write(85);
          
        }
		  }
		  else {
        if(prevpitch>3){
          rightleftturning(-1 * prevyaw);
          ESC1.write(0);
          ESC2.write(0);
          ESC3.write(0);
          ESC4.write(0);
          
          ontek1.write(motpower5);
          ontek2.write(motpower6);
          arkatek.write(throttle);
        }
        else if(prevpitch<-3){
          rightleftturning(prevyaw);
          ESC1.write(0);
          ESC2.write(0);
          ESC3.write(0);
          ESC4.write(0);
          
          ontek1.write(85 - motpower5);
          ontek2.write(85 - motpower6);
          arkatek.write(85 - throttle);
        }
        else {
          ontek1.write(85);
          ontek2.write(85);
          arkatek.write(85);
        }  
      }
    }
  
}