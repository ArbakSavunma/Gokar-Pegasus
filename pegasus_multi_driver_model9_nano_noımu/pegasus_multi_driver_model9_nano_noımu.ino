//KOD AÇIKLAMASI:
//  Kod pegasus_multi_driver_model4 ün devamıdır. Eklenen tek şey kara sürüşünde geriye doğru harekettir.

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
unsigned long lastmillis=0;
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
    long int rec[]= { receiver_values[0], receiver_values[1], receiver_values[2]};    
    prevpitch=map(rec[1],1000,2000,-20,20);
    if(prevpitch<5 && prevpitch>-5) prevpitch=0;

    prevyaw=-1*(map(rec[2],1000,2000,-20,20));
    //prevyaw=0;
    if(prevyaw<5 && prevyaw>-5) prevyaw=0; 
    if(prevpitch>5){
      
     throttle=(prevpitch*17)-85;
     if(throttle<0) throttle=0; if(throttle>255) throttle=255;
    }
    else {
      
        throttle=-1*((prevpitch*17)+85);
        if(throttle<0) throttle=0; if(throttle>255) throttle=255;
       
      
      
    }  
     
    Serial.print("                                         ");
    Serial.print(prevyaw);
    Serial.print(" , ");
    Serial.print(prevpitch);
    Serial.print(" , ");
    Serial.print(throttle);    
    Serial.println(" ");  
  return prevyaw, prevpitch, throttle;
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
        Serial.println(imu_error);
        while (1) {
          digitalWrite(enA, LOW);
          digitalWrite(enB, LOW);
        } 
        return 0;
    }
  }
}
void rightleftturning(uint32_t yawturn) {
 int incdelay;
	if(receiver_values[0]>1500) {
		
		if(yawturn>=3){
      Serial.println("                                                      sola gidiyor");
			if(yawturn>=3 && yawturn<6) incdelay=10;
			else if(yawturn>=6 && yawturn<10) incdelay=30;
			else if(yawturn>=10 && yawturn<13) incdelay=60;
			else  incdelay=100;
		
		
				soltek_val=throttle+incdelay;
				sagtek_val=throttle-incdelay;
        if(soltek_val>255) soltek_val=255; if(soltek_val<0) soltek_val=0;
        if(sagtek_val>255) sagtek_val=255; if(sagtek_val<0) sagtek_val=0;
				analogWrite(sagtek_pwm, sagtek_val);
        analogWrite(soltek_pwm, soltek_val);
        Serial.print("soltek_val:");
        Serial.print(soltek_val);
        Serial.println(" ");
        Serial.print("sagtek_val:");
        Serial.print(sagtek_val);
        Serial.println(" "); 
			
		}
		else if(yawturn<=-3){
      Serial.println("                                                      saga gidiyor");
			if(yawturn<=-3 && yawturn>-6) incdelay=10;
			else if(yawturn<=-6 && yawturn>-10) incdelay=30;
			else if(yawturn<=-10 && yawturn>-13) incdelay=60;
			else  incdelay=100;

			
			
				sagtek_val=throttle+incdelay;
				soltek_val=throttle-incdelay;
        if(soltek_val>255) soltek_val=255; if(soltek_val<0) soltek_val=0;
        if(sagtek_val>255) sagtek_val=255; if(sagtek_val<0) sagtek_val=0;
				analogWrite(sagtek_pwm, sagtek_val);
        analogWrite(soltek_pwm, soltek_val);
        Serial.print("                                       soltek_val:");
        Serial.print(soltek_val);
        Serial.println(" ");
        Serial.print("sagtek_val:");
        Serial.print(sagtek_val);
        Serial.println(" "); 
			
		}
	
  }
  else {
    analogWrite(sagtek_pwm, 0);
    analogWrite(soltek_pwm, 0);
    digitalWrite(enA, LOW);
    digitalWrite(enB, LOW);
  }
}

void landdrivecontrol(float prYaw, int imu_values){

	if(receiver_values[0]>1500) {
		//double yawError = prYaw - imu_values;
    //Serial.println(yawError);
		//yaw_control_signal =  getControlSignal(yawError, KP_yaw, KI_yaw, KD_yaw, yaw_pid_i, yaw_last_error, imu_values.DeltaTime);
		//yaw_last_error= yawError;
    
		sagtek_val= throttle;
		if(sagtek_val>255) sagtek_val=255; if(sagtek_val<0) sagtek_val=0;
		soltek_val= throttle;
		if(soltek_val>255) soltek_val=255; if(soltek_val<0) soltek_val=0;
    analogWrite(sagtek_pwm, sagtek_val);
    analogWrite(soltek_pwm, soltek_val);
    Serial.print("mot1:");
    Serial.print(sagtek_val);
    Serial.println(" ");
    Serial.print("mot2:");
    Serial.print(soltek_val);
    Serial.println(" "); 
		
	}
  else {
    analogWrite(sagtek_pwm, 0);
    analogWrite(soltek_pwm, 0);
    digitalWrite(enA, LOW);
    digitalWrite(enB, LOW);
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
  //initializeIMU();
  delay(2000);
  mot_calib_setup();

}

void loop() {
  //struct IMU_Values imuValues = GetIMU_Values();
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


  
  
    if( receiver_values[0]<1500) {
        Serial.println("Uçuşa geçildi");
          digitalWrite(enA, LOW);
          digitalWrite(enB, LOW);
          analogWrite(sagtek_pwm, 0);
          analogWrite(soltek_pwm, 0);
          
    }
           
       
    else {
      Serial.println("kara sürüşünde");
        
      
      
      if(prevyaw<3 && prevyaw>-3) {
        Serial.println("burada1");
        if(prevpitch>3){
          
          //prevyaw=imuValues.CurrentOrientation.YawAngle;
          
          digitalWrite(enA, HIGH);
          digitalWrite(enB, LOW);
          landdrivecontrol(0,0);
          
          
        }
        else if(prevpitch<-3){ 
          
          //prevyaw=imuValues.CurrentOrientation.YawAngle;
          digitalWrite(enA, LOW);
          digitalWrite(enB, HIGH);
          landdrivecontrol(0,0);
          
          
          
        }
        else {
          digitalWrite(enA, LOW);
          digitalWrite(enB, LOW);
          
        }
		  }
		  else {
        if(prevpitch>3){
          digitalWrite(enA, HIGH);
          digitalWrite(enB, LOW);
          
          rightleftturning(prevyaw);
        }
        else if(prevpitch<-3){
          digitalWrite(enA, LOW);
          digitalWrite(enB, HIGH);
         
          rightleftturning(prevyaw);
        }
        else {
          digitalWrite(enA, LOW);
          digitalWrite(enB, LOW);
        }  
      }
    }
  
}