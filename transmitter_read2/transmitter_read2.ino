int receiver_pins[] = {A0, A1, A2, A3, A4, A5};
long int receiver_values[] = {0, 0, 0, 0, 0, 0};
long int throttle=0, prevyaw=0, prevpitch=0, prevroll=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

/*  for(int i = 0; i <6; i++){

    Serial.print(receiver_pins[i] + ' ' + receiver_values[i] + '\n');

   }
Serial.println(" ");*/
     for(int i=0; i<6; i++) {
      receiver_values[i]=pulseIn (receiver_pins[i], HIGH, 30000);
     }
     long int rec[]= { receiver_values[0], receiver_values[1], receiver_values[2], receiver_values[3]};    
     //for(int i=0; i<6; i++) {
     //  rec[i]=map(rec[i],1000,2000,0,180);
     //}
    
    // throttle=map(rec[2],5,202,0,180);
    // if(throttle<20) throttle=0;
    // if(throttle>180) throttle=180; 
    // prevyaw=map(rec[0],36,144,-20,20);
    // if(prevyaw<6 && prevyaw>-6) prevyaw=0;
    // else if(prevyaw>20) prevyaw=20;
    // else if(prevyaw<-20) prevyaw=-20;
    // prevpitch=map(rec[1],44,150,-20,20);
    // if(prevpitch<6 && prevpitch>-6) prevpitch=0;
    // else if(prevpitch>20) prevpitch=20;
    // else if(prevpitch<-20) prevpitch=-20;
    // prevroll=map(rec[3],30,145,-20,20);
    // if(prevroll<6 && prevroll>-6) prevroll=0; 
    // else if(prevroll>20) prevroll=20;
    // else if(prevroll<-20) prevroll=-20;
    Serial.print(receiver_values[0]); 
     Serial.print(" , ");
     Serial.print(receiver_values[1]); 
     Serial.print(" , ");
     Serial.print(receiver_values[2]);
     Serial.print(" , ");
     Serial.print(receiver_values[3]);
     Serial.print(" , ");
     Serial.print(receiver_values[4]);
     Serial.print(" , ");
     Serial.println(receiver_values[5]);
}
