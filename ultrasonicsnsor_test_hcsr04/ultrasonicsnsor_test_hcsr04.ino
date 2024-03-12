#define echo 39
#define triq 41
float mesafe;
float sure; 
float sicalik = 0;
float hiz ;
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
  
  Serial.print("mesafe:");
  Serial.print(mesafe);
  Serial.println(" ");
  
  return mesafe;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(triq,OUTPUT);
  pinMode(echo,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
mesafe_yer();
}
