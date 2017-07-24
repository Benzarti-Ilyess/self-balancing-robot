  #include <PID_v1.h>
  #include<Wire.h>
  #include <Kalman.h>
  #include <PID_AutoTune_v0.h>
  #define rad_to_deg 57.295786 // 1 rad=57 degree
  int moteurA=10,moteurB=11;
  int sensA=12,sensB=13;
  int mpu_addr=0x68; //adresse de mpu6050
  int16_t acy,acz,gyx;
  double angle_x,gyro_x_rate,comp_angle_x;
  double kp=0,ki=0,kd=0.0;
  double pid_pwm,angle_reference=-2.0,input;
  uint32_t timer; //enregistrer le temp
  int port_kp=3,port_ki=2,port_kd=1;
  Kalman kalmanx;
  PID pid(&input,&pid_pwm,&angle_reference,kp,ki,kd,DIRECT);
  PID_ATune aTune(&input, &pid_pwm);
  void setup() {
    // put your setup code here, to run once:
  pinMode(moteurA,OUTPUT);
  pinMode(moteurB,OUTPUT);
  pinMode(sensA,OUTPUT);
  pinMode(sensB,OUTPUT);
  Wire.begin();
  //Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x6B);//ecrire dans ce registre 0 pour desactiver le sleepmode pour 
  //avoir l acces de lire et ecrire
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);//vitesse de transmission terminal
  delay(100);
  timer=micros();
  }
  
  void loop() {
    pid.SetMode(AUTOMATIC);
    //lecture de l acc et le gyroscope
   Wire.beginTransmission(mpu_addr);
   Wire.write(0x3D);//adresse de registre acy_h 8bit
   Wire.endTransmission(false);
   Wire.requestFrom(mpu_addr,4);
   acy=Wire.read()<<8|Wire.read();
   acz=Wire.read()<<8|Wire.read();
   Wire.endTransmission(true);
   Wire.beginTransmission(mpu_addr);
   Wire.write(0x43); //adresse de registre gyx_h 8bit
   Wire.endTransmission(false);
   Wire.requestFrom(mpu_addr,2);
   gyx=Wire.read()<<8|Wire.read();
   
   //fin lecture;
  
   //calcul dt 
   double dt=(double)(micros()-timer)/1000000;
   timer=micros(); // rendre dt en sec 1sec=10^6 microsec
   //fin calcul dt
   
   // extraction des informations
   angle_x=atan2(acy,acz)*rad_to_deg; //calcul approximative de l angle x
   gyro_x_rate=gyx/131.0; //calcul vitesse angulaire en deg/sec 131 d apres datasheet
   //fin extraction
   
   //filtre complementaire
   //comp_angle_x=0.95*(comp_angle_x+gyro_x_rate*dt)+0.05*angle_x;
  comp_angle_x=kalmanx.getAngle(angle_x,gyro_x_rate,dt);
  Serial.print(comp_angle_x);//affichage de l angle apres filtrage
  kp = analogRead(port_kp);
  kd = analogRead(port_ki);
  kp = map(kp,0,1024,0,100);
  kd = 0.000988*kd*4;
  //kd=analogRead(port_kd);
  //kd=map(kd,0,1024,0,100)-3;
  Serial.print("    kp    ");
  Serial.print(kp);
  Serial.print("    ki ");
  Serial.print(ki);
  Serial.print("    kd ");
  Serial.print(kd);
  if(comp_angle_x>-2.0)
  { input=-comp_angle_x-4.0;
    pid.SetTunings(kp,ki,kd);
    pid.Compute();
    digitalWrite(sensB,HIGH);
   digitalWrite(sensA,HIGH);
   analogWrite(moteurA,pid_pwm);  
   analogWrite(moteurB,pid_pwm);
   Serial.print("   pwm sens1= ");
   Serial.println(pid_pwm);
  }
  else
  {
    input=comp_angle_x;
    pid.SetTunings(kp,ki,kd);
    pid.Compute();
  digitalWrite(sensB,LOW);
   digitalWrite(sensA,LOW);
   analogWrite(moteurA,pid_pwm);
   analogWrite(moteurB,pid_pwm);
    Serial.print("pwm sens2= ");
   Serial.println(pid_pwm);
   }
  
  }
