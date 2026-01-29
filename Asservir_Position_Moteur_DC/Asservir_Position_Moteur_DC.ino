/*
Date de modification : 29/01/2024
Asservissement en position d'un moteur DC

Club E-Gab Centrale Marseille
Coupe de France de la robotique
Adnane Lamnaouar

PID inspiré de Curio Res

Plateforme : ESP8266
*/

#define ENCA 2 // White
#define ENCB 3 // Yellow
#define PWMPin 10

#define IN1 11
#define IN2 12
//chaque tour (c a d 360 degres) corr a 158 excitation du capteur
//pour convertir en degres on multiplie par : 360/158 = 2.27
int pos= 0;
long prevT=0;
float eprev =0;
float eintegral=0;

//PID constants
float kp=8;   
float kd=0.02;
float ki=0.01;

void  readEncoder();


void setup() {

pinMode(ENCA, INPUT);
pinMode(ENCB, INPUT);

pinMode(IN1,OUTPUT);

pinMode(IN2,OUTPUT);
pinMode(PWMPin,OUTPUT);

attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

Serial.begin(9600);
}

void loop() {
//set target position
int target =400;  // c est 1362 degres

//time difference
long currT = micros();
long et = currT-prevT;
float deltaT= ((float)(currT-prevT))/1.0e6;  //TE echantilonnage il est le temps pour revenir
prevT = currT;

//error
int e= (target-pos);

//derivative
float dedt = (e-eprev)/(deltaT);

//integral
eintegral = eintegral + e*deltaT;

//control signal
float u = kp*e + kd*dedt + ki*eintegral;

//motor power to turn the motor with
float pwr = fabs(u);   //donne la valeur absolue
if(pwr > 255){   // 
  pwr = 255;
}

//motor direction
int dir = 1;
if(u<0){
  dir = -1;

}

//signal the motor
setMotor(dir,pwr,PWMPin,IN1,IN2);

//store previous error
eprev = e;

Serial.print(target);
Serial.print (" ");
Serial.print(pos);
Serial.println();

}




void setMotor(int dir, int pwmVal, int pwmPin, int in1, int in2){
  analogWrite(pwmPin,pwmVal);
  if(dir==1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);  
    }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    }
  }


void readEncoder(){
  int b= digitalRead(ENCB);
  if(b>0){
    pos++;
    }
   else {
    pos--;
    }
  //Serial.println(pos);
  }
