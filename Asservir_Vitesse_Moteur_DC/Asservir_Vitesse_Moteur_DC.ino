/*
Date de modification : 29/01/2024
Asservissement en vitesse d'un moteur DC

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

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

// Low-pass filter constants
float alpha = 0.08;  // Adjust this value based on the desired cutoff frequency
float filteredSpeed = 0;

// PID constants for speed control
float kp = 1;   
float kd = 0.01;
float ki = 0.1;

int prevPos = 0; // Previous position for speed calculation

void readEncoder(); 

void setup() {
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWMPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  Serial.begin(9600);
}

void loop() {
  // Set target speed (number of encoder pulses per sampling interval)
  int targetSpeed = 200; // Adjust this value based on your requirements

  // Time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6; // Sampling time in seconds
  prevT = currT;

  // Calculate current speed
  int currentSpeed = (pos - prevPos) / deltaT;
  prevPos = pos;

   // Low-pass filter to smooth the speed signal
  filteredSpeed = alpha * currentSpeed + (1 - alpha) * filteredSpeed;


  // Error in speed
  int speedError = targetSpeed - filteredSpeed;

  // Derivative
  float dedt = (speedError - eprev) / deltaT;

  // Integral
  eintegral = eintegral + speedError * deltaT;

  // Control signal
  float u = kp * speedError + kd * dedt + ki * eintegral;

  // Motor power to turn the motor with
  float pwr = fabs(u);
  if (pwr > 255) pwr = 255;

  // Motor direction
  int dir = 1;
  if (u < 0) dir = -1;

  // Signal the motor
  setMotor(dir, pwr, PWMPin, IN1, IN2);

  // Store previous error
  eprev = speedError;

  Serial.print("Target Speed: ");
  Serial.print(targetSpeed);
  Serial.print(" Current Speed: ");
  Serial.println(currentSpeed);
  delay(5);
}

void setMotor(int dir, int pwmVal, int pwmPin, int in1, int in2) {
  analogWrite(pwmPin, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);  
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) pos++;
  else pos--;
}
