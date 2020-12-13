int RPWM=4;
int LPWM=13;
// timer 0
int R_EN=6;
int L_EN=7;


//arm encoder-interrupt
const int interruptPinA1 = 2;
const int interruptPinB1 = 3;
//motor encoder-interrupt
const int interruptPinA2 = 18;
const int interruptPinB2 = 19;

//pushbutton
const int interruptPinbut = 20;
const int interruptPinbut2 = 21;

unsigned long time1 = 0;
unsigned long time2 = 0;
unsigned long time3 = 0;
unsigned long time4 = 0;

int warning = 0; //test condtition
int Count1 = 0.0; //arm encoder counter
int Count2 = 0.0; //motor encoder counter
int button = 0; //pushbutton

double degree1 = 0.0; //arm encoder degree
double degree2 = 0.0; //motor encoder degree
double predeg2;
double predeg1;

double degref_old = 0.0; // old position of end effector
double degref_new; //new position of end effector
double delta_deg;
double keq = 10.9; //10.83; //equal k spring
double deg0 = 0.05; //a const value for opposing force

double torqj; //joint torque

double velocity = 0; //velocity of motor

//pd controler
double setpoint_pd, input_pd, output_pd;
double Kd = 240, Ki = 0, Bd = 0;
unsigned long currentTime_pd ;
unsigned long previousTime_pd = 0;
double elapsedTime_pd;
double error_pd = 0.0;
double lastError_pd;
double rateError_pd, rateError_pd1, rateError_pd2;
double alfa2 = 0.9;


//pi1 controler- this pi controll Afect on speed
double setpoint_pi1, input_pi1, output_pi1;
double Kp1 = 5.5, Ki1 = 2.7, Kd1 = 0;
unsigned long currentTime_pi1 = 0;
unsigned long previousTime_pi1 = 0;
double elapsedTime_pi1;
double error_pi1 = 0.0;
double lastError_pi1;
double cumError_pi1 = 0.0;
double rateError_pi1 = 0.0 ;


//pi2 controler - this pi controll Afect on torq
double setpoint_pi2, input_pi2, output_pi2;
double Kp2 = 1.5, Ki2 = 0.7, Kd2 = 0; 
unsigned long currentTime_pi2 = 0;
unsigned long previousTime_pi2 = 0;
double elapsedTime_pi2;
double error_pi2 = 0.0;
double lastError_pi2;
double cumError_pi2 = 0.0;

int n = 1;
double x;
int pwm ;


void setup() {
  TCCR4B = TCCR4B & B11111000 | B00000001;
  TCCR0B = TCCR0B & B11111000 | B00000001;
  Serial.begin(19200);
  pinMode(interruptPinA1, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinA1), func1, RISING);
  pinMode(interruptPinB1, INPUT);
  //  attachInterrupt(digitalPinToInterrupt(interruptPinB1), func, RISING);
  pinMode(interruptPinA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinA2), func2, RISING);
  pinMode(interruptPinB2, INPUT);
  //  attachInterrupt(digitalPinToInterrupt(interruptPinB2), func, RISING);
  pinMode(interruptPinbut, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinbut), butfunc, RISING);
  pinMode(interruptPinbut2, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinbut2), butfunc2, FALLING);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  predeg2 = degree2;
  
  button = 0;
  time1 = time2;
//  degref_old = degref_new;
}

void loop() {
//  time3 = millis();
  degree1 = Count1 * 0.00314 ; //radian
  degree2 = Count2 * 0.00314 ; //radian
  ///////motor velocity
  time2 = millis();
  if ((time2 - time1) > 50 ) {
    velocity = (degree2 - predeg2) / ((double(time2 - time1)) / 500);
    time1 = time2;
  }
  predeg2 = degree2;

  //// joint  torque = torque(t) - torque(t - 1)
  torqj = (degree2 - degree1) * keq; //joint torque


  //////////////  CONTROLER   ////////////////////

  //CONDITIONS PARTS
  delta_deg = degree1 - degref_old ;
  if (delta_deg > deg0) { 
    output_pd = Kd * (-1 * deg0);
    degref_new = degree1 - deg0;
    lastError_pd = -1 * deg0;
  }
  else if (delta_deg < (-1 * deg0)) {
    output_pd = Kd * deg0;
    degref_new = degree1 + deg0;
    lastError_pd = deg0;
  }
  else {
    error_pd = degref_new - degree1;
    rateError_pd = (error_pd - lastError_pd) / 50;
    lastError_pd = error_pd;
    rateError_pd2 = alfa2 * rateError_pd1 + (1 - alfa2) * rateError_pd ;
    rateError_pd1 = rateError_pd2;
    output_pd = error_pd * Kd + Bd * rateError_pd2;
  }

  /////////// pi1 control /////////
  input_pi1 = torqj;
  setpoint_pi1 = output_pd;
  output_pi1 = computePI1(input_pi1, setpoint_pi1);

  /////////// pi2 control /////////
  input_pi2 = velocity;
  setpoint_pi2 = output_pi1;
  output_pi2 = computePI2(input_pi2, setpoint_pi2);
  time3 = millis();
  if (time3 - time4 >= 1500) {
    cumError_pi1 = 0;
    cumError_pi2 = 0;
    time4 = time3;
  }

  //  x = output_pi2 - torqj;
  x = output_pi2 + (torqj);
  if (x >= 110) {
    x = 110;
  }
  else if (x <= (-110)) {
    x = -110;
  }

  pwm = abs(x);
  //  pwm = map (pwm,0,27,0,255);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  if (x > 0) {
    motorfor(pwm);
  }
  else if (x < 0) {
    motorback(pwm);
  }
  else {
    motorstop();
  }

//
//  Serial.println( String(Count1) + "/ +" + String(button) + "*" + " Count2: " + String(Count2) + "  degree1: " + String(degree1) + "  degree2: "
//                  + String(degree2) + " torqj: " + String(torqj) + " output_pd: " + String(output_pd) + " output_pi1: " + String(output_pi1)
//                  + "output_pi2: " + String(output_pi2) +  " velocity " + String(velocity) + " x: " + String(x) + " elapsedTime_pi1: " + String(elapsedTime_pi1)
//                  + " elapsedTime_pd: " + String(elapsedTime_pd) + " degref_old: " + String(degref_old)
//                  + " degref_new: " + String(degref_new) + " delta_deg: " + String(delta_deg) + " warning: " + String(warning) );
//    Serial.print(torqj);
//    Serial.print("\t");
    Serial.println(x);
//    Serial.println(output_pi2);
  
  
  degref_old = degref_new;
//  Serial.println(time3 - millis());
  delay(200);
}

//double computePD(double inpd, double Setpoint_pd) {
//  double out_pd = 0.0;
//  //  currentTime_pd = millis();                //get current time
//  //  elapsedTime_pd = double(currentTime_pd - previousTime_pd);        //compute time elapsed from previous computation
//  elapsedTime_pd = 50.0;
//  error_pd = Setpoint_pd - inpd;                                // determine error
//  rateError_pd = (error_pd - lastError_pd) / ((elapsedTime_pd) / 500) ; // compute derivative
//
//  out_pd = Kd * error_pd + Bd * rateError_pd;            //PID output
//
//  lastError_pd = error_pd;                                //remember current error
//  previousTime_pd = currentTime_pd;                        //remember current time
//
//  return out_pd;                                        //have function return the PID output
//}

double computePI1(double inpi1, double Setpoint_pi1) {
  double out_pi1 = 0.0;
//    currentTime_pi1 = millis();                //get current time
//    elapsedTime_pi1 = double(currentTime_pi1 - previousTime_pi1);        //compute time elapsed from previous computation
  elapsedTime_pi1 = 50.0;
  error_pi1 = Setpoint_pi1 - inpi1;                                // determine error
  cumError_pi1 += (error_pi1 * (elapsedTime_pi1 / 500));              // compute integral
  rateError_pi1 = (error_pi1 - lastError_pi1) / ((elapsedTime_pi1) / 500);
  
  out_pi1 = (Kp1*error_pi1) + (Ki1*cumError_pi1) + (Kd1*rateError_pi1);            //PID output

  lastError_pi1 = error_pi1;                                //remember current error
  previousTime_pi1 = currentTime_pi1;                        //remember current time

  return out_pi1;                                        //have function return the PID output
}

double computePI2(double inpi2, double Setpoint_pi2) {
  double out_pi2 = 0.0;
//    currentTime_pi2 = millis();                //get current time
//    elapsedTime_pi2 = double(currentTime_pi2 - previousTime_pi2);        //compute time elapsed from previous computation
  elapsedTime_pi2 = 50.0;
  error_pi2 = Setpoint_pi2 - inpi2;                                // determine error
  cumError_pi2 += (error_pi2 * (elapsedTime_pi2 / 500));              // compute integral

  out_pi2 = Kp2 * error_pi2 + Ki2 * cumError_pi2;            //PID output

  lastError_pi2 = error_pi2;                                //remember current error
  previousTime_pi2 = currentTime_pi2;                        //remember current time

  return out_pi2;                                        //have function return the PID output
}




void func1() {
  if (digitalRead(interruptPinB1) == HIGH) {
    Count1 += 1;
  }
  else if (digitalRead(interruptPinB1) == LOW) {
    Count1 -= 1;
  }
  else {
    Count1 = Count1;
  }
}

void func2() {
  if (digitalRead(interruptPinB2) == HIGH) {
    Count2 -= 1;
  }
  else if (digitalRead(interruptPinB2) == LOW) {
    Count2 += 1;
  }

  else {
    Count2 = Count2;
  }
}

void butfunc() {
  button = 0;

}
void butfunc2() {
  button = 1;

}

void motorfor(int rpm) {

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  analogWrite(RPWM, rpm);
  
}
void motorback(int rpm) {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  analogWrite(LPWM, rpm);

}
void motorstop() {
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}
