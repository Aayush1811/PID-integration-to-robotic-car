#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

#define r 11
#define r1 12
#define r2 10
#define l 6
#define l1 7
#define l2 8

float angle;
float sp = 180;
float pp;
float error;

float kp = 7;
float ki = 0.000001;
float kd= 20 ;

float P,I,D;
float integral = 0;
float p_error = 0;

float op;
float pid_op;

float speed;

float pid_cal(float error, float kp, float ki, float kd)
{
  // P
  P = error*kp;

  //I
  integral = integral + error;
  I = integral*ki;

  //D
  D = error*(error - p_error);
  p_error = error;

  op = P + I + D;

  return op;

}


void setup()
{
  Serial.begin(9600);
  pinMode(r,OUTPUT);
  pinMode(r1,OUTPUT);
  pinMode(r2,OUTPUT);
  pinMode(l,OUTPUT);
  pinMode(l1,OUTPUT);
  pinMode(l2,OUTPUT);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {


  while(true)
  {

    mpu6050.update();
  angle = mpu6050.getAngleZ();

  
  pp = angle;
  error = sp - pp;
  

  pid_op = pid_cal(error, kp, ki , kd);
  if(pid_op >= 100)
  {
    pid_op = 100;
  }
  if(pid_op<=-100)
  {
    pid_op = -100;
  }


  speed = map(pid_op, 0 , 100, 50, 255);
  speed = abs(speed);


  if(error>0)
  {
    analogWrite(r,speed);
    analogWrite(l,speed);

    digitalWrite(r1,LOW);
    digitalWrite(r2,HIGH);
    digitalWrite(l1,LOW);
    digitalWrite(l2,HIGH);

    // if(error>-2 && error<2)
    // {
    //   analogWrite(r,0);
    //   analogWrite(l,0);
    //   break;

    // }

  }
  else if(error<0)
  {
    analogWrite(r,speed);
    analogWrite(l,speed);

    digitalWrite(r2,LOW);
    digitalWrite(r1,HIGH);
    digitalWrite(l2,LOW);
    digitalWrite(l1,HIGH);

    // if(error>-2 && error<2)
    // {
    //   analogWrite(r,0);
    //   analogWrite(l,0);
    //   break;

    // }

  }



  Serial.print("pp: ");
  Serial.print(angle);
  Serial.print("  sp: ");
  Serial.print(sp);
  Serial.print("  error: ");
  Serial.print(error);
  Serial.print("  op: ");
  Serial.print(pid_op);
  Serial.print("  speed: ");
  Serial.println(speed);

  }
  
}
