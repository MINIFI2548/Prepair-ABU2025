#include <Arduino.h>
#include "Motor.h"
#include "config.h"
#include "RPi_Pico_TimerInterrupt.h"
#include "RPi_Pico_ISR_Timer.h"

#define KP_SPEED_CONTROLL 0.03

Motor frontLeftMotor  ( FORNT_LEFT_MOTOR_PWM, FORNT_LEFT_MOTOR_IN1, FORNT_LEFT_MOTOR_IN2, 
                        FORNT_LEFT_MOTOR_ECA, FORNT_LEFT_MOTOR_ECB  );
Motor frontRightMotor (  FORNT_RIGHT_MOTOR_PWM, FORNT_RIGHT_MOTOR_IN1, FORNT_RIGHT_MOTOR_IN2, 
                        FORNT_RIGHT_MOTOR_ECA, FORNT_RIGHT_MOTOR_ECB  );
Motor backLeftMotor   ( BACK_LEFT_MOTOR_PWM, BACK_LEFT_MOTOR_IN1, BACK_LEFT_MOTOR_IN2, 
                        BACK_LEFT_MOTOR_ECA, BACK_LEFT_MOTOR_ECB  );
Motor backRightMotor  (  BACK_RIGHT_MOTOR_PWM, BACK_RIGHT_MOTOR_IN1, BACK_RIGHT_MOTOR_IN2, 
                        BACK_RIGHT_MOTOR_ECA, BACK_RIGHT_MOTOR_ECB  );

volatile double curruntPosiX, curruntPosiY, yaw;
long lastTime;

void upDatePosition(){
  curruntPosiY = RADIUS_OF_WHEELS * (frontLeftMotor.getEncoder() + frontRightMotor.getEncoder() + backLeftMotor.getEncoder() + backRightMotor.getEncoder()) * PI / 78000;
  curruntPosiX = RADIUS_OF_WHEELS * (frontLeftMotor.getEncoder() - frontRightMotor.getEncoder() - backLeftMotor.getEncoder() + backRightMotor.getEncoder()) * PI / 78000; 
  yaw = RADIUS_OF_WHEELS * 180 * (frontRightMotor.getPosition() - frontLeftMotor.getPosition() - backLeftMotor.getPosition() + backRightMotor.getPosition()) / (PI * 4 * (DISTANCE_FROM_WHEELS_X + DISTANCE_FROM_WHEELS_Y));
}

void wait_OK() {
  pinMode(OK_BUTTOM_PIN, INPUT_PULLUP);
  while(digitalRead(OK_BUTTOM_PIN)) {};
}

void showInfo(){
      frontLeftMotor.updateVelocity();
      frontRightMotor.updateVelocity();
      backLeftMotor.updateVelocity();
      backRightMotor.updateVelocity();
      if(millis() - lastTime > 250){
      Serial.print("Position X : ");
      Serial.print(curruntPosiX);
      Serial.print("\tPosition Y : ");
      Serial.print(curruntPosiY);
      Serial.print("\tYax : ");
      Serial.println(yaw);

      Serial.print("FL Velocity : ");
      Serial.print(frontLeftMotor.getVelocity());
      Serial.print("\tFR Velocity : ");
      Serial.println(frontRightMotor.getVelocity());
      Serial.print("BL Velocity: ");
      Serial.print(backLeftMotor.getVelocity());
      Serial.print("\tBR Velocity : ");
      Serial.println(backRightMotor.getVelocity());
      Serial.println("\n###############################################################\n");
      lastTime = millis();
    }
}

RPI_PICO_Timer motorVelocityClock(0);

bool motorVelocityCalculate(struct repeating_timer *t)
{
  (void) t;
  frontLeftMotor.calculateVelocity();
  frontRightMotor.calculateVelocity();
  backLeftMotor.calculateVelocity();
  backRightMotor.calculateVelocity();
  return true;
}
void setup() {
  frontLeftMotor.init(0);
  frontRightMotor.init(1);
  backLeftMotor.init(2);
  backRightMotor.init(3);
  // wait_OK();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(OK_BUTTOM_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  motorVelocityClock.attachInterrupt(1000, motorVelocityCalculate);
}
 
void loop() {
  for(int i = 1; i <= 20; i++){
    if((((millis() % 10000) / 500) > i) && (millis() % 10000) < 5000){
      frontRightMotor.setPulse(i*25.5);
    }else if(((((millis() % 10000) / 500) > i) && (millis() % 10000) > 5000)){
      frontRightMotor.setPulse(510 - (i*25.5));
    }
  }

  Serial.print(">");
  Serial.print("Raw_velocity: ");
  Serial.print(frontRightMotor.rawVelocity);
  Serial.print(",");
  Serial.print("filter_velocity: ");
  Serial.print(frontRightMotor.velocity);
  Serial.println();

  delay(10);
}