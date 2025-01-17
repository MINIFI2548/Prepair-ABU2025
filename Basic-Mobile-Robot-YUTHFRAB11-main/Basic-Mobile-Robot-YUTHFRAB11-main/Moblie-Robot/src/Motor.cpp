#include <Arduino.h>
#include <Motor.h>

Motor* Motor::Encoder = nullptr;

Motor :: Motor(uint8_t PWM, uint8_t IN1, uint8_t IN2, uint8_t ENCODER_A, uint8_t ENCODER_B){
    this -> PWM = PWM;
    this -> IN1 = IN1;
    this -> IN2 = IN2;
    this -> ENC_A = ENCODER_B;
    this -> ENC_B = ENCODER_B;

    
    pinMode(IN1, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    Encoder = this;
    attachInterrupt(digitalPinToInterrupt(ENC_A) , Motor :: updateEncoderISR  , CHANGE);
    // attachInterrupt(digitalPinToInterrupt(ENC_B), Motor :: updateEncoder, CHANGE);
}   

void Motor :: updateEncoder() {
  // อ่านค่า A และ B จาก encoder
  bool MSB = digitalRead(ENC_A); // ค่า bit สูงสุด
  bool LSB = digitalRead(ENC_A); // ค่า bit ต่ำสุด

  // เข้ารหัสค่า
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // อัปเดตค่า encoderValue ตามทิศทางการหมุน
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount--;

  // บันทึกค่า encoded ล่าสุด
  lastEncoded = encoded;
}

void Motor::updateEncoderISR(){
    if(Encoder != nullptr){
        Encoder -> updateEncoder();
    }
}