#include <Arduino.h>
#include <Motor.h>
#include <RPi_Pico_TimerInterrupt.h>

Motor *Motor::Encoder[] = {nullptr, nullptr, nullptr, nullptr};

Motor :: Motor(uint8_t PWM, uint8_t IN1, uint8_t IN2, uint8_t ENCODER_A, uint8_t ENCODER_B){
    this -> PWM = PWM;
    this -> IN1 = IN1;
    this -> IN2 = IN2;
    this -> ENC_A = ENCODER_A;
    this -> ENC_B = ENCODER_B;
}   

void Motor :: updateEncoder() {
    // เข้ารหัสค่า
    int encoded = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
    int sum = (lastEncoded << 2) | encoded;
    int increment;

    // อัปเดตค่า encoderValue ตามทิศทางการหมุน
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
      increment = 1; 
    }
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
      increment = -1;
    }
    this -> encoderCount += increment;
    lastEncoded = encoded;
}

void Motor::updateEncoderISR_ID0(){
    if(Encoder[0] != nullptr){
        Encoder[0] -> updateEncoder();
    }
}
void Motor::updateEncoderISR_ID1(){
    if(Encoder[1] != nullptr){
        Encoder[1] -> updateEncoder();
    }
}
void Motor::updateEncoderISR_ID2(){
    if(Encoder[2] != nullptr){
        Encoder[2] -> updateEncoder();
    }
}
void Motor::updateEncoderISR_ID3(){
    if(Encoder[3] != nullptr){
        Encoder[3] -> updateEncoder();
    }
}

long Motor :: getEncoder() { 
    return this->encoderCount;
}

double Motor :: getPosition() { 
    return (this -> encoderCount) * 2 * PI / ENCODER_PER_REVOLUTION;
}

double Motor :: getVelocity(){
    return velocity;
}

void Motor :: info() { 
    // Serial.print("IN1 -> ");
    // Serial.print(IN1);
    // Serial.print(" | IN2 -> ");
    // Serial.print(IN2);
    // Serial.print(" | PWM -> ");
    // Serial.print(PWM);
    // Serial.print("\n EncoderA -> ");
    // Serial.print(ENC_A);
    // Serial.print(" | EncoderB -> ");
    // Serial.print(ENC_B);
    // Serial.println();
    Serial.println((long)&Encoder);
}

void Motor :: init(int motorID) {
    this -> motorID = motorID;
    pinMode(IN1, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    Encoder[motorID] = this;
    switch (motorID){
    case 0:
        attachInterrupt(digitalPinToInterrupt(ENC_A), Motor :: updateEncoderISR_ID0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENC_B), Motor :: updateEncoderISR_ID0, CHANGE);
        break;
    case 1:
        attachInterrupt(digitalPinToInterrupt(ENC_A), Motor :: updateEncoderISR_ID1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENC_B), Motor :: updateEncoderISR_ID1, CHANGE);
        break;
    case 2:
        attachInterrupt(digitalPinToInterrupt(ENC_A), Motor :: updateEncoderISR_ID2, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENC_B), Motor :: updateEncoderISR_ID2, CHANGE);
        break;
    case 3:
        attachInterrupt(digitalPinToInterrupt(ENC_A), Motor :: updateEncoderISR_ID3, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENC_B), Motor :: updateEncoderISR_ID3, CHANGE);
        break;
    default:
        break;
    }
}

void Motor :: setPulse(int pulse) { 
    digitalWrite(IN1, pulse >= 0);
    digitalWrite(IN2, pulse <= 0);
    if (pulse == 0) {
        analogWrite(PWM, 255);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, HIGH);
    }
    else {
        analogWrite(PWM, abs(pulse));
    }
}

void Motor :: calculateVelocity(){
    long currentTime = millis(); // ms
    long deltaTime = currentTime - lastTimeCalculate; // ms
    
    /*
    deltaTime / 1000 = 1 sec
    multiply by 60 to convert to minutes
    divide by 4000 to convert encoder to round
    velocity = (encoderCount -  lastEncoderCount) * 1000 / deltaTime * 60 / 4000;
    */
    rawVelocity = (abs(encoderCount) - abs(lastEncoderCount) > 0) ? (encoderCount -  lastEncoderCount) * 15.0 / (deltaTime) : rawVelocity; // Reduced equations
    

    // ! low pass filter 
    float a[] ={1.9112,  -0.9150};
    float b[] ={0.0009, 0.0019, 0.0009};
    velocity = a[0] * prevVelocityFiltered + a[1] * prevVelocityFiltered2nd  + b[0] * rawVelocity + b[1] * prevRawVelocity1st + b[2] * prevRawVelocity2nd;


    lastEncoderCount = encoderCount;
    lastTimeCalculate = currentTime;
    prevRawVelocity2nd = prevRawVelocity1st;
    prevRawVelocity1st = rawVelocity;
    prevVelocityFiltered2nd = prevVelocityFiltered;
    prevVelocityFiltered = velocity;
} 