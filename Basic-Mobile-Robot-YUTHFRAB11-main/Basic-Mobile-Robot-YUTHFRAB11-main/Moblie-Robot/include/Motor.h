#include <Arduino.h>

class Motor{
    private :
        volatile long encoderCount, lastEncoded;
        static Motor* Encoder;
        uint8_t PWM, IN1, IN2, ENC_A, ENC_B;
        static void updateEncoderISR();
    public :
        Motor(uint8_t PWM, uint8_t IN1, uint8_t IN2, uint8_t ENCODER_A, uint8_t ENCODER_B);
        void position();
        void updateEncoder();
};