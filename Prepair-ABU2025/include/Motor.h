#include <Arduino.h>
#include <RPi_Pico_TimerInterrupt.h>

#ifdef NUM_MOTORS
#else 
    #define NUM_MOTORS 4 
#endif

#define RADIUS_OF_WHEELS 39 //mm 
#define ENCODER_PER_REVOLUTION 3900 // tick per revolurion 

class Motor{
    private :
        volatile long lastEncoded;
        volatile long encoderCount;

        // velocity in this class mean angular velocity(RPM)
        long lastTime;
        long encoderOffset;
        
        // todo /////////////////////////////////////////////////////////////////// 
        volatile long lastEncoderCount;
        // volatile double velocity, rawVelocity, lastRawVelocity;
        long lastTimeCalculate = 0; // time between encoder tick 

        int motorID = 0;

        static Motor *Encoder[NUM_MOTORS];
        static void updateEncoderISR_ID0();
        static void updateEncoderISR_ID1();
        static void updateEncoderISR_ID2();
        static void updateEncoderISR_ID3();
        
     public :
        uint8_t PWM, IN1, IN2, ENC_A, ENC_B;
        Motor(uint8_t PWM, uint8_t IN1, uint8_t IN2, uint8_t ENCODER_A, uint8_t ENCODER_B);

        long getEncoder();
        double getPosition();
        double getVelocity();

        void updateVelocity();
        void info();
        void init(int motorID);
        void updateEncoder();
        void setPulse(int pluse);

        volatile double velocity = 0, rawVelocity =0 , prevRawVelocity1st = 0;
        volatile double prevVelocityFiltered = 0,prevVelocityFiltered2nd =0, prevRawVelocity2nd = 0;

        // todo ///////////////////////////////////////////////////////////////////
        void calculateVelocity();
};