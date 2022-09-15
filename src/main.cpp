#include <Arduino.h>
#include "stmpu6050.h"
SMPU6050 mpu6050;


# define Enable       8            //D8                 //PORTB 0                    
# define Step_3       7            //D7                 //PORTD 7                    
# define Step_2       6            //D6                 //PORTD 6                    
# define Step_1       5            //D5                 //PORTD 5                    
# define Dir_3        4            //D4                 //PORTD 4                    
# define Dir_2        3            //D3                 //PORTD 3                    
# define Dir_1        2            //D2                 //PORTD 2  

void  pin_INI() {
  pinMode(Enable, OUTPUT);
  pinMode(Step_1, OUTPUT);
  pinMode(Step_2, OUTPUT);
  pinMode(Step_3, OUTPUT);
  pinMode(Dir_1, OUTPUT);
  pinMode(Dir_2, OUTPUT);
  pinMode(Dir_3, OUTPUT);
  digitalWrite(Enable, LOW);
}

//     HÀM KHAI BÁO TIMER2
void timer_INI() {
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000Hz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode Chế độ CTC bộ đếm được xóa về 0 khi giá trị bộ đếm (TCNT0) khớp với OCR0A
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
}


int8_t Dir_M1, Dir_M2, Dir_M3;                                         
volatile int Count_timer1, Count_timer2, Count_timer3;                       
volatile int32_t Step1, Step2, Step3;
int16_t Count_TOP1, Count_BOT1, Count_TOP2, Count_BOT2, Count_TOP3, Count_BOT3;  
float input, Offset, Output, I, input_last, OutputL, OutputR, M_L, M_R, MotorL, MotorR;




//     CHƯƠNG TRÌNH NGẮT CỦA TIMER2
//....................................
ISR(TIMER2_COMPA_vect) {

  if (Dir_M1 != 0) {                                                          //nếu MOTOR cho phép quay
    Count_timer1++;
    if (Count_timer1 <= Count_TOP1)PORTD |= 0b00100000;                        
    else PORTD &= 0b11011111;                                                 
    if (Count_timer1 > Count_BOT1) {
      Count_timer1 = 0;                          
      if (Dir_M1 > 0)Step1++;
      else if (Dir_M1 < 0)Step1--;
    }
  }


  if (Dir_M2 != 0) {
    Count_timer2++;
    if (Count_timer2 <= Count_TOP2)PORTD |= 0b01000000;
    else PORTD &= 0b10111111;
    if (Count_timer2 > Count_BOT2) {
      Count_timer2 = 0;
      if (Dir_M2 > 0)Step2++;
      else if (Dir_M2 < 0)Step2--;
    }
  }

  if (Dir_M3 != 0) {
    Count_timer3++;
    if (Count_timer3 <= Count_TOP3)PORTD |= 0b10000000;
    else PORTD &= 0b01111111;
    if (Count_timer3 > Count_BOT3) {
      Count_timer3 = 0;
      if (Dir_M3 > 0)Step3++;
      else if (Dir_M3 < 0)Step3--;
    }
  }
}


//     HÀM TỐC ĐỘ DI CHUYỂN MOTOR1
//....................................
void Speed_M1(int16_t x) {
  if (x < 0) {
    Dir_M1 = -1;
    PORTD &= 0b11111011;
  }
  else if (x > 0) {
    Dir_M1 = 1;
    PORTD |= 0b00000100;
  }
  else Dir_M1 = 0;

  Count_BOT1 = abs(x);
  Count_TOP1 = Count_BOT1 / 2;
}

//     HÀM TỐC ĐỘ DI CHUYỂN MOTOR2
//....................................
void Speed_L(int16_t x) {
  if (x < 0) {
    Dir_M2 = -1;
    PORTD &= 0b11110111;
  }
  else if (x > 0) {
    Dir_M2 = 1;
    PORTD |= 0b00001000;
  }
  else Dir_M2 = 0;

  Count_BOT2 = abs(x);
  Count_TOP2 = Count_BOT2 / 2;
}

//     HÀM TỐC ĐỘ DI CHUYỂN MOTOR3
//....................................
void Speed_R(int16_t x) {
  if (x < 0) {
    Dir_M3 = -1;
    PORTD &= 0b11101111;
  }
  else if (x > 0) {
    Dir_M3 = 1;
    PORTD |= 0b00010000;
  }
  else Dir_M3 = 0;

  Count_BOT3 = abs(x);
  Count_TOP3 = Count_BOT3 / 2;
}


void setup() {
  mpu6050.init(0x68);
  Serial.begin(9600);         
  pin_INI();                      
  timer_INI();                 
  delay(500);
}

void loop() {
  float Kp=22;
  float Ki=0.99;
  float Kd=0;
  float AngleY = mpu6050.getYAngle();
  Serial.println(AngleY);
  Offset = 0;
  input = AngleY + Offset;
  I += input;
  Output = Kp * input + Ki * I + Kd * (input - input_last);
  input_last = input;
  if (Output > -5 && Output < 5)Output = 0;
  Output = constrain(Output, -400, 400);
  OutputL = Output;
  OutputR = Output;

  if (Output > 0) {
    M_L = 405 - (1 / (OutputL + 9)) * 5500;
    M_R = 405 - (1 / (OutputR + 9)) * 5500; //OutputR = 1    ----> M_R = -145
                 //OutputR = 4.58 ----> M_R = 0
                 //OutputR = 10   ----> M_R = 115.52
                 //OutputR = 400  ----> M_R = 391.55
  }
  else if (Output < 0) {
    M_L = -405 - (1 / (OutputL - 9)) * 5500;
    M_R = -405 - (1 / (OutputR - 9)) * 5500;
  }
  else {
    M_L = 0;
    M_R = 0;
  }
  if (M_L > 0)MotorL = 400 - M_L;
  else if (M_L < 0)MotorL = -400 - M_L;
  else MotorL = 0;
  if (M_R > 0)MotorR = 400 - M_R;
  else if (M_R < 0)MotorR = -400 - M_R;
  else MotorR = 0;
  Speed_L(MotorL);
  Speed_R(MotorR);
}