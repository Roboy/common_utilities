#include <SPI.h>
#include "ardprintf.h"

static const int spiClk = 2000000; // 2 MHz

const int ss_n[2] = {0,2};
#define POSITION 0
#define VELOCITY 1
#define DISPLACEMENT 2
#define DIRECT_PWM 3
#define maxPWM 500

float Kp[2] = {0.1,0.1}, Kd[2] = {0.01,0.01}, err[2] = {0,0}, err_prev[2] = {0,0}, setpoint[2] = {300,0}, result[2] = {0,0};
int control_mode[2] = {0,3};

int32_t position[2] = {0,0};
int16_t velocity[2] = {0,0}, current[2] = {0,0}, pwmRef[2] = {0,0};

void setup() {
  Serial.begin(115200);
  Serial.println("welcome stranger");
  SPI.begin();
  for(int motor=0;motor<2;motor++){
    pinMode (ss_n[motor], OUTPUT);
    digitalWrite(ss_n[motor],HIGH);
  }
}

void loop() {
  Serial.println("loopnoob");
  for(int motor=0;motor<2;motor++){
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
    uint16_t data[2];
    for(int i=0;i<12;i++){
      digitalWrite(ss_n[motor],LOW);
      delayMicroseconds(1);
      if(i==0)
        SPI.transfer16(0x8000); // header
      else if(i==1)
        SPI.transfer16((pwmRef[motor]& 0x7fff));
      else{
        switch(i){
          case 4:
            data[0] = SPI.transfer16(0);
            break;  
          case 5:
            data[1] = SPI.transfer16(0);
            position[motor] =  ((data[0]>>8)<<24|(data[0]&0xff)<<16|(data[1]>>8)<<8|(data[1]&0xff));
            break; 
          case 6:
            velocity[motor] = SPI.transfer16(0);
            break; 
          case 7:
            current[motor] = SPI.transfer16(0);
            break; 
          default:
            SPI.transfer16(0);
           break;
         }
      }
      digitalWrite(ss_n[motor],HIGH);
    }
    SPI.endTransaction();
  }
  // controller
  for(int motor=0;motor<2;motor++){
    switch(control_mode[motor]){
      case POSITION:
        err[motor] = setpoint[motor]-position[motor];
        break;
      case VELOCITY:
        err[motor] = setpoint[motor]-velocity[motor];
        break;
      case DISPLACEMENT: // not implemented
        err[motor] = 0;
        break;
      case DIRECT_PWM:
        result[motor] = setpoint[motor];
        break;
    }
    if(control_mode[motor]!=DIRECT_PWM){
      result[motor] = Kp[motor]*err[motor] + Kd[motor]*(err_prev[motor]-err[motor]);
      err_prev[motor] = err[motor];
    }
    if(result[motor] > maxPWM){
      result[motor] = maxPWM;
    }
    if(result[motor] < -maxPWM){
      result[motor] = -maxPWM;
    }
    pwmRef[motor] = result[motor];
  }
  for(int motor=0;motor<2;motor++){
    ardprintf("pwmRef: %d positions: %d velocities: %d currents: %d\n", 
       pwmRef[motor], position[motor], velocity[motor], current[motor]
    );
  }
  delay(10);
}
