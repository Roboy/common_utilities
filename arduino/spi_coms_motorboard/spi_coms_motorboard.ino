#include <SPI.h>
#include "ardprintf.h"

struct SPISTREAM {
  union {
    struct {
      uint16_t startOfFrame;
      int16_t pwmRef;
      uint16_t controlFlags1 : 16;
      uint16_t controlFlags2 : 16;
      uint16_t dummy : 16;
      int32_t actualPosition : 32;
      int16_t actualVelocity : 16;
      int16_t actualCurrent : 16;
      int16_t springDisplacement : 16;
      int16_t sensor1 : 16;
      int16_t sensor2 : 16;
    };
    uint8_t TxBuffer[24];
    uint16_t TxBuffer2[12];
  };
};

SPISTREAM tx_frame[2], rx_frame[2];
#define SS_0 6
#define SS_1 7
#define POSITION 0
#define VELOCITY 1
#define DISPLACEMENT 2
#define maxPWM 500

float Kp[2] = {1,1}, Kd[2] = {0,0}, err[2] = {0,0}, err_prev[2] = {0,0}, setpoint[2] = {0,0}, result[2] = {0,0};
int control_mode[2];

void setup() {
  for(int motor;motor<2;motor++){
    tx_frame[motor].startOfFrame = 0x8000;
    tx_frame[motor].pwmRef = 0;
    tx_frame[motor].controlFlags1 = 0;
    tx_frame[motor].controlFlags2 = 0;
    tx_frame[motor].dummy = 0;
    tx_frame[motor].actualPosition = 0;
    tx_frame[motor].actualVelocity = 0;
    tx_frame[motor].actualCurrent = 0;
    tx_frame[motor].springDisplacement = 0;
    tx_frame[motor].sensor1 = 0;
    tx_frame[motor].sensor2 = 0;
  }
  Serial.begin(115200);
  SPI.begin();
  pinMode (SS_0, OUTPUT);
  pinMode (SS_1, OUTPUT);
}

void loop() {
  for(int motor=0;motor<2;motor++){
    for(int i=0;i<12;i++){
      if(i<5){
        SPI.transfer16(tx_frame[motor].TxBuffer2[i]);
      }else{
        rx_frame[motor].TxBuffer2[i] = SPI.transfer16(i);
      }
    }
  }
  // controller
  for(int motor=0;motor<2;motor++){
    switch(control_mode[motor]){
      case POSITION:
        err[motor] = setpoint[motor]-rx_frame[motor].actualPosition;
        break;
      case VELOCITY:
        err[motor] = setpoint[motor]-rx_frame[motor].actualVelocity;
        break;
      case DISPLACEMENT:
        err[motor] = setpoint[motor]-rx_frame[motor].springDisplacement;
        break;
    }
    result[motor] = Kp[motor]*err[motor] + Kd[motor]*(err_prev[motor]-err[motor]);
    err_prev[motor] = err[motor];
    if(result[motor] > maxPWM){
      result[motor] = maxPWM;
    }
    if(result[motor] < -maxPWM){
      result[motor] = -maxPWM;
    }
    tx_frame[motor].pwmRef = result[motor];
  }
  ardprintf("pwmRef: %d %d\n"
     "setpoints: %d %d\n"
     "positions: %d %d\n"
     "velocities: %d %d\n"
     "currents: %d %d\n"
     "displacements: %d %d\n", 
     rx_frame[0].pwmRef, rx_frame[1].pwmRef,
     setpoint[0], setpoint[1],
     rx_frame[0].actualPosition, rx_frame[1].actualPosition,
     rx_frame[0].actualVelocity, rx_frame[1].actualVelocity,
     rx_frame[0].actualCurrent, rx_frame[1].actualCurrent,
     rx_frame[0].springDisplacement, rx_frame[1].springDisplacement
     );
  
}
