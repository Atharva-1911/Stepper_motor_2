#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include "globals.h"

#define SDA1 21
#define SCL1 22
#define SDA2 16
#define SCL2 17

TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

#define STEP1 26
#define DIR1 27 
#define STEP2 12 
#define DIR2 14
#define ENCODER_ADDR 0x36


int f_max = 4000;
int f_min = 100;
double setpoint, positionError, output;
double setpoint2, positionError2, output2;


double Kp = 4, Ki = 0.001, Kd = 1;
double Kp2 = 0, Ki2 = 0, Kd2 = 0;


PID positionController(&positionError, &output, &setpoint, Kp, Ki, Kd, DIRECT);
PID positionController2(&positionError2, &output2, &setpoint2, Kp2, Ki2, Kd2, DIRECT);
SemaphoreHandle_t angleMutex = NULL;
SemaphoreHandle_t angleMutex2 = NULL;
SemaphoreHandle_t positionMutex = NULL;
SemaphoreHandle_t positionMutex2 = NULL;
SemaphoreHandle_t setpointMutex = NULL;

hw_timer_t *pulseTimer = NULL;
hw_timer_t *pulseTimer2 = NULL;
int SemaphoreTimeout = 50;


//float zeroOffset;

// void i2c_scanner(){
//   byte error, address;
//   int nDevices;
//   Serial.println("Scanning...");
//   nDevices = 0;
//   for (address = 1; address < 127; address++)
//   {
//     // The i2c_scanner uses the return value of the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();
//     // Serial.print("Address: ");
//     // Serial.println(address);
//     if (error == 0)
//      {  
//       Serial.print("Found at Address ");
//       Serial.println(address, HEX);
//       nDevices++;
//      }
//   }
//   if (nDevices == 0)
//     {Serial.println("No I2C devices found\n");}
//   else
//     {Serial.println("done\n");}
// }

//create a structure containing various variables with different datatypes 
//name it "EncoderData" and create an instance of it called "encoderData"
 struct EncoderData{
  TwoWire* i2c;
  volatile float* angle;
  volatile float* prevAngle;
  volatile float* currentAngle;
  volatile int* totalRev;
  SemaphoreHandle_t mutex;
} ;

EncoderData encoderData1;
EncoderData encoderData2;
//create a task where the encoder data is used 
//use same logic for transmission and getting the angle
//use pointer enc where the variables are used 
//to access the variable in struct use arrow operator ->

void readEncoder(void *pvParameter){
  EncoderData* enc =  (EncoderData*) pvParameter;

   while (1)
    {    
    enc->i2c->beginTransmission(ENCODER_ADDR);
    enc->i2c->write(0x0E); 
    enc->i2c->endTransmission(false);
    // End transmission to the address
    // Request 2 bytes of data (0x0E + 0x0F)
    
    enc->i2c->requestFrom(ENCODER_ADDR, 2);
    
    byte high_byte = enc->i2c->read();
    byte low_byte  = enc->i2c->read();

      
    uint16_t angle_byte = (high_byte << 8) | low_byte;
    float angle_deg = (angle_byte * 360.0) / 4096;
     

    if ((*(enc->prevAngle) - angle_deg) > 180)
      {
        (*(enc->totalRev))++;
      } 
    else if ((*(enc->prevAngle) - angle_deg) < -180)
      {
        (*(enc->totalRev))--;
      }

    *(enc->prevAngle) = angle_deg;

    angle_deg = ((*(enc->totalRev)) * 360.0) + angle_deg;
      
      
    if (xSemaphoreTake(enc->mutex, SemaphoreTimeout))
      {
        *(enc->angle) = angle_deg;
        *(enc->currentAngle) = angle_deg;
        xSemaphoreGive(enc->mutex);
      }
      
      
    vTaskDelay(pdMS_TO_TICKS(1));
}
}




void IRAM_ATTR stepGenerator(){
  stepState = !stepState;
  digitalWrite(STEP1, stepState);
}
void IRAM_ATTR stepGenerator2(){
  stepState2 = !stepState2;
  digitalWrite(STEP2, stepState2);
}



// set target angle, directio and targettime in sec.
void moveMotor(void *pvParameter){
  //steps per degree = angle*1600/360

  //set direction HIGH OR LOW (HIGH= clockwise,LOW=anticlockwise)
  // int numSteps = 0;
  while (1)
  {
    double local_sp = 0;
    if(xSemaphoreTake(setpointMutex, SemaphoreTimeout)){
      local_sp = setpoint;
      xSemaphoreGive(setpointMutex);
    
  }
  

    if (xSemaphoreTake(angleMutex, SemaphoreTimeout))
    {
      currentAngle = angle / 2;
      xSemaphoreGive(angleMutex);
    }
    
    if (xSemaphoreTake(positionMutex, SemaphoreTimeout))
    {
      positionError = local_sp - currentAngle;
      xSemaphoreGive(positionMutex);
    }
    
    presentTime = micros();
    dt = (presentTime - lastTime)/ 1000000.0;
    derivative = 0;
    if (dt > 0){
      derivative = (positionError - previouserror)/dt;
    }
    
     integral = integral+(positionError*dt);

     if (integral > 1000)integral = 1000;
     if (integral < -1000)integral = -1000;
    
    output = (Kp * (positionError)) + (Ki * (integral)) + (Kd * (derivative)) ;
    previouserror = positionError;
    lastTime = presentTime;

    if (output < 0)
    {
      direction = 1;
    }
    else
    {
      direction = 0;
    }

    digitalWrite(DIR1, direction);
    output = constrain(output, -2000.0, 2000.0);

    deadband = 0.1;
    if(fabs(positionError) < deadband){
      timerAlarmDisable(pulseTimer);
    } 
    else{
    
    frequency = 4000 - (fabs(output) / 2000.0) * (f_max - f_min);
    // Serial.printf("%f\t%f\t%f\t%f\t%f\n", currentAngle, positionError, output, frequency, integral);
    timerAlarmWrite(pulseTimer, (uint64_t)frequency, true);
    timerAlarmEnable(pulseTimer);
    vTaskDelay(pdMS_TO_TICKS(1));
    }
  }  
}







void moveMotor1(void *pvParameter){
  while (1)
  {
    double local_sp2 = 0;
    if(xSemaphoreTake(setpointMutex, SemaphoreTimeout)){
      local_sp2 = setpoint2;
      xSemaphoreGive(setpointMutex);

   
  }


    if (xSemaphoreTake(angleMutex2, SemaphoreTimeout))
    {
      currentAngle2 = angle2 / 2;
      xSemaphoreGive(angleMutex2);
    }
    
    if (xSemaphoreTake(positionMutex2, SemaphoreTimeout))
    {
      positionError2 = local_sp2 - currentAngle2;
      xSemaphoreGive(positionMutex2);
    }
    
    float presentTime2 = micros();
    float dt2 = (presentTime2 - lastTime2)/ 1000000.0;
    float derivative2 = 0;
    if (dt2 > 0){
      derivative2 = (positionError2 - previouserror2)/dt;
    }
    
     integral2 = integral2+(positionError2*dt);

     if (integral2 > 1000)integral2 = 1000;
     if (integral2 < -1000)integral2 = -1000;
    
    output2 = (Kp2 * (positionError2)) + (Ki2 * (integral2)) + (Kd2 * (derivative2)) ;
    previouserror2 = positionError2;
    lastTime2 = presentTime2;

    if (output2 < 0)
    {
      direction = 1;
    }
    else
    {
      direction = 0;
    }

    digitalWrite(DIR2, direction);
    output2 = constrain(output2, -2000.0, 2000.0);

    float deadband1 = 0.1;
    if(fabs(positionError2) < deadband1){
      timerAlarmDisable(pulseTimer2);
    } 
    else{
    
    float frequency2 = 4000 - (fabs(output2) / 2000.0) * (f_max - f_min);
    // Serial.printf("%f\t%f\t%f\t%f\t%f\n", currentAngle, positionError, output, frequency, integral);
    timerAlarmWrite(pulseTimer2, (uint64_t)frequency2, true);
    timerAlarmEnable(pulseTimer2);
    vTaskDelay(pdMS_TO_TICKS(1));
    }
  }  
}

void Print(void *pvParameter){
  float local_angle = 0;
  float local_angle2 = 0;
  float local_position = 0;
  float local_position2 = 0;
  float local_setpoint1 = 0;
  float local_setpoint2 = 0;

  while(1){    
    if(xSemaphoreTake(angleMutex, SemaphoreTimeout))
    {
      local_angle = angle/2;
      xSemaphoreGive(angleMutex);
    }

    if(xSemaphoreTake(angleMutex2, SemaphoreTimeout))
    {
      local_angle2 = angle2;
      xSemaphoreGive(angleMutex2);
    }

    if(xSemaphoreTake(positionMutex, SemaphoreTimeout))
    {
      local_position = positionError;
      xSemaphoreGive(positionMutex);
    }
    if(xSemaphoreTake(positionMutex2, SemaphoreTimeout))
    {
      local_position2 = positionError2;
      xSemaphoreGive(positionMutex2);
    }
    if(xSemaphoreTake(setpointMutex, SemaphoreTimeout))
    {
      local_setpoint1 = setpoint;
      local_setpoint2 = setpoint2;
      xSemaphoreGive(setpointMutex);
    }
    

    float local_Time = micros();

    Serial.print(local_angle);
    Serial.print("\t");
    Serial.println(local_angle2);
    // float deadband_1 = 0.1;
    // if(fabs(local_position) >= deadband_1){
    //   Serial.printf("%f\t%f\t%f\t%f\n", local_angle, local_position, local_Time, local_angle2);
       
    // }

    vTaskDelay(pdMS_TO_TICKS(50));   
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(STEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);


  I2C_1.begin(SDA1, SCL1);
  I2C_2.begin(SDA2, SCL2);


  // setpoint = 90.0;
  positionController.SetMode(AUTOMATIC);
  positionController2.SetMode(AUTOMATIC);

  angleMutex = xSemaphoreCreateMutex();
  angleMutex2 = xSemaphoreCreateMutex();
  positionMutex = xSemaphoreCreateMutex();
  positionMutex2 = xSemaphoreCreateMutex();
  setpointMutex = xSemaphoreCreateMutex();

  // pulseTimer = timerBegin(0, 80, true);
  // timerAttachInterrupt(pulseTimer, &stepGenerator, true);
  // pulseTimer2 = timerBegin(1, 80, true);
  // timerAttachInterrupt(pulseTimer2, &stepGenerator2, true);

  encoderData1 = {&I2C_1, &angle, &prevAngle, &currentAngle, &totalRev, angleMutex};
  encoderData2 = {&I2C_2, &angle2, &prevAngle2, &currentAngle2, &totalRev2, angleMutex2};


  xTaskCreatePinnedToCore(
    readEncoder,
    "read_encoder",
    8192,
    &encoderData1,
    1,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    readEncoder,
    "read_encoder2",
    8192,
    &encoderData2,
    1,
    NULL,
    0
  );

  // xTaskCreatePinnedToCore(
  //   Serial_task,
  //   "Write_setpoints",
  //   8192,
  //   NULL,
  //   2,
  //   NULL,
  //   1
  // );

  // xTaskCreatePinnedToCore(
  //   moveMotor,
  //   "move_motor",
  //   4096,
  //   NULL,
  //   2,
  //   NULL,
  //   1
  // );
 
  // xTaskCreatePinnedToCore(
  //   moveMotor1,
  //   "move_motor",
  //   4096,
  //   NULL,
  //   2,
  //   NULL,
  //   1
  // );
 

  xTaskCreatePinnedToCore(
    Print,
    "Serial_Print",
    8192,
    NULL,
    1,
    NULL,
    1
  );

  

}

void loop() {
}