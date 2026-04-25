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
#define ENCODER_ADDR 0x36


int f_max = 4000;
int f_min = 100;
double setpoint, positionError, output;



double Kp = 4, Ki = 0.001, Kd = 1;


PID positionController(&positionError, &output, &setpoint, Kp, Ki, Kd, DIRECT);
SemaphoreHandle_t angleMutex = NULL;
SemaphoreHandle_t angleMutex2 = NULL;
SemaphoreHandle_t positionMutex = NULL;

hw_timer_t *pulseTimer = NULL;
int SemaphoreTimeout = 50;


// float zeroOffset;

// void i2c_scanner()
// {
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

void readEncoder(void *pvParameter)
  {
    while (1)
    
    {    //Being transmission to the address
    I2C_1.beginTransmission(ENCODER_ADDR);
    // Write the instruction register 0x0E
    I2C_1.write(0x0E); 
    // Angle is stored in the large endian format.
    // End transmission without reseting the bus
    byte error = I2C_1.endTransmission(false);
    // End transmission to the address
    // Request 2 bytes of data (0x0E + 0x0F)
    if (error == 0){
      if(I2C_1.requestFrom(ENCODER_ADDR, 2)==2)
      {
        byte high_byte = I2C_1.read();
        byte low_byte  = I2C_1.read();

      // raw angle = high bytes << 8 | low bytes
        uint16_t angle_byte = (high_byte << 8) | low_byte;
        float angle_deg = (angle_byte * 360.0) / 4096;
      // Serial.print(angle_deg); Serial.print("\t"); Serial.println(prevAngle);

      if ((prevAngle - angle_deg) > 180)
      {
        totalRev++;
      } 
      else if ((prevAngle - angle_deg) < -180)
      {
        totalRev--;
      }
      prevAngle = angle_deg;
      angle_deg = (totalRev * 360.0) + angle_deg;
      
      //float angle = raw * 360.0 / 4096;
      
      if (xSemaphoreTake(angleMutex, SemaphoreTimeout))
      {
        angle = angle_deg;
        xSemaphoreGive(angleMutex);
      }
      }
    }
    //Serial.println(angle);
    vTaskDelay(10 / portTICK_PERIOD_MS);}
    
}


void readEncoder2(void *pvParameter)
  {
    while (1)
    {
    I2C_2.beginTransmission(ENCODER_ADDR);
    I2C_2.write(0x0E); 
    byte error = I2C_2.endTransmission(false);

    if (error == 0)
    {
      if(I2C_2.requestFrom(ENCODER_ADDR, 2)==2){
        
        
        
        byte high_byte = I2C_2.read();
        byte low_byte  = I2C_2.read();
        
        uint16_t angle_byte = (high_byte << 8) | low_byte;
        float angle_deg2 = (angle_byte * 360.0) / 4096;
        
        if ((prevAngle2 - angle_deg2) > 180)
        {
            totalRev2++;
          } 
          else if ((prevAngle2 - angle_deg2) < -180)
          {
              totalRev2--;
            }
            prevAngle2 = angle_deg2;
            angle_deg2 = (totalRev2 * 360.0) + angle_deg2;
            
            
            if (xSemaphoreTake(angleMutex2, SemaphoreTimeout))
            {
              angle2 = angle_deg2;
              xSemaphoreGive(angleMutex2);
            }
            
          }
          }
      //Serial.print(angle2);
    vTaskDelay(20 / portTICK_PERIOD_MS);}
    
}



void IRAM_ATTR stepGenerator(){
  stepState = !stepState;
  digitalWrite(STEP1, stepState);
}



// set target angle, directio and targettime in sec.
void moveMotor(void *pvParameter){
  //steps per degree = angle*1600/360

  //set direction HIGH OR LOW (HIGH= clockwise,LOW=anticlockwise)
  // int numSteps = 0;
  while (1)
  {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');  // wait for full line
      input.trim();  

    if (input.length() > 0) {
      float newsetpoint = input.toFloat();

    setpoint = newsetpoint;
    Serial.println(setpoint);

    integral = 0;
    previouserror = 0;
    lastTime = micros();
  }
}

    if (xSemaphoreTake(angleMutex, SemaphoreTimeout))
    {
      currentAngle = angle / 2;
      xSemaphoreGive(angleMutex);
    }
    
    

    if (xSemaphoreTake(positionMutex, SemaphoreTimeout))
    {
      positionError = setpoint - currentAngle;
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


void Print(void *pvParameter){
  float local_angle = 0;
  float local_angle2 = 0;
  float local_position=0;

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

    float local_Time = micros();

    // Serial.print(local_angle);
    // Serial.print("\t");
    // Serial.println(local_angle2);
    if(fabs(local_position) >= deadband){
      Serial.printf("%f\t%f\t%f\t%f\n", local_angle, local_position, local_Time, local_angle2);
       
    }

    vTaskDelay(pdMS_TO_TICKS(10));   
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(STEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);


  I2C_1.begin(SDA1, SCL1, 100000);
  I2C_2.begin(SDA2, SCL2, 100000);


  // setpoint = 90.0;
  positionController.SetMode(AUTOMATIC);

  angleMutex = xSemaphoreCreateMutex();
  angleMutex2 = xSemaphoreCreateMutex();
  positionMutex = xSemaphoreCreateMutex();


  pulseTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(pulseTimer, &stepGenerator, true);


  xTaskCreatePinnedToCore(
    readEncoder,
    "read_encoder",
    8192,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    readEncoder2,
    "read_encoder2",
    8192,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    moveMotor,
    "move_motor",
    4096,
    NULL,
    2,
    NULL,
    1
  );
 
 
  xTaskCreatePinnedToCore(
    Print,
    "Serial_Print",
    8192,
    NULL,
    1,
    NULL,
    0
  );

  

}

void loop() {

  
}