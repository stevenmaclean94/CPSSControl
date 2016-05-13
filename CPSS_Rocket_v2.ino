#include "Kalman.h"
#include "Control.h"
#include <Wire.h>;

#define BAUD_RATE 9600
#define SOL_Y_POS 4
#define SOL_Y_NEG 12
#define SOL_Z_POS 6
#define SOL_Z_NEG 8
#define MAX_CTRL_TIME 0.2


struct dataset{
  float gyr[3];
  float mag[3];
  float acc[3];
  float bar[3];
};


/* Global Variables*/
float timer;
float control[2];
float initAngleY;
float initAngleZ;
int timer1_counter;
int timer3_counter;
float controlTime = 0.2;
float cal[3] = {0,0,0};
float rotation[9] = {0,1,0,0,0,1,1,0,0};
struct dataset data;


int off =0;
/* Function Prototypes */
byte readReg(byte reg,byte address);
int getOffset();
int Convert2TimerCounts(float controlValue);
void FireThruster();
void WriteReg(byte reg, byte value, byte device);
void beep (float hz, float dur);
float* MultiplyRotations(float rot, float pos);
void ReadData(struct dataset *data, char control);
float angleZ = 0;
float angleY = 0;
/* Instantiate Objects*/
  Kalman kalmanX;
  Kalman kalmanY;
  Kalman kalmanZ; 
  Control controller;
//------------------------------------------
// Harrison's
//-----------------------------------------
//Barometer:
  uint16_t C[7] = {0};
  const float sea_press = 1013.25;
  const byte baroAdd = 0x77; //or 0x76

  byte Pcode = 0x48;
  byte Tcode = 0x58;

//Acc,Gyr,Mag:
  //conversion factors for sensors
  float accfact = 0.000732;
  float gyrfact = 0.00875;
  float magfact = 0.00029;
  
  //device address where SDO is tied to Vdd
  byte accAdd = 0B1101011;
  byte gyrAdd = 0B1101011; //Not necessary, as acc and gyr share an address. Makes code more readable.
  byte magAdd = 0B0011110;
  
  //first register of measurements for each device (start register)
  byte accSt = 0B00101000;
  byte gyrSt = 0B00011000;
  byte magSt = 0B00101000;
  byte TSt   = 0B00010101;
  
  //other necessary registers
  byte CTRL_REG4    = 0B00011110;
  byte CTRL_REG5_XL = 0B00011111;
  byte CTRL_REG6_XL = 0B00100000;
  byte CTRL_REG1_G  = 0B00010000;
  byte CTRL_REG2_M  = 0B00100001;
  byte CTRL_REG3_M  = 0B00100010;

  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  Wire.begin();

  // initialize solenoids
  pinMode(5,OUTPUT);
  pinMode(SOL_Y_POS,OUTPUT);
  digitalWrite(SOL_Y_POS,LOW);
  pinMode(SOL_Y_NEG,OUTPUT);
  digitalWrite(SOL_Y_NEG,LOW);
  pinMode(SOL_Z_POS,OUTPUT);
  digitalWrite(SOL_Z_POS,LOW);
  pinMode(SOL_Z_NEG,OUTPUT);
  digitalWrite(SOL_Z_NEG,LOW);

  // initialize sensors
  Wire.begin();
  pinMode(13,OUTPUT);
  pinMode(5,OUTPUT);
  digitalWrite(13,HIGH);
  delay(200); 
  digitalWrite(13,LOW);

  
//OpenLog:
  Serial.begin(9600);
  while(!Serial);
  delay(1000);
  //Serial.print("t ");
  //Serial.print("AccX AccY AccZ ");
  //Serial.print("GyrX GyrY GyrZ ");
  //Serial.print("MagX MagY MagZ ");
  //Serial.println("A P Tbar");

  beep(440,200);

//Acc,Gyr,Mag:
  WriteReg(CTRL_REG4,0B00111000,accAdd);    //enable gyro
  WriteReg(CTRL_REG5_XL,0B00111000,accAdd); //enable accelerometer
  WriteReg(CTRL_REG6_XL,0B00001000,accAdd); //set accelerometer scale to 16Gs
  WriteReg(CTRL_REG1_G,0B01000000,accAdd);  //gyro/accel odr and bw
  WriteReg(CTRL_REG2_M,0B00100000,magAdd);  //set mag sensitivity to 8 gauss
  WriteReg(CTRL_REG3_M,0B00000000,magAdd);  //enable mag continuous
  beep(550,200);

//Barometer:
  Wire.beginTransmission(baroAdd);
  Wire.write(0x1E); // reset
  Wire.endTransmission();
  delay(10);


   for (int i=0; i<6  ; i++) {
  
     Wire.beginTransmission(baroAdd);
     Wire.write(0xA2 + (i * 2));
     Wire.endTransmission();
  
     //Wire.beginTransmission(baroAdd);
     Wire.requestFrom(baroAdd, (uint8_t) 2);
     while (Wire.available()==0);
     C[i+1] = Wire.read() << 8 | Wire.read();
   }
   
  beep(660,400);
  //Calibrate Sensors
  for (int i=0; i < 1000; i++){
    ReadData(&data,0b1000);
    cal[0] =+ data.acc[0]/1000;
    cal[1] =+ data.acc[1]/1000;
    cal[2] =+ data.acc[2]/1000;
    delay(2);
  }  
  // Setup Controller
  controller.setDerivGain(0);
  controller.setPropGain(1);
  controller.setDeadband(1);
  // Setup interrupt code
  timer1_counter = 0;   // preload timer 65536-16MHz/256/35Hz
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

  timer3_counter = 0;   // preload timer 65536-16MHz/256/35Hz
  TCNT3 = timer1_counter;   // preload timer
  TCCR3B |= (1 << CS32);    // 256 prescaler 
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

  beep(2500,200);
/*
  delay(100);
  digitalWrite(SOL_Y_NEG,HIGH);
  delay(100);
  digitalWrite(SOL_Y_NEG,LOW);
  delay(2000);
  digitalWrite(SOL_Y_POS,HIGH);
  delay(100);
  digitalWrite(SOL_Y_POS,LOW);
  delay(2000);
  digitalWrite(SOL_Z_NEG,HIGH);
  delay(100);
  digitalWrite(SOL_Z_NEG,LOW);
  delay(2000);
  digitalWrite(SOL_Z_POS,HIGH);
  delay(100);
  digitalWrite(SOL_Z_POS,LOW);
  */
  timer = millis();
}

void loop() {
  // put your main code here, to run repeatedly:

  float dt = (millis() - timer) / 1000; //Convert dt to sec
  timer = millis();
  //ReadData(gyr, gyrAdd, gyrSt);  //read gyroscope
  ReadData(&data,0b1100);
  for (byte i=0;i<3;i++){
      data.acc[i] *= accfact;  //multiply by conversion factor for Gs
      data.gyr[i] *= gyrfact;  //multiply by conversion factor for deg/s
      data.mag[i] *= magfact;  //multiply by conversion factor for Gauss
    }
  //float angleY = 0.99*data.gyr[0] + 0.01*data.acc[0];
  //float angleY = kalmanY.getAngle(data.gyr[0], dt, 0.0, 0.0);
  //float angleZ = kalmanZ.getAngle(data.gyr[2], dt, 0.0, 0.0);
  angleY += data.gyr[0]*dt * RAD_TO_DEG;
  angleZ += data.gyr[2]*dt * RAD_TO_DEG; 
  
  float* control = controller.PDControl(angleY, angleZ, 0, 0);

  Serial.print(control[0]);
  Serial.print(control[1]);
  Serial.print("\t");
  Serial.print(angleY);
  Serial.println(angleZ);
  
  if (millis()-controlTime<0){
    int timerCountY = Convert2TimerCounts(control[0]);
    int timerCountZ = Convert2TimerCounts(control[1]);
    if (control[1]>0&&control[0]<0){
      digitalWrite(SOL_Z_POS,HIGH);
      TCNT3 = min(timerCountY, timerCountZ);
    }
    else{
      if(control[0]<0){ //y axis
        digitalWrite(SOL_Y_POS, HIGH);    
        TCNT1 = timerCountY;
      } else{
        digitalWrite(SOL_Z_POS, HIGH);
        digitalWrite(SOL_Z_NEG, HIGH);
        TCNT1 = timerCountY;      
      }
      if (control[1]>0){ //z axis
        digitalWrite(SOL_Z_POS, HIGH);
        digitalWrite(SOL_Y_POS, HIGH);
        TCNT3 = timerCountZ;      
      }else{
        digitalWrite(SOL_Z_NEG, HIGH);
        TCNT3 = timerCountZ;      
      }
    }
  }
  delay(20);
}

ISR(TIMER1_OVF_vect){
  TCNT1 = 0; //Reset timer
  digitalWrite(SOL_Y_POS, 0);
  digitalWrite(SOL_Y_NEG, 0);
  digitalWrite(SOL_Z_POS, 0);
  digitalWrite(SOL_Z_NEG, 0);
}

ISR(TIMER3_OVF_vect){
  TCNT3 = 0; //Reset timer
  digitalWrite(SOL_Z_POS, 0);
  digitalWrite(SOL_Z_NEG, 0);
  digitalWrite(SOL_Y_POS, 0);
  digitalWrite(SOL_Y_NEG, 0);
}

int Convert2TimerCounts(float controlValue){
  //Scale the control input to a time value. Shouldn't be necessary but preparing
  float scalingValue = 1; 
  float targetTime = controlValue * scalingValue;
  if (targetTime > MAX_CTRL_TIME){
    targetTime = MAX_CTRL_TIME; 
  }
  float timerResolution = 1.6e-5; //256 prescalar
  int timerCount = floor(targetTime/timerResolution);
  return timerCount;
}

void beep (float hz, float dur){  //beep frequency in Hz for duration in ms (not us)
  int halfT = 500000/hz;
  for (int i = 0; i<(500*dur/halfT) ;i++){
    digitalWrite(5,HIGH);
    delayMicroseconds(halfT);
    digitalWrite(5,LOW);
    delayMicroseconds(halfT);
  }
}

float* MultiplyRotations(float rot[], float pos[]){
  float rotated[3];
  for (int i=0; i<9; i++){
    int j = floor(i/3);
      rotated[j] =+ rot[i]*pos[j];
  }
  return rotated;
}

void ReadData(struct dataset *data, char control){

  // break down the control command: gyr,acc,mag,bar
  bool acc = (control&&0b1000);
  bool gyr = (control&&0b0100);
  bool mag = (control&&0b0010);
  bool bar = (control&&0b0001);
  
//get current measurements
    if (bar){
    PrimeBaro(baroAdd,Pcode);       //ask barometer to prepare a pressure value (then do other things while we wait)
    delay(6);                       //wait for barometer to finish prepping pressure value
    }
    if (mag){
    ReadAccel(data->mag, magAdd, magSt);  //read magnetometer
    }
    if (acc){
    ReadAccel(data->acc, accAdd, accSt);  //read accelerometer
    }
    if (gyr){
    ReadAccel(data->gyr, gyrAdd, gyrSt);  //read gyroscope
    }
    if (bar){
    uint32_t Praw = ReadBaro(baroAdd);       //collect prepared pressure value
    PrimeBaro(baroAdd,Tcode);       //ask barometer to prepare temperature value
    delay(10);                       //wait for barometer to prepare temperature value (nothing to do while we wait, so wait full duration)
    uint32_t Traw = ReadBaro(baroAdd);       //collect temperature value
    
      int64_t dT = 0;
      int32_t TEMP = 0;
      int64_t OFF = 0; 
      int64_t SENS = 0; 
      int32_t P = 0;
      
      float Temperature;
      float Pressure;
      float Altitude;
    //Get altitude
    dT   = Traw - ((uint32_t)C[5] << 8);
    OFF  = ((int64_t)C[2] << 16) + ((dT * C[4]) >> 7);
    SENS = ((int32_t)C[1] << 15) + ((dT * C[3]) >> 8);
   
    TEMP = (int64_t)dT * (int64_t)C[6] / 8388608 + 2000;
   
    if(TEMP < 2000) // if temperature lower than 20 Celsius 
    {
      int32_t T1    = 0;
      int64_t OFF1  = 0;
      int64_t SENS1 = 0;
   
      T1    = pow(dT, 2) / 2147483648;
      OFF1  = 5 * pow((TEMP - 2000), 2) / 2;
      SENS1 = 5 * pow((TEMP - 2000), 2) / 4;
      
      if(TEMP < -1500) // if temperature lower than -15 Celsius 
      {
        OFF1  = OFF1 + 7 * pow((TEMP + 1500), 2); 
        SENS1 = SENS1 + 11 * pow((TEMP + 1500), 2) / 2;
      }
      
      TEMP -= T1;
      OFF -= OFF1; 
      SENS -= SENS1;
    }
   
    
    Temperature = (float)TEMP / 100; 
    
    P  = ((int64_t)Praw * SENS / 2097152 - OFF) / 32768;
   
    Pressure = (float)P / 100;
    Altitude = ((pow((sea_press / Pressure), 1/5.257) - 1.0) * (Temperature + 273.15)) / 0.0065;
    data->bar[0] = Altitude;
    data->bar[1] = Pressure;
    data->bar[2] = Temperature;
    }
    
    // Convert inertial measurements to useful units
    for (byte i=0;i<3;i++){
      if (acc){
      data->acc[i] *= accfact;  //multiply by conversion factor for Gs
      }
      if(gyr){
      data->gyr[i] *= gyrfact;  //multiply by conversion factor for deg/s
      }
      if(mag){
      data->mag[i] *= magfact;  //multiply by conversion factor for Gauss
      }
    }
}


//____________________________Other Functions______________________________________
void printArray (float array[]){
for (int i = 0; i<3; i++){
  Serial.print(array[i]);
  Serial.print(" ");
}
}

//_____________________________Supporting Functions___________________________________

void ReadAccel(float data[], byte devAdd, byte starTcode){
  Wire.beginTransmission(devAdd);
  Wire.write(starTcode);
  Wire.endTransmission();
  Wire.requestFrom(devAdd,(uint8_t)6);
  while (Wire.available() == 0){
  }
  for (byte i=0;i<3;i++){
    data[i] = (Wire.read())|(Wire.read()<<8);
  }
}
void PrimeBaro(byte devAdd, byte code){
  Wire.beginTransmission(devAdd); 
  Wire.write(code);
  Wire.endTransmission();
}
long ReadBaro(byte devAdd){
  unsigned long data = 0;
  Wire.beginTransmission(devAdd);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(devAdd,3);
  while (Wire.available() == 0);
  data = Wire.read() * (unsigned long)65536 + Wire.read() * (unsigned long)256 + Wire.read();
  return data;
}
float Tread(){
  float T;
  Wire.beginTransmission(accAdd);
  Wire.write(TSt);
  Wire.endTransmission();
  Wire.requestFrom(accAdd,2);
  while(Wire.available()==0);
  T = (((Wire.read())|(Wire.read()<<8))/16+25);
  return T;
}
void WriteReg(byte reg, byte value, byte device){
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


