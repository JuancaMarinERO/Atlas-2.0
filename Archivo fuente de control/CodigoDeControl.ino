// THERE IS NO WARRANTY FOR THE SOFTWARE... (mantener el comentario original)

#include <Wire.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"  // Agregar la librería Bluetooth
#define Bomba 23

// Cambios en la sección de declaraciones
BluetoothSerial SerialBT;  // Objeto Bluetooth

volatile int ReceiverValue[6] = {1000, 1000, 1000, 1000, 1000, 1000}; // Inicializar con valores neutrales

uint32_t lastPrintTime = 0;
const uint32_t printInterval = 100;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;

int ESCfreq=500;
float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0.5; float IAnglePitch=IAngleRoll;
float DAngleRoll=0.007; float DAnglePitch=DAngleRoll;

float PRateRoll = 0.625;
float IRateRoll = 2.1;
float DRateRoll = 0.0088;

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

uint32_t LoopTimer;
float t=0.004;      //time cycle


Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;

volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;

volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;


float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState + (t*KalmanInput);
  KalmanUncertainty=KalmanUncertainty + (t*t*4*4); //here 4 is the vairnece of IMU i.e 4 deg/s
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //std deviation of error is 3 deg
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

// Eliminar las funciones de interrupción y lectura de pines

void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29; //*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;
}


void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm +( I * (Error + PrevError) * (t/2));
  if (Iterm > 400)
  {
    Iterm = 400;
  }
  else if (Iterm < -400)
  {
  Iterm = -400;
  }
  float Dterm = D *( (Error - PrevError)/t);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
  {
    PIDOutput = 400;
  }
  else if (PIDOutput < -400)
  {
    PIDOutput = -400;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void setup(void) {
  Serial.begin(115200);
  pinMode(Bomba, OUTPUT);
    
    // Inicialización Bluetooth
  SerialBT.begin("ESP32-Dron");
  Serial.println("Bluetooth listo. Esperando conexión...");
  Serial.println("Inicio del sistema - Bomba de Agua");

int led_time=100;
  pinMode(15, OUTPUT);
    digitalWrite(15, LOW);
    delay(led_time);
    digitalWrite(15, HIGH);
    delay(led_time);
    digitalWrite(15, LOW);
    delay(led_time);
    digitalWrite(15, HIGH);
    delay(led_time);
    digitalWrite(15, LOW);
    delay(led_time);
    digitalWrite(15, HIGH);
    delay(led_time);
    digitalWrite(15, LOW);
    delay(led_time);
    digitalWrite(15, HIGH);
    delay(led_time);
    digitalWrite(15, LOW);
    delay(led_time);
      
    // Eliminar las inicializaciones de pines de radio
    // Eliminar attachInterrupt
    
    // Mantener inicialización de LED

    // ... (código LED original)
    
    // Inicialización I2C y MPU
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    // ... (código MPU original)
    
    // Inicialización de motores
    ESP32PWM::allocateTimer(0);
	  ESP32PWM::allocateTimer(1);
	  ESP32PWM::allocateTimer(2);
	  ESP32PWM::allocateTimer(3);
    // ... (código de inicialización de motores original)
    delay(1000);
    mot1.attach(mot1_pin,1000,2000);
    delay(1000);
    mot1.setPeriodHertz(ESCfreq);
    delay(100);
    mot2.attach(mot2_pin,1000,2000);
    delay(1000);
    mot2.setPeriodHertz(ESCfreq);
    delay(100);
    mot3.attach(mot3_pin,1000,2000);
    delay(1000);
    mot3.setPeriodHertz(ESCfreq);
    delay(100);
    mot4.attach(mot4_pin,1000,2000);
    delay(1000);
    mot4.setPeriodHertz(ESCfreq);
    delay(100);

    mot1.writeMicroseconds(1000);
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
      delay(500);
    digitalWrite(15, LOW);
    digitalWrite(15, HIGH);
    delay(500);
    digitalWrite(15, LOW);
    delay(500);
    // Valores de calibración
  RateCalibrationRoll=-1.37;
  RateCalibrationPitch=-1.80;
  RateCalibrationYaw=-0.28;
  AccXCalibration=0.19;
  AccYCalibration=-0.01;
  AccZCalibration=0.07;
  
  LoopTimer = micros();
}

uint8_t buffer[5];       // Buffer para datos Bluetooth

void loop() {
    // Leer datos Bluetooth si están disponibles
    if (SerialBT.available() >= 5) {
      SerialBT.readBytes(buffer, 5);
        
        // Mapear valores Bluetooth a canales de control (0-125 → 1000-2000μs)
      int x1 = buffer[0];
      int y1 = 125 - buffer[1];  // Invertir eje Y
      int x2 = buffer[2];
      int y2 = 125 - buffer[3];  // Invertir eje Y
      int aspersor = buffer[4];
      Serial.println(aspersor);


      ReceiverValue[0] = 1000 + (constrain(x1, 0, 125) * 8);      // Roll
      ReceiverValue[1] = 1000 + (constrain(y2, 0, 125) * 8);      // Pitch
      ReceiverValue[2] = 1000 + (constrain(y1, 0, 125) * 8);      // Throttle
      ReceiverValue[3] = 1000 + (constrain(x2, 0, 125) * 8);      // Yaw
    
      Serial.print(ReceiverValue[0]);
      if (aspersor == 0) {
        Serial.println("Bomba apagada");
        digitalWrite(Bomba, HIGH);
      }
      else if (aspersor == 1) {
        Serial.println("Bomba encendito");
        digitalWrite(Bomba, LOW);
      } 
    //uint32_t currentMillis = millis();
    //if (currentMillis - lastPrintTime >= printInterval) {
      //lastPrintTime = currentMillis;
    
      //Serial.print("Receiver Values: ");
      //Serial.print(ReceiverValue[0]); // Roll
      //Serial.print(" ");
      //Serial.print(ReceiverValue[1]); // Pitch
      //Serial.print(" ");
      //Serial.print(ReceiverValue[2]); // Throttle
      //Serial.print(" ");
      //Serial.println(ReceiverValue[3]); // Yaw
    //}
Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;


RateRoll -= RateCalibrationRoll;
Serial.print("RateRoll");
Serial.println(RateRoll);
RatePitch -= RateCalibrationPitch;
Serial.print("RatePitch");
Serial.println(RatePitch);
RateYaw -= RateCalibrationYaw;
Serial.print("RateYaw");
Serial.println(RateYaw);

AccX -= AccXCalibration ;
Serial.print("AccX");
Serial.println(AccX);
AccY -= AccYCalibration ;
Serial.print("AccY");
Serial.println(AccY);
AccZ -= AccZCalibration;
Serial.print("AccX");
Serial.println(AccZ);



  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;

// // Inlined Kalman Filter computation in the loop
// KalmanAngleRoll += t * RateRoll;
// KalmanUncertaintyAngleRoll += t * t * 16; // Variance of IMU (4 deg/s) squared
// KalmanGainRoll = KalmanUncertaintyAngleRoll / (KalmanUncertaintyAngleRoll + 9); // Error variance (3 deg) squared
// KalmanAngleRoll += KalmanGainRoll * (AngleRoll - KalmanAngleRoll);
// KalmanUncertaintyAngleRoll *= (1 - KalmanGainRoll);

// // Set output for Roll Kalman
// Kalman1DOutput[0] = KalmanAngleRoll;
// Kalman1DOutput[1] = KalmanUncertaintyAngleRoll;

// // Inlined Kalman Filter computation for Pitch
// KalmanAnglePitch += t * RatePitch;
// KalmanUncertaintyAnglePitch += t * t * 16; // Variance of IMU (4 deg/s) squared
// KalmanGainPitch = KalmanUncertaintyAnglePitch / (KalmanUncertaintyAnglePitch + 9); // Error variance (3 deg) squared
// KalmanAnglePitch += KalmanGainPitch * (AnglePitch - KalmanAnglePitch);
// KalmanUncertaintyAnglePitch *= (1 - KalmanGainPitch);

// // Set output for Pitch Kalman
// Kalman1DOutput[0] = KalmanAnglePitch;
// Kalman1DOutput[1] = KalmanUncertaintyAnglePitch;

// KalmanAngleRoll = (KalmanAngleRoll > 20) ? 20 : ((KalmanAngleRoll < -20) ? -20 : KalmanAngleRoll);
// KalmanAnglePitch = (KalmanAnglePitch > 20) ? 20 : ((KalmanAnglePitch < -20) ? -20 : KalmanAnglePitch);


complementaryAngleRoll=0.991*(complementaryAngleRoll+RateRoll*t) + 0.009*AngleRoll;
complementaryAnglePitch=0.991*(complementaryAnglePitch+RatePitch*t) + 0.009*AnglePitch;
// Clamping complementary filter roll angle to ±20 degrees
complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);



DesiredAngleRoll=0.1*(ReceiverValue[0]-1500);
DesiredAnglePitch=0.1*(ReceiverValue[1]-1500);
InputThrottle=ReceiverValue[2];
DesiredRateYaw=0.15*(ReceiverValue[3]-1500);


// Inlined PID equation for Roll
ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
PtermRoll = PAngleRoll * ErrorAngleRoll;
ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
DesiredRateRoll = PIDOutputRoll;
PrevErrorAngleRoll = ErrorAngleRoll;
PrevItermAngleRoll = ItermRoll;

ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
PtermPitch = PAnglePitch * ErrorAnglePitch;
ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
DesiredRatePitch = PIDOutputPitch;
PrevErrorAnglePitch = ErrorAnglePitch;
PrevItermAnglePitch = ItermPitch;

// Compute errors
ErrorRateRoll = DesiredRateRoll - RateRoll;
ErrorRatePitch = DesiredRatePitch - RatePitch;
ErrorRateYaw = DesiredRateYaw - RateYaw;

// Roll Axis PID
PtermRoll = PRateRoll * ErrorRateRoll;
ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

// Update output and previous values for Roll
InputRoll = PIDOutputRoll;
PrevErrorRateRoll = ErrorRateRoll;
PrevItermRateRoll = ItermRoll;

// Pitch Axis PID
PtermPitch = PRatePitch * ErrorRatePitch;
ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

// Update output and previous values for Pitch
InputPitch = PIDOutputPitch;
PrevErrorRatePitch = ErrorRatePitch;
PrevItermRatePitch = ItermPitch;

// Yaw Axis PID
PtermYaw = PRateYaw * ErrorRateYaw;
ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);  // Clamp ItermYaw to [-400, 400]
DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);  // Clamp PIDOutputYaw to [-400, 400]


// Update output and previous values for Yaw
InputYaw = PIDOutputYaw;
PrevErrorRateYaw = ErrorRateYaw;
PrevItermRateYaw = ItermYaw;


  if (InputThrottle > 1800)
  {
    InputThrottle = 1800;
  }

  
  MotorInput1 =  (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
  MotorInput2 =  (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
  MotorInput3 =  (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise
  MotorInput4 =  (InputThrottle + InputRoll - InputPitch + InputYaw); //front left - clockwise


  if (MotorInput1 > 2000)
  {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000)
  {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000)
  {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000)
  {
    MotorInput4 = 1999;
  }


// int ThrottleIdle = 1150;
// int ThrottleCutOff = 1000;
  if (MotorInput1 < ThrottleIdle)
  {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle)
  {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle)
  {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle)
  {
    MotorInput4 = ThrottleIdle;
  }

 if (ReceiverValue[2] < 1030 ) // dont Arm the motors
  {
   
  MotorInput1 = ThrottleCutOff;
  MotorInput2 = ThrottleCutOff;
  MotorInput3 = ThrottleCutOff;
  MotorInput4 = ThrottleCutOff;

  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
  
  }

// Calculate motor control values directly
mot1.writeMicroseconds(MotorInput1);
mot2.writeMicroseconds(MotorInput2);
mot3.writeMicroseconds(MotorInput3);
mot4.writeMicroseconds(MotorInput4);



//Reciever signals
  // Serial.print(ReceiverValue[0]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[1]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[2]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[3]);
  // Serial.print(" ");
 
  // Serial.print(ReceiverValue[4]);
  // Serial.print(" - ");
  // Serial.print(ReceiverValue[5]);
  // Serial.print(" - ");

// //Motor PWMs in us
//   Serial.print("MotVals-");
  // Serial.print(MotorInput1);
  // Serial.print("  ");
  // Serial.print(MotorInput2);
  // Serial.print("  ");
  // Serial.print(MotorInput3);
  // Serial.print("  ");
  // Serial.print(MotorInput4);
  // Serial.println(" ");

// //Reciever translated rates
//   Serial.print(DesiredRateRoll);
//   Serial.print("  ");
//   Serial.print(DesiredRatePitch);
//   Serial.print("  ");
//   Serial.print(DesiredRateYaw);
//   Serial.print(" -- ");

// // //IMU values
  // Serial.print("Acc values: ");
  // Serial.print("AccX:");
  // Serial.print(AccX);
  // Serial.print("  ");
  // Serial.print("AccY:");
  // Serial.print(AccY);
  // Serial.print("  ");
  // Serial.print("AccZ:");
  // Serial.print(AccZ);
  // Serial.print(" -- ");
  // Print the gyroscope values
  // Serial.print("Gyro values: ");
  // Serial.print(RateRoll);
  // Serial.print("  ");
  // Serial.print(RatePitch);
  // Serial.print("  ");
  // Serial.print(RateYaw);
  // Serial.print("  ");
  // Serial.print(" -- ");

//PID outputs
// Serial.print("PID O/P ");
// Serial.print(InputPitch);
//   Serial.print("  ");
// Serial.print(InputRoll);
//   Serial.print("  ");
// Serial.print(InputYaw);
//   Serial.print(" -- ");

//Angles from MPU
  // Serial.print("AngleRoll:");
  // Serial.print(AngleRoll);
  // //serial.print("  ");
  //   Serial.print("AnglePitch:");
  // Serial.print(AnglePitch);

  // Serial.print("KalmanAngleRoll:");
  // Serial.print(KalmanAngleRoll);
  // //serial.print("  ");
  //   Serial.print("KalmanAnglePitch:");
  // Serial.print(KalmanAnglePitch);

  // Serial.print("ComplementaryAngleRoll: ");
  // Serial.print(complementaryAngleRoll);
  // Serial.print("ComplementaryAnglePitch: ");
  // Serial.print(complementaryAnglePitch);

  // Serial.println(" ");  

  //  serial plotter comparison
  // Serial.print(KalmanAngleRoll);
  // Serial.print(" ");
  // Serial.print(KalmanAnglePitch);
  // Serial.print(" ");
  // Serial.print(complementaryAngleRoll);
  // Serial.print(" ");
  // Serial.println(complementaryAnglePitch);

  while (micros() - LoopTimer < (t*1000000)) {
      LoopTimer = micros();
    }
}}