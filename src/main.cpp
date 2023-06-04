#include <Arduino.h>
#include <Defines.h>
#include <Config.h>
#include <SimpleFOC.h>

// Hall sensor instance
// HallSensor(int hallA, int hallB , int hallC , int pp)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(2, 3, 4, 11);

// Interrupt routine initialization
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}


#ifdef HOVER_SERIAL
  #include <Hoverserial.h>
  Hoverserial hoverserial(Serial2);
#endif

#ifdef REMOTE
  #include <RCSwitch.h>
  RCSwitch mySwitch = RCSwitch();
#endif

#ifdef DEBUG_SERIAL
  #include <RTTStream.h>
  RTTStream rtt;
#endif

#ifdef MPU6050
  #include <Adafruit_MPU6050.h>
  Adafruit_MPU6050 mpu;
  #define IMU MPU6050
#endif

#ifdef ICM20948
  #include <Adafruit_ICM20948.h>
  #include <Adafruit_ICM20X.h>
  Adafruit_ICM20948 mpu;
  #define IMU ICM20948
#endif

#ifdef IMU
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
#endif //IMU

class CIO
{	
private:
	int m_iPin;
public:
	CIO(int iPin);
	void Setup(int iType = INPUT_PULLUP);
	void Set(bool bOn = true);
	bool Get(void);
};

CIO::CIO(int iPin){	m_iPin = iPin;  }
void CIO::Setup(int iType){  pinMode(m_iPin, iType);  }	
void CIO::Set(bool bOn){ digitalWrite(m_iPin,bOn); }	
bool CIO::Get(void){  return digitalRead(m_iPin); }

CIO oLedRed = CIO(LED1_PIN);
CIO oLedGreen = CIO(LED3_PIN);
CIO oLedGreen2 = CIO(LED2_PIN);

CIO aoLed[5] = {oLedGreen, oLedGreen2, oLedRed, CIO(LED4_PIN), CIO(LED5_PIN) };


// ########################## SETUP ##########################
void setup(){
  
  for (int i=0; i<5; i++) 
  {
    aoLed[i].Setup(OUTPUT);
    aoLed[i].Set(HIGH);
    delay(200);
  }
  delay(800);
  for (int i=0; i<5; i++) aoLed[i].Set(LOW);

  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);

  #ifdef HOVER_SERIAL
    Serial2.begin(HOVER_SERIAL_BAUD);
    //robo using HOVER_SERIAL instead of DEBUG_SERIAL=ST-Link
    Serial2.println("Hoverboard Serial v1.0, this is HOVER_SERIAL");
  #endif

  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Hoverboard Serial v1.0");
  #endif

  #ifdef REMOTE
    mySwitch.enableReceive(digitalPinToInterrupt(RECEIVER_PIN));  // Receiver on interrupt
  #endif
  
  #ifdef IMU
    #ifdef MPU6050
    if (!mpu.begin()){
    #endif
    #ifdef ICM20948
    if (!mpu.begin_I2C()){
    #endif
      digitalWrite(LED1_PIN,HIGH);
      #ifdef DEBUG_SERIAL
      DEBUG_SERIAL.println(F("Failed to find IMU chip"));
      #endif
    }else{
      digitalWrite(LED2_PIN,HIGH);
      #ifdef DEBUG_SERIAL
      DEBUG_SERIAL.println(F("IMU found!"));
      #endif

      mpu_temp = mpu.getTemperatureSensor();
      #ifdef DEBUG_SERIAL
        mpu_temp->printSensorDetails();
      #endif

      mpu_accel = mpu.getAccelerometerSensor();
      #ifdef DEBUG_SERIAL
        mpu_accel->printSensorDetails();
      #endif

      mpu_gyro = mpu.getGyroSensor();
      #ifdef DEBUG_SERIAL
        mpu_gyro->printSensorDetails();
      #endif
    }
  #endif //IMU 
}


// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iAnalog1 = 250;   // global variable for simple low-pass filter
int iAnalog2 = 1000;  // global variable for simple low-pass filter

#ifdef TEST
int iTest = 0;
int iStep = SPEED_STEP;
#endif

void loop(){

  #ifdef HOVER_SERIAL
    // Check for new received data
    hoverserial.receive();
  #endif

  // Delay
  unsigned long timeNow = millis();

  iAnalog1 = analogRead(SENSOR1_PIN);
  //delay(10);
  iAnalog2 = analogRead(SENSOR2_PIN);
  digitalWrite(LED1_PIN, (timeNow % (iAnalog1*2)) < iAnalog1 );
  digitalWrite(LED2_PIN, (timeNow % (iAnalog2*2)) < iAnalog2 );
  
  //iAnalog1 = (9*iAnalog1 + analogRead(SENSOR1_PIN) ) / 10;
  //iAnalog2 = (9*iAnalog2 + analogRead(SENSOR2_PIN) ) / 10;
  //oLedRed.Set( (timeNow % (iAnalog1*2)) < iAnalog1 );
  //oLedGreen.Set( (timeNow % (iAnalog2*2)) < iAnalog2 );


  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;

  //robo using HOVER_SERIAL instead of DEBUG_SERIAL=ST-Link
  Serial2.print("A4: ");Serial2.print(iAnalog1);Serial2.print("\t C14: ");Serial2.println(iAnalog2);

  #ifdef IMU
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    
    mpu_temp->getEvent(&temp);
    mpu_accel->getEvent(&accel);
    mpu_gyro->getEvent(&gyro);

    #ifdef DEBUG_SERIAL
      DEBUG_SERIAL.print("Temperature ");
      DEBUG_SERIAL.print(temp.temperature);
      DEBUG_SERIAL.println(" deg C");

      /* Display the results (acceleration is measured in m/s^2) */
      DEBUG_SERIAL.print("Accel X: ");
      DEBUG_SERIAL.print(accel.acceleration.x);
      DEBUG_SERIAL.print(" \tY: ");
      DEBUG_SERIAL.print(accel.acceleration.y);
      DEBUG_SERIAL.print(" \tZ: ");
      DEBUG_SERIAL.print(accel.acceleration.z);
      DEBUG_SERIAL.println(" m/s^2 ");

      /* Display the results (rotation is measured in rad/s) */
      DEBUG_SERIAL.print("Gyro X: ");
      DEBUG_SERIAL.print(gyro.gyro.x);
      DEBUG_SERIAL.print(" \tY: ");
      DEBUG_SERIAL.print(gyro.gyro.y);
      DEBUG_SERIAL.print(" \tZ: ");
      DEBUG_SERIAL.print(gyro.gyro.z);
      DEBUG_SERIAL.println(" radians/s ");
      DEBUG_SERIAL.println();
    #endif
  #endif //IMU

  #ifdef REMOTE
    if (mySwitch.available()) {
      #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("Received ");
        DEBUG_SERIAL.print( mySwitch.getReceivedValue() );
        DEBUG_SERIAL.print(" / ");
        DEBUG_SERIAL.print( mySwitch.getReceivedBitlength() );
        DEBUG_SERIAL.print("bit ");
        DEBUG_SERIAL.print("Protocol: ");
        DEBUG_SERIAL.println( mySwitch.getReceivedProtocol() );
      #endif

      switch(mySwitch.getReceivedValue()){
        case REMOTE_BUTTON1:
          hoverserial.setCtrlTyp(FOC_CTRL);
          break;
        case REMOTE_BUTTON2:
          hoverserial.setCtrlTyp(SIN_CTRL);
          break;
        case REMOTE_BUTTON3:
          hoverserial.setCtrlTyp(COM_CTRL);
          break;
        case REMOTE_BUTTON4:
          break;
      }

      mySwitch.resetAvailable();
    }
  #endif

  #ifdef HOVER_SERIAL
    #ifdef TEST
      hoverserial.setCmd2(iTest);
      // Calculate test command signal
      iTest += iStep;

      // invert step if reaching limit
      if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
        iStep *= -1;
    #else
      hoverserial.setCmd2(0);
    #endif
    //robo using HOVER_SERIAL instead of DEBUG_SERIAL=ST-Link
    //hoverserial.send();
  #endif
}