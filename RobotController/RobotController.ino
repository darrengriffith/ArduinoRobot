#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <CurieBLE.h>

//
// Globals for Gyro
//
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

//
// Globals for BLE
//
BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)
BLEService inclinometerService("B1433ADF-3BB6-4BCE-BC92-58F2A637FC7B"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEFloatCharacteristic pitchCharacteristic("472E0238-2829-4CB4-9DB9-4CC3667B776D", BLERead);
BLEFloatCharacteristic rollCharacteristic("C6E426D3-729E-45D4-8D73-B15148026B36", BLERead);
BLEFloatCharacteristic yawCharacteristic("024629AD-5363-45DA-8760-3CF259356B8C", BLERead);

BLECentral central = blePeripheral.central();

//
// Globals for Tracks
//
int currentDirection = 0;
long previousDistance = 0; 
long distance = 0;

void setup() {

  //Serial.begin (9600);

  //
  // SETUP I/O
  //
  pinMode(3, OUTPUT); // Distance
  pinMode(4, INPUT);
  
  pinMode(5, OUTPUT); // Speed control
  pinMode(6, OUTPUT);
  
  pinMode(8, OUTPUT); // Track control
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  
  pinMode(13, OUTPUT); // BLE status
  
  //
  // SETUP GYRO
  // 
  
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();

  //
  // SETUP BLE
  // 

  // set advertised local name and service UUID:
  blePeripheral.setLocalName("ROBOT 1");
  blePeripheral.setAdvertisedServiceUuid(inclinometerService.uuid());

  // add service and characteristic:
  blePeripheral.addAttribute(inclinometerService);
  blePeripheral.addAttribute(pitchCharacteristic);
  blePeripheral.addAttribute(rollCharacteristic);
  blePeripheral.addAttribute(yawCharacteristic);

  // set the initial value for the characeristic:
  pitchCharacteristic.setValue(0);
  rollCharacteristic.setValue(0);
  yawCharacteristic.setValue(0);

  // begin advertising BLE service:
  blePeripheral.begin();

  for(int i = 0; i < 10; i++) {
    digitalWrite(13, HIGH);  
    delay(50);
    digitalWrite(13, LOW);  
    delay(50);       
  }
} 
void trackSpeed(int speed) {
  analogWrite(5, speed);
  analogWrite(6, speed);
}

void stopBoth() {
  digitalWrite(8, LOW);  // Turn off the right motor
  digitalWrite(9, LOW);  // Turn off the right motor
  digitalWrite(10, LOW);  // Turn off the left motor
  digitalWrite(11, LOW);  // Turn off the left motor

  trackSpeed(0);

  currentDirection = 0;
}

void leftForward() {
  digitalWrite(8, LOW);  
  digitalWrite(9, HIGH);  
}

void leftBackward() {
  digitalWrite(8, HIGH);  
  digitalWrite(9, LOW);  
}

void rightForward() {
  digitalWrite(10, HIGH);  
  digitalWrite(11, LOW);  
}

void rightBackward() {
  digitalWrite(10, LOW);  
  digitalWrite(11, HIGH);  
}

void forward() {
  leftForward();
  rightForward();

  currentDirection = 1;
}

void left() {
  leftBackward();
  rightForward();

  currentDirection = 2;
}

void loop() {
  long duration;

  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long microsNow;

  digitalWrite(3, HIGH);
  delayMicroseconds(10);
  digitalWrite(3, LOW);
  duration = pulseIn(4, HIGH);
  previousDistance = distance;
  distance = (duration/2) / 29.1;
  //Serial.println(distance);

  // if a central is connected to peripheral:
  if (!central) {
    central = blePeripheral.central();
  }
  else
  {
    if (!central.connected()) {
      digitalWrite(13, LOW); // will turn the LED off
        
      stopBoth();
    }
    else
    {
      digitalWrite(13, HIGH); // will turn the LED on

      if(previousDistance == distance) {
        if(distance < 5) {
          stopBoth();
        } else if(distance > 30) {
          trackSpeed(150);
          forward();
        } else {
          trackSpeed(200);
          left();
        }
      }

      // check if it's time to read data and update the filter
      microsNow = micros();
      if (microsNow - microsPrevious >= microsPerReading) {
            
        // read raw data from CurieIMU
        CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
    
        // convert from raw data to gravity and degrees/second units
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);
    
        // update the filter, which computes orientation
        filter.updateIMU(gx, gy, gz, ax, ay, az);
    
        // print the heading, pitch and roll
        pitchCharacteristic.setValue(filter.getPitch());
        rollCharacteristic.setValue(filter.getRoll());
        yawCharacteristic.setValue(filter.getYaw());
    
        // increment previous time, so we keep proper pace
        microsPrevious = microsPrevious + microsPerReading;
      
      }
    }
  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
