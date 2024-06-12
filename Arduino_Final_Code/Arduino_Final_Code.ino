#include "Motor2PWM.h"
#include <AccelStepper.h>
#include <ArduinoBLE.h>

//
// BLE SETUP
//
BLEService botanistArduino("1214");
BLECharacteristic drivetrain("9873", BLERead | BLEWrite, 2);
BLECharacteristic lift("7758", BLERead | BLEWrite, 2);
BLECharacteristic directionSwap("e94a", BLERead | BLEWrite, 4);
int swap = 0;
//
//
//

//
// DRIVETRAIN SETTINGS
//
// maxPWM not 255 because DBH-12V motor driver requires <98% demand cycle
#define maxPWM 250

#define R_S_f 3  //ir sensor Right front
#define L_S_f 5  //ir sensor Left front
#define R_S_b 6  //ir sensor Right back
#define L_S_b 4  //ir sensor Left back

Motor motorA(11, 12, maxPWM);
Motor motorB(7, 8, maxPWM);
int pmw = 255;
// 0 is tank side 1 is lift side???
bool dir = 1;
//
//
//

//
// LIFT SETTINGS
//
AccelStepper liftMotor(1, 16, 17); // pulse, dir
#define liftEnable 18

int loopCount = 0;
int16_t liftValue = 0;
//
//
//

void setup() {
  pinMode(R_S_f, INPUT);
  pinMode(L_S_f, INPUT);
  pinMode(R_S_b, INPUT);
  pinMode(L_S_b, INPUT);

  liftMotor.setMaxSpeed(1000);
  liftMotor.setAcceleration(500);

  Serial.begin(9600);
  //while(!Serial);

  // BLE Initialization
  if(!BLE.begin()){
    Serial.println("starting BLE Failed");
    while(1) {
      digitalWrite(LEDR, LOW);
      delay(500);
      digitalWrite(LEDR, HIGH);
      delay(500);
    }
  }

  digitalWrite(LEDR, LOW);

  // ble setup
  BLE.setLocalName("botanistArduino");
  BLE.setAdvertisedService(botanistArduino);
  botanistArduino.addCharacteristic(drivetrain);
  botanistArduino.addCharacteristic(lift);
  botanistArduino.addCharacteristic(directionSwap);
  BLE.addService(botanistArduino);
  Serial.println(BLE.address());

  // Set event handlers
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  uint8_t initialDrive[2];
  initialDrive[0] = initialDrive[1] = 0;
  drivetrain.writeValue(initialDrive,2);
  drivetrain.setEventHandler(BLEWritten, drivetrainActivation);
  
  lift.setValue(0);
  liftMotor.setSpeed(0);
  digitalWrite(liftEnable, HIGH);
  lift.setEventHandler(BLEWritten, liftActivation);

  directionSwap.writeValue(&swap, 4);

  BLE.advertise();
  Serial.println("Advertising");

}


void loop() {
  // Poll funciton just refers to event handlers as the events happen, as set in setup
  if (liftValue == 0) {
    BLE.poll();
  }
  else {
    if(loopCount > 5) {
      BLE.poll();
      loopCount = 0;
    }

    liftMotor.runSpeed();
    loopCount++;
  }
}

/*
  bool irR;
  bool irL;

  Serial.println(String(dir) + ' ' + String(digitalRead(R_S_f)) + ' ' + String(digitalRead(L_S_f)) + ' ' + String(digitalRead(R_S_b)) + ' ' + String(digitalRead(L_S_b)));

  if(dir) {
    irR = digitalRead(R_S_f);
    irL = digitalRead(L_S_f); 
  } else {
    irR = digitalRead(R_S_b);
    irL = digitalRead(L_S_b); 
  }

  if (!irR && !irL)
    forward(getMod(dir));
  if (irR && !irL)
    turnRight(getMod(dir));
  if (!irR && irL)
    turnLeft(getMod(dir));
  if (irR && irL) {
    liftMotor.setSpeed(80); // 80/100
    liftMotor.step(STEPS_PER_REV);
    liftMotor.step(-STEPS_PER_REV);
    dir = !dir;
  }
}
*/

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDB, LOW);
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());

  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDR, LOW);

  Stop();

  //liftMotor.setSpeed(50);
  //for(int i = 0; i<300; i++)
  // liftMotor.step(STEPS_PER_REV/100); // >0 away from motor  
}

void liftActivation(BLEDevice central, BLECharacteristic characteristic) {
  // eek ahh oww
  Serial.println("lift event");

  lift.readValue(&liftValue, 2);
  Serial.println(liftValue);

  if(liftValue == 0) {
    digitalWrite(liftEnable, HIGH);
  }
  else {
    digitalWrite(liftEnable, LOW);
  }

  liftMotor.setSpeed(liftValue);
}

void drivetrainActivation(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.println("drivetrain event, written: ");

  // Store and print values: first element is PWM input
  uint8_t charValue[2];
  drivetrain.readValue(charValue, 2);
  Serial.println(String(charValue[0]) + ' ' + String(charValue[1]));

  // turn 0/1 to -1/1 direction
  int PWM = charValue[0];
  int dir = getDirMod(charValue[1]);
  
  if(PWM==0) {
    Stop();
    return;
  }

  bool irR, irL;
  if(charValue[1]) {
    irR = digitalRead(R_S_b);
    irL = digitalRead(L_S_b); 
  } else {
    irR = digitalRead(R_S_f);
    irL = digitalRead(L_S_f); 
  }

  if (!irR && !irL)
    forward(dir, PWM);
  if (irR && !irL)
    turnRight(dir, PWM);
  if (!irR && irL)
    turnLeft(dir, PWM);
  if (irR && irL) {
    swap = 1;
    directionSwap.writeValue(&swap, 4);
  }
}

void forward(int mod, int pwm) {
  motorA.drive(mod*pwm);
  motorB.drive(mod*pwm);
}

void turnRight(int mod, int pwm) {
  motorA.drive(mod*pwm);
  motorB.drive(-mod*pwm);
}

void turnLeft(int mod, int pwm) {
  motorA.drive(-mod*pwm);
  motorB.drive(mod*pwm);
}

int getDirMod(bool dir) {
  if(dir)
    return -1;
  else
    return 1;
}

void Stop() {
  motorA.drive(0);
  motorB.drive(0);
}

void oh_no() {
  digitalWrite(LEDB, HIGH);
  while(1) {
    digitalWrite(LEDR, LOW);
    delay(500);
    digitalWrite(LEDR, HIGH);
    delay(500);
  }
}
