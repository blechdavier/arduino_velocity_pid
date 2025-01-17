#include <Encoder.h>
#include <L298NX2.h>

typedef union {
  float floatingPoint;
  uint8_t binary[4];
} binaryFloat;

typedef union {
  long longInteger;
  uint8_t binary[4];
} binaryLong;

// initialize encoders
Encoder leftEncoder(A0, A1);
Encoder rightEncoder(A2, A3);

const unsigned int EN_A = 10;
const unsigned int IN1_A = 9;
const unsigned int IN2_A = 8;

const unsigned int IN1_B = 7;
const unsigned int IN2_B = 6;
const unsigned int EN_B = 5;

// initialize motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

#define FILTER_CONSTANT 0.93
#define FILTER_CONSTANT_2 (1.0-FILTER_CONSTANT)/2.0

#define ODOM_MS 20

long prevT = 0;
long prevSentOdomTimestamp = 0;

binaryLong leftPosition;
binaryLong rightPosition;

float leftVelFiltered = 0.0;
float leftVelPrevious = 0.0;
float rightVelFiltered = 0.0;
float rightVelPrevious = 0.0;

float kP = 0.6;
binaryFloat leftSetpoint;
binaryFloat rightSetpoint;

void setup() {
  Serial.begin(115200);
}

void loop() {
  updateFilteredVelocities();

  float leftMotorOutput = controller(leftVelFiltered, leftSetpoint.floatingPoint);
  float rightMotorOutput = controller(rightVelFiltered, rightSetpoint.floatingPoint);

  setLeftMotorOutput(leftMotorOutput);
  setRightMotorOutput(rightMotorOutput);
  handlePackets();
}

float controller(float currentVel, float setpoint) {
  float error = setpoint - currentVel;
  float feedForward = 0;
  if (setpoint > 300) {
    feedForward = -1.52945e-12 * (setpoint*setpoint*setpoint*setpoint) + 2.06452e-8 * (setpoint*setpoint*setpoint) - 0.0000546066 * (setpoint*setpoint) + 0.074348 * setpoint + 28.35074;
  } else if (setpoint < -300) {
    feedForward = 1.52945e-12 * (setpoint*setpoint*setpoint*setpoint) + 2.06452e-8 * (setpoint*setpoint*setpoint) + 0.0000546066 * (setpoint*setpoint) + 0.074348 * setpoint - 28.35074;
  }
  return feedForward + kP * error;
}

void handlePackets() {
  while (Serial.available() >= 5) {
    // read packet
    uint8_t packetId = Serial.read();
    uint8_t bytes[4] = {Serial.read(), Serial.read(), Serial.read(), Serial.read()};

    switch (packetId) {
      case 0:
      memcpy(&leftSetpoint, bytes, 4);
      break;
      case 1:
      memcpy(&rightSetpoint, bytes, 4);
      break;
    }
  }
  long currT = millis();

  // target 50hz, don't send old data though
  if (currT - prevSentOdomTimestamp > 2 * ODOM_MS) {
    sendOdometry();
    prevSentOdomTimestamp = currT;
  } else if (currT - prevSentOdomTimestamp > ODOM_MS) {
    sendOdometry();
    prevSentOdomTimestamp += ODOM_MS;
  }
}

void sendOdometry() {
  Serial.write(leftPosition.binary, 4);
  Serial.write(rightPosition.binary, 4);
  // Serial.println(leftPosition.longInteger);
  // Serial.println(rightPosition.longInteger);
}

void updateFilteredVelocities() {
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  prevT = currT;

  long newLeftPosition = leftEncoder.read();
  long newRightPosition = rightEncoder.read();

  float leftVel = (newLeftPosition - leftPosition.longInteger) / deltaT;
  float rightVel = (newRightPosition - rightPosition.longInteger) / deltaT;
  leftPosition.longInteger = newLeftPosition;
  rightPosition.longInteger = newRightPosition;

  leftVelFiltered = FILTER_CONSTANT*leftVelFiltered + FILTER_CONSTANT_2*leftVel + FILTER_CONSTANT_2*leftVelPrevious;
  leftVelPrevious = leftVel;
  rightVelFiltered = FILTER_CONSTANT*rightVelFiltered + FILTER_CONSTANT_2*rightVel + FILTER_CONSTANT_2*rightVelPrevious;
  rightVelPrevious = rightVel;
}

void setLeftMotorOutput(float outputFloat) {
  int output = max(min(int(outputFloat), 255), -255);
  if (output > 0) {
    motors.backwardA();
  } else {
    motors.forwardA();
  }
  motors.setSpeedA(abs(output));
}

void setRightMotorOutput(float outputFloat) {
  int output = max(min(int(outputFloat), 255), -255);
  if (output > 0) {
    motors.forwardB();
  } else {
    motors.backwardB();
  }
  motors.setSpeedB(abs(output));
}
