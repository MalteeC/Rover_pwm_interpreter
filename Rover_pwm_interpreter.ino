// Rover PWM and serial interpreter for Arduino Mega.
//
// Input control:
//   FC PWM inputs are read on S1-S5 pins below.
//   Serial override accepts one line: s1,s2,s3,s4,s5\n at 115200 baud.
//   Example from Python: 1500,1500,1500,1200,1500
//
// FC PWM inputs:
//   S1 pin 40 = drive forward/back, 1500 stop
//   S2 pin 38 = steering left/right, 1500 center
//   S3 pin 36 = arm up/down, 1500 stop
//   S4 pin 34 = shovel wheel stepper speed, 1200 off, 1800 max
//   S5 pin 32 = bunker up/down, <1400 one way, >1600 other way
//
// L298N motor pins, two wires per motor:
//   A pins are direction pins, B pins are native Mega PWM pins.
//   M1 left front:   A 24, B 11
//   M2 right front:  A 28, B 7
//   M3 left middle:  A 22, B 12
//   M4 right middle: A 26, B 8
//   M5 left rear:    A 23, B 13
//   M6 right rear:   A 27, B 9
//   M7 arm:          A 25, B 10
//   M8 bunker:       A 29, B 6
//
// DRV8825 for NEMA17 shovel wheel rotation:
//   STEP pin 45
//   DIR  pin 44
//   EN is permanently wired to GND.

const byte CHANNEL_COUNT = 5;
const byte pwmInputPins[CHANNEL_COUNT] = {40, 38, 36, 34, 32};

const int RC_MIN_US = 1000;
const int RC_CENTER_US = 1500;
const int RC_MAX_US = 2000;
const int RC_DEADBAND_US = 45;
const unsigned long PWM_SIGNAL_TIMEOUT_US = 300000UL;
const unsigned long SERIAL_CONTROL_TIMEOUT_MS = 300UL;

int rcUs[CHANNEL_COUNT] = {1500, 1500, 1500, 1200, 1500};
int serialUs[CHANNEL_COUNT] = {1500, 1500, 1500, 1200, 1500};
unsigned long lastValidPwmUs[CHANNEL_COUNT] = {0, 0, 0, 0, 0};
unsigned long lastSerialPacketMs = 0;

struct MotorPins {
  byte a;
  byte b;
};

const byte MOTOR_COUNT = 8;
const byte DRIVE_MOTORS_PER_SIDE = 3;

const MotorPins motors[MOTOR_COUNT] = {
  {24, 11},  // M1: left front, B is PWM
  {28, 7},   // M2: right front, B is PWM
  {22, 12},  // M3: left middle, B is PWM
  {26, 8},   // M4: right middle, B is PWM
  {23, 13},  // M5: left rear, B is PWM
  {27, 9},   // M6: right rear, B is PWM
  {25, 10},  // M7: arm, B is PWM
  {29, 6}    // M8: bunker, B is PWM
};

const byte leftDriveMotorNumbers[DRIVE_MOTORS_PER_SIDE] = {1, 3, 5};
const byte rightDriveMotorNumbers[DRIVE_MOTORS_PER_SIDE] = {2, 4, 6};
const byte ARM_MOTOR_NUMBER = 7;
const byte BUNKER_MOTOR_NUMBER = 8;

const byte SHOVEL_DIR_PIN = 44;
const byte SHOVEL_STEP_PIN = 45;
const byte SHOVEL_FIXED_DIR = HIGH;
const int SHOVEL_OFF_US = 1200;
const int SHOVEL_MAX_US = 1800;
const unsigned int SHOVEL_MIN_STEPS_PER_SEC = 80;
const unsigned int SHOVEL_MAX_STEPS_PER_SEC = 1800;

unsigned long lastShovelStepUs = 0;
unsigned long shovelStepIntervalUs = 0;

void setupMotor(const MotorPins &motor) {
  pinMode(motor.a, OUTPUT);
  pinMode(motor.b, OUTPUT);
  digitalWrite(motor.a, LOW);
  analogWrite(motor.b, 0);
}

void setMotor(const MotorPins &motor, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed == 0) {
    digitalWrite(motor.a, LOW);
    analogWrite(motor.b, 0);
    return;
  }

  if (speed > 0) {
    digitalWrite(motor.a, HIGH);
    analogWrite(motor.b, 255 - speed);
  } else {
    digitalWrite(motor.a, LOW);
    analogWrite(motor.b, -speed);
  }
}

void setMotorByNumber(byte motorNumber, int speed) {
  if (motorNumber < 1 || motorNumber > MOTOR_COUNT) {
    return;
  }

  setMotor(motors[motorNumber - 1], speed);
}

void setDriveMotorGroup(const byte motorNumbers[], byte count, int speed) {
  for (byte i = 0; i < count; i++) {
    setMotorByNumber(motorNumbers[i], speed);
  }
}

int centeredRcToSpeed(int rcValue) {
  int centered = constrain(rcValue, RC_MIN_US, RC_MAX_US) - RC_CENTER_US;

  if (abs(centered) <= RC_DEADBAND_US) {
    return 0;
  }

  if (centered > 0) {
    return map(centered, RC_DEADBAND_US, 500, 0, 255);
  }

  return map(centered, -RC_DEADBAND_US, -500, 0, -255);
}

int centeredRcToFullSpeed(int rcValue) {
  int centered = constrain(rcValue, RC_MIN_US, RC_MAX_US) - RC_CENTER_US;

  if (abs(centered) <= RC_DEADBAND_US) {
    return 0;
  }

  return centered > 0 ? 255 : -255;
}

void driveFromChannels(int throttleUs, int steeringUs) {
  int throttle = centeredRcToSpeed(throttleUs);
  int steering = centeredRcToSpeed(steeringUs);

  int leftSpeed = constrain(throttle + steering, -255, 255);
  int rightSpeed = constrain(throttle - steering, -255, 255);

  setDriveMotorGroup(leftDriveMotorNumbers, DRIVE_MOTORS_PER_SIDE, leftSpeed);
  setDriveMotorGroup(rightDriveMotorNumbers, DRIVE_MOTORS_PER_SIDE, rightSpeed);
}

void armFromChannel(int armUs) {
  setMotorByNumber(ARM_MOTOR_NUMBER, centeredRcToFullSpeed(armUs));
}

void bunkerFromChannel(int bunkerUs) {
  if (bunkerUs < 1400) {
    setMotorByNumber(BUNKER_MOTOR_NUMBER, -255);
  } else if (bunkerUs > 1600) {
    setMotorByNumber(BUNKER_MOTOR_NUMBER, 255);
  } else {
    setMotorByNumber(BUNKER_MOTOR_NUMBER, 0);
  }
}

void updateShovelStepperSpeed(int shovelUs) {
  if (shovelUs <= SHOVEL_OFF_US) {
    shovelStepIntervalUs = 0;
    return;
  }

  shovelUs = constrain(shovelUs, SHOVEL_OFF_US, SHOVEL_MAX_US);
  unsigned int stepsPerSec = map(shovelUs, SHOVEL_OFF_US, SHOVEL_MAX_US,
                                 SHOVEL_MIN_STEPS_PER_SEC, SHOVEL_MAX_STEPS_PER_SEC);

  shovelStepIntervalUs = 1000000UL / stepsPerSec;
}

void runShovelStepper() {
  if (shovelStepIntervalUs == 0) {
    digitalWrite(SHOVEL_STEP_PIN, LOW);
    return;
  }

  unsigned long nowUs = micros();

  if (nowUs - lastShovelStepUs >= shovelStepIntervalUs) {
    lastShovelStepUs = nowUs;
    digitalWrite(SHOVEL_STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(SHOVEL_STEP_PIN, LOW);
  }
}

void updatePwmInputs() {
  unsigned long nowUs = micros();

  for (byte i = 0; i < CHANNEL_COUNT; i++) {
    unsigned long pulseWidth = pulseIn(pwmInputPins[i], HIGH, 2500UL);

    if (pulseWidth >= 800 && pulseWidth <= 2200) {
      rcUs[i] = (int)pulseWidth;
      lastValidPwmUs[i] = nowUs;
    }
  }
}

bool parseSerialPacket(char *line) {
  int parsed[CHANNEL_COUNT];
  char *token = strtok(line, ",");

  for (byte i = 0; i < CHANNEL_COUNT; i++) {
    if (token == NULL) {
      return false;
    }

    parsed[i] = constrain(atoi(token), 800, 2200);
    token = strtok(NULL, ",");
  }

  for (byte i = 0; i < CHANNEL_COUNT; i++) {
    serialUs[i] = parsed[i];
  }

  lastSerialPacketMs = millis();
  return true;
}

void updateSerialControl() {
  static char lineBuffer[48];
  static byte lineLength = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      lineBuffer[lineLength] = '\0';
      parseSerialPacket(lineBuffer);
      lineLength = 0;
      continue;
    }

    if (lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = c;
    } else {
      lineLength = 0;
    }
  }
}

void readChannels(int channels[]) {
  bool useSerial = (millis() - lastSerialPacketMs) <= SERIAL_CONTROL_TIMEOUT_MS;
  unsigned long nowUs = micros();

  if (useSerial) {
    for (byte i = 0; i < CHANNEL_COUNT; i++) {
      channels[i] = serialUs[i];
    }
    return;
  }

  for (byte i = 0; i < CHANNEL_COUNT; i++) {
    channels[i] = ((nowUs - lastValidPwmUs[i]) > PWM_SIGNAL_TIMEOUT_US) ? RC_CENTER_US : rcUs[i];
  }

  if ((nowUs - lastValidPwmUs[3]) > PWM_SIGNAL_TIMEOUT_US) {
    channels[3] = SHOVEL_OFF_US;
  }
}

void printChannels(const int channels[]) {
  Serial.print("S1=");
  Serial.print(channels[0]);
  Serial.print(" S2=");
  Serial.print(channels[1]);
  Serial.print(" S3=");
  Serial.print(channels[2]);
  Serial.print(" S4=");
  Serial.print(channels[3]);
  Serial.print(" S5=");
  Serial.println(channels[4]);
}

void setup() {
  Serial.begin(115200);

  for (byte i = 0; i < CHANNEL_COUNT; i++) {
    pinMode(pwmInputPins[i], INPUT);
    lastValidPwmUs[i] = micros();
  }

  for (byte i = 0; i < MOTOR_COUNT; i++) {
    setupMotor(motors[i]);
  }

  pinMode(SHOVEL_STEP_PIN, OUTPUT);
  pinMode(SHOVEL_DIR_PIN, OUTPUT);
  digitalWrite(SHOVEL_STEP_PIN, LOW);
  digitalWrite(SHOVEL_DIR_PIN, SHOVEL_FIXED_DIR);
}

void loop() {
  static unsigned long lastPrintMs = 0;
  int channels[CHANNEL_COUNT];

  updateSerialControl();
  updatePwmInputs();
  readChannels(channels);

  driveFromChannels(channels[0], channels[1]);
  armFromChannel(channels[2]);
  updateShovelStepperSpeed(channels[3]);
  bunkerFromChannel(channels[4]);
  runShovelStepper();

  if (millis() - lastPrintMs >= 250) {
    lastPrintMs = millis();
    printChannels(channels);
  }
}



