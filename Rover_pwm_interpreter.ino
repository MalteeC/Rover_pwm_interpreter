// Rover PWM interpreter for Arduino Mega + INAV PWM outputs.
//
// FC PWM inputs, stored as integer pulse widths in microseconds:
//   S1 pin 2  = drive forward/back, 1500 stop
//   S2 pin 3  = steering left/right, 1500 center
//   S3 pin 18 = arm / shovel wheel lift up/down, 1500 stop
//   S4 pin 19 = shovel wheel rotation speed, 1200 off, 1800 max
//   S5 pin 20 = bunker up/down, <1400 slow one way, >1600 slow other way
//
// L298N outputs:
//   6 drive motors: 3 left, 3 right
//   1 arm lift motor
//   1 bunker lift motor
//
// DRV8825 for NEMA17 shovel wheel rotation:
//   EN is permanently wired to GND, so only STEP/DIR are controlled here.

const byte PWM_CHANNEL_COUNT = 5;
const byte pwmInputPins[PWM_CHANNEL_COUNT] = {2, 3, 18, 19, 20};

const int RC_MIN_US = 1000;
const int RC_CENTER_US = 1500;
const int RC_MAX_US = 2000;
const int RC_DEADBAND_US = 45;
const unsigned long SIGNAL_TIMEOUT_US = 300000UL;

volatile unsigned long pulseStartUs[PWM_CHANNEL_COUNT];
volatile unsigned long lastValidPulseUs[PWM_CHANNEL_COUNT];
volatile int rcUs[PWM_CHANNEL_COUNT] = {1500, 1500, 1500, 1200, 1500};

struct DcMotorPins {
  byte en;
  byte in1;
  byte in2;
};

const byte DRIVE_MOTORS_PER_SIDE = 3;

const DcMotorPins leftDriveMotors[DRIVE_MOTORS_PER_SIDE] = {
  {4, 22, 23},
  {5, 24, 25},
  {6, 26, 27}
};

const DcMotorPins rightDriveMotors[DRIVE_MOTORS_PER_SIDE] = {
  {7, 28, 29},
  {8, 30, 31},
  {9, 32, 33}
};

const DcMotorPins armLiftMotor = {10, 34, 35};
const DcMotorPins bunkerLiftMotor = {11, 36, 37};

const byte SHOVEL_STEP_PIN = 46;
const byte SHOVEL_DIR_PIN = 48;
const byte SHOVEL_FIXED_DIR = HIGH;
const int SHOVEL_OFF_US = 1200;
const int SHOVEL_MAX_US = 1800;
const unsigned int SHOVEL_MIN_STEPS_PER_SEC = 80;
const unsigned int SHOVEL_MAX_STEPS_PER_SEC = 1800;

unsigned long lastShovelStepUs = 0;
unsigned long shovelStepIntervalUs = 0;

void handlePwmChange(byte channel) {
  if (digitalRead(pwmInputPins[channel]) == HIGH) {
    pulseStartUs[channel] = micros();
    return;
  }

  unsigned long nowUs = micros();
  unsigned long pulseWidth = nowUs - pulseStartUs[channel];

  if (pulseWidth >= 800 && pulseWidth <= 2200) {
    rcUs[channel] = (int)pulseWidth;
    lastValidPulseUs[channel] = nowUs;
  }
}

void handleChannel1() { handlePwmChange(0); }
void handleChannel2() { handlePwmChange(1); }
void handleChannel3() { handlePwmChange(2); }
void handleChannel4() { handlePwmChange(3); }
void handleChannel5() { handlePwmChange(4); }

void setupMotor(const DcMotorPins &motor) {
  pinMode(motor.en, OUTPUT);
  pinMode(motor.in1, OUTPUT);
  pinMode(motor.in2, OUTPUT);
  analogWrite(motor.en, 0);
  digitalWrite(motor.in1, LOW);
  digitalWrite(motor.in2, LOW);
}

void setMotor(const DcMotorPins &motor, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed == 0) {
    analogWrite(motor.en, 0);
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, LOW);
    return;
  }

  if (speed > 0) {
    digitalWrite(motor.in1, HIGH);
    digitalWrite(motor.in2, LOW);
    analogWrite(motor.en, speed);
  } else {
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, HIGH);
    analogWrite(motor.en, -speed);
  }
}

void setMotorGroup(const DcMotorPins motors[], byte count, int speed) {
  for (byte i = 0; i < count; i++) {
    setMotor(motors[i], speed);
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

void driveFromChannels(int throttleUs, int steeringUs) {
  int throttle = centeredRcToSpeed(throttleUs);
  int steering = centeredRcToSpeed(steeringUs);

  int leftSpeed = constrain(throttle + steering, -255, 255);
  int rightSpeed = constrain(throttle - steering, -255, 255);

  setMotorGroup(leftDriveMotors, DRIVE_MOTORS_PER_SIDE, leftSpeed);
  setMotorGroup(rightDriveMotors, DRIVE_MOTORS_PER_SIDE, rightSpeed);
}

void armFromChannel(int armUs) {
  setMotor(armLiftMotor, centeredRcToSpeed(armUs));
}

void bunkerFromChannel(int bunkerUs) {
  const int bunkerSlowPwm = 90;

  if (bunkerUs < 1400) {
    setMotor(bunkerLiftMotor, -bunkerSlowPwm);
  } else if (bunkerUs > 1600) {
    setMotor(bunkerLiftMotor, bunkerSlowPwm);
  } else {
    setMotor(bunkerLiftMotor, 0);
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

void readChannelsSafe(int channels[]) {
  unsigned long nowUs = micros();
  unsigned long lastPulseCopy[PWM_CHANNEL_COUNT];

  noInterrupts();
  for (byte i = 0; i < PWM_CHANNEL_COUNT; i++) {
    channels[i] = rcUs[i];
    lastPulseCopy[i] = lastValidPulseUs[i];
  }
  interrupts();

  for (byte i = 0; i < PWM_CHANNEL_COUNT; i++) {
    if ((nowUs - lastPulseCopy[i]) > SIGNAL_TIMEOUT_US) {
      channels[i] = RC_CENTER_US;
    }
  }

  if ((nowUs - lastPulseCopy[3]) > SIGNAL_TIMEOUT_US) {
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

  for (byte i = 0; i < PWM_CHANNEL_COUNT; i++) {
    pinMode(pwmInputPins[i], INPUT);
    lastValidPulseUs[i] = micros();
  }

  for (byte i = 0; i < DRIVE_MOTORS_PER_SIDE; i++) {
    setupMotor(leftDriveMotors[i]);
    setupMotor(rightDriveMotors[i]);
  }

  setupMotor(armLiftMotor);
  setupMotor(bunkerLiftMotor);

  pinMode(SHOVEL_STEP_PIN, OUTPUT);
  pinMode(SHOVEL_DIR_PIN, OUTPUT);
  digitalWrite(SHOVEL_STEP_PIN, LOW);
  digitalWrite(SHOVEL_DIR_PIN, SHOVEL_FIXED_DIR);

  attachInterrupt(digitalPinToInterrupt(pwmInputPins[0]), handleChannel1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInputPins[1]), handleChannel2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInputPins[2]), handleChannel3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInputPins[3]), handleChannel4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInputPins[4]), handleChannel5, CHANGE);
}

void loop() {
  static unsigned long lastPrintMs = 0;
  int channels[PWM_CHANNEL_COUNT];

  readChannelsSafe(channels);

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

