// EV 2026, travel a set distance in a set time, gyro only
// Time-based control (no encoders)

#include <TektiteRotEv.h>

//COMP PARAMETERS 
#define TARGET_DISTANCE_M   2.00f   // meters
#define TARGET_TIME_S       15.0f   // seconds

#define MOTOR_LEFT          2
#define MOTOR_RIGHT         1
#define MOTOR_LEFT_SIGN     1.0f
#define MOTOR_RIGHT_SIGN   -1.0f

#define DEADBAND_LEFT       0.16f
#define DEADBAND_RIGHT      0.22f
#define KFF_LEFT            0.58f
#define KFF_RIGHT           0.73f

#define BRAKE_DUTY          0.30f
#define BRAKE_DURATION_MS   80

//Heading PID
#define HEADING_KP          0.5f
#define HEADING_KD          0.05f

#define MAX_ACCEL           0.40f   // m/s*s
#define MAX_VELOCITY        1.00f   // m/s cap
#define MAX_MOTOR_VOLTAGE   6.0f


RotEv rotev;

enum State { IDLE, RUNNING, BRAKING, STOPPED };
State state = IDLE;

// gyro
float heading = 0;
float gyroBias = 0;

// motion profile
float vMax, tAccel, tCruise;

// timing
unsigned long startTime, lastLoopTime, brakeStartTime;

// Helpers

void writeMotors(float leftDuty, float rightDuty) {
  leftDuty *= MOTOR_LEFT_SIGN;
  rightDuty *= MOTOR_RIGHT_SIGN;
  float vbat = rotev.getVoltage();
  if (vbat > 0) {
    float cap = MAX_MOTOR_VOLTAGE / vbat;
    leftDuty = constrain(leftDuty, -cap, cap);
    rightDuty = constrain(rightDuty, -cap, cap);
  }
  leftDuty = constrain(leftDuty, -1.0f, 1.0f);
  rightDuty = constrain(rightDuty, -1.0f, 1.0f);
  if (MOTOR_LEFT == 1) {
    rotev.motorWrite1(leftDuty);
    rotev.motorWrite2(rightDuty);
  } else {
    rotev.motorWrite2(leftDuty);
    rotev.motorWrite1(rightDuty);
  }
}

void computeProfile() {
  float D = TARGET_DISTANCE_M, T = TARGET_TIME_S, a = MAX_ACCEL;
  float disc = T * T - 4.0f * D / a;
  if (disc < 0)
    vMax = min(2.0f * D / T, sqrt(D * a));
  else
    vMax = (T - sqrt(disc)) * a / 2.0f;
  if (vMax > MAX_VELOCITY) vMax = MAX_VELOCITY;
  tAccel = vMax / a;
  tCruise = T - 2.0f * tAccel;
  if (tCruise < 0) tCruise = 0;
}

float getTargetVelocity(float t) {
  if (t < tAccel) return MAX_ACCEL * t;
  if (t < tAccel + tCruise) return vMax;
  if (t < tAccel + tCruise + tAccel) return vMax - MAX_ACCEL * (t - tAccel - tCruise);
  return 0;
}

float feedforward(float vel, float deadband, float kff) {
  if (vel <= 0) return 0;
  return deadband + kff * vel;
}

// loop

void setup() {
  rotev.begin();

  // Calibrate gyro bias (red means don't move)
  rotev.ledWrite(1, 0, 0);
  float sum = 0;
  for (int i = 0; i < 200; i++) {
    sum += rotev.readYawRate();
    delay(5);
  }
  gyroBias = sum / 200.0f;

  rotev.ledWrite(0, 0, 1); // blue ready
  state = IDLE;
}

void loop() {
  switch (state) {

    case IDLE:
      if (rotev.goButtonPressed()) {
        computeProfile();
        heading = 0;
        rotev.motorEnable(true);
        startTime = micros();
        lastLoopTime = startTime;
        rotev.ledWrite(0, 1, 0); // green = running
        state = RUNNING;
      }
      break;

    case RUNNING: {
      unsigned long now = micros();
      float dt = (now - lastLoopTime) / 1e6f;
      lastLoopTime = now;
      if (dt <= 0 || dt > 0.05f) break;

      float elapsed = (now - startTime) / 1e6f;
      float targetVel = getTargetVelocity(elapsed);

      if (targetVel <= 0) {
        brakeStartTime = micros();
        rotev.ledWrite(1, 1, 0); // yellow = braking
        state = BRAKING;
        break;
      }

      // gyro heading correction
      float yawRate = rotev.readYawRate() - gyroBias;
      heading += yawRate * dt;
      float correction = HEADING_KP * heading + HEADING_KD * yawRate;
      correction = constrain(correction, -0.15f, 0.15f);

      // feedforward per wheel + heading correction
      float leftDuty  = feedforward(targetVel, DEADBAND_LEFT, KFF_LEFT) + correction;
      float rightDuty = feedforward(targetVel, DEADBAND_RIGHT, KFF_RIGHT) - correction;

      writeMotors(leftDuty, rightDuty);
      break;
    }

    case BRAKING: {
      unsigned long now = micros();
      if ((now - brakeStartTime) / 1000 < BRAKE_DURATION_MS) {
        writeMotors(-BRAKE_DUTY, -BRAKE_DUTY);
      } else {
        writeMotors(0, 0);
        rotev.motorEnable(false);
        rotev.ledWrite(0, 0, 0);
        state = STOPPED;
      }
      break;
    }

    case STOPPED:
      if (rotev.stopButtonPressed()) {
        rotev.ledWrite(0, 0, 1);
        state = IDLE;
      }
      break;
  }
}
