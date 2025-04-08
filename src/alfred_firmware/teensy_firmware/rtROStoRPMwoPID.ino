#include <Encoder.h>

#define PWM1 5   // Left Motor PWM
#define DIR1 6   // Left Motor Direction
#define PWM2 8   // Right Motor PWM
#define DIR2 9   // Right Motor Direction

#define ENC1_A 2
#define ENC1_B 3
#define ENC2_A 4
#define ENC2_B 7

// Encoder constants
#define CPR 153550          // Counts per revolution
#define SAMPLE_TIME 10     // Sample time in milliseconds
#define MAX_RPM 20        // Max RPM for PWM mapping

// Velocity commands in RPM
double left_wheel_joint_vel = 0, right_wheel_joint_vel = 0;

// Encoder objects
Encoder motor1Encoder(ENC1_A, ENC1_B);
Encoder motor2Encoder(ENC2_A, ENC2_B);

long prevCount1 = 0, prevCount2 = 0;
unsigned long prevTime = 0;

// Compute RPM function
double computeRPM(long deltaCount, int motorID) {
  // SAMPLE_TIME is in milliseconds, so this converts to samples per minute
  double timeFactor = (60000.0 / SAMPLE_TIME);

  // RPM = (deltaCount / CPR) * (60000 / SAMPLE_TIME)
  // motorID == 1 → left wheel → invert sign if needed for direction consistency
  return (motorID == 1)
           ? -(deltaCount * timeFactor) / CPR
           :  (deltaCount * timeFactor) / CPR;
}

// Convert RPM to angular velocity (rad/s)
double rpmToAngularVelocity(double rpm) {
  return (rpm * 2.0 * PI) / 60.0;  // RPM to rad/s conversion
}

// Check for new JOINT_VELOCITIES from Serial
void checkSerialForJointVelocities() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("JOINT_VELOCITIES")) {
      sscanf(command.c_str(), "JOINT_VELOCITIES r%lf,l%lf", &right_wheel_joint_vel, &left_wheel_joint_vel);
    }
  }
}

// Convert RPM to Motor PWM value (Basic Proportional Mapping)
int rpmToPWM(double rpm) {
  return constrain(map(abs(rpm), 0, MAX_RPM, 0, 255), 0, 255);
}

void setup() {
  Serial.begin(115200);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
}

void loop() {
  unsigned long currentTime = millis();

  checkSerialForJointVelocities();

  if (currentTime - prevTime >= SAMPLE_TIME) {
    long count1 = motor1Encoder.read();
    long count2 = motor2Encoder.read();

    double currentRPM1 = computeRPM(count1 - prevCount1, 1);
    double currentRPM2 = computeRPM(count2 - prevCount2, 2);

    prevCount1 = count1;
    prevCount2 = count2;
    prevTime = currentTime;

    moveMotors(left_wheel_joint_vel, right_wheel_joint_vel);

    // Convert RPM to angular velocity (rad/s)
    double angVel1 = rpmToAngularVelocity(currentRPM1);
    double angVel2 = rpmToAngularVelocity(currentRPM2);

    // motor1 = left wheel, motor2 = right wheel
    Serial.print("r");
    Serial.print(angVel2, 3);  // 3 decimal places
    Serial.print(",");
    Serial.print("l");
    Serial.println(angVel1, 3);
  }
}

void moveMotors(double rpm1, double rpm2) {
  int pwm1 = rpmToPWM(rpm1);
  int pwm2 = rpmToPWM(rpm2);

  digitalWrite(DIR1, rpm1 >= 0 ? LOW : HIGH);
  analogWrite(PWM1, pwm1);

  digitalWrite(DIR2, rpm2 >= 0 ? LOW : HIGH);
  analogWrite(PWM2, pwm2);
}
