// PID Position Control for GA12 N20 Motor with Encoder
// Adapted for L298N Motor Driver and Arduino Uno// ************ PIN DEFINITIONS ************
// L298N Motor Driver Pins
const int PWMA = 6;    // Enable A (PWM speed control)
const int AIN1 = 7;    // Direction control 1
const int AIN2 = 8;    // Direction control 2// Encoder Pins
const int encoderPinA = 2;  // Channel A (interrupt pin)
const int encoderPinB = 3;  // Channel B (interrupt pin)// ************ ENCODER VARIABLES ************
volatile long encoderCount = 0;
volatile bool lastStateA = 0;// ************ PID VARIABLES ************
float targetAngle = 0.0;      // Desired angle (degrees) - from Serial input
float currentAngle = 0.0;     // Current angle (degrees) - from encoder// PID Constants - Start with these values and tune as needed
float kp = 8.0;               // Proportional gain
float ki = 20;               // Integral gain
float kd = 0.8;               // Derivative gain// PID Calculation Variables
float error = 0.0;
float previousError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float pidOutput = 0.0;// Timing Variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
float deltaTime = 0.0;// Motor Control Variables
float motorVoltage = 0.0;
float maxVoltage = 6.0;       // Maximum voltage (6V power supply)
float minVoltage = -6.0;      // Minimum voltage
int pwmValue = 0;// Control Parameters
float tolerance = 2.0;        // Position tolerance in degrees
bool positionReached = false;void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("PID Position Control for GA12 N20 Motor");
  Serial.println("Commands:");
  Serial.println("g<angle> = Go to angle (e.g., g90 for 90 degrees)");
  Serial.println("p<value> = Set Kp (e.g., p2.5)");
  Serial.println("i<value> = Set Ki (e.g., i0.1)");
  Serial.println("d<value> = Set Kd (e.g., d0.5)");
  Serial.println("r = Reset position (set current as 0°)");
  Serial.println("s = Show current status");  // Initialize motor control pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);  // Initialize encoder pins
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);  // Stop motor initially
  stopMotor();  // Initialize timing
  previousTime = millis();  Serial.println("System ready. Enter target angle (e.g., g90):");
}void loop() {
  // Check for serial commands
  if (Serial.available()) {
    processSerialCommand();
  }  // Get current time and calculate delta time
  currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds  // Only run PID if enough time has passed (10ms minimum)
  if (deltaTime >= 0.01) {
    // Update current angle from encoder
    updateCurrentAngle();    // Calculate PID output
    calculatePID();    // Apply motor control
    applyMotorControl();    // Check if position is reached
    checkPositionReached();    // Print status
    printStatus();    // Update previous values
    previousTime = currentTime;
    previousError = error;
  }  delay(5); // Small delay to prevent overwhelming the system
}void processSerialCommand() {
  String command = Serial.readString();
  command.trim();  if (command.length() > 0) {
    char cmd = command.charAt(0);
    float value = command.substring(1).toFloat();    switch (cmd) {
      case 'g':
      case 'G':
        targetAngle = value;
        positionReached = false;
        integral = 0; // Reset integral term
        Serial.print("Target angle set to: ");
        Serial.print(targetAngle);
        Serial.println("°");
        break;      case 'p':
      case 'P':
        kp = value;
        Serial.print("Kp set to: ");
        Serial.println(kp);
        break;      case 'i':
      case 'I':
        ki = value;
        integral = 0; // Reset integral when changing Ki
        Serial.print("Ki set to: ");
        Serial.println(ki);
        break;      case 'd':
      case 'D':
        kd = value;
        Serial.print("Kd set to: ");
        Serial.println(kd);
        break;      case 'r':
      case 'R':
        encoderCount = 0;
        currentAngle = 0;
        targetAngle = 0;
        integral = 0;
        Serial.println("Position reset to 0°");
        break;      case 's':
      case 'S':
        showStatus();
        break;      default:
        Serial.println("Invalid command! Use g<angle>, p<kp>, i<ki>, d<kd>, r, or s");
        break;
    }
  }
}void updateCurrentAngle() {
  // Convert encoder counts to angle
  // GA12 N20: 3 PPR * 2 (edges) * 50 (gear ratio) = 300 counts per revolution
  currentAngle = (encoderCount / 700.0) * 360.0;
}void calculatePID() {
  // Calculate error
  error = targetAngle - currentAngle;  // Calculate integral (with windup protection)
  if (abs(error) < 90.0) { // Only integrate if error is reasonable
    integral += error * deltaTime;
  }  // Integral windup protection
  float integralLimit = maxVoltage / ki;
  if (integral > integralLimit) integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;  // Calculate derivative
  derivative = (error - previousError) / deltaTime;  // Calculate PID output
  pidOutput = (kp * error) + (ki * integral) + (kd * derivative);  // Constrain output to motor voltage limits
  if (pidOutput > maxVoltage) {
    pidOutput = maxVoltage;
    integral = integral - (error * deltaTime); // Anti-windup
  }
  if (pidOutput < minVoltage) {
    pidOutput = minVoltage;
    integral = integral - (error * deltaTime); // Anti-windup
  }  motorVoltage = pidOutput;
}
void applyMotorControl() {
  // Convert voltage to PWM value (0-255)
  pwmValue = int(255 * abs(motorVoltage) / maxVoltage);  // Constrain PWM value
  if (pwmValue > 255) pwmValue = 255;  // Set motor direction and speed
  if (abs(error) < tolerance) {
    // Stop motor if within tolerance
    stopMotor();
  } else if (motorVoltage > 0) {
    // Rotate in positive direction
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwmValue);
  } else if (motorVoltage < 0) {
    // Rotate in negative direction
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, pwmValue);
  } else {
    // Stop motor
    stopMotor();
  }
}void stopMotor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
}void checkPositionReached() {
  if (abs(error) <= tolerance && !positionReached) {
    positionReached = true;
    Serial.println("*** POSITION REACHED ***");  }else if (abs(error) > tolerance) {
    positionReached = false;
  }}void encoderISR() {
  // Read current state of encoder pins
  bool stateA = digitalRead(encoderPinA);
  bool stateB = digitalRead(encoderPinB);  // Determine direction and increment/decrement counter
  if (stateA != lastStateA) {
    if (stateA == stateB) {
      encoderCount--;
    } else {
      encoderCount++;
    }
  }
  lastStateA = stateA;
}void printStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) { // Print every 500ms
    Serial.print("Target: ");
    Serial.print(targetAngle, 1);
    Serial.print("° | Current: ");
    Serial.print(currentAngle, 1);
    Serial.print("° | Error: ");
    Serial.print(error, 1);
    Serial.print("° | PWM: ");
    Serial.print(pwmValue);    if (positionReached) {
      Serial.print(" | STATUS: AT POSITION");
    } else {
      Serial.print(" | STATUS: MOVING");
    }
    Serial.println();    lastPrint = millis();
  }
}void showStatus() {
  Serial.println("=== CURRENT STATUS ===");
  Serial.print("Target Angle: ");
  Serial.print(targetAngle);
  Serial.println("°");
  Serial.print("Current Angle: ");
  Serial.print(currentAngle);
  Serial.println("°");
  Serial.print("Error: ");
  Serial.print(error);
  Serial.println("°");
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
  Serial.println("=== PID PARAMETERS ===");
  Serial.print("Kp: ");
  Serial.println(kp);
  Serial.print("Ki: ");
  Serial.println(ki);
  Serial.print("Kd: ");
  Serial.println(kd);
  Serial.print("Tolerance: ");
  Serial.print(tolerance);
  Serial.println("°");
  Serial.println("=====================");
}