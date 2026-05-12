/**
 * ============================================================
 * Wheeled Mobile Robot — Arduino Mega 2560
 * Motors  : JGY-370 with 2-channel quadrature encoder
 * Drivers : BTS7960 H-bridge (dual PWM, no DIR pin)
 *
 * Changes: 
 * - Fixed PID divide-by-zero vulnerability.
 * - Rebuilt serial parser for robustness and bounds safety.
 * - Fixed heading PID windup carryover.
 * - Added string memory reservation to prevent heap fragmentation.
 * ============================================================
 */

#include <Arduino.h>

// ─── Encoder Pins ───────────────────────────────────────────
#define ENC_LEFT_A       3
#define ENC_LEFT_B       2
#define ENC_RIGHT_A     20
#define ENC_RIGHT_B     21

// ─── BTS7960 Motor Driver Pins ──────────────────────────────
#define MOTOR_L_RPWM     9
#define MOTOR_L_LPWM     8
#define MOTOR_L_REN     24
#define MOTOR_L_LEN     25
#define MOTOR_R_RPWM    11
#define MOTOR_R_LPWM    10
#define MOTOR_R_REN     22
#define MOTOR_R_LEN     23

// ─── JGY-370 Parameters ─────────────────────────────────────
#define GEAR_RATIO      46
#define ENC_PPR         11
const float COUNTS_PER_REV = (float)(ENC_PPR * 4 * GEAR_RATIO); // 3960
const float WHEEL_DIAMETER = 0.065f;
const float WHEEL_BASE     = 0.165f;
const float WHEEL_CIRC     = PI * WHEEL_DIAMETER;
const float DIST_PER_COUNT = WHEEL_CIRC / COUNTS_PER_REV;

// ─── Control Loop ───────────────────────────────────────────
const uint16_t CONTROL_HZ     =                     50;
const uint32_t CONTROL_PERIOD = 1000000UL / CONTROL_HZ;

// ─── PID Gains ──────────────────────────────────────────────
const float KP_SPEED_L =  70.0f;
const float KI_SPEED_L =  40.0f;
const float KD_SPEED_L =   0.0f;

const float KP_SPEED_R =  70.0f;
const float KI_SPEED_R =  40.0f;
const float KD_SPEED_R =  0.0f;

const float KP_HEAD  =     1.5f;
const float KI_HEAD  =    0.05f;
const float KD_HEAD  =    0.10f;

// ─── Quadrature Lookup Table ────────────────────────────────
static const int8_t QEM[4][4] = {
  {  0, -1,  1,  0 },
  {  1,  0,  0, -1 },
  {  0,  1, -1,  0 },
  { -1,  0,  0,  1 }
};

volatile int32_t encoderLeft  = 0;
volatile int32_t encoderRight = 0;
volatile uint8_t stateLeft    = 0;
volatile uint8_t stateRight   = 0;

// ─── Odometry & Speed ───────────────────────────────────────
float   posX = 0, posY = 0, theta = 0;
int32_t prevLeft = 0, prevRight = 0;
float   speedLeft = 0, speedRight = 0;

// ─── Navigation ─────────────────────────────────────────────
float targetX = 0, targetY = 0, targetTheta = 0;
bool  navActive = false;
float setpointLeft  = 0;
float setpointRight = 0;

// ─── ROS2 watchdog ──────────────────────────────────────────
uint32_t lastCmdMs  = 0;
bool     rosControl = false;   

// ─── PID Struct ─────────────────────────────────────────────
struct PID { float kp, ki, kd, integral, prevError, outMin, outMax; };
PID pidL = { KP_SPEED_L, KI_SPEED_L, KD_SPEED_L, 0, 0, -255.0f, 255.0f };
PID pidR = { KP_SPEED_R, KI_SPEED_R, KD_SPEED_R, 0, 0, -255.0f, 255.0f };
PID pidH = { KP_HEAD,  KI_HEAD,  KD_HEAD,  0, 0,   -0.5f,   0.5f };

String   serialBuf;
uint32_t lastControlUs = 0;

// ═══════════════════════════════════════════════════════════
//  ISRs
// ═══════════════════════════════════════════════════════════
void ISR_EncLeft() {
  uint8_t cur = ((uint8_t)digitalRead(ENC_LEFT_A) << 1) | (uint8_t)digitalRead(ENC_LEFT_B);
  encoderLeft += QEM[stateLeft][cur];
  stateLeft    = cur;
}
void ISR_EncRight() {
  uint8_t cur = ((uint8_t)digitalRead(ENC_RIGHT_A) << 1) | (uint8_t)digitalRead(ENC_RIGHT_B);
  encoderRight -= QEM[stateRight][cur];
  stateRight    = cur;
}

// ═══════════════════════════════════════════════════════════
//  PID
// ═══════════════════════════════════════════════════════════
float computePID(PID &pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  pid.integral += error * dt;
  
  // FIXED: Divide-by-zero vulnerability patched
  if (pid.ki != 0.0f) {
    pid.integral = constrain(pid.integral, pid.outMin/pid.ki, pid.outMax/pid.ki);
  } else {
    pid.integral = 0.0f;
  }
  
  float derivative = (error - pid.prevError) / dt;
  pid.prevError = error;
  
  return constrain(pid.kp*error + pid.ki*pid.integral + pid.kd*derivative, pid.outMin, pid.outMax);
}

// ═══════════════════════════════════════════════════════════
//  BTS7960 MOTOR DRIVER
// ═══════════════════════════════════════════════════════════
void setMotorBTS(uint8_t rpwm, uint8_t lpwm, float pwmVal) {
  uint8_t speed = (uint8_t)constrain(fabs(pwmVal), 0, 255);
  if (pwmVal >= 0.0f) { analogWrite(rpwm, speed); analogWrite(lpwm, 0); }
  else                { analogWrite(rpwm, 0);     analogWrite(lpwm, speed); }
}

void stopMotors() {
  analogWrite(MOTOR_L_RPWM, 0); analogWrite(MOTOR_L_LPWM, 0);
  analogWrite(MOTOR_R_RPWM, 0); analogWrite(MOTOR_R_LPWM, 0);
  setpointLeft = setpointRight = 0.0f;
  pidL.integral = pidR.integral = 0.0f;
}

// ═══════════════════════════════════════════════════════════
//  ODOMETRY
// ═══════════════════════════════════════════════════════════
void updateOdometry(float dt) {
  noInterrupts();
  int32_t snapL = encoderLeft;
  int32_t snapR = encoderRight;
  interrupts();

  int32_t dL = snapL - prevLeft;
  int32_t dR = snapR - prevRight;
  prevLeft  = snapL;
  prevRight = snapR;

  float distL = (float)dL * DIST_PER_COUNT;
  float distR = (float)dR * DIST_PER_COUNT;
  speedLeft   = distL / dt;
  speedRight  = distR / dt;

  float distCentre = (distL + distR) * 0.5f;
  float dTheta     = (distR - distL) / WHEEL_BASE;

  theta += dTheta;
  while (theta >  PI)  theta -= TWO_PI;
  while (theta <= -PI) theta += TWO_PI;

  posX += distCentre * cos(theta);
  posY += distCentre * sin(theta);
}

// ═══════════════════════════════════════════════════════════
//  NAVIGATION CONTROLLER
// ═══════════════════════════════════════════════════════════
const float NAV_LINEAR_SPEED = 0.20f;
const float NAV_GOAL_RADIUS  = 0.02f;
const float NAV_ANGLE_THRESH = 0.05f;

float wrapAngle(float a) {
  while (a >  PI) a -= TWO_PI;
  while (a <= -PI) a += TWO_PI;
  return a;
}

void updateNavigation(float dt) {
  if (!navActive) return;
  float dx   = targetX - posX;
  float dy   = targetY - posY;
  float dist = sqrt(dx*dx + dy*dy);

  if (dist < NAV_GOAL_RADIUS) {
    float headErr = wrapAngle(targetTheta - theta);
    if (fabs(headErr) < NAV_ANGLE_THRESH) {
      navActive = false; stopMotors();
      Serial.println(F("NAV: Goal reached."));
      return;
    }
    float c = computePID(pidH, 0, -headErr, dt);
    setpointLeft = -c; setpointRight = c;
    return;
  }

  float bearing = atan2(dy, dx);
  float headErr = wrapAngle(bearing - theta);
  if (fabs(headErr) > 0.5f) {
    float c = computePID(pidH, 0, -headErr, dt);
    setpointLeft = -c;
    setpointRight = c;
    return;
  }

  float linearSpeed = min(NAV_LINEAR_SPEED, dist * 2.0f);
  float c = computePID(pidH, 0, -headErr, dt);
  setpointLeft  = linearSpeed - c;
  setpointRight = linearSpeed + c;
}

// ═══════════════════════════════════════════════════════════
//  SERIAL COMMAND PARSER
// ═══════════════════════════════════════════════════════════
void parseSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuf.trim();
      if (serialBuf.length() == 0) { serialBuf = ""; return; }

      // FIXED: Safely trap and process ROS CMD strings first
      if (serialBuf.startsWith("CMD,")) {
        String cmdArgs = serialBuf.substring(4);
        int comma = cmdArgs.indexOf(',');
        if (comma > 0) {
          setpointLeft  = cmdArgs.substring(0, comma).toFloat();
          setpointRight = cmdArgs.substring(comma + 1).toFloat();
          navActive  = false;
          rosControl = true;
          lastCmdMs  = millis();
        }
        serialBuf = "";
        return; // Done with this line
      }

      // FIXED: Safe bounds check for single-character standard commands
      char cmd = toupper(serialBuf.charAt(0));
      String args = (serialBuf.length() > 2) ? serialBuf.substring(2) : "";

      switch (cmd) {
        case 'G': {
          int sp = args.indexOf(' ');
          if (sp > 0) {
            targetX = args.substring(0, sp).toFloat();
            targetY = args.substring(sp + 1).toFloat();
            navActive  = true;
            rosControl = false;
            pidH.integral = pidH.prevError = 0;
            Serial.print(F("NAV: Goto (")); Serial.print(targetX);
            Serial.print(F(",")); Serial.print(targetY); Serial.println(F(")"));
          }
          break;
        }
        case 'H': {
          targetTheta = radians(args.toFloat());
          navActive   = true;
          rosControl  = false;
          // FIXED: Integral windup reset added here
          pidH.integral = pidH.prevError = 0; 
          Serial.print(F("NAV: Heading ")); Serial.println(args);
          break;
        }
        case 'S': {
          int sp = args.indexOf(' ');
          if (sp > 0) {
            setpointLeft  = args.substring(0, sp).toFloat();
            setpointRight = args.substring(sp + 1).toFloat();
            navActive  = false;
            rosControl = false;
          }
          break;
        }
        case 'P': {
          noInterrupts();
          int32_t eL = encoderLeft; int32_t eR = encoderRight; interrupts();
          Serial.print(F("ODO x="));  Serial.print(posX, 4);
          Serial.print(F(" y="));     Serial.print(posY, 4);
          Serial.print(F(" θ="));
          Serial.print(degrees(theta), 2);
          Serial.print(F("° vL="));   Serial.print(speedLeft,  3);
          Serial.print(F(" vR="));    Serial.print(speedRight, 3);
          Serial.print(F(" encL="));  Serial.print(eL);
          Serial.print(F(" encR="));  Serial.println(eR);
          break;
        }
        case 'R': {
          posX = posY = theta = 0.0f;
          noInterrupts(); encoderLeft = encoderRight = 0; interrupts();
          prevLeft = prevRight = 0;
          navActive = false;
          stopMotors();
          Serial.println(F("ODO: Reset."));
          break;
        }
        default:
          Serial.println(F("Commands: CMD,vL,vR | G x y | H deg | S vL vR | P | R"));
      }
      serialBuf = "";
    } else {
      serialBuf += c;
    }
  }
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  
  // FIXED: Reserve 64 bytes of memory for the string buffer to stop heap fragmentation
  serialBuf.reserve(64); 

  Serial.println(F("=== JGY-370 + BTS7960 + ROS2 Bridge ==="));
  pinMode(MOTOR_L_REN, OUTPUT); digitalWrite(MOTOR_L_REN, HIGH);
  pinMode(MOTOR_L_LEN, OUTPUT); digitalWrite(MOTOR_L_LEN, HIGH);
  pinMode(MOTOR_R_REN, OUTPUT); digitalWrite(MOTOR_R_REN, HIGH);
  pinMode(MOTOR_R_LEN, OUTPUT); digitalWrite(MOTOR_R_LEN, HIGH);
  pinMode(MOTOR_L_RPWM, OUTPUT); pinMode(MOTOR_L_LPWM, OUTPUT);
  pinMode(MOTOR_R_RPWM, OUTPUT); pinMode(MOTOR_R_LPWM, OUTPUT);
  stopMotors();

  pinMode(ENC_LEFT_A,  INPUT_PULLUP); pinMode(ENC_LEFT_B,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP); pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  stateLeft  = ((uint8_t)digitalRead(ENC_LEFT_A)  << 1) | (uint8_t)digitalRead(ENC_LEFT_B);
  stateRight = ((uint8_t)digitalRead(ENC_RIGHT_A) << 1) | (uint8_t)digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  ISR_EncLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B),  ISR_EncLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), ISR_EncRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), ISR_EncRight, CHANGE);

  lastControlUs = micros();
  lastCmdMs     = millis();
}

// ═══════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════

void loop() {

 parseSerial();
  // ── ROS2 watchdog: stop if no CMD for >500 ms ──────────
  if (rosControl && (millis() - lastCmdMs > 500)) {
    stopMotors();
    rosControl = false;
  }

  uint32_t now     = micros();
  uint32_t elapsed = now - lastControlUs;
  
  if (elapsed >= CONTROL_PERIOD) {
    float dt      = elapsed * 1e-6f;
    lastControlUs = now;

    updateOdometry(dt);
    if (!rosControl) updateNavigation(dt);

    float pwmL = 0;
    if (setpointLeft == 0.0f) {
      pidL.integral = 0; 
    } else {
      pwmL = computePID(pidL, setpointLeft, speedLeft, dt);
    }

    float pwmR = 0;
    if (setpointRight == 0.0f) {
      pidR.integral = 0;
    } else {
      pwmR = computePID(pidR, setpointRight, speedRight, dt);
    }
    
    setMotorBTS(MOTOR_L_LPWM, MOTOR_L_RPWM, pwmL);
    setMotorBTS(MOTOR_R_RPWM, MOTOR_R_LPWM, pwmR);
    
    // ── Odometry stream to ROS2 (20 Hz) ──────────────────
    static uint8_t pubDiv = 0;
    if (++pubDiv >= (CONTROL_HZ / 20)) {
      pubDiv = 0;
      noInterrupts(); int32_t eL = encoderLeft;
      int32_t eR = encoderRight; interrupts();
      Serial.print(F("ODO,"));
      Serial.print(posX,          4); Serial.print(',');
      Serial.print(posY,          4); Serial.print(',');
      Serial.print(degrees(theta),4); Serial.print(',');
      Serial.print(speedLeft,     4); Serial.print(',');
      Serial.print(speedRight,    4); Serial.print(',');
      Serial.print(eL);               Serial.print(',');
      Serial.println(eR);
    }
  }
  
}