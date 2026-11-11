#include <MeMCore.h>
#include <avr/pgmspace.h>
// --- Hardware Initialisation ---
MeDCMotor leftMotor(M1);
MeDCMotor rightMotor(M2);
MeLineFollower lineFinder(PORT_2); 
MeUltrasonicSensor ultrasonic(PORT_1); 
MeRGBLed statusLEDs(PORT_7, 2);
MeBuzzer buzzer;

// 1 = show Serial messafes while debugging
// 0 = disable Serial messages during the maze run
#define SERIAL_MONITOR_ON 1

// Note frequencies
#define REST     0
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784

// Confirmed physical mapping: M1 is RIGHT, M2 is LEFT.
// Existing variable names are retained for compatibility.
// Port 4 S1/S2 = decoder A0/A1.
// Port 3 S1/S2 = LDR A2 / IR receiver A3.
#define DECODER_A0 A0
#define DECODER_A1 A1
#define IR_RX_PIN A3
#define LDR_PIN A2

// --- Timing Constants ---
const byte RGBWait = 75; // Delay before next RGB color turns ON
const byte LDRWait = 7;  // Delay before taking another LDR reading
// timing when the car travels ONE grid
const unsigned long BLUE_ONE_GRID_TRAVEL_MS = 820;
const unsigned long PINK_ONE_GRID_TRAVEL_MS = 755;


const unsigned long AVOID_FORWARD_MS = 150; // after 45deg, forward sec
const int AVOID_SPEED = 100; //temp speed after emergency maneuver

// --- Tuning Constants ---
#define FORWARD_SPEED 225
#define RIGHT_MOTOR_OFFSET 30
#define TURN_SPEED 255
#define UTURN_SPEED 255

//Higher = sharper nudge
// Does NOT affect driveBetweenWaypoints(), which uses driveCentred().
#define NUDGE_DEVIATION 150

// Start IR-based left correction when reflection is above this value.
// Lower = reacts earlier, but may trigger when the wall is still far away.
#define IR_THRESHOLD 130

#define TURN_90_MS 355     
#define UTURN_180_MS 632

// Initial trial: calibrate the LEFT 180-degree turn separately.
const unsigned long UTURN_LEFT_180_MS = UTURN_180_MS;

const unsigned long AVOID_TURN_30_MS = TURN_90_MS / 2.5;

//CAR TO STOP WHEN IR IS VERY CLOSE TO THE WALL
#define IR_EMERGENCY_THRESHOLD 325
#define IR_EMERGENCY_RELEASE   160

#define US_EMERGENCY_TRIGGER_CM 3.5f
#define US_EMERGENCY_RELEASE_CM 7.0f

#define US_CLOSE_TRIGGER_CM 6.0f
#define US_CLOSE_CORRECTION 60

// measured when your robot is physically centred.
#define TARGET_LEFT_CM 9.0f

// Ultrasonic steering strength per cm beyond the allowed deadband.
// Higher = stronger correction for the same distance error.
#define US_KP 5.0f
// Maximum wheel-speed correction requested by ultrasonic centring.
// Higher = allows sharper ultrasonic steering.
#define US_MAX_CORRECTION 50

// IR steering strength per reflection unit above IR_THRESHOLD.
// Higher = stronger left correction for the same IR reading.
#define IR_KP 0.25f

//controls how much US distance variation is allowed before correcting
#define US_DEADBAND_CM 1.0f 

#define MAX_CENTRE_CORRECTION 60
#define IR_AVOID_CORRECTION 60

// false: test wall correction and drive past black strips.
// true: calibrate at startup and solve colour challenges.
const bool ENABLE_COLOUR_CHALLENGE = true;

// Starting limit for rejecting distant/missing left walls.
// Adjust for your corridor geometry.
#define US_MAX_WALL_CM 20.0f

// byte invalidColourAttempts = 0;
bool mazeCompleted = false;
bool restartButtonReady = false;

bool rightWallAvoidanceArmed = true;
bool leftWallAvoidanceArmed = true;

// --- Colour Calibration Arrays ---
// Arrays hold values for [0] = Red, [1] = Green, [2] = Blue
float whiteArray[] = {973.43f, 1005.00f, 987.57f};
float blackArray[] = {916.30f, 963.10f, 899.00f};
float greyDiff[]   = {0,0,0};


void showChallengeColour(int colour) {
  switch (colour) {
    case 1: // Red
      statusLEDs.setColor(0, 40, 0, 0);
      break;

    case 2: // Green
      statusLEDs.setColor(0, 0, 40, 0);
      break;

    case 3: // Light blue
      statusLEDs.setColor(0, 0, 25, 40);
      break;

    case 4: // Orange
      statusLEDs.setColor(0, 40, 15, 0);
      break;

    case 5: // Pink
      statusLEDs.setColor(0, 40, 0, 20);
      break;

    case 6: // White
      statusLEDs.setColor(0, 30, 30, 30);
      break;

    default: // Off
      statusLEDs.setColor(0, 0, 0, 0);
      break;
  }

  statusLEDs.show();
}

void stopMotor() {
  leftMotor.stop();
  rightMotor.stop();
}

void moveForward() {
  leftMotor.run(FORWARD_SPEED);
  
  rightMotor.run(-(FORWARD_SPEED + RIGHT_MOTOR_OFFSET));
}

void turnLeft() {
  leftMotor.run(TURN_SPEED);
  rightMotor.run(TURN_SPEED - RIGHT_MOTOR_OFFSET);
  delay(TURN_90_MS);
  stopMotor();
}

void turnRight() {
  leftMotor.run(-TURN_SPEED);
  rightMotor.run(-(TURN_SPEED - RIGHT_MOTOR_OFFSET));
  delay(TURN_90_MS);
  stopMotor();
}

void uTurn() {
    leftMotor.run(-UTURN_SPEED);
  rightMotor.run(-(UTURN_SPEED - RIGHT_MOTOR_OFFSET));
  delay(UTURN_180_MS);
  stopMotor();
}

void driveCentred(int correction) {
  // Positive correction = steer RIGHT
  // Negative correction = steer LEFT

  correction = constrain(
    correction,
    -MAX_CENTRE_CORRECTION,
    MAX_CENTRE_CORRECTION
  );

  // M1 = physical RIGHT wheel
  int rightWheelSpeed = constrain(
    FORWARD_SPEED - correction,
    0, 255);

  // M2 = physical LEFT wheel
  int leftWheelSpeed = constrain(
    FORWARD_SPEED + RIGHT_MOTOR_OFFSET + correction,
    0, 255);

  leftMotor.run(rightWheelSpeed);
  rightMotor.run(-leftWheelSpeed);
  
}

void nudgeRight() {
  // Physical RIGHT wheel: slow down
  int rightWheelSpeed = constrain(
    FORWARD_SPEED - NUDGE_DEVIATION,
    0, 255
  );

  // Physical LEFT wheel: speed up
  int leftWheelSpeed = constrain(
    FORWARD_SPEED + 15 + NUDGE_DEVIATION,
    0, 255
  );

  leftMotor.run(rightWheelSpeed);   // M1 = right wheel
  rightMotor.run(-leftWheelSpeed); // M2 = left wheel
}

void nudgeLeft() {
  // Physical RIGHT wheel: speed up
  int rightWheelSpeed = constrain(
    FORWARD_SPEED + NUDGE_DEVIATION,
    0, 255
  );

  // Physical LEFT wheel: slow down
  int leftWheelSpeed = constrain(
    FORWARD_SPEED + 15 - NUDGE_DEVIATION,
    0, 255
  );

  leftMotor.run(rightWheelSpeed);   // M1 = right wheel
  rightMotor.run(-leftWheelSpeed); // M2 = left wheel
}

// --- Colour Sensor LED Logic ---
void setLEDColor(int colorIndex) {
  // colorIndex: 0=Red, 1=Green, 2=Blue, 3=RGB OFF / IR ON
  switch (colorIndex) {
    case 0: // Red (1Y1)
      digitalWrite(DECODER_A1, LOW);
      digitalWrite(DECODER_A0, HIGH);
      break;
    case 1: // Green (1Y2)
      digitalWrite(DECODER_A1, HIGH);
      digitalWrite(DECODER_A0, LOW);
      break;
    case 2: // Blue (1Y3)
      digitalWrite(DECODER_A1, HIGH);
      digitalWrite(DECODER_A0, HIGH);
      break;
    default: // Y0: RGB OFF, IR emitter ON
      digitalWrite(DECODER_A1, LOW);
      digitalWrite(DECODER_A0, LOW);
      break;
  }
}

long getAvgReading(int times) {
  long total = 0;
  for (int i = 0; i < times; i++) {
    total += analogRead(LDR_PIN);
    delay(LDRWait);
  }
  return total / times;
}


int detectColour() {
  float rgbArray[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    if (greyDiff[i] < 10.0f) {
      #if SERIAL_MONITOR_ON
      Serial.println(F("Colour calibration invalid."));
      #endif
      return 0;
     }
  }
  for (int c = 0; c < 3; ++c) {
    setLEDColor(c);
    delay(RGBWait);
    float rawReading = getAvgReading(5);
    float normalized = (rawReading - blackArray[c]) / greyDiff[c] * 255.0f;
    rgbArray[c] = normalized;
    if (rgbArray[c] < 0) rgbArray[c] = 0;
    if (rgbArray[c] > 255) rgbArray[c] = 255;
    setLEDColor(3);
    delay(10);
  }
  float r = rgbArray[0], g = rgbArray[1], b = rgbArray[2];
  #if SERIAL_MONITOR_ON
  Serial.print(F("R: ")); Serial.print((int)r);
  Serial.print(F(" | G: ")); Serial.print((int)g);
  Serial.print(F(" | B: ")); Serial.println((int)b);
  #endif

  // Measured rules from your stationary paper tests.
  // 0=unknown, 1=red, 2=green, 3=blue-like, 4=orange, 5=pink.

// WHITE: finish marker.
  if (r >= 160 && g >= 250 && b >= 250) 
  return 6;

//ORANGE
if (r >= 185 &&
    g >= 185 && g <= 225 &&
    b >= 75 && b <= 160) {

      //ORIGINAL CODE: (r >= 210 && g >= 180 && g <= 215 && 
      // b >= 75 && b <= 160) {

  return 4;
}


//PINK
  if (r >= 170 && r <= 255 && g >= 155 && g <= 250 &&
      b >= 190 && b <= 245) 
      return 5;

//RED
  if (r >= 160 &&
    g >= 75 && g <= 175 &&
    b >= 80 && b <= 145) {

      /*(r >= 200 &&
    g >= 75 && g <= 165 &&
    b >= 80 && b <= 145) {*/

  return 1;
}

//GREEN
  if (r <= 150  &&
    g >= 195 && g <= 255 &&
    b >= 165 && b <= 225 &&
    g - b >= 20) {

      /*(r >= 70 && r <= 125 &&
    g >= 195 && g <= 255 &&
    b >= 165 && b <= 225) {*/

  return 2;
  }

//BLUE
  if (b > r && b > g) 
  return 3;

//UNKNOWN
  return 0;
}

int getIRReflection() {
  // IR OFF: select Y3
  digitalWrite(DECODER_A1, HIGH);
  digitalWrite(DECODER_A0, HIGH);
  delay(3);

  analogRead(IR_RX_PIN); // Discard first ADC reading
  int ambientLight = analogRead(IR_RX_PIN);

  // IR ON: select Y0
  digitalWrite(DECODER_A1, LOW);
  digitalWrite(DECODER_A0, LOW);
  delay(3);

  analogRead(IR_RX_PIN);
  int activeLight = analogRead(IR_RX_PIN);

  // With your pull-up receiver, reflected IR lowers voltage.
  int reflection = ambientLight - activeLight;

  return reflection > 0 ? reflection : 0;
}

bool uTurnWithClearance() {
  stopMotor();

  // Read walls while stationary.
  float leftCm = ultrasonic.distanceCm();
  int ir = getIRReflection();

  bool leftValid =
      leftCm >= 3.0f &&
      leftCm <= US_MAX_WALL_CM;

  bool leftClose =
      leftValid &&
      leftCm < TARGET_LEFT_CM - US_DEADBAND_CM;

  bool rightClose = ir > IR_THRESHOLD;

#if SERIAL_MONITOR_ON
  Serial.print(F("Orange turn: US left="));
  Serial.print(leftCm, 1);
  Serial.print(F(" cm | IR right="));
  Serial.println(ir);
#endif

  // Missing left reading, or both sides reporting close:
  // remain stopped and retry.
  if (!leftValid || (leftClose && rightClose)) {
#if SERIAL_MONITOR_ON
    Serial.println(
      F("Orange turn held: missing or conflicting wall readings.")
    );
#endif
    return false;
  }

  // Prefer LEFT when the right wall appears close,
  // or when the car is farther from the left wall.
  bool chooseLeft =
      rightClose ||
      leftCm > TARGET_LEFT_CM + US_DEADBAND_CM;

  if (chooseLeft) {
#if SERIAL_MONITOR_ON
    Serial.println(F("ORANGE -> LEFT 180 degrees"));
#endif

    // M1 = physical RIGHT wheel: forward.
    // M2 = physical LEFT wheel: backward.
    leftMotor.run(UTURN_SPEED);
    rightMotor.run(UTURN_SPEED - RIGHT_MOTOR_OFFSET);

    delay(UTURN_LEFT_180_MS);
    stopMotor();
  }
  else {
#if SERIAL_MONITOR_ON
    Serial.println(F("ORANGE -> RIGHT 180 degrees"));
#endif

    uTurn(); // Your original RIGHT 180-degree turn.
  }

  return true;
}

// ==================================================
// WAYPOINT MOVEMENT STATES
// ==================================================
enum { DRIVING, CHECKING_COLOUR, LEAVING_STRIP };
int navigationState = DRIVING;
bool calibrationReady = false;
bool clearTimerRunning = false;
unsigned long clearStarted = 0;

// Both line detectors must see white continuously before another challenge.
const unsigned long STRIP_CLEAR_MS = 60;

void restartMazeRun() {
  stopMotor();

  // Reset the maze-run state.
  mazeCompleted = false;
  navigationState = DRIVING;
  clearTimerRunning = false;
  clearStarted = 0;

  rightWallAvoidanceArmed = true;
  leftWallAvoidanceArmed = true;
  restartButtonReady = false;

  setLEDColor(3); // Custom RGB off; IR on.

#if SERIAL_MONITOR_ON
  Serial.println(F("Restart: release button to begin countdown."));
#endif

  // Stay stopped while the button is held.
  while (analogRead(A7) <= 10) {
    delay(10);
  }

  // Five-second restart countdown.
  for (int seconds = 2; seconds > 0; seconds--) {
#if SERIAL_MONITOR_ON
    Serial.print(F("Restarting in: "));
    Serial.println(seconds);
#endif

    buzzer.tone(1000, 100);
    delay(900);
  }

  buzzer.tone(2000, 300);
}

void rampAvoidanceForward() {
  unsigned long started = millis();

  while (true) {
    unsigned long elapsed = millis() - started;

    int speed = FORWARD_SPEED;

    if (elapsed < AVOID_FORWARD_MS) {
      speed = AVOID_SPEED +
              ((long)(FORWARD_SPEED - AVOID_SPEED) * elapsed)
              / AVOID_FORWARD_MS;
    }

    leftMotor.run(constrain(speed, 0, 255));

    rightMotor.run(
      -constrain(speed + RIGHT_MOTOR_OFFSET, 0, 255)
    );

    if (elapsed >= AVOID_FORWARD_MS) {
      break;
    }

    delay(5);
  }
}

void avoidRightWall45() {
  stopMotor();
  #if SERIAL_MONITOR_ON
  Serial.println(F("Right wall: starting avoidance"));
  #endif

  // LEFT approximately 30 degrees
  leftMotor.run(TURN_SPEED);
  rightMotor.run(TURN_SPEED - 15);
  delay(AVOID_TURN_30_MS);

  stopMotor();
  delay(100);

  // Short diagonal movement away from the right wall
  rampAvoidanceForward();

  stopMotor();
  delay(100);

  

  stopMotor();
  #if SERIAL_MONITOR_ON
  Serial.println(F("Avoidance finished: resuming centring"));
  #endif
}

void avoidLeftWall30() {
  stopMotor();
  #if SERIAL_MONITOR_ON
  Serial.println(F("Emergency: left wall too close - RIGHT 30 degrees"));
  #endif

  // Turn RIGHT approximately 30 degrees.
  leftMotor.run(-TURN_SPEED);
  rightMotor.run(-(TURN_SPEED - 15));
  delay(AVOID_TURN_30_MS);

  stopMotor();
  delay(100);

  // Short forward movement away from the left wall.
  // Uses your existing emergency speed and duration.
  rampAvoidanceForward();

  stopMotor();
  #if SERIAL_MONITOR_ON
  Serial.println(F("Left-wall emergency finished"));
  #endif
}



void driveBetweenWaypoints(bool checkBlack) {

// Stop immediately if black is already detected.
  if (checkBlack &&
      lineFinder.readSensors() != S1_OUT_S2_OUT) {
    stopMotor();
    return;
  }

  float distance = ultrasonic.distanceCm();
  int irReflection = getIRReflection();

  if (checkBlack && lineFinder.readSensors() != S1_OUT_S2_OUT) {
    stopMotor();
    return;
  }
  

  bool emergencyUsValid =
      distance > 0.0f && distance <= US_MAX_WALL_CM;

  // Allow another emergency turn after reaching 7 cm.
  if (emergencyUsValid &&
      distance >= US_EMERGENCY_RELEASE_CM) {
    leftWallAvoidanceArmed = true;
  }

  // Turn RIGHT approximately 30° at 3.5 cm or below.
  if (checkBlack &&
      emergencyUsValid &&
      leftWallAvoidanceArmed &&
      distance <= US_EMERGENCY_TRIGGER_CM) {

    leftWallAvoidanceArmed = false;

    #if SERIAL_MONITOR_ON
    Serial.print(F("US emergency distance: "));
    Serial.println(distance, 1);
    #endif

    avoidLeftWall30();
    return;
  }


  // Rearm after the right wall reflection falls below 160
if (irReflection < IR_EMERGENCY_RELEASE) {
  rightWallAvoidanceArmed = true;
}

// Trigger once when reflection exceeds 200
if (checkBlack &&
    rightWallAvoidanceArmed &&
    irReflection > IR_EMERGENCY_THRESHOLD &&
    !(emergencyUsValid &&
      distance <= US_CLOSE_TRIGGER_CM)) {

  rightWallAvoidanceArmed = false;

  avoidRightWall45();

  // Read fresh sensor values on the next loop
  return;
}



  int correction = 0;
  const char* action = "FORWARD";
  bool leftWallValid = distance >= 3.0f && distance <= US_MAX_WALL_CM;
  float error = leftWallValid ? TARGET_LEFT_CM - distance : 0.0f;

  // Subtract the deadband so correction starts at zero, without a 15-point jump.
  // Preserve left-wall protection priority when both sensors request avoidance.
  if (leftWallValid && distance <= US_CLOSE_TRIGGER_CM) {
  correction = US_CLOSE_CORRECTION;
  action = "RIGHT - strong left-wall correction";
}
else if (leftWallValid && error > US_DEADBAND_CM) {
    correction = constrain(
      (int)(US_KP * (error - US_DEADBAND_CM)),
      0, US_MAX_CORRECTION
    );
    action = correction > 0 ? "RIGHT - proportional ultrasonic" : "FORWARD";
  }
  else {
    if (leftWallValid && error < -US_DEADBAND_CM) {
      correction = -constrain(
        (int)(US_KP * (-error - US_DEADBAND_CM)),
        0, US_MAX_CORRECTION
      );
      action = correction < 0 ? "LEFT - proportional ultrasonic" : "FORWARD";
    }
    else if (!leftWallValid) {
      action = "FORWARD - ultrasonic ignored";
    }

    // Gradual IR avoidance: a small excess no longer immediately requests -40.
    // Take the stronger left request from the two sensors.
    if (irReflection > IR_THRESHOLD) {
      int irStrength = constrain(
        (int)(IR_KP * (irReflection - IR_THRESHOLD)),
        0, IR_AVOID_CORRECTION
      );
      if (-irStrength < correction) {
        correction = -irStrength;
        action = "LEFT - proportional IR protection";
      }
    }
  }

  correction = constrain(correction,
                         -MAX_CENTRE_CORRECTION,
                         MAX_CENTRE_CORRECTION);
  driveCentred(correction);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    #if SERIAL_MONITOR_ON
    Serial.print(F("US: ")); Serial.print(distance, 1);
    Serial.print(F(" cm | IR: ")); Serial.print(irReflection);
    Serial.print(F(" | Correction: ")); Serial.print(correction);
    Serial.print(F(" | Action: ")); Serial.println(action);
    #endif
  }
}



void twoRightTurns() {
  #if SERIAL_MONITOR_ON
  Serial.println(F("LIGHT BLUE: first right turn"));
  #endif
  turnRight();
  delay(50);
  
  #if SERIAL_MONITOR_ON
  Serial.println(F("LIGHT BLUE: travelling one grid"));
  #endif

  unsigned long started = millis();

  while (millis() - started < BLUE_ONE_GRID_TRAVEL_MS) {
    // Keep wall correction active.
    // Ignore black strips during this movement.
    driveBetweenWaypoints(false);
  }

  stopMotor();
  delay(50);
  
  #if SERIAL_MONITOR_ON
  Serial.println(F("LIGHT BLUE: second right turn"));
  #endif
  turnRight();
}

void twoLeftTurns() {
  #if SERIAL_MONITOR_ON
  Serial.println(F("PINK: first left turn"));
  #endif
  turnLeft();
  delay(50);
  #if SERIAL_MONITOR_ON
  Serial.println(F("PINK: travelling one grid"));
  #endif

  unsigned long started = millis();

  while (millis() - started < PINK_ONE_GRID_TRAVEL_MS) {
    // Wall correction stays active.
    // No second black strip is needed.
    driveBetweenWaypoints(false);
  }

  stopMotor();
  delay(50);
  
  #if SERIAL_MONITOR_ON
  Serial.println(F("PINK: second left turn"));
  #endif
  turnLeft();
}



//------------------------MUSIC------------------------------------//

const uint16_t melody[] PROGMEM = {
  NOTE_A4, REST, NOTE_B4, REST, NOTE_C5, REST, NOTE_A4, REST,
  NOTE_D5, REST, NOTE_E5, REST, NOTE_D5, REST,

  NOTE_G4, NOTE_A4, NOTE_C5, NOTE_A4, NOTE_E5, NOTE_E5, REST,
  NOTE_D5, REST,

  NOTE_G4, NOTE_A4, NOTE_C5, NOTE_A4, NOTE_D5, NOTE_D5, REST,
  NOTE_C5, REST, NOTE_B4, NOTE_A4, REST,

  NOTE_G4, NOTE_A4, NOTE_C5, NOTE_A4, NOTE_C5, NOTE_D5, REST,
  NOTE_B4, NOTE_A4, NOTE_G4, REST, NOTE_G4, REST,
  NOTE_D5, REST, NOTE_C5, REST,

  NOTE_G4, NOTE_A4, NOTE_C5, NOTE_A4, NOTE_E5, NOTE_E5, REST,
  NOTE_D5, REST,

  NOTE_G4, NOTE_A4, NOTE_C5, NOTE_A4, NOTE_G5, NOTE_B4, REST,
  NOTE_C5, REST, NOTE_B4, NOTE_A4, REST,

  NOTE_G4, NOTE_A4, NOTE_C5, NOTE_A4, NOTE_C5, NOTE_D5, REST,
  NOTE_B4, NOTE_A4, NOTE_G4, REST, NOTE_G4, REST,
  NOTE_D5, REST, NOTE_C5, REST,

  NOTE_C5, REST, NOTE_D5, REST, NOTE_G4, REST,
  NOTE_D5, REST, NOTE_E5, REST,
  NOTE_G5, NOTE_F5, NOTE_E5, REST,

  NOTE_C5, REST, NOTE_D5, REST, NOTE_G4, REST
};

const uint8_t durations[] PROGMEM = {
  8, 8, 8, 8, 8, 8, 8, 4,
  8, 8, 8, 8, 2, 2,

  8, 8, 8, 8, 2, 8, 8,
  2, 8,

  8, 8, 8, 8, 2, 8, 8,
  4, 8, 8, 8, 8,

  8, 8, 8, 8, 2, 8, 8,
  2, 8, 4, 8, 8, 8, 8, 8, 1, 4,

  8, 8, 8, 8, 2, 8, 8,
  2, 8,

  8, 8, 8, 8, 2, 8, 8,
  2, 8, 8, 8, 8,

  8, 8, 8, 8, 2, 8, 8,
  4, 8, 3, 8, 8, 8, 8, 8, 1, 4,

  2, 6, 2, 6, 4, 4, 2, 6, 2, 3,
  8, 8, 8, 8,

  2, 6, 2, 6, 2, 1
};

void playCompletionMusic() {
  const size_t notes =
    sizeof(melody) / sizeof(melody[0]);

  static_assert(
    sizeof(melody) / sizeof(melody[0]) ==
    sizeof(durations) / sizeof(durations[0]),
    "Melody and durations must have equal lengths"
  );

  stopMotor();

  for (size_t note = 0; note < notes; note++) {
    // Cycle: red → green → light blue → orange → pink.
    showChallengeColour((note % 5) + 1);
    uint16_t frequency =
      pgm_read_word(&melody[note]);

    uint8_t noteType =
      pgm_read_byte(&durations[note]);

    int duration = 1000 / noteType;

    if (frequency == REST) {
      delay(duration);
    }
    else {
      buzzer.tone(frequency, duration);
    }

    delay(duration * 3 / 10);
  }

  stopMotor();
}

//--------------------------SETUP-----------------------------------------
void setup() {
#if SERIAL_MONITOR_ON
  Serial.begin(9600);
#endif

  stopMotor();



  pinMode(DECODER_A0, OUTPUT);
  pinMode(DECODER_A1, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(IR_RX_PIN, INPUT);
  setLEDColor(3); // RGB off; IR on.

if (ENABLE_COLOUR_CHALLENGE) {
  // Use saved calibration instead of measuring again.
  calibrationReady = true;

  for (int i = 0; i < 3; i++) {
    greyDiff[i] = whiteArray[i] - blackArray[i];

    if (greyDiff[i] < 10.0f) {
      calibrationReady = false;
    }
  }
}

// Wait for the onboard user button before starting.
pinMode(A7, INPUT);
stopMotor();

#if SERIAL_MONITOR_ON
  Serial.println(F("Press the onboard button to start."));
#endif

while (true) {
  if (analogRead(A7) <= 10) {
    delay(30); // Brief debounce check.

    if (analogRead(A7) <= 10) {
      break; // Button confirmed: begin countdown.
    }
  }

  delay(10);
}


for (int seconds = 3; seconds > 0; seconds--) {
#if SERIAL_MONITOR_ON
  Serial.print(F("Starting in: "));
  Serial.println(seconds);
#endif

  buzzer.tone(1000, 100);  // Short beep: 100 ms
  delay(900);             // Total: 1 second per count
}

// Higher, longer beep signals the start
buzzer.tone(2000, 300);
  // Start automatically; a black reading is handled by loop().
  clearTimerRunning = false;
  navigationState = DRIVING;

}

//-----------------------------------------------------------------//
//--------------------------LOOP-----------------------------------//
//-----------------------------------------------------------------//
void loop() {

  // Arm restart after the initial start button is released.
int buttonValue = analogRead(A7);

if (buttonValue > 10) {
  restartButtonReady = true;
}

// A later button press requests a restart.
if (restartButtonReady && buttonValue <= 10) {
  stopMotor();
  delay(30); // Debounce.

  if (analogRead(A7) <= 10) {
    restartMazeRun();
    return;
  }
}

  if (mazeCompleted) {
    stopMotor();
    return;
  }

  
  int sensorState = lineFinder.readSensors();

  // Depart once. Ignore the handled strip until both detectors clear it.
  if (navigationState == LEAVING_STRIP) {
    if (sensorState == S1_OUT_S2_OUT) {
      if (!clearTimerRunning) {
        clearStarted = millis();
        clearTimerRunning = true;
      }
      if (millis() - clearStarted >= STRIP_CLEAR_MS) {
        navigationState = DRIVING;


#if SERIAL_MONITOR_ON
        Serial.println(F("Strip cleared: next waypoint armed."));
#endif
        return;
      }
    } 
    else {
      clearTimerRunning = false;
    }

    driveBetweenWaypoints(false);
    return;
  }

  // scan colours only after a black-strip trigger
  if (navigationState == CHECKING_COLOUR || sensorState != S1_OUT_S2_OUT) {

    stopMotor();
    
    navigationState = CHECKING_COLOUR;

    if (!ENABLE_COLOUR_CHALLENGE || !calibrationReady) {
#if SERIAL_MONITOR_ON
      Serial.println(F("Colour turns disabled: continuing forward."));
#endif
      clearTimerRunning = false;
      navigationState = LEAVING_STRIP;
      driveBetweenWaypoints(false);
      return;
    }
    showChallengeColour(0);
    delay(50);
    int first = detectColour();
   // int second = detectColour();

#if SERIAL_MONITOR_ON
    Serial.print(F("Colour ID: ")); 
    Serial.println(first);

#endif

// WHITE confirmed : complete the maze.
    if (first == 6 /* && second == 6*/) {
      mazeCompleted = true;
      stopMotor();
      showChallengeColour(6);
      playCompletionMusic();

      stopMotor();
      return;
    }



// Invalid or mismatched colours: sound, wait and retry 
// No attempt limit
    if (first != 1 && first != 2 && first != 3 &&
     first != 4 && first != 5) {

  stopMotor();

  buzzer.tone(300, 50);
  delay(200);

  return;
}

#if SERIAL_MONITOR_ON
Serial.println(F("Sending challenge colour to onboard LEDs"));
#endif


showChallengeColour(first);

    if (first == 1) {
#if SERIAL_MONITOR_ON
      Serial.println(F("RED -> LEFT 90-degree turn."));
#endif
      turnLeft();
    }
    else if (first == 2) {
#if SERIAL_MONITOR_ON
      Serial.println(F("GREEN -> RIGHT 90-degree turn."));
#endif
      turnRight();
    }

    else if (first == 3) {
#if SERIAL_MONITOR_ON
      Serial.println(F("LIGHT BLUE -> two successive right turns"));
#endif
      twoRightTurns();
    }

    else if (first == 5) {
#if SERIAL_MONITOR_ON
      Serial.println(F("PINK -> two successive left turns"));
#endif
      twoLeftTurns();
    }

else if (first == 4) {

  if (!uTurnWithClearance()) {
    // Stay in CHECKING_COLOUR.
    // No forward movement; retry on the next loop.
    return;
  }

  delay(50);

  // Existing straight exit after completing the turn.
  moveForward();
  delay(150);

  clearTimerRunning = false;
  navigationState = LEAVING_STRIP;
  return;
}

    delay(50);
    clearTimerRunning = false;
    navigationState = LEAVING_STRIP;
    return;
  }
  driveBetweenWaypoints(true);
}