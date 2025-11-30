// Sequence with Start & Emergency buttons and LEDs
// M1 gearbox, M2 mixer, M3 diaphragm, M4 centrifugal
// Relay (peristaltic) D53
// Start A8->GND, Emergency A9->GND
// A0 pre turbidity, A1 post turbidity
// Green LED D30 (running), Red LED D31 (emergency)

#include <AFMotor.h>

// Motor shield
AF_DCMotor motorGearbox(1, MOTOR12_1KHZ);
AF_DCMotor motorMixer(2, MOTOR12_1KHZ);
AF_DCMotor pumpDiaphragm(3, MOTOR34_1KHZ);
AF_DCMotor pumpCentrifugal(4, MOTOR34_1KHZ);

// Relay
const int RELAY_PIN    = 53;
const int RELAY_ACTIVE = HIGH;
const int RELAY_IDLE   = LOW;

// Buttons
const int PIN_BTN_START = A8;
const int PIN_BTN_ESTOP = A9;

// LEDs
const int LED_GREEN = 30;
const int LED_RED   = 31;

// Turbidity sensors
const int PIN_TURB_PRE  = A0;
const int PIN_TURB_POST = A1;
const float ANALOG_REF_V = 5.0;

// Speeds
const int SPEED_GEARBOX     = 255;
const int SPEED_MIX_FAST    = 255;
const int SPEED_MIX_SLOW    = 140;
const int SPEED_DIAPHRAGM   = 255;
const int SPEED_CENTRIFUGAL = 255;

// Timings
const unsigned long PRECHECK_TIME_MS    = 30000;    // pre turbidity duration
const unsigned long POSTCHECK_TIME_MS   = 30000;    // post turbidity duration
const unsigned long FILL_TIME_MS        = 28000;
const unsigned long ALUM_TIME_MS        = 5800;
const unsigned long MIX_FAST_TIME_MS    = 40000;
const unsigned long MIX_SLOW_TIME_MS    = 45000;
const unsigned long PRESS_PAUSE_MS      = 120000;   // 120 s pause before press plate
const unsigned long PRESS_LOWER_TIME_MS = 20000;
const unsigned long TRANSFER_TIME_MS    = 85000;

const unsigned long LOG_INTERVAL_MS = 1000;
const unsigned long DEBOUNCE_MS     = 50;

enum State {
  WAIT_START,
  PRECHECK,
  FILL,
  ALUM,
  MIX_FAST,
  MIX_SLOW,
  WAIT_BEFORE_PRESS,  
  PRESS_LOWER,
  TRANSFER,
  POSTCHECK,
  DONE,
  EMERGENCY
};

State state = WAIT_START;
unsigned long t0 = 0;
unsigned long tLastLog = 0;

// setup
void setup() {
  Serial.begin(9600);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_IDLE);

  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_ESTOP, INPUT_PULLUP);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  allOff();

  Serial.println("Ready. Press Start (A8) to begin sequence.");
}

// loop
void loop() {

  // emergency stop 
  if (digitalRead(PIN_BTN_ESTOP) == LOW && state != EMERGENCY) {
    enterEmergency();
  }

  switch (state) {

    case WAIT_START:
      if (pressed(PIN_BTN_START)) {
        delay(DEBOUNCE_MS);
        if (pressed(PIN_BTN_START)) {
          Serial.println("Start pressed -> Stage 1: Pre turbidity check");
          digitalWrite(LED_GREEN, HIGH);
          digitalWrite(LED_RED, LOW);
          startPrecheck();
        }
      }
      break;

    case PRECHECK:
      logTurbidityIfDue(PIN_TURB_PRE);
      if (millis() - t0 >= PRECHECK_TIME_MS) {
        Serial.println("Precheck done -> Stage 2: Fill dirty water");
        startFill();
      }
      break;

    case FILL:
      if (millis() - t0 >= FILL_TIME_MS) {
        pumpCentrifugal.run(RELEASE);
        Serial.println("Fill done -> Stage 3: Fill alum");
        startAlum();
      }
      break;

    case ALUM:
      if (millis() - t0 >= ALUM_TIME_MS) {
        pumpDiaphragm.run(RELEASE);
        Serial.println("Alum done -> Stage 4: Mix fast");
        startMixFast();
      }
      break;

    case MIX_FAST:
      if (millis() - t0 >= MIX_FAST_TIME_MS) {
        Serial.println("Stage 5: Switch to mix slow");
        startMixSlow();
      }
      break;

    case MIX_SLOW:
      if (millis() - t0 >= MIX_SLOW_TIME_MS) {
        motorMixer.run(RELEASE);
        Serial.println("Mixing done -> Stage 6: Waiting 120s for settling");
        t0 = millis();
        state = WAIT_BEFORE_PRESS;
      }
      break;

    case WAIT_BEFORE_PRESS:
      if (millis() - t0 >= PRESS_PAUSE_MS) {
        Serial.println("45 s pause done -> Stage 7: Lower plate");
        startPressLower();
      }
      break;

    case PRESS_LOWER:
      if (millis() - t0 >= PRESS_LOWER_TIME_MS) {
        motorGearbox.run(RELEASE);
        Serial.println("Plate lowered -> Stage 8: Transfer to clean water tank");
        startTransfer();
      }
      break;

    case TRANSFER:
      if (millis() - t0 >= TRANSFER_TIME_MS) {
        digitalWrite(RELAY_PIN, RELAY_IDLE);
        Serial.println("Transfer done -> Stage 9: Post turbidity check ");
        startPostcheck();
      }
      break;

    case POSTCHECK:
      logTurbidityIfDue(PIN_TURB_POST);
      if (millis() - t0 >= POSTCHECK_TIME_MS) {
        Serial.println("Postcheck complete");
        state = DONE;
      }
      break;

    case DONE:
      allOff();
      digitalWrite(LED_GREEN, LOW);
      if (pressed(PIN_BTN_START)) {
        delay(DEBOUNCE_MS);
        if (pressed(PIN_BTN_START)) {
          Serial.println("Restart");
          digitalWrite(LED_GREEN, HIGH);
          digitalWrite(LED_RED, LOW);
          startPrecheck();
        }
      }
      break;

    case EMERGENCY:
      if (pressed(PIN_BTN_START)) {
        delay(DEBOUNCE_MS);
        if (pressed(PIN_BTN_START)) {
          Serial.println("Emergency cleared -> Waiting for restart");
          digitalWrite(LED_RED, LOW);
          state = WAIT_START;
        }
      }
      break;
  }
}

// Stage starters
void startPrecheck() {
  t0 = millis();
  tLastLog = 0;
  printHeader();
  state = PRECHECK;
}

void startFill() {
  pumpCentrifugal.setSpeed(SPEED_CENTRIFUGAL);
  pumpCentrifugal.run(FORWARD);
  t0 = millis();
  state = FILL;
}

void startAlum() {
  pumpDiaphragm.setSpeed(SPEED_DIAPHRAGM);
  pumpDiaphragm.run(FORWARD);
  t0 = millis();
  state = ALUM;
}

void startMixFast() {
  motorMixer.setSpeed(SPEED_MIX_FAST);
  motorMixer.run(BACKWARD);
  t0 = millis();
  state = MIX_FAST;
}

void startMixSlow() {
  motorMixer.setSpeed(SPEED_MIX_SLOW);
  t0 = millis();
  state = MIX_SLOW;
}

void startPressLower() {
  motorGearbox.setSpeed(SPEED_GEARBOX);
  motorGearbox.run(BACKWARD);
  t0 = millis();
  state = PRESS_LOWER;
}

void startTransfer() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE);
  t0 = millis();
  state = TRANSFER;
}

void startPostcheck() {
  t0 = millis();
  tLastLog = 0;
  printHeader();
  state = POSTCHECK;
}

// Utility
bool pressed(int pin) {
  return digitalRead(pin) == LOW;
}

void enterEmergency() {
  Serial.println("EMERGENCY STOP");
  allOff();
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);
  state = EMERGENCY;
}

void allOff() {
  motorGearbox.run(RELEASE);
  motorMixer.run(RELEASE);
  pumpDiaphragm.run(RELEASE);
  pumpCentrifugal.run(RELEASE);
  digitalWrite(RELAY_PIN, RELAY_IDLE);
}

void printHeader() {
  Serial.println("time_s voltage_V");
}

void logTurbidityIfDue(int analogPin) {
  unsigned long now = millis();
  if (now - tLastLog >= LOG_INTERVAL_MS) {
    unsigned long t_s = (now - t0) / 1000;
    int adc = analogRead(analogPin);
    float volts = (adc * ANALOG_REF_V) / 1023.0f;
    Serial.print(t_s);
    Serial.print(" ");
    Serial.println(volts, 2);
    tLastLog = now;
  }
}
