#include <AccelStepper.h>

// ---------------------- BOARD ID ----------------------
#define BOARD_ID 1   // 2. UNO için 2 yapacaksın

// ---------------------- PINLER ------------------------
#define M1_STEP 2
#define M1_DIR  5
#define M1_EN   9

#define M2_STEP 3
#define M2_DIR  6
#define M2_EN   10

#define M3_STEP 4
#define M3_DIR  7
#define M3_EN   11

// ---------------------- STEPPER NESNELERI -------------
AccelStepper stepper1(AccelStepper::DRIVER, M1_STEP, M1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, M2_STEP, M2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, M3_STEP, M3_DIR);

// Varsayılan ayarlar
const float DEFAULT_MAX_SPEED = 800.0;
const float DEFAULT_ACCEL     = 300.0;

// Smart token parser
int splitTokens(String line, String tokens[], int maxTokens) {
  line.trim();
  int count = 0;

  while (line.length() > 0 && count < maxTokens) {
    int idx = line.indexOf(' ');
    if (idx == -1) {
      tokens[count++] = line;
      break;
    } else {
      tokens[count++] = line.substring(0, idx);
      line = line.substring(idx + 1);
      line.trim();
    }
  }
  return count;
}

// Enable motor (LOW active)
void enableMotor(int m, bool en) {
  int pin = -1;
  if      (m == 1) pin = M1_EN;
  else if (m == 2) pin = M2_EN;
  else if (m == 3) pin = M3_EN;

  if (pin == -1) return;

  digitalWrite(pin, en ? LOW : HIGH);
}

// --------------------- KOMUT ISLEME ---------------------
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String tokens[10];
  int n = splitTokens(line, tokens, 10);
  if (n == 0) return;

  String cmd = tokens[0];
  cmd.toUpperCase();

  // ------ ENABLE Tek Motor ------
  if (cmd == "EN") {
    if (n < 3) {
      Serial.println(F("ERR EN usage: EN m v"));
      return;
    }
    int m = tokens[1].toInt();
    int v = tokens[2].toInt();
    enableMotor(m, v == 1);
    Serial.println(F("OK EN"));
  }

  // ------ ENABLE ALL ------
  else if (cmd == "ENALL") {
    if (n < 2) {
      Serial.println(F("ERR ENALL v"));
      return;
    }
    int v = tokens[1].toInt();
    enableMotor(1, v == 1);
    enableMotor(2, v == 1);
    enableMotor(3, v == 1);
    Serial.println(F("OK ENALL"));
  }

  // ------ SET SPEED ------
  else if (cmd == "SET") {
    if (n < 4) {
      Serial.println(F("ERR SET m maxSpeed accel"));
      return;
    }
    int m = tokens[1].toInt();
    float maxS = tokens[2].toFloat();
    float acc  = tokens[3].toFloat();

    AccelStepper* st = nullptr;
    if      (m == 1) st = &stepper1;
    else if (m == 2) st = &stepper2;
    else if (m == 3) st = &stepper3;
    else {
      Serial.println(F("ERR MOTOR"));
      return;
    }

    st->setMaxSpeed(maxS);
    st->setAcceleration(acc);

    Serial.println(F("OK SET"));
  }

  // ------ MOVR (relative) ------
  else if (cmd == "MOVR") {
    if (n < 3) {
      Serial.println(F("ERR MOVR m steps"));
      return;
    }
    int m = tokens[1].toInt();
    long steps = tokens[2].toInt();

    AccelStepper* st = nullptr;
    if      (m == 1) st = &stepper1;
    else if (m == 2) st = &stepper2;
    else if (m == 3) st = &stepper3;
    else {
      Serial.println(F("ERR MOTOR"));
      return;
    }

    st->move(steps);

    Serial.println(F("OK MOVR"));
  }

  // ------ MOVA (absolute) ------
  else if (cmd == "MOVA") {
    if (n < 3) {
      Serial.println(F("ERR MOVA m pos"));
      return;
    }
    int m = tokens[1].toInt();
    long pos = tokens[2].toInt();

    AccelStepper* st = nullptr;
    if      (m == 1) st = &stepper1;
    else if (m == 2) st = &stepper2;
    else if (m == 3) st = &stepper3;
    else {
      Serial.println(F("ERR MOTOR"));
      return;
    }

    st->moveTo(pos);

    Serial.println(F("OK MOVA"));
  }

  // ------ 3 Motor Relative ------
  else if (cmd == "MOV3R") {
    if (n < 6) {
      Serial.println(F("ERR MOV3R d1 d2 d3 maxSpeed accel"));
      return;
    }

    long d1 = tokens[1].toInt();
    long d2 = tokens[2].toInt();
    long d3 = tokens[3].toInt();
    float maxS = tokens[4].toFloat();
    float acc  = tokens[5].toFloat();

    stepper1.setMaxSpeed(maxS);
    stepper2.setMaxSpeed(maxS);
    stepper3.setMaxSpeed(maxS);

    stepper1.setAcceleration(acc);
    stepper2.setAcceleration(acc);
    stepper3.setAcceleration(acc);

    stepper1.move(d1);
    stepper2.move(d2);
    stepper3.move(d3);

    Serial.println(F("OK MOV3R"));
  }

  // ------ 3 Motor Absolute ------
  else if (cmd == "MOV3A") {
    if (n < 6) {
      Serial.println(F("ERR MOV3A p1 p2 p3 maxSpeed accel"));
      return;
    }

    long p1 = tokens[1].toInt();
    long p2 = tokens[2].toInt();
    long p3 = tokens[3].toInt();
    float maxS = tokens[4].toFloat();
    float acc  = tokens[5].toFloat();

    stepper1.setMaxSpeed(maxS);
    stepper2.setMaxSpeed(maxS);
    stepper3.setMaxSpeed(maxS);

    stepper1.setAcceleration(acc);
    stepper2.setAcceleration(acc);
    stepper3.setAcceleration(acc);

    stepper1.moveTo(p1);
    stepper2.moveTo(p2);
    stepper3.moveTo(p3);

    Serial.println(F("OK MOV3A"));
  }

  // ------ ZERO ------
  else if (cmd == "ZERO") {
    if (n < 2) {
      Serial.println(F("ERR ZERO ALL | m"));
      return;
    }

    String w = tokens[1];
    w.toUpperCase();

    if (w == "ALL") {
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      stepper3.setCurrentPosition(0);
    } else {
      int m = w.toInt();
      if (m == 1) stepper1.setCurrentPosition(0);
      else if (m == 2) stepper2.setCurrentPosition(0);
      else if (m == 3) stepper3.setCurrentPosition(0);
      else {
        Serial.println(F("ERR ZERO"));
        return;
      }
    }

    Serial.println(F("OK ZERO"));
  }

  // ------ POS ------
  else if (cmd == "POS?") {
    Serial.print(F("POS "));
    Serial.print(stepper1.currentPosition()); Serial.print(' ');
    Serial.print(stepper2.currentPosition()); Serial.print(' ');
    Serial.print(stepper3.currentPosition()); Serial.println();
  }

  else {
    Serial.println(F("ERR UNKNOWN"));
  }
}

// ------------------------- SETUP -------------------------
void setup() {
  Serial.begin(115200);

  pinMode(M1_EN, OUTPUT);
  pinMode(M2_EN, OUTPUT);
  pinMode(M3_EN, OUTPUT);

  enableMotor(1, false);
  enableMotor(2, false);
  enableMotor(3, false);

  stepper1.setMaxSpeed(DEFAULT_MAX_SPEED);
  stepper2.setMaxSpeed(DEFAULT_MAX_SPEED);
  stepper3.setMaxSpeed(DEFAULT_MAX_SPEED);

  stepper1.setAcceleration(DEFAULT_ACCEL);
  stepper2.setAcceleration(DEFAULT_ACCEL);
  stepper3.setAcceleration(DEFAULT_ACCEL);

  Serial.println(F("3-Eksen Controller Hazır"));
  Serial.print(F("BOARD ")); Serial.println(BOARD_ID);
}

// -------------------------- LOOP -------------------------
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  stepper1.run();
  stepper2.run();
  stepper3.run();
}
