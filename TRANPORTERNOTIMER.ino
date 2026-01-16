#include <Ps3Controller.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <EEPROM.h>
#include "var.h"

//PIN MOTOR motor
#define M5A 32
#define M5B 33

#define M1A 27
#define M1B 13
#define M2A 19
#define M2B 18
#define M3A 4
#define M3B 17
#define M4A 25
#define M4B 26
#define encA_M1 36
#define encB_M1 35

//Encoder pins
#define encA_M1 36
#define encB_M1 35

// ✅ UBAH:   Tambah Mode 5
#define ADDR_POSITION_TARGETS_MODE1 300  // Mode 1: 3 target (3x4 bytes = 12 bytes)
#define ADDR_POSITION_TARGETS_MODE2 312  // Mode 2: 3 target (3x4 bytes = 12 bytes)
#define ADDR_POSITION_TARGETS_MODE3 324  // Mode 3: 3 target (3x4 bytes = 12 bytes)
#define ADDR_POSITION_TARGETS_MODE4 336  // Mode 4: 3 target (3x4 bytes = 12 bytes)
#define ADDR_POSITION_TARGETS_MODE5 348  // ✅ TAMBAH: Mode 5: 3 target (3x4 bytes = 12 bytes)
#define ADDR_POSITION_BUTTONS_MODE1 360  // ✅ UBAH: Mode 1: 3 button (3x1 bytes = 3 bytes)
#define ADDR_POSITION_BUTTONS_MODE2 363  // ✅ UBAH: Mode 2: 3 button (3x1 bytes = 3 bytes)
#define ADDR_POSITION_BUTTONS_MODE3 366  // ✅ UBAH:   Mode 3: 3 button (3x1 bytes = 3 bytes)
#define ADDR_POSITION_BUTTONS_MODE4 369  // ✅ UBAH:  Mode 4: 3 button (3x1 bytes = 3 bytes)
#define ADDR_POSITION_BUTTONS_MODE5 372  // ✅ TAMBAH: Mode 5: 3 button (3x1 bytes = 3 bytes)

// ============ THROTTLE SYSTEM VARIABLES ============
// EEPROM addresses untuk throttle settings
#define ADDR_THROTTLE_ENABLED 375    // 1 byte - throttle on/off
#define ADDR_THROTTLE_PWM_STEP 376   // 4 bytes - int
#define ADDR_THROTTLE_INTERVAL 380   // 4 bytes - int (microseconds)
#define ADDR_THROTTLE_THRESHOLD 384  // 4 bytes - int

// Throttle parameters
bool throttleEnabled = false;           // Throttle aktif/nonaktif
int throttlePwmStep = 50;               // PWM increment per tick
int throttleIntervalUs = 20000;         // Timer interval (20ms)
int throttleActivationThreshold = 100;  // Throttle aktif jika PWM > threshold

// Hardware timer untuk throttle
hw_timer_t *throttleTimer = NULL;
volatile SemaphoreHandle_t throttleTimerSemaphore;

// Throttle state variables - PWM based
volatile int currentPwmMotor[4] = { 0, 0, 0, 0 };  // Current PWM motor 1-4
volatile int targetPwmMotor[4] = { 0, 0, 0, 0 };   // Target PWM dari kinematik

// Menu throttle variables
int throttleMenuSelected = 0;  // 0=PWM Step, 1=Interval, 2=Threshold, 3=Enable/Disable

// ✅ UBAH:  Position control arrays untuk 5 mode
long positionTargets[5][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };     // [mode][target]
uint8_t positionButtons[5][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };  // [mode][target]

// ✅ UBAH: Position control states untuk 5 mode
bool positionButtonStates[5][3] = { { false, false, false }, { false, false, false }, { false, false, false }, { false, false, false }, { false, false, false } };
unsigned long positionLastTrigger[5][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

int currentPositionMode = 0;  // 0-4 untuk mode 1-5
bool modeChangeDebounce = false;
unsigned long lastModeChange = 0;

//  Encoder variables
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile long currentPosition = 0;
volatile int lastEncoderStateA = 0;
long targetPosition = 0;
int encoderMotorSpeed = 1023;  // Kecepatan motor encoder (0-255)

unsigned long lastModeChangeTime = 0;

bool manualPositionMode = false;     // Mode untuk set target manual
bool positionControlActive = false;  // Apakah position control sedang aktif

// PARAMETER MOTOR
#define FREQ_MOTOR 16000
#define RESOLUTION_MOTOR 10

// ---- OLED SETUP ----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define WHITE 1
#define BLACK 0

// ---- PCA9685 SETUP untuk Servo MG90S ----
#define SERVO_FREQ 50
#define USMIN 600
#define USMAX 2400

// ---- EEPROM ADDRESSES ----
#define ADDR_SERVO_OPEN 16
#define ADDR_SERVO_CLOSE 32
#define ADDR_SERVO_OPEN2 48
#define ADDR_LAMBDA 64
#define ADDR_LENGTH_ALPHA 68
#define ADDR_BUTTON_MAPPING 72

// ---- TIMER SETUP ----
hw_timer_t *servoTimer = NULL;
volatile SemaphoreHandle_t servoTimerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

int limitTimeOut = 100;
int countTimeOutOled = 0;
int oledTimeOut = false;



//pinmotor
const int pinMotor[] = { M1A, M1B, M2A, M2B, M3A, M3B, M4A, M4B, M5A, M5B };

// Struct untuk button mapping servo dengan 2 button per servo
struct ServoButtonMapping {
  uint8_t buttonType1;
  uint8_t buttonType2;
  bool state1;
  bool state2;
  bool lastButtonState1;
  bool lastButtonState2;
  unsigned long lastToggleTime1;
  unsigned long lastToggleTime2;
};
ServoButtonMapping servoButtonMap[4];

// Mode OLED - ✅ HAPUS motor timer modes
enum OledMode {
  OLED_INFO,
  OLED_INFO_PWM,
  OLED_MENU_SETTING_SERVO,
  OLED_MENU_SETTING_LAMBDA_LENGTHALPHA,
  OLED_MENU_BUTTON_MAPPING,
  OLED_MENU_POSITION_CONTROL,  // Menu position control
  OLED_MENU_THROTTLE_SETTING
};

// ✅ UBAH: Position menu states untuk navigasi yang lebih mudah
enum PositionMenuState {
  MENU_POSITION_MAIN,         // Menu utama position control
  MENU_POSITION_MODE_SELECT,  //   Pilih mode 1/2/3/4/5
  MENU_POSITION_TARGET_LIST,  // Pilih target 1/2/3 dari mode yang dipilih
  MENU_POSITION_SET_TARGET,   // Set nilai target
  MENU_POSITION_SET_BUTTON    // Set button trigger
};
PositionMenuState positionMenuState = MENU_POSITION_MAIN;

// Position control menu variables
int selectedPositionTarget = 0;  // 0-2 untuk target 1-3
int selectedPositionMode = 0;
bool waitingForPositionButton = false;
OledMode oledMode = OLED_INFO;

enum ServoMenuState {
  MENU_SERVO_LIST,
  MENU_SERVO_OCC
};
ServoMenuState servoMenuState = MENU_SERVO_LIST;

int lambdaMenuSelected = 0;
int selectedServoForMapping = 0;
int selectedButtonForMapping = 0;
bool waitingForButtonPress = false;

// Data servo dan parameter
int servoOpen[4] = { 90, 90, 90, 90 };
int servoOpen2[4] = { 45, 45, 45, 45 };
int servoClose[4] = { 0, 0, 0, 0 };
int selectedSetting = 0;
int editServoIndex = 0;
int editOCCSelected = 0;

float lambda = 1.0f;
const float minLambda = 0.1f, maxLambda = 5.683f, stepLambda = 0.1f;

float LengthAlpha = 0.2f;
const float minLengthAlpha = 0.1f, maxLengthAlpha = 1.0f, stepLengthAlpha = 0.01f;

int m1, m2, m3, m4;
int lx, ly, rx, l2Value, r2Value;

#define d2r(x) x *(PI / 180)

bool isConnected = false;
bool lastCross = false, lastCircle = false, lastSquare = false, lastTriangle = false;
bool lastL1 = false, lastR1 = false, lastL2 = false, lastR2 = false;
bool lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
bool lastSelect = false, lastStart = false, lastPs = false, lastL3 = false;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

unsigned long lastButtonPress = 0;
const unsigned long BUTTON_DEBOUNCE = 150;
const unsigned long AUTO_REPEAT_DELAY = 100;
const unsigned long SERVO_DEBOUNCE = 150;

// ============ ENCODER FUNCTIONS ============

void ARDUINO_ISR_ATTR encoder_ISR() {
  portENTER_CRITICAL_ISR(&encoderMux);
  int currentStateA = digitalRead(encA_M1);
  int currentStateB = digitalRead(encB_M1);

  if (currentStateA != lastEncoderStateA) {
    if (currentStateA == HIGH) {
      if (currentStateB == LOW) {
        currentPosition++;
      } else {
        currentPosition--;
      }
    }
  }
  lastEncoderStateA = currentStateA;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void ARDUINO_ISR_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  if (oledMode == OLED_INFO && countTimeOutOled < limitTimeOut) {
    if (++countTimeOutOled == limitTimeOut) {
      oledTimeOut = true;
    }
  }

  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

// ============ THROTTLE TIMER ISR ============
void ARDUINO_ISR_ATTR onThrottleTimer() {
  xSemaphoreGiveFromISR(throttleTimerSemaphore, NULL);
}

void setupThrottleTimer() {
  throttleTimerSemaphore = xSemaphoreCreateBinary();
  throttleTimer = timerBegin(1000000);  // 1MHz
  timerAttachInterrupt(throttleTimer, &onThrottleTimer);
  timerAlarm(throttleTimer, throttleIntervalUs, true, 0);
  Serial.println("Throttle Timer initialized");
}


void loadPositionData() {
  Serial.println("📖 Loading position data from EEPROM...");

  for (int mode = 0; mode < 5; mode++) {
    for (int i = 0; i < 3; i++) {
      int targetAddr = ADDR_POSITION_TARGETS_MODE1 + (mode * 12) + (i * sizeof(long));
      int buttonAddr = ADDR_POSITION_BUTTONS_MODE1 + (mode * 3) + i;

      EEPROM.get(targetAddr, positionTargets[mode][i]);
      EEPROM.get(buttonAddr, positionButtons[mode][i]);

      Serial.printf("Mode %d Target %d: %ld, Button:   %s\n",
                    mode + 1, i + 1, positionTargets[mode][i],
                    getButtonName(positionButtons[mode][i]).c_str());
    }
  }
}

void savePositionData() {
  Serial.println("💾 Saving position data to EEPROM...");

  for (int mode = 0; mode < 5; mode++) {
    for (int i = 0; i < 3; i++) {
      int targetAddr = ADDR_POSITION_TARGETS_MODE1 + (mode * 12) + (i * sizeof(long));
      int buttonAddr = ADDR_POSITION_BUTTONS_MODE1 + (mode * 3) + i;

      EEPROM.put(targetAddr, positionTargets[mode][i]);
      EEPROM.put(buttonAddr, positionButtons[mode][i]);
    }
  }

  EEPROM.commit();
  Serial.println("✅ Position data saved!");
}

void setupEncoder() {
  pinMode(encA_M1, INPUT_PULLUP);
  pinMode(encB_M1, INPUT_PULLUP);

  lastEncoderStateA = digitalRead(encA_M1);
  attachInterrupt(digitalPinToInterrupt(encA_M1), encoder_ISR, CHANGE);

  Serial.println("Encoder initialized");
}

void kontrolMotorWithEncoder(int pwm) {
  pwm = constrain(pwm, -1023, 1023);

  static int lastPwm = 0;
  if (pwm != lastPwm) {
    Serial.printf("🔧 Motor PWM:  %d (M5A=%d, M5B=%d)\n",
                  pwm,
                  (pwm > 0) ? pwm : 0,
                  (pwm < 0) ? abs(pwm) : 0);
    lastPwm = pwm;
  }

  if (pwm > 0) {
    ledcWrite(M5A, pwm);
    ledcWrite(M5B, 0);
  } else if (pwm < 0) {
    ledcWrite(M5A, 0);
    ledcWrite(M5B, abs(pwm));
  } else {
    ledcWrite(M5A, 0);
    ledcWrite(M5B, 0);
  }
}

void moveToPosition(long target) {
  targetPosition = target;
  positionControlActive = true;
  Serial.printf("Moving to position: %ld\n", target);
}

void resetEncoderPosition() {
  portENTER_CRITICAL(&encoderMux);
  currentPosition = 0;
  portEXIT_CRITICAL(&encoderMux);
  targetPosition = 0;
  positionControlActive = false;
  kontrolMotorWithEncoder(0);
  Serial.println("Encoder position reset to 0");
}

long getCurrentPosition() {
  long pos;
  portENTER_CRITICAL(&encoderMux);
  pos = currentPosition;
  portEXIT_CRITICAL(&encoderMux);
  return pos;
}

void updatePositionControl() {
  if (!positionControlActive) return;

  static unsigned long positionStartTime = 0;
  if (positionStartTime == 0) {
    positionStartTime = millis();
  }

  if (millis() - positionStartTime > 10000) {
    Serial.println("⚠️ Position control TIMEOUT - stopping motor");
    kontrolMotorWithEncoder(0);
    positionControlActive = false;
    positionStartTime = 0;
    return;
  }

  long currentPos = getCurrentPosition();
  long error = targetPosition - currentPos;

  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 1000) {
    Serial.printf("🎯 Pos:%ld->%ld Err:%ld\n", currentPos, targetPosition, error);
    lastLog = millis();
  }

  if (abs(error) > 10) {
    const int FIXED_PWM = 1023;

    if (error > 0) {
      kontrolMotorWithEncoder(FIXED_PWM);
    } else {
      kontrolMotorWithEncoder(-FIXED_PWM);
    }
  } else {
    kontrolMotorWithEncoder(0);
    positionControlActive = false;
    positionStartTime = 0;
    Serial.printf(" Posisi:   %ld\n", currentPos);
  }
}

void handlePositionControlTriggers() {
  if (!Ps3.isConnected()) return;

  if (oledMode != OLED_INFO && oledMode != OLED_INFO_PWM) {
    return;
  }

  unsigned long now = millis();

  if (Ps3.data.button.start && !lastStart) {
    if (now - lastButtonPress >= 50) {
      resetEncoderPosition();
      lastButtonPress = now;

      if (oledMode == OLED_INFO) drawInfo();
      else if (oledMode == OLED_INFO_PWM) drawPwmMotor();
    }
  }
  lastStart = Ps3.data.button.start;

  for (int i = 0; i < 3; i++) {
    if (positionButtons[currentPositionMode][i] != 0) {
      bool currentState = getButtonState(positionButtons[currentPositionMode][i]);

      if (currentState && !positionButtonStates[currentPositionMode][i]) {
        if (now - positionLastTrigger[currentPositionMode][i] >= 50) {
          Serial.printf("🚀 Mode %d Target %d:   %ld\n",
                        currentPositionMode + 1, i + 1, positionTargets[currentPositionMode][i]);
          moveToPosition(positionTargets[currentPositionMode][i]);
          positionLastTrigger[currentPositionMode][i] = now;

          if (oledMode == OLED_INFO) drawInfo();
          else if (oledMode == OLED_INFO_PWM) drawPwmMotor();
        }
      }
      positionButtonStates[currentPositionMode][i] = currentState;
    }
  }
}

void drawPositionTargetMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if (positionMenuState == MENU_POSITION_MODE_SELECT) {
    display.setCursor(23, 1);
    display.println("SET LEVEL LIFT");

    display.drawLine(0, 9, 127, 10, 1);


    for (int i = 0; i < 5; i++) {
      int yPos = 12 + i * 10;

      if (selectedPositionMode == i) {
        display.fillRect(5, yPos, 118, 9, WHITE);
        display.setTextColor(BLACK, WHITE);
      } else {
        display.setTextColor(WHITE);
      }

      display.setCursor(7, yPos + 1);
      display.printf("MODE %d", i + 1);

      int setCount = 0;
      for (int j = 0; j < 3; j++) {
        if (positionButtons[i][j] != 0) setCount++;
      }
      display.printf(" (%d/3)", setCount);
      display.setTextColor(WHITE);
    }

  } else if (positionMenuState == MENU_POSITION_TARGET_LIST) {
    display.setCursor(0, 0);
    display.printf("MODE %d Targets:", selectedPositionMode + 1);

    for (int i = 0; i < 3; i++) {
      int yPos = 18 + i * 14;

      if (selectedPositionTarget == i) {
        display.fillRect(2, yPos, 124, 12, WHITE);
        display.setTextColor(BLACK, WHITE);
      } else {
        display.setTextColor(WHITE);
      }

      display.setCursor(4, yPos + 2);
      display.print("TARGET");
      display.print(i + 1);
      display.print(":");

      long target = positionTargets[selectedPositionMode][i];
      uint8_t btn = positionButtons[selectedPositionMode][i];

      display.printf("%ld ", target);
      display.print(getButtonName(btn));
      display.setTextColor(WHITE);
    }

  } else if (positionMenuState == MENU_POSITION_SET_TARGET) {
    display.setCursor(0, 0);
    display.printf("SET MODE %d TARGET %d:", selectedPositionMode + 1, selectedPositionTarget + 1);

    display.setCursor(0, 15);
    display.println("Gerakkan Motor Manual");
    display.setCursor(0, 25);
    display.println("Kemudian tekan CROSS");

    long currentPos = getCurrentPosition();
    display.setTextSize(2);
    display.setCursor(10, 40);
    display.printf("%ld", currentPos);

    display.setTextSize(1);

  } else if (positionMenuState == MENU_POSITION_SET_BUTTON) {
    if (waitingForPositionButton) {
      display.clearDisplay();
      display.drawBitmap(10, 16, _PRESSBUTTON, 111, 32, 1);
    } else {
      display.setCursor(0, 0);
      display.clearDisplay();
      display.printf("M%d T%d - Button:", selectedPositionMode + 1, selectedPositionTarget + 1);

      display.setTextSize(2);
      display.setCursor(35, 29);
      uint8_t btn = positionButtons[selectedPositionMode][selectedPositionTarget];
      display.print(getButtonName(btn));
    }
  }

  display.display();
}

// ✅ HAPUS pengecekan motor timer dari fungsi ini
bool isButtonAlreadyAssigned(uint8_t buttonType, bool isForPosition = false) {
  // Check servo mappings
  for (int i = 0; i < 4; i++) {
    if (servoButtonMap[i].buttonType1 == buttonType || servoButtonMap[i].buttonType2 == buttonType) {
      return true;
    }
  }

  // ✅ BARU:  Jika untuk position control, IZINKAN penggunaan ulang button
  if (isForPosition) {
    return false;  // Button boleh digunakan untuk position targets
  }

  // ✅ UBAH: Check position buttons hanya jika bukan untuk position assignment
  for (int mode = 0; mode < 5; mode++) {
    for (int i = 0; i < 3; i++) {
      if (positionButtons[mode][i] == buttonType) {
        return true;
      }
    }
  }

  return false;
}

void handlePositionTargetMenuNavigation() {
  static unsigned long lastInput = 0;
  static unsigned long lastDisplayUpdate = 0;
  unsigned long now = millis();

  if (positionMenuState == MENU_POSITION_MODE_SELECT) {
    static bool lastUpNav = false;
    static bool lastDownNav = false;
    bool upPressed = Ps3.data.button.up && !lastUpNav;
    bool downPressed = Ps3.data.button.down && !lastDownNav;
    lastUpNav = Ps3.data.button.up;
    lastDownNav = Ps3.data.button.down;

    bool crossPressed = Ps3.data.button.cross && !lastCross;
    bool circlePressed = Ps3.data.button.circle && !lastCircle;

    lastCross = Ps3.data.button.cross;
    lastCircle = Ps3.data.button.circle;

    if (upPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      selectedPositionMode = (selectedPositionMode - 1 + 5) % 5;
      drawPositionTargetMenu();
      lastInput = now;
    } else if (downPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      selectedPositionMode = (selectedPositionMode + 1) % 5;
      drawPositionTargetMenu();
      lastInput = now;
    } else if (crossPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      positionMenuState = MENU_POSITION_TARGET_LIST;
      selectedPositionTarget = 0;
      drawPositionTargetMenu();
      lastInput = now;
    } else if (circlePressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      oledMode = OLED_INFO;
      stopMotor();
      lastInput = now;
    }

  } else if (positionMenuState == MENU_POSITION_TARGET_LIST) {
    static bool lastUpNav = false;
    static bool lastDownNav = false;
    bool upPressed = Ps3.data.button.up && !lastUpNav;
    bool downPressed = Ps3.data.button.down && !lastDownNav;
    lastUpNav = Ps3.data.button.up;
    lastDownNav = Ps3.data.button.down;

    bool crossPressed = Ps3.data.button.cross && !lastCross;
    bool circlePressed = Ps3.data.button.circle && !lastCircle;

    lastCross = Ps3.data.button.cross;
    lastCircle = Ps3.data.button.circle;

    if (upPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      selectedPositionTarget = (selectedPositionTarget - 1 + 3) % 3;
      drawPositionTargetMenu();
      lastInput = now;
    } else if (downPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      selectedPositionTarget = (selectedPositionTarget + 1) % 3;
      drawPositionTargetMenu();
      lastInput = now;
    } else if (crossPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      positionMenuState = MENU_POSITION_SET_TARGET;
      manualPositionMode = true;
      drawPositionTargetMenu();
      lastInput = now;
      lastDisplayUpdate = now;
    } else if (circlePressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      positionMenuState = MENU_POSITION_MODE_SELECT;
      drawPositionTargetMenu();
      lastInput = now;
    }

  } else if (positionMenuState == MENU_POSITION_SET_TARGET) {
    bool crossPressed = Ps3.data.button.cross && !lastCross;
    bool circlePressed = Ps3.data.button.circle && !lastCircle;

    lastCross = Ps3.data.button.cross;
    lastCircle = Ps3.data.button.circle;

    if (crossPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      long currentPos = getCurrentPosition();
      positionTargets[selectedPositionMode][selectedPositionTarget] = currentPos;

      display.clearDisplay();
      display.drawBitmap(48, 8, _saved, 32, 49, 1);
      display.display();
      delay(800);

      Serial.printf("✅ Saved Mode %d Target %d:   %ld\n",
                    selectedPositionMode + 1, selectedPositionTarget + 1, currentPos);

      manualPositionMode = false;
      positionMenuState = MENU_POSITION_SET_BUTTON;
      waitingForPositionButton = false;
      drawPositionTargetMenu();
      lastInput = now;
    } else if (circlePressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      manualPositionMode = false;
      positionMenuState = MENU_POSITION_TARGET_LIST;
      drawPositionTargetMenu();
      lastInput = now;
    }

    if (now - lastDisplayUpdate >= 200) {
      drawPositionTargetMenu();
      lastDisplayUpdate = now;
    }

  } else if (positionMenuState == MENU_POSITION_SET_BUTTON) {
    if (waitingForPositionButton) {
      uint8_t detectedButton = 0;

      if (Ps3.data.button.l1 && !lastL1) detectedButton = 5;
      else if (Ps3.data.button.r1 && !lastR1) detectedButton = 6;
      else if (Ps3.data.button.cross && !lastCross) detectedButton = 1;
      else if (Ps3.data.button.circle && !lastCircle) detectedButton = 2;
      else if (Ps3.data.button.square && !lastSquare) detectedButton = 3;
      else if (Ps3.data.button.triangle && !lastTriangle) detectedButton = 4;
      else if (Ps3.data.button.r3 && !lastL3) detectedButton = 13;

      lastL1 = Ps3.data.button.l1;
      lastR1 = Ps3.data.button.r1;
      lastCross = Ps3.data.button.cross;
      lastCircle = Ps3.data.button.circle;
      lastSquare = Ps3.data.button.square;
      lastTriangle = Ps3.data.button.triangle;
      lastL3 = Ps3.data.button.r3;

      if (detectedButton > 0) {
        bool alreadyUsed = isButtonAlreadyAssigned(detectedButton, true);

        if (alreadyUsed) {
          display.clearDisplay();
          display.drawBitmap(20, 6, _assigned, 89, 52, 1);
          display.display();
          delay(800);
          Serial.printf("⚠️ Button %s already assigned to servo!\n", getButtonName(detectedButton).c_str());
        } else {
          positionButtons[selectedPositionMode][selectedPositionTarget] = detectedButton;

          display.clearDisplay();
          display.drawBitmap(46, -2, _SUCCESS, 36, 48, 1);
          display.setTextSize(1);
          display.setCursor(50, 46);
          display.print(getButtonName(detectedButton));
          display.display();
          delay(500);

          Serial.printf("✅ Assigned button %s to Mode %d Target %d (reuse allowed)\n",
                        getButtonName(detectedButton).c_str(),
                        selectedPositionMode + 1, selectedPositionTarget + 1);
        }

        waitingForPositionButton = false;
        drawPositionTargetMenu();
        delay(300);
      }

    } else {
      bool crossPressed = Ps3.data.button.cross && !lastCross;
      bool trianglePressed = Ps3.data.button.triangle && !lastTriangle;
      bool circlePressed = Ps3.data.button.circle && !lastCircle;

      lastCross = Ps3.data.button.cross;
      lastTriangle = Ps3.data.button.triangle;
      lastCircle = Ps3.data.button.circle;

      if (crossPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
        waitingForPositionButton = true;
        drawPositionTargetMenu();
        lastInput = now;
        delay(300);
      } else if (trianglePressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
        positionButtons[selectedPositionMode][selectedPositionTarget] = 0;

        display.clearDisplay();
        display.drawBitmap(23, 9, _cleared, 83, 47, 1);
        display.display();
        delay(800);

        drawPositionTargetMenu();
        lastInput = now;
      } else if (circlePressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
        savePositionData();

        display.clearDisplay();
        display.drawBitmap(48, 8, _saved, 32, 49, 1);
        display.display();
        delay(500);

        positionMenuState = MENU_POSITION_TARGET_LIST;
        drawPositionTargetMenu();
        lastInput = now;
      }
    }
  }
}

// ============ TIMER ISR ============
void ARDUINO_ISR_ATTR onServoTimer() {
  xSemaphoreGiveFromISR(servoTimerSemaphore, NULL);
}

int angleToUs(int angle) {
  return map(angle, 0, 180, USMIN, USMAX);
}

void setServoAngleStrong(uint8_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  int us = angleToUs(angle);
  pca.writeMicroseconds(ch, us);
}

void pcaSetup() {
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);
}

String getButtonName(uint8_t buttonType) {
  switch (buttonType) {
    case 1: return "CROSS";
    case 2: return "CIRCLE";
    case 3: return "SQUARE";
    case 4: return "TRIANGLE";
    case 5: return "L1";
    case 6: return "R1";
    case 7: return "L2";
    case 8: return "R2";
    case 9: return "UP";
    case 10: return "DOWN";
    case 11: return "LEFT";
    case 12: return "RIGHT";
    case 13: return "L3";
    default: return "NONE";
  }
}

bool getButtonState(uint8_t buttonType) {
  switch (buttonType) {
    case 1: return Ps3.data.button.cross;
    case 2: return Ps3.data.button.circle;
    case 3: return Ps3.data.button.square;
    case 4: return Ps3.data.button.triangle;
    case 5: return Ps3.data.button.l1;
    case 6: return Ps3.data.button.r1;
    case 7: return (Ps3.data.analog.button.l2 > 200);
    case 8: return (Ps3.data.analog.button.r2 > 200);
    case 9: return Ps3.data.button.up;
    case 10: return Ps3.data.button.down;
    case 11: return Ps3.data.button.left;
    case 12: return Ps3.data.button.right;
    case 13: return Ps3.data.button.r3;
    default: return false;
  }
}

void handleServoToggle() {
  if (!Ps3.isConnected() || (oledMode != OLED_INFO && oledMode != OLED_INFO_PWM)) {
    return;
  }

  unsigned long now = millis();

  // ========== SERVO 1 - LOGIKA KHUSUS ==========
  if (servoButtonMap[0].buttonType1 == 6) {  // R1 button
    bool currentButtonState = Ps3.data.button.r1;

    if (currentButtonState && !servoButtonMap[0].lastButtonState1) {
      if (now - servoButtonMap[0].lastToggleTime1 >= SERVO_DEBOUNCE) {
        if (servoButtonMap[1].state1 || servoButtonMap[1].state2) {
          servoButtonMap[0].state1 = !servoButtonMap[0].state1;
          if (servoButtonMap[0].state1) {
            setServoAngleStrong(0, servoOpen[0]);
            Serial.printf("Servo 1: OPEN (sudut 1) - Servo 2 terbuka\n");
          } else {
            setServoAngleStrong(0, servoOpen2[0]);
            Serial.printf("Servo 1: CLOSE (menggunakan sudut open2)\n");
          }
          servoButtonMap[0].state2 = false;
        } else {
          servoButtonMap[0].state2 = !servoButtonMap[0].state2;
          if (servoButtonMap[0].state2) {
            setServoAngleStrong(0, servoOpen2[0]);
            Serial.printf("Servo 1: OPEN2 (sudut 2) - Servo 2 tertutup\n");
          } else {
            setServoAngleStrong(0, servoClose[0]);
            Serial.printf("Servo 1: CLOSE (normal)\n");
          }
          servoButtonMap[0].state1 = false;
        }
        servoButtonMap[0].lastToggleTime1 = now;
      }
    }
    servoButtonMap[0].lastButtonState1 = currentButtonState;
  }

  // ========== SERVO 2 - LOGIKA DENGAN AUTO TRIGGER SERVO 1 ==========
  if (servoButtonMap[1].buttonType1 != 0) {
    bool currentButtonState1 = false;

    if (servoButtonMap[1].buttonType1 == 7) {
      currentButtonState1 = (Ps3.data.analog.button.l2 > 250);
    } else if (servoButtonMap[1].buttonType1 == 8) {
      currentButtonState1 = (Ps3.data.analog.button.r2 > 250);
    } else {
      currentButtonState1 = getButtonState(servoButtonMap[1].buttonType1);
    }

    if (currentButtonState1 && !servoButtonMap[1].lastButtonState1) {
      if (now - servoButtonMap[1].lastToggleTime1 >= SERVO_DEBOUNCE) {
        bool wasOpen = servoButtonMap[1].state1;
        servoButtonMap[1].state1 = !servoButtonMap[1].state1;

        if (servoButtonMap[1].state1) {
          setServoAngleStrong(1, servoOpen[1]);
          Serial.printf("Servo 2: OPEN (Btn1)\n");

          if (!wasOpen) {
            setServoAngleStrong(0, servoOpen2[0]);
            servoButtonMap[0].state2 = false;
            servoButtonMap[0].state1 = false;
            Serial.printf("🔄 AUTO:   Servo 1 -> CLOSE (menggunakan sudut open2 karena Servo 2 dibuka)\n");
          }
        } else {
          setServoAngleStrong(1, servoClose[1]);
          Serial.printf("Servo 2: CLOSE (Btn1)\n");

          if (wasOpen) {
            setServoAngleStrong(0, servoClose[0]);
            servoButtonMap[0].state1 = false;
            servoButtonMap[0].state2 = false;
            Serial.printf("🔄 AUTO:  Servo 1 -> CLOSE (normal karena Servo 2 ditutup)\n");
          }
        }
        servoButtonMap[1].lastToggleTime1 = now;
        servoButtonMap[1].state2 = false;
      }
    }
    servoButtonMap[1].lastButtonState1 = currentButtonState1;
  }

  if (servoButtonMap[1].buttonType2 != 0) {
    bool currentButtonState2 = false;

    if (servoButtonMap[1].buttonType2 == 7) {
      currentButtonState2 = (Ps3.data.analog.button.l2 > 250);
    } else if (servoButtonMap[1].buttonType2 == 8) {
      currentButtonState2 = (Ps3.data.analog.button.r2 > 250);
    } else {
      currentButtonState2 = getButtonState(servoButtonMap[1].buttonType2);
    }

    if (currentButtonState2 && !servoButtonMap[1].lastButtonState2) {
      if (now - servoButtonMap[1].lastToggleTime2 >= SERVO_DEBOUNCE) {
        bool wasOpen = servoButtonMap[1].state2;
        servoButtonMap[1].state2 = !servoButtonMap[1].state2;

        if (servoButtonMap[1].state2) {
          setServoAngleStrong(1, servoOpen2[1]);
          Serial.printf("Servo 2: OPEN2 (Btn2)\n");

          if (!wasOpen) {
            setServoAngleStrong(0, servoOpen2[0]);
            servoButtonMap[0].state2 = false;
            servoButtonMap[0].state1 = false;
            Serial.printf("🔄 AUTO:  Servo 1 -> CLOSE (menggunakan sudut open2 karena Servo 2 dibuka)\n");
          }
        } else {
          setServoAngleStrong(1, servoClose[1]);
          Serial.printf("Servo 2: CLOSE (Btn2)\n");

          if (wasOpen) {
            setServoAngleStrong(0, servoClose[0]);
            servoButtonMap[0].state1 = false;
            servoButtonMap[0].state2 = false;
            Serial.printf("🔄 AUTO:  Servo 1 -> CLOSE (normal karena Servo 2 ditutup)\n");
          }
        }
        servoButtonMap[1].lastToggleTime2 = now;
        servoButtonMap[1].state1 = false;
      }
    }
    servoButtonMap[1].lastButtonState2 = currentButtonState2;
  }

  // ========== SERVO 3, 4 - LOGIKA NORMAL ==========
  for (int i = 2; i < 4; i++) {
    if (servoButtonMap[i].buttonType1 != 0) {
      bool currentButtonState1 = false;

      if (servoButtonMap[i].buttonType1 == 7) {
        currentButtonState1 = (Ps3.data.analog.button.l2 > 250);
      } else if (servoButtonMap[i].buttonType1 == 8) {
        currentButtonState1 = (Ps3.data.analog.button.r2 > 250);
      } else {
        currentButtonState1 = getButtonState(servoButtonMap[i].buttonType1);
      }

      if (currentButtonState1 && !servoButtonMap[i].lastButtonState1) {
        if (now - servoButtonMap[i].lastToggleTime1 >= SERVO_DEBOUNCE) {
          servoButtonMap[i].state1 = !servoButtonMap[i].state1;
          if (servoButtonMap[i].state1) {
            setServoAngleStrong(i, servoOpen[i]);
            Serial.printf("Servo %d: OPEN (Btn1)\n", i + 1);
          } else {
            setServoAngleStrong(i, servoClose[i]);
            Serial.printf("Servo %d: CLOSE (Btn1)\n", i + 1);
          }
          servoButtonMap[i].lastToggleTime1 = now;
          servoButtonMap[i].state2 = false;
        }
      }
      servoButtonMap[i].lastButtonState1 = currentButtonState1;
    }

    if (servoButtonMap[i].buttonType2 != 0) {
      bool currentButtonState2 = false;

      if (servoButtonMap[i].buttonType2 == 7) {
        currentButtonState2 = (Ps3.data.analog.button.l2 > 250);
      } else if (servoButtonMap[i].buttonType2 == 8) {
        currentButtonState2 = (Ps3.data.analog.button.r2 > 250);
      } else {
        currentButtonState2 = getButtonState(servoButtonMap[i].buttonType2);
      }

      if (currentButtonState2 && !servoButtonMap[i].lastButtonState2) {
        if (now - servoButtonMap[i].lastToggleTime2 >= SERVO_DEBOUNCE) {
          servoButtonMap[i].state2 = !servoButtonMap[i].state2;
          if (servoButtonMap[i].state2) {
            setServoAngleStrong(i, servoOpen2[i]);
            Serial.printf("Servo %d:   OPEN2 (Btn2)\n", i + 1);
          } else {
            setServoAngleStrong(i, servoClose[i]);
            Serial.printf("Servo %d:  CLOSE (Btn2)\n", i + 1);
          }
          servoButtonMap[i].lastToggleTime2 = now;
          servoButtonMap[i].state1 = false;
        }
      }
      servoButtonMap[i].lastButtonState2 = currentButtonState2;
    }
  }
}

void onConnect() {
  Serial.println("Ps3 Controller Connected!");
  isConnected = true;
  if (oledMode == OLED_INFO) {
    drawInfo();
  }
}

void onDisconnect() {
  Serial.println("Ps3 Controller Disconnected!");
  isConnected = false;
  stopMotor();
  kontrolMotorWithEncoder(0);
  positionControlActive = false;
}

void processGamepad() {
  if (!Ps3.isConnected()) return;

  if (Ps3.data.button.ps && !lastPs) {
    display.clearDisplay();
    display.drawBitmap(44, 10, _restart_, 41, 44, 1);
    display.display();
    delay(500);
    ESP.restart();
  }
  lastPs = Ps3.data.button.ps;

  lx = Ps3.data.analog.stick.lx;
  ly = Ps3.data.analog.stick.ly;
  rx = Ps3.data.analog.stick.rx;
  l2Value = Ps3.data.analog.button.l2;
  r2Value = Ps3.data.analog.button.r2;

  ly = (ly * 250) / 128;
  lx = (lx * 250) / 128;
  rx = (rx * 250) / 128;

  ly = constrain(ly, -250, 250);
  lx = constrain(lx, -250, 250);
  rx = constrain(rx, -250, 250);

  if (abs(lx) < 25) lx = 0;
  if (abs(ly) < 25) ly = 0;
  if (abs(rx) < 25) rx = 0;

  const int THRESHOLD = 150;
  if (abs(ly) > THRESHOLD && abs(lx) < THRESHOLD) {
    lx = 0;
  } else if (abs(lx) > THRESHOLD && abs(ly) < THRESHOLD) {
    ly = 0;
  } else if (abs(ly) < 20 && abs(lx) < 20) {
    ly = 0;
    lx = 0;
  }

  // ✅ KONTROL MOTOR 5 dengan Priority System yang disederhanakan
  if (oledMode == OLED_INFO || oledMode == OLED_INFO_PWM || oledMode == OLED_MENU_POSITION_CONTROL) {

    int l2Mapped = map(l2Value, 0, 255, 0, 1023);
    int r2Mapped = map(r2Value, 0, 255, 0, 1023);

    if (l2Mapped < 100) l2Mapped = 0;
    if (r2Mapped < 100) r2Mapped = 0;

    // Priority: Position Control > Manual Control > Stop
    if (positionControlActive) {
      // Position control aktif - motor dikontrol oleh updatePositionControl()
    } else if (l2Mapped > 100 || r2Mapped > 100) {
      // Manual control
      if (l2Mapped > 100) {
        kontrolMotorWithEncoder(l2Mapped);
      } else if (r2Mapped > 100) {
        kontrolMotorWithEncoder(-r2Mapped);
      }
    } else {
      // Stop motor jika tidak ada input
      kontrolMotorWithEncoder(0);
    }

  } else {
    // Di menu setting - stop semua motor
    kontrolMotorWithEncoder(0);
  }

  // Kinematik untuk motor 1-4
  if (oledMode == OLED_INFO || oledMode == OLED_INFO_PWM) {

    kinematik(-lx, -ly, rx);
  } else {
    stopMotor();
  }

  // Serial.printf("ly:%d lx:%d rx:%d\n",ly,lx,rx);
}

void kinematik(int x, int y, int th) {
  m1 = lambda * (cos(d2r(45)) * x + sin(d2r(45)) * y + -LengthAlpha * th);
  m2 = lambda * (cos(d2r(-45)) * x + sin(d2r(-45)) * y + -LengthAlpha * th);
  m3 = lambda * (cos(d2r(-135)) * x + sin(d2r(-135)) * y + -LengthAlpha * th);
  m4 = lambda * (cos(d2r(135)) * x + sin(d2r(135)) * y + -LengthAlpha * th);

  m1 = constrain(m1, -1023, 1023);
  m2 = constrain(m2, -1023, 1023);
  m3 = constrain(m3, -1023, 1023);
  m4 = constrain(m4, -1023, 1023);

  int pwm[4] = { m1, m2, m3, m4 };
  // ✅ UBAH:  Gunakan throttle jika enabled
  if (throttleEnabled) {
    // Set sebagai target PWM untuk throttle system
    portENTER_CRITICAL(&timerMux);
    targetPwmMotor[0] = m1;
    targetPwmMotor[1] = m2;
    targetPwmMotor[2] = m3;
    targetPwmMotor[3] = m4;
    portEXIT_CRITICAL(&timerMux);
  } else {
    // Langsung apply tanpa throttle
    setPwm(pwm);
  }
}

// ============ DRAW FUNCTIONS ============

void drawInfo() {
  // countTimeOutOled=0;
  if (oledTimeOut) {
    display.oled_command(SH110X_DISPLAYOFF);
  } else {
    display.oled_command(SH110X_DISPLAYON);
  }
  display.clearDisplay();
  // display.setContrast(5);
  if (isConnected) {
    display.drawBitmap(112, 5, image_GameMode_bits, 11, 8, 1);
  } else {
    display.drawBitmap(110, 2, image_Layer_5_bits, 14, 15, 1);
  }
  display.drawBitmap(69, 3, image_Layer_34_bits, 55, 61, 1);
  display.drawBitmap(3, 12, image_Layer_7_bits, 65, 41, 1);

  long currentPos = getCurrentPosition();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(3, 55);
  display.printf("P:%ld M%d", currentPos, currentPositionMode + 1);

  if (positionControlActive) {
    display.setCursor(3, 2);
    display.printf("SetPoint-> %ld", targetPosition);
  }

  display.display();
}

void drawPwmMotor() {
  display.oled_command(SH110X_DISPLAYON);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(7, 6);
  display.print("MOTOR 1:");
  display.setCursor(7, 20);
  display.print("MOTOR 2:");
  display.setCursor(7, 34);
  display.print("MOTOR 3:");
  display.setCursor(7, 48);
  display.print("MOTOR 4:");

  display.setCursor(56, 6);
  display.print(m1);
  display.setCursor(56, 20);
  display.print(m2);
  display.setCursor(56, 34);
  display.print(m3);
  display.setCursor(56, 48);
  display.print(m4);

  long currentPos = getCurrentPosition();
  display.setCursor(80, 6);
  display.printf("P:%ld", currentPos);
  display.setCursor(80, 20);
  display.printf("M%d", currentPositionMode + 1);

  display.display();
}

void updateDisplayRealTime() {
  static unsigned long lastDisplayUpdate = 0;

  if (millis() - lastDisplayUpdate >= 100) {
    if (oledMode == OLED_INFO) {
      drawInfo();
    } else if (oledMode == OLED_INFO_PWM) {
      drawPwmMotor();
    }
    lastDisplayUpdate = millis();
  }
}

// void drawSettingServoMenu() {
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(WHITE);

//   if (servoMenuState == MENU_SERVO_LIST) {
//     display.setCursor(0, 0);
//     display.println("Setting Servo:");
//     for (int i = 0; i < 4; i++) {
//       display.setCursor(2, 18 + i * 12);
//       if (selectedSetting == i) display.print("> ");
//       else display.print("  ");
//       display.print("Servo ");
//       display.print(i + 1);
//     }
//   } else if (servoMenuState == MENU_SERVO_OCC) {
//     char buf[24];
//     sprintf(buf, "Servo %d Setting:", editServoIndex + 1);
//     display.setCursor(0, 0);
//     display.println(buf);

//     display.setTextSize(1);
//     display.setCursor(5, 16);
//     if (editOCCSelected == 0) display.print("> ");
//     else display.print("  ");
//     display.print("Open:");
//     display.setCursor(50, 16);
//     display.print(servoOpen[editServoIndex]);

//     display.setCursor(5, 30);
//     if (editOCCSelected == 1) display.print("> ");
//     else display.print("  ");
//     display.print("Open2:");
//     display.setCursor(56, 30);
//     display.print(servoOpen2[editServoIndex]);

//     display.setCursor(5, 44);
//     if (editOCCSelected == 2) display.print("> ");
//     else display.print("  ");
//     display.print("Close:");
//     display.setCursor(55, 44);
//     display.print(servoClose[editServoIndex]);
//   }

//   display.display();
// }

void drawThrottleSettingMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(17, 0);
  display.println("THROTTLE SETTING");
  display.drawLine(0, 10, 127, 10, 1);

  // PWM Step
  if (throttleMenuSelected == 0) {
    display.fillRect(5, 16, 118, 10, WHITE);
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(7, 17);
  display.print("PWM Step:");
  display.setCursor(75, 17);
  display.print(throttlePwmStep);
  display.setTextColor(WHITE);

  // Interval (ms)
  if (throttleMenuSelected == 1) {
    display.fillRect(5, 28, 118, 10, WHITE);
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(7, 29);
  display.print("Interval:");
  display.setCursor(75, 29);
  display.print(throttleIntervalUs / 1000);
  display.print(" ms");
  display.setTextColor(WHITE);

  // Threshold
  if (throttleMenuSelected == 2) {
    display.fillRect(5, 40, 118, 10, WHITE);
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(7, 41);
  display.print("Threshold:");
  display.setCursor(75, 41);
  display.print(throttleActivationThreshold);
  display.setTextColor(WHITE);

  // Enable/Disable
  if (throttleMenuSelected == 3) {
    display.fillRect(5, 52, 118, 10, WHITE);
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(7, 53);
  display.print("Throttle:");
  display.setCursor(75, 53);
  display.print(throttleEnabled ? "ON" : "OFF");
  display.setTextColor(WHITE);

  display.display();
}

void drawSettingServoMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if (servoMenuState == MENU_SERVO_LIST) {
    display.setCursor(26, 0);
    display.println("SETTING SERVO");
    display.drawLine(0, 10, 127, 10, 1);
    for (int i = 0; i < 4; i++) {
      int yPos = 18 + i * 12;

      if (selectedSetting == i) {
        // ✅ Inverse:  background putih, text hitam
        display.fillRect(2, yPos, 124, 10, WHITE);
        display.setTextColor(BLACK, WHITE);
        display.setCursor(5, yPos + 1);
      } else {
        display.setTextColor(WHITE);
        display.setCursor(5, yPos + 1);
      }

      display.print("SERVO ");
      display.print(i + 1);
      display.setTextColor(WHITE);  // Reset ke normal
    }
  } else if (servoMenuState == MENU_SERVO_OCC) {
    char buf[24];
    sprintf(buf, "SERVO %d SETTING:", editServoIndex + 1);
    display.setCursor(0, 0);
    display.println(buf);

    display.setTextSize(1);

    // Open
    if (editOCCSelected == 0) {
      display.fillRect(5, 16, 118, 10, WHITE);
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE);
    }
    display.setCursor(7, 17);
    display.print("OPEN:");
    display.setCursor(50, 17);
    display.print(servoOpen[editServoIndex]);
    display.setTextColor(WHITE);

    // Open2
    if (editOCCSelected == 1) {
      display.fillRect(5, 30, 118, 10, WHITE);
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE);
    }
    display.setCursor(7, 31);
    display.print("OPEN2:");
    display.setCursor(56, 31);
    display.print(servoOpen2[editServoIndex]);
    display.setTextColor(WHITE);

    // Close
    if (editOCCSelected == 2) {
      display.fillRect(5, 44, 118, 10, WHITE);
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE);
    }
    display.setCursor(7, 45);
    display.print("CLOSE:");
    display.setCursor(55, 45);
    display.print(servoClose[editServoIndex]);
    display.setTextColor(WHITE);
  }

  display.display();
}
// void drawSettingLambdaLengthAlphaMenu() {
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(WHITE);
//   display.setCursor(0, 0);
//   display.println("Setting Parameters:");

//   display.setCursor(10, 20);
//   if (lambdaMenuSelected == 0) display.print("> ");
//   else display.print("  ");
//   display.print("Lambda:");
//   display.setTextSize(2);
//   display.setCursor(70, 16);
//   display.print(lambda, 2);

//   display.setTextSize(1);
//   display.setCursor(10, 44);
//   if (lambdaMenuSelected == 1) display.print("> ");
//   else display.print("  ");
//   display.print("Rotasi:");
//   display.setTextSize(2);
//   display.setCursor(70, 40);
//   display.print(LengthAlpha, 2);

//   display.display();
// }
void drawSettingLambdaLengthAlphaMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(14, 0);
  display.println("SETTING PARAMETER");

  display.drawLine(0, 10, 127, 10, 1);


  // Lambda
  if (lambdaMenuSelected == 0) {
    display.fillRect(10, 20, 108, 20, WHITE);
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(12, 21);
  display.print("Lambda:");
  display.setTextSize(2);
  display.setCursor(70, 20);
  display.print(lambda, 2);
  display.setTextColor(WHITE);

  // LengthAlpha
  display.setTextSize(1);
  if (lambdaMenuSelected == 1) {
    display.fillRect(10, 44, 108, 20, WHITE);
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(12, 45);
  display.print("Rotasi:");
  display.setTextSize(2);
  display.setCursor(70, 44);
  display.print(LengthAlpha, 2);
  display.setTextColor(WHITE);

  display.display();
}
// void drawButtonMappingMenu() {
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(WHITE);
//   display.setCursor(0, 0);
//   display.println("Button Mapping:");

//   if (waitingForButtonPress) {
//     display.drawBitmap(10, 16, _PRESSBUTTON, 111, 32, 1);
//   } else {
//     int servoIdx = selectedServoForMapping;

//     display.setCursor(2, 18);
//     display.printf("> Servo %d:", servoIdx + 1);

//     display.setCursor(10, 30);
//     if (selectedButtonForMapping == 0) display.print("> ");
//     else display.print("  ");
//     display.print("Btn1:  ");
//     String btn1Name = getButtonName(servoButtonMap[servoIdx].buttonType1);
//     display.print(btn1Name);
//     if (servoButtonMap[servoIdx]. buttonType1 != 0) {
//       display.print(servoButtonMap[servoIdx]. state1 ? " [O]" : " [C]");
//     }

//     display.setCursor(10, 42);
//     if (selectedButtonForMapping == 1) display.print("> ");
//     else display.print("  ");
//     display.print("Btn2: ");
//     String btn2Name = getButtonName(servoButtonMap[servoIdx].buttonType2);
//     display.print(btn2Name);
//     if (servoButtonMap[servoIdx]. buttonType2 != 0) {
//       display.print(servoButtonMap[servoIdx]. state2 ? " [O2]" : " [C]");
//     }
//   }

//   display.display();
// }

void drawButtonMappingMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(5, 0);
  display.println("BUTTON SERVO MAPPING");
  display.drawLine(0, 10, 127, 10, 1);
  if (waitingForButtonPress) {
    display.drawBitmap(10, 16, _PRESSBUTTON, 111, 32, 1);
  } else {
    int servoIdx = selectedServoForMapping;

    display.setCursor(2, 18);
    display.printf("* SERVO %d:", servoIdx + 1);

    // Button 1
    if (selectedButtonForMapping == 0) {
      display.fillRect(10, 30, 110, 10, WHITE);
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE);
    }
    display.setCursor(12, 31);
    display.print("BUTTON1: ");
    String btn1Name = getButtonName(servoButtonMap[servoIdx].buttonType1);
    display.print(btn1Name);
    if (servoButtonMap[servoIdx].buttonType1 != 0) {
      display.print(servoButtonMap[servoIdx].state1 ? " [O]" : " [C]");
    }
    display.setTextColor(WHITE);

    // Button 2
    if (selectedButtonForMapping == 1) {
      display.fillRect(10, 42, 110, 10, WHITE);
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE);
    }
    display.setCursor(12, 43);
    display.print("BUTTON2: ");
    String btn2Name = getButtonName(servoButtonMap[servoIdx].buttonType2);
    display.print(btn2Name);
    if (servoButtonMap[servoIdx].buttonType2 != 0) {
      display.print(servoButtonMap[servoIdx].state2 ? " [O2]" : " [C]");
    }
    display.setTextColor(WHITE);
  }

  display.display();
}

void handleThrottleSettingMenuNavigation() {
  static unsigned long lastInput = 0;
  static unsigned long lastRepeat = 0;
  unsigned long now = millis();

  bool upPressed = Ps3.data.button.up && !lastUp;
  bool downPressed = Ps3.data.button.down && !lastDown;
  bool leftPressed = Ps3.data.button.left && !lastLeft;
  bool rightPressed = Ps3.data.button.right && !lastRight;
  bool crossPressed = Ps3.data.button.cross && !lastCross;
  bool circlePressed = Ps3.data.button.circle && !lastCircle;

  lastUp = Ps3.data.button.up;
  lastDown = Ps3.data.button.down;
  lastLeft = Ps3.data.button.left;
  lastRight = Ps3.data.button.right;
  lastCross = Ps3.data.button.cross;
  lastCircle = Ps3.data.button.circle;

  // Navigasi UP/DOWN
  if (upPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
    throttleMenuSelected = (throttleMenuSelected - 1 + 4) % 4;
    Serial.printf("▲ Selected: %d\n", throttleMenuSelected);
    drawThrottleSettingMenu();
    lastInput = now;
    return;
  }

  if (downPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
    throttleMenuSelected = (throttleMenuSelected + 1) % 4;
    Serial.printf("▼ Selected: %d\n", throttleMenuSelected);
    drawThrottleSettingMenu();
    lastInput = now;
    return;
  }

  // Save dengan CROSS
  // Save dengan CROSS
  if (crossPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
    Serial.println("💾 Saving throttle settings...");

    // ✅ UBAH:  Save sebagai uint8_t untuk konsistensi
    uint8_t enabledByte = throttleEnabled ? 1 : 0;
    EEPROM.put(ADDR_THROTTLE_ENABLED, enabledByte);

    EEPROM.put(ADDR_THROTTLE_PWM_STEP, throttlePwmStep);
    EEPROM.put(ADDR_THROTTLE_INTERVAL, throttleIntervalUs);
    EEPROM.put(ADDR_THROTTLE_THRESHOLD, throttleActivationThreshold);
    EEPROM.commit();

    Serial.printf("✅ Saved:   Enabled=%d, Step=%d, Interval=%d, Threshold=%d\n",
                  enabledByte, throttlePwmStep, throttleIntervalUs, throttleActivationThreshold);

    if (throttleTimer) {
      timerAlarm(throttleTimer, throttleIntervalUs, true, 0);
    }

    display.clearDisplay();
    display.drawBitmap(48, 8, _saved, 32, 49, 1);
    display.display();
    delay(500);

    oledMode = OLED_INFO;
    drawInfo();
    lastInput = now;
    return;
  }

  // Exit dengan CIRCLE
  if (circlePressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
    Serial.println("❌ Cancel throttle settings");
    oledMode = OLED_INFO;
    drawInfo();
    lastInput = now;
    return;
  }

  // ✅ THROTTLE ON/OFF TOGGLE - DENGAN DEBUG LENGKAP
  if (throttleMenuSelected == 3) {
    if (leftPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      // ✅ TAMBAH:  Print state SEBELUM toggle
      Serial.printf("🔵 BEFORE toggle: throttleEnabled = %d (address: 0x%p)\n",
                    throttleEnabled, &throttleEnabled);

      // Toggle dengan explicit assignment
      if (throttleEnabled == true) {
        throttleEnabled = false;
      } else {
        throttleEnabled = true;
      }

      // ✅ TAMBAH:  Print state SETELAH toggle
      Serial.printf("🟢 AFTER toggle: throttleEnabled = %d\n", throttleEnabled);
      Serial.printf("◄ Toggle Result: %s\n", throttleEnabled ? "ON" : "OFF");

      drawThrottleSettingMenu();
      lastInput = now;
      return;
    }

    if (rightPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      Serial.printf("🔵 BEFORE toggle: throttleEnabled = %d (address: 0x%p)\n",
                    throttleEnabled, &throttleEnabled);

      if (throttleEnabled == true) {
        throttleEnabled = false;
      } else {
        throttleEnabled = true;
      }

      Serial.printf("🟢 AFTER toggle: throttleEnabled = %d\n", throttleEnabled);
      Serial.printf("► Toggle Result: %s\n", throttleEnabled ? "ON" : "OFF");

      drawThrottleSettingMenu();
      lastInput = now;
      return;
    }
  } else {
    // NUMERIC PARAMETERS
    if (now - lastRepeat >= AUTO_REPEAT_DELAY) {
      bool changed = false;

      if (Ps3.data.button.left) {
        if (throttleMenuSelected == 0) {
          throttlePwmStep -= 5;
          if (throttlePwmStep < 1) throttlePwmStep = 1;
          Serial.printf("◄ PWM Step:  %d\n", throttlePwmStep);
        } else if (throttleMenuSelected == 1) {
          throttleIntervalUs -= 5000;
          if (throttleIntervalUs < 5000) throttleIntervalUs = 5000;
          Serial.printf("◄ Interval: %dms\n", throttleIntervalUs / 1000);
        } else if (throttleMenuSelected == 2) {
          throttleActivationThreshold -= 10;
          if (throttleActivationThreshold < 0) throttleActivationThreshold = 0;
          Serial.printf("◄ Threshold: %d\n", throttleActivationThreshold);
        }
        changed = true;
      } else if (Ps3.data.button.right) {
        if (throttleMenuSelected == 0) {
          throttlePwmStep += 5;
          if (throttlePwmStep > 500) throttlePwmStep = 500;
          Serial.printf("► PWM Step: %d\n", throttlePwmStep);
        } else if (throttleMenuSelected == 1) {
          throttleIntervalUs += 5000;
          if (throttleIntervalUs > 1000000) throttleIntervalUs = 1000000;
          Serial.printf("► Interval: %dms\n", throttleIntervalUs / 1000);
        } else if (throttleMenuSelected == 2) {
          throttleActivationThreshold += 10;
          if (throttleActivationThreshold > 500) throttleActivationThreshold = 500;
          Serial.printf("► Threshold: %d\n", throttleActivationThreshold);
        }
        changed = true;
      }

      if (changed) {
        drawThrottleSettingMenu();
        lastRepeat = now;
      }
    }
  }
}

void handleSettingServoMenuNavigation() {
  static unsigned long lastInput = 0;
  static unsigned long lastRepeat = 0;
  unsigned long now = millis();

  if (servoMenuState == MENU_SERVO_LIST) {
    bool upPressed = Ps3.data.button.up && !lastUp;
    bool downPressed = Ps3.data.button.down && !lastDown;
    bool crossPressed = Ps3.data.button.cross && !lastCross;
    bool circlePressed = Ps3.data.button.circle && !lastCircle;

    lastUp = Ps3.data.button.up;
    lastDown = Ps3.data.button.down;
    lastCross = Ps3.data.button.cross;
    lastCircle = Ps3.data.button.circle;

    if (upPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      selectedSetting = (selectedSetting - 1 + 4) % 4;
      drawSettingServoMenu();
      lastInput = now;
    } else if (downPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      selectedSetting = (selectedSetting + 1) % 4;
      drawSettingServoMenu();
      lastInput = now;
    } else if (crossPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      editServoIndex = selectedSetting;
      editOCCSelected = 0;
      servoMenuState = MENU_SERVO_OCC;
      drawSettingServoMenu();
      lastInput = now;
    } else if (circlePressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      oledMode = OLED_INFO;
      drawInfo();
      lastInput = now;
    }
  } else if (servoMenuState == MENU_SERVO_OCC) {
    bool upPressed = Ps3.data.button.up && !lastUp;
    bool downPressed = Ps3.data.button.down && !lastDown;
    bool crossPressed = Ps3.data.button.cross && !lastCross;
    bool circlePressed = Ps3.data.button.circle && !lastCircle;

    lastUp = Ps3.data.button.up;
    lastDown = Ps3.data.button.down;
    lastCross = Ps3.data.button.cross;
    lastCircle = Ps3.data.button.circle;

    if (upPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      editOCCSelected = (editOCCSelected - 1 + 3) % 3;
      drawSettingServoMenu();
      lastInput = now;
    } else if (downPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      editOCCSelected = (editOCCSelected + 1) % 3;
      drawSettingServoMenu();
      lastInput = now;
    } else if (crossPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      EEPROM.put(ADDR_SERVO_OPEN + editServoIndex * sizeof(int), servoOpen[editServoIndex]);
      EEPROM.put(ADDR_SERVO_OPEN2 + editServoIndex * sizeof(int), servoOpen2[editServoIndex]);
      EEPROM.put(ADDR_SERVO_CLOSE + editServoIndex * sizeof(int), servoClose[editServoIndex]);
      EEPROM.commit();

      display.clearDisplay();
      display.drawBitmap(48, 8, _saved, 32, 49, 1);
      display.display();
      delay(500);

      servoMenuState = MENU_SERVO_LIST;
      drawSettingServoMenu();
      lastInput = now;
    } else if (circlePressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
      servoMenuState = MENU_SERVO_LIST;
      drawSettingServoMenu();
      lastInput = now;
    }

    if (Ps3.data.button.left && (now - lastRepeat >= AUTO_REPEAT_DELAY)) {
      if (editOCCSelected == 0) {
        servoOpen[editServoIndex]--;
        if (servoOpen[editServoIndex] < 0) servoOpen[editServoIndex] = 0;
        setServoAngleStrong(editServoIndex, servoOpen[editServoIndex]);
      } else if (editOCCSelected == 1) {
        servoOpen2[editServoIndex]--;
        if (servoOpen2[editServoIndex] < 0) servoOpen2[editServoIndex] = 0;
        setServoAngleStrong(editServoIndex, servoOpen2[editServoIndex]);
      } else {
        servoClose[editServoIndex]--;
        if (servoClose[editServoIndex] < 0) servoClose[editServoIndex] = 0;
        setServoAngleStrong(editServoIndex, servoClose[editServoIndex]);
      }
      drawSettingServoMenu();
      lastRepeat = now;
    } else if (Ps3.data.button.right && (now - lastRepeat >= AUTO_REPEAT_DELAY)) {
      if (editOCCSelected == 0) {
        servoOpen[editServoIndex]++;
        if (servoOpen[editServoIndex] > 180) servoOpen[editServoIndex] = 180;
        setServoAngleStrong(editServoIndex, servoOpen[editServoIndex]);
      } else if (editOCCSelected == 1) {
        servoOpen2[editServoIndex]++;
        if (servoOpen2[editServoIndex] > 180) servoOpen2[editServoIndex] = 180;
        setServoAngleStrong(editServoIndex, servoOpen2[editServoIndex]);
      } else {
        servoClose[editServoIndex]++;
        if (servoClose[editServoIndex] > 180) servoClose[editServoIndex] = 180;
        setServoAngleStrong(editServoIndex, servoClose[editServoIndex]);
      }
      drawSettingServoMenu();
      lastRepeat = now;
    }
  }
}

void handleSettingLambdaLengthAlphaMenuNavigation() {
  static unsigned long lastInput = 0;
  static unsigned long lastRepeat = 0;
  unsigned long now = millis();

  bool upPressed = Ps3.data.button.up && !lastUp;
  bool downPressed = Ps3.data.button.down && !lastDown;
  bool crossPressed = Ps3.data.button.cross && !lastCross;
  bool circlePressed = Ps3.data.button.circle && !lastCircle;

  lastUp = Ps3.data.button.up;
  lastDown = Ps3.data.button.down;
  lastCross = Ps3.data.button.cross;
  lastCircle = Ps3.data.button.circle;

  if ((upPressed || downPressed) && (now - lastInput >= BUTTON_DEBOUNCE)) {
    lambdaMenuSelected = 1 - lambdaMenuSelected;
    drawSettingLambdaLengthAlphaMenu();
    lastInput = now;
  } else if (crossPressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
    EEPROM.put(ADDR_LAMBDA, lambda);
    EEPROM.put(ADDR_LENGTH_ALPHA, LengthAlpha);
    EEPROM.commit();

    display.clearDisplay();
    display.drawBitmap(48, 8, _saved, 32, 49, 1);
    display.display();
    delay(500);

    oledMode = OLED_INFO;
    drawInfo();
    lastInput = now;
  } else if (circlePressed && (now - lastInput >= BUTTON_DEBOUNCE)) {
    oledMode = OLED_INFO;
    drawInfo();
    lastInput = now;
  }

  if (Ps3.data.button.left && (now - lastRepeat >= AUTO_REPEAT_DELAY)) {
    if (lambdaMenuSelected == 0) {
      lambda -= stepLambda;
      if (lambda < minLambda) lambda = minLambda;
      lambda = ((int)(lambda * 100 + 0.5f)) / 100.0f;
    } else {
      LengthAlpha -= stepLengthAlpha;
      if (LengthAlpha < minLengthAlpha) LengthAlpha = minLengthAlpha;
      LengthAlpha = ((int)(LengthAlpha * 100 + 0.5f)) / 100.0f;
    }
    drawSettingLambdaLengthAlphaMenu();
    lastRepeat = now;
  } else if (Ps3.data.button.right && (now - lastRepeat >= AUTO_REPEAT_DELAY)) {
    if (lambdaMenuSelected == 0) {
      lambda += stepLambda;
      if (lambda > maxLambda) lambda = maxLambda;
      lambda = ((int)(lambda * 100 + 0.5f)) / 100.0f;
    } else {
      LengthAlpha += stepLengthAlpha;
      if (LengthAlpha > maxLengthAlpha) LengthAlpha = maxLengthAlpha;
      LengthAlpha = ((int)(LengthAlpha * 100 + 0.5f)) / 100.0f;
    }
    drawSettingLambdaLengthAlphaMenu();
    lastRepeat = now;
  }
}

void handleButtonMappingMenuNavigation() {
  static unsigned long lastInput = 0;
  unsigned long now = millis();

  if (waitingForButtonPress) {
    uint8_t detectedButton = 0;

    if (Ps3.data.button.l1 && !lastL1) detectedButton = 5;
    else if (Ps3.data.button.r1 && !lastR1) detectedButton = 6;
    else if (Ps3.data.button.l2 && !lastL2) detectedButton = 7;
    else if (Ps3.data.button.r2 && !lastR2) detectedButton = 8;
    else if (Ps3.data.button.cross && !lastCross) detectedButton = 1;
    else if (Ps3.data.button.circle && !lastCircle) detectedButton = 2;
    else if (Ps3.data.button.square && !lastSquare) detectedButton = 3;
    else if (Ps3.data.button.triangle && !lastTriangle) detectedButton = 4;
    else if (Ps3.data.button.up && !lastUp) detectedButton = 9;
    else if (Ps3.data.button.down && !lastDown) detectedButton = 10;
    else if (Ps3.data.button.left && !lastLeft) detectedButton = 11;
    else if (Ps3.data.button.right && !lastRight) detectedButton = 12;
    else if (Ps3.data.button.r3 && !lastL3) detectedButton = 13;

    lastL1 = Ps3.data.button.l1;
    lastR1 = Ps3.data.button.r1;
    lastL2 = Ps3.data.button.l2;
    lastR2 = Ps3.data.button.r2;
    lastCross = Ps3.data.button.cross;
    lastCircle = Ps3.data.button.circle;
    lastSquare = Ps3.data.button.square;
    lastTriangle = Ps3.data.button.triangle;
    lastUp = Ps3.data.button.up;
    lastDown = Ps3.data.button.down;
    lastLeft = Ps3.data.button.left;
    lastRight = Ps3.data.button.right;
    lastL3 = Ps3.data.button.r3;

    if (detectedButton > 0) {
      bool alreadyAssigned = isButtonAlreadyAssigned(detectedButton);

      if (alreadyAssigned) {
        display.clearDisplay();
        display.drawBitmap(20, 6, _assigned, 89, 52, 1);
        display.display();
        delay(800);
        Serial.printf("⚠️ Button %s already assigned elsewhere!\n", getButtonName(detectedButton).c_str());
      } else {
        if (selectedButtonForMapping == 0) {
          servoButtonMap[selectedServoForMapping].buttonType1 = detectedButton;
          servoButtonMap[selectedServoForMapping].state1 = false;
          servoButtonMap[selectedServoForMapping].lastButtonState1 = false;
          servoButtonMap[selectedServoForMapping].lastToggleTime1 = 0;
        } else {
          servoButtonMap[selectedServoForMapping].buttonType2 = detectedButton;
          servoButtonMap[selectedServoForMapping].state2 = false;
          servoButtonMap[selectedServoForMapping].lastButtonState2 = false;
          servoButtonMap[selectedServoForMapping].lastToggleTime2 = 0;
        }

        display.clearDisplay();
        display.drawBitmap(46, -2, _SUCCESS, 36, 48, 1);
        display.setTextSize(1);
        display.setCursor(50, 46);
        display.print(getButtonName(detectedButton));
        display.display();
        delay(500);
      }

      waitingForButtonPress = false;
      drawButtonMappingMenu();
      delay(300);
    }
  } else {
    static bool lastLeftNav = false;
    static bool lastRightNav = false;
    bool leftPressed = Ps3.data.button.left && !lastLeftNav;
    bool rightPressed = Ps3.data.button.right && !lastRightNav;
    lastLeftNav = Ps3.data.button.left;
    lastRightNav = Ps3.data.button.right;

    bool upPressed = Ps3.data.button.up && !lastUp;
    bool downPressed = Ps3.data.button.down && !lastDown;
    bool crossPressed = Ps3.data.button.cross && !lastCross;
    bool trianglePressed = Ps3.data.button.triangle && !lastTriangle;
    bool circlePressed = Ps3.data.button.circle && !lastCircle;

    lastUp = Ps3.data.button.up;
    lastDown = Ps3.data.button.down;
    lastCross = Ps3.data.button.cross;
    lastTriangle = Ps3.data.button.triangle;
    lastCircle = Ps3.data.button.circle;

    if (now - lastInput < BUTTON_DEBOUNCE) return;

    if (rightPressed) {
      selectedServoForMapping = (selectedServoForMapping - 1 + 4) % 4;
      drawButtonMappingMenu();
      lastInput = now;
    } else if (leftPressed) {
      selectedServoForMapping = (selectedServoForMapping + 1) % 4;
      drawButtonMappingMenu();
      lastInput = now;
    } else if (upPressed) {
      selectedButtonForMapping = (selectedButtonForMapping - 1 + 2) % 2;
      drawButtonMappingMenu();
      lastInput = now;
    } else if (downPressed) {
      selectedButtonForMapping = (selectedButtonForMapping + 1) % 2;
      drawButtonMappingMenu();
      lastInput = now;
    } else if (crossPressed) {
      waitingForButtonPress = true;
      drawButtonMappingMenu();
      lastInput = now;
      delay(300);
    } else if (trianglePressed) {
      if (selectedButtonForMapping == 0) {
        servoButtonMap[selectedServoForMapping].buttonType1 = 0;
        servoButtonMap[selectedServoForMapping].state1 = false;
        servoButtonMap[selectedServoForMapping].lastButtonState1 = false;
        servoButtonMap[selectedServoForMapping].lastToggleTime1 = 0;
      } else {
        servoButtonMap[selectedServoForMapping].buttonType2 = 0;
        servoButtonMap[selectedServoForMapping].state2 = false;
        servoButtonMap[selectedServoForMapping].lastButtonState2 = false;
        servoButtonMap[selectedServoForMapping].lastToggleTime2 = 0;
      }

      display.clearDisplay();
      display.drawBitmap(23, 9, _cleared, 83, 47, 1);
      display.display();
      delay(800);

      drawButtonMappingMenu();
      lastInput = now;
    } else if (circlePressed) {
      for (int i = 0; i < 4; i++) {
        EEPROM.put(ADDR_BUTTON_MAPPING + i * sizeof(ServoButtonMapping), servoButtonMap[i]);
      }
      EEPROM.commit();

      display.clearDisplay();
      display.drawBitmap(48, 8, _saved, 32, 49, 1);
      display.display();
      delay(500);

      oledMode = OLED_INFO;
      drawInfo();
      lastInput = now;
    }
  }
}

// ✅ 5-Mode Position Control dengan L3 trigger
void handlePositionModeSelection() {
  if (!Ps3.isConnected()) return;

  if (oledMode != OLED_INFO && oledMode != OLED_INFO_PWM) {
    return;
  }

  unsigned long now = millis();

  // Mode switching dengan UP/LEFT/RIGHT/DOWN/L3
  if (Ps3.data.button.up && !lastUp) {
    if (now - lastModeChange >= 300) {
      currentPositionMode = 0;
      showModeNotification(1);
      lastModeChange = now;
    }
  }
  lastUp = Ps3.data.button.up;

  if (Ps3.data.button.left && !lastLeft) {
    if (now - lastModeChange >= 300) {
      currentPositionMode = 1;
      showModeNotification(2);
      lastModeChange = now;
    }
  }
  lastLeft = Ps3.data.button.left;

  if (Ps3.data.button.right && !lastRight) {
    if (now - lastModeChange >= 300) {
      currentPositionMode = 2;
      showModeNotification(3);
      lastModeChange = now;
    }
  }
  lastRight = Ps3.data.button.right;

  if (Ps3.data.button.down && !lastDown) {
    if (now - lastModeChange >= 300) {
      currentPositionMode = 3;
      showModeNotification(4);
      lastModeChange = now;
    }
  }
  lastDown = Ps3.data.button.down;

  // ✅ Mode 5 dengan L3 (R3) button
  if (Ps3.data.button.r3 && !lastL3) {
    if (now - lastModeChange >= 300) {
      currentPositionMode = 4;  // Mode 5 (index 4)
      showModeNotification(5);
      lastModeChange = now;
    }
  }
  lastL3 = Ps3.data.button.r3;
}

void showModeNotification(int mode) {
  Serial.printf("🔄 Switched to Position Mode %d\n", mode);

  display.oled_command(SH110X_DISPLAYON);
  oledTimeOut = false;
  limitTimeOut = 50;
  countTimeOutOled = 0;
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(29, 10);
  display.printf("MODE %d", mode);
  display.setTextSize(1);
  display.setCursor(41, 39);
  display.print("SELECTED");
  display.display();
  delay(500);

  if (oledMode == OLED_INFO) drawInfo();
  else if (oledMode == OLED_INFO_PWM) drawPwmMotor();
}

// ============ SETUP ============

void setup() {
  Serial.begin(115200);


  Serial.println("\n\n=== EEPROS PS3 ROBOT ===");
  Serial.println("Starting system.. .");

  EEPROM.begin(512);
  Serial.println("EEPROM initialized");

  if (!display.begin(SCREEN_ADDRESS, true)) {
    Serial.println(F("SH1106 allocation failed"));
  } else {
    Serial.println("Display initialized");
  }

  display.clearDisplay();
  display.drawBitmap(12, 3, _AVATAR, 89, 59, 1);
  display.display();

  setupEncoder();

  servoTimerSemaphore = xSemaphoreCreateBinary();
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Set timer frequency to 1Mhz
  timer = timerBegin(1000000);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, 100000, true, 0);

  servoTimer = timerBegin(1000000);
  timerAttachInterrupt(servoTimer, &onServoTimer);
  timerAlarm(servoTimer, 5000, true, 0);

  Serial.println("Hardware Timer initialized (5ms)");

  setupMotor();
  Serial.println("Motors initialized");

  pcaSetup();
  Serial.println("PCA9685 initialized");

  // Load position control data untuk 5 mode
  Serial.println("\n=== Loading 5-Mode Position Control ===");
  for (int mode = 0; mode < 5; mode++) {
    for (int i = 0; i < 3; i++) {
      int targetAddr = ADDR_POSITION_TARGETS_MODE1 + (mode * 12) + (i * sizeof(long));
      int buttonAddr = ADDR_POSITION_BUTTONS_MODE1 + (mode * 3) + i;

      EEPROM.get(targetAddr, positionTargets[mode][i]);
      EEPROM.get(buttonAddr, positionButtons[mode][i]);

      if (positionButtons[mode][i] > 13) positionButtons[mode][i] = 0;

      positionButtonStates[mode][i] = false;
      positionLastTrigger[mode][i] = 0;

      Serial.printf("Mode %d Target %d:  %ld, Button:  %s\n",
                    mode + 1, i + 1, positionTargets[mode][i],
                    getButtonName(positionButtons[mode][i]).c_str());
    }
  }
  Serial.println("5-Mode Position Control loaded successfully!");

  loadPositionData();

  for (int i = 0; i < 4; i++) {
    EEPROM.get(ADDR_SERVO_OPEN + i * sizeof(int), servoOpen[i]);
    EEPROM.get(ADDR_SERVO_OPEN2 + i * sizeof(int), servoOpen2[i]);
    EEPROM.get(ADDR_SERVO_CLOSE + i * sizeof(int), servoClose[i]);

    if (servoOpen[i] < 0 || servoOpen[i] > 180) servoOpen[i] = 120;
    if (servoOpen2[i] < 0 || servoOpen2[i] > 180) servoOpen2[i] = 60;
    if (servoClose[i] < 0 || servoClose[i] > 180) servoClose[i] = 30;
  }

  EEPROM.get(ADDR_LAMBDA, lambda);
  if (isnan(lambda) || lambda < minLambda || lambda > maxLambda) lambda = 1.0f;

  EEPROM.get(ADDR_LENGTH_ALPHA, LengthAlpha);
  if (isnan(LengthAlpha) || LengthAlpha < minLengthAlpha || LengthAlpha > maxLengthAlpha) LengthAlpha = 0.2f;

  Serial.printf("Loaded - Lambda: %.2f, LengthAlpha:  %.2f\n", lambda, LengthAlpha);

  Serial.println("\n=== Button Mapping ===");
  for (int i = 0; i < 4; i++) {
    EEPROM.get(ADDR_BUTTON_MAPPING + i * sizeof(ServoButtonMapping), servoButtonMap[i]);

    if (servoButtonMap[i].buttonType1 > 13) servoButtonMap[i].buttonType1 = 0;
    if (servoButtonMap[i].buttonType2 > 13) servoButtonMap[i].buttonType2 = 0;

    servoButtonMap[i].state1 = false;
    servoButtonMap[i].state2 = false;
    servoButtonMap[i].lastButtonState1 = false;
    servoButtonMap[i].lastButtonState2 = false;
    servoButtonMap[i].lastToggleTime1 = 0;
    servoButtonMap[i].lastToggleTime2 = 0;

    Serial.printf("Servo %d => Btn1: %s, Btn2: %s\n",
                  i + 1,
                  getButtonName(servoButtonMap[i].buttonType1).c_str(),
                  getButtonName(servoButtonMap[i].buttonType2).c_str());
  }

  Serial.println("======================\n");

  // ============ LOAD THROTTLE SETTINGS ============
  Serial.println("\n=== Loading Throttle Settings ===");

  // ✅ UBAH:   Load sebagai uint8_t dulu untuk cek nilai
  uint8_t throttleEnabledByte;
  EEPROM.get(ADDR_THROTTLE_ENABLED, throttleEnabledByte);
  EEPROM.get(ADDR_THROTTLE_PWM_STEP, throttlePwmStep);
  EEPROM.get(ADDR_THROTTLE_INTERVAL, throttleIntervalUs);
  EEPROM.get(ADDR_THROTTLE_THRESHOLD, throttleActivationThreshold);

  // ✅ VALIDASI:  Jika bukan 0 atau 1, set default
  if (throttleEnabledByte != 0 && throttleEnabledByte != 1) {
    Serial.printf("⚠️ Invalid throttle enabled value: %d (resetting to default)\n", throttleEnabledByte);
    throttleEnabled = false;  // Default OFF

    // Save default value ke EEPROM
    uint8_t defaultEnabled = 0;
    EEPROM.put(ADDR_THROTTLE_ENABLED, defaultEnabled);
    EEPROM.commit();
  } else {
    throttleEnabled = (throttleEnabledByte == 1);
  }

  // Validasi parameter lainnya
  if (throttlePwmStep < 1 || throttlePwmStep > 500) throttlePwmStep = 50;
  if (throttleIntervalUs < 5000 || throttleIntervalUs > 1000000) throttleIntervalUs = 20000;
  if (throttleActivationThreshold < 0 || throttleActivationThreshold > 500) throttleActivationThreshold = 100;

  Serial.printf("Throttle:  %s (raw byte: %d)\n", throttleEnabled ? "ENABLED" : "DISABLED", throttleEnabledByte);
  Serial.printf("PWM Step: %d\n", throttlePwmStep);
  Serial.printf("Interval: %d us (%d ms)\n", throttleIntervalUs, throttleIntervalUs / 1000);
  Serial.printf("Threshold: %d\n", throttleActivationThreshold);

  // Initialize throttle timer
  setupThrottleTimer();
  for (int i = 0; i < 4; i++) {
    setServoAngleStrong(i, servoClose[i]);
    servoButtonMap[i].state1 = false;
    servoButtonMap[i].state2 = false;
  }

  Serial.println("Starting PS3 Controller Setup...");
  Ps3.begin(PS3_ADDRESS);
  Ps3.attach(processGamepad);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);

  Serial.printf("Allowed PS3 Controller: %s\n", PS3_ADDRESS);
  Serial.println("Waiting for PS3 controller connection...");

  delay(1500);
  drawInfo();
  Serial.println("Setup complete!");
}

// ============ LOOP ============

void loop() {
  if (xSemaphoreTake(servoTimerSemaphore, 0) == pdTRUE) {
    if ((oledMode == OLED_INFO || oledMode == OLED_INFO_PWM) && Ps3.isConnected()) {
      handleServoToggle();
    }
  }

  updatePositionControl();
  if (throttleEnabled && xSemaphoreTake(throttleTimerSemaphore, 0) == pdTRUE) {
    updatePwmThrottle();
  }
  updateDisplayRealTime();

  handlePositionControlTriggers();
  handlePositionModeSelection();

  static unsigned long lastDisplayUpdate = 0;
  if ((oledMode == OLED_INFO || oledMode == OLED_INFO_PWM) && (millis() - lastDisplayUpdate >= 100)) {
    if (oledMode == OLED_INFO) drawInfo();
    else drawPwmMotor();
    lastDisplayUpdate = millis();
  }

  static bool lastSelectForMenu = false;
  static unsigned long lastMiscPress = 0;

  if (Ps3.isConnected()) {
    bool selectPressed = Ps3.data.button.select;

    if (selectPressed && !lastSelectForMenu && (millis() - lastMiscPress > 300)) {
      if (oledMode == OLED_INFO) {
        oledMode = OLED_INFO_PWM;
        drawPwmMotor();
        Serial.println("Enter PWM INFO menu");
      } else if (oledMode == OLED_INFO_PWM) {
        oledMode = OLED_MENU_SETTING_SERVO;
        servoMenuState = MENU_SERVO_LIST;
        selectedSetting = 0;
        drawSettingServoMenu();
        Serial.println("Enter SERVO menu");
      } else if (oledMode == OLED_MENU_SETTING_SERVO) {
        oledMode = OLED_MENU_SETTING_LAMBDA_LENGTHALPHA;
        lambdaMenuSelected = 0;
        drawSettingLambdaLengthAlphaMenu();
        Serial.println("Enter LAMBDA/LENGTHALPHA menu");
      } else if (oledMode == OLED_MENU_SETTING_LAMBDA_LENGTHALPHA) {
        oledMode = OLED_MENU_BUTTON_MAPPING;
        selectedServoForMapping = 0;
        selectedButtonForMapping = 0;
        waitingForButtonPress = false;
        drawButtonMappingMenu();
        Serial.println("Enter BUTTON MAPPING menu");
      } else if (oledMode == OLED_MENU_BUTTON_MAPPING) {
        oledMode = OLED_MENU_POSITION_CONTROL;
        positionMenuState = MENU_POSITION_MODE_SELECT;
        selectedPositionMode = 0;
        drawPositionTargetMenu();
        Serial.println("Enter POSITION CONTROL - 5-Mode Selection");
      } else if (oledMode == OLED_MENU_POSITION_CONTROL) {
        oledMode = OLED_MENU_THROTTLE_SETTING;  // ✅ Arahkan ke throttle menu
        throttleMenuSelected = 0;
        drawThrottleSettingMenu();
        Serial.println("Enter THROTTLE SETTING menu");
      } else if (oledMode == OLED_MENU_THROTTLE_SETTING) {
        oledMode = OLED_INFO;
        oledTimeOut = false;
        countTimeOutOled = 0;
        drawInfo();
        Serial.println("Back to INFO");
      }
      lastMiscPress = millis();
    }
    lastSelectForMenu = selectPressed;

    // ✅ Handle menu navigation
    if (oledMode == OLED_MENU_SETTING_SERVO) {
      handleSettingServoMenuNavigation();
    } else if (oledMode == OLED_MENU_SETTING_LAMBDA_LENGTHALPHA) {
      handleSettingLambdaLengthAlphaMenuNavigation();
    } else if (oledMode == OLED_MENU_BUTTON_MAPPING) {
      handleButtonMappingMenuNavigation();
    } else if (oledMode == OLED_MENU_POSITION_CONTROL) {
      handlePositionTargetMenuNavigation();
    } else if (oledMode == OLED_MENU_THROTTLE_SETTING) {
      handleThrottleSettingMenuNavigation();
    }
  }
}


// ============ MOTOR FUNCTIONS ============

void setupMotor() {
  for (int i = 0; i < 10; i++) {
    ledcAttach(pinMotor[i], FREQ_MOTOR, RESOLUTION_MOTOR);
    ledcWrite(pinMotor[i], 0);
  }
}

void stopMotor() {
  // Reset throttle state
  portENTER_CRITICAL(&timerMux);
  for (int i = 0; i < 4; i++) {
    currentPwmMotor[i] = 0;
    targetPwmMotor[i] = 0;
  }
  portEXIT_CRITICAL(&timerMux);

  // Stop semua motor
  for (int i = 0; i < 10; i++) {
    ledcWrite(pinMotor[i], 0);
  }
}

void updatePwmThrottle() {
  if (!throttleEnabled) return;  // Skip jika throttle disabled

  int localTarget[4];
  int localCurrent[4];

  // Baca target dan current dengan thread safety
  portENTER_CRITICAL(&timerMux);
  for (int i = 0; i < 4; i++) {
    localTarget[i] = targetPwmMotor[i];
    localCurrent[i] = currentPwmMotor[i];
  }
  portEXIT_CRITICAL(&timerMux);

  // Update setiap motor
  for (int i = 0; i < 4; i++) {
    int target = localTarget[i];
    int current = localCurrent[i];
    int newCurrent = current;

    // Jika target PWM <= threshold, langsung set (bypass throttle)
    if (abs(target) <= throttleActivationThreshold) {
      newCurrent = target;
    }
    // Jika > threshold, gunakan smooth ramping
    else {
      int diff = target - current;

      if (abs(diff) <= throttlePwmStep) {
        newCurrent = target;
      } else {
        if (diff > 0) {
          newCurrent = current + throttlePwmStep;
        } else {
          newCurrent = current - throttlePwmStep;
        }
      }

      newCurrent = constrain(newCurrent, -1023, 1023);
    }

    localCurrent[i] = newCurrent;
  }

  // Tulis kembali current PWM
  portENTER_CRITICAL(&timerMux);
  for (int i = 0; i < 4; i++) {
    currentPwmMotor[i] = localCurrent[i];
  }
  portEXIT_CRITICAL(&timerMux);

  // Apply ke motor
  setPwmDirect(localCurrent);
}


void setPwmDirect(int pwm[4]) {
  int chA[] = { M1A, M2A, M3A, M4A };
  int chB[] = { M1B, M2B, M3B, M4B };

  for (int i = 0; i < 4; i++) {
    if (pwm[i] > 0) {
      ledcWrite(chA[i], abs(pwm[i]));
      ledcWrite(chB[i], 0);
    } else if (pwm[i] < 0) {
      ledcWrite(chA[i], 0);
      ledcWrite(chB[i], abs(pwm[i]));
    } else {
      ledcWrite(chA[i], 0);
      ledcWrite(chB[i], 0);
    }
  }
}
void setPwm(int pwm[4]) {
  int chA[] = { M1A, M2A, M3A, M4A };
  int chB[] = { M1B, M2B, M3B, M4B };

  for (int i = 0; i < 4; i++) {
    if (pwm[i] > 0) {
      ledcWrite(chA[i], pwm[i]);
      ledcWrite(chB[i], 0);
    } else if (pwm[i] < 0) {
      ledcWrite(chA[i], 0);
      ledcWrite(chB[i], abs(pwm[i]));
    } else {
      ledcWrite(chA[i], 0);
      ledcWrite(chB[i], 0);
    }
  }
}