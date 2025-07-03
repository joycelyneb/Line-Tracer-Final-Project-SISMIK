#include "esp_system.h"
#include "esp_task_wdt.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#define IN1 27
#define IN2 14
#define IN3 25
#define IN4 26

#define SENSOR_PALING_KIRI 19
#define SENSOR_KIRI 32
#define SENSOR_TENGAH 35
#define SENSOR_KANAN 33
#define SENSOR_PALING_KANAN 18

int sensorPins[5] = {SENSOR_PALING_KIRI, SENSOR_KIRI, SENSOR_TENGAH, SENSOR_KANAN, SENSOR_PALING_KANAN};
volatile int sensorStatus[5];
volatile int prevSensorStatus[5];

#define TIMER_DIVIDER         80
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)
#define TIMER_INTERVAL_SEC    (0.01)
#define TIMER_INTERVAL        (TIMER_INTERVAL_SEC * TIMER_SCALE)

#define WDT_TIMEOUT 5
#define STUCK_THRESHOLD 5000

volatile unsigned long lastMovementTime = 0;
volatile unsigned long lastWdtFeed = 0;
volatile bool isStuck = false;
volatile bool sensorChanged = false;

typedef enum {
  ROBOT_FORWARD,
  ROBOT_LEFT,
  ROBOT_RIGHT,
  ROBOT_STOP,
  ROBOT_RECOVERY
} robot_state_t;

volatile robot_state_t currentState = ROBOT_STOP;
volatile robot_state_t prevState = ROBOT_STOP;

void IRAM_ATTR timer_group_isr(void *para);
void IRAM_ATTR sensor_change_isr(void);
void processLineFollowing(void);
void executeMotorCommand(robot_state_t state);

void setup() {
  Serial.begin(115200);
  initWatchdog();
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  setupSensorInterrupts();
  setupTimerInterrupt();
  lastMovementTime = millis();
  Serial.println("Robot Line Follower 5 Sensor dengan Interrupt Siap!");
  Serial.println("  `() function akan kosong - semua proses menggunakan interrupt");
}

void loop() {
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    Serial.print("Robot State: ");
    switch(currentState) {
      case ROBOT_FORWARD: Serial.println("FORWARD"); break;
      case ROBOT_LEFT: Serial.println("LEFT"); break;
      case ROBOT_RIGHT: Serial.println("RIGHT"); break;
      case ROBOT_STOP: Serial.println("STOP"); break;
      case ROBOT_RECOVERY: Serial.println("RECOVERY"); break;
    }
    lastDebug = millis();
  }
  delay(1);
}

void setupSensorInterrupts() {
  for(int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
    sensorStatus[i] = digitalRead(sensorPins[i]);
    prevSensorStatus[i] = sensorStatus[i];
  }
  attachInterrupt(digitalPinToInterrupt(SENSOR_TENGAH), sensor_change_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SENSOR_KIRI), sensor_change_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SENSOR_KANAN), sensor_change_isr, CHANGE);
  if (digitalPinToInterrupt(SENSOR_PALING_KIRI) != -1) {
    attachInterrupt(digitalPinToInterrupt(SENSOR_PALING_KIRI), sensor_change_isr, CHANGE);
  }
  if (digitalPinToInterrupt(SENSOR_PALING_KANAN) != -1) {
    attachInterrupt(digitalPinToInterrupt(SENSOR_PALING_KANAN), sensor_change_isr, CHANGE);
  }
  Serial.println("Sensor interrupts berhasil disetup");
}

void setupTimerInterrupt() {
  timer_config_t config;
  config.divider = TIMER_DIVIDER;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_PAUSE;
  config.alarm_en = TIMER_ALARM_EN;
  config.intr_type = TIMER_INTR_LEVEL;
  config.auto_reload = true;
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL); 
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group_isr, (void*) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
  timer_start(TIMER_GROUP_0, TIMER_0);
  Serial.println("Timer interrupt berhasil disetup");
}

void IRAM_ATTR sensor_change_isr(void) {
  sensorChanged = true;
  for(int i = 0; i < 5; i++) {
    sensorStatus[i] = digitalRead(sensorPins[i]);
  }
  lastMovementTime = millis();
}

void IRAM_ATTR timer_group_isr(void *para) {
  int timer_idx = (int) para;
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);
  static unsigned long lastWdtFeedISR = 0;
  unsigned long currentTime = millis();
  if(currentTime - lastWdtFeedISR >= 100) {
    esp_task_wdt_reset();
    lastWdtFeedISR = currentTime;
  }
  if ((currentTime - lastMovementTime) > STUCK_THRESHOLD) {
    if (isPatternISR(1,1,1,1,1) || isPatternISR(0,0,0,0,0)) {
      if(!isStuck) {
        isStuck = true;
        currentState = ROBOT_RECOVERY;
      }
    }
  }
  if (sensorChanged || isStuck) {
    processLineFollowing();
    sensorChanged = false;
  }
}

void processLineFollowing(void) {
  robot_state_t newState = ROBOT_STOP;
  if(isStuck) {
    performStuckRecoveryISR();
    return;
  }
  if(isPatternISR(0,0,1,0,0) || isPatternISR(0,1,1,1,0) || isPatternISR(1,1,1,1,1)) {
    newState = ROBOT_FORWARD;
  }
  else if(isPatternISR(0,0,0,0,0)) {
    newState = ROBOT_STOP;
  }
  else if(isPatternISR(1,0,0,0,0) || isPatternISR(1,1,0,0,0) || isPatternISR(1,1,1,0,0) || 
          isPatternISR(1,1,1,1,0) || isPatternISR(0,1,0,0,0) || isPatternISR(0,1,1,0,0)) {
    newState = ROBOT_LEFT;
  }
  else if(isPatternISR(0,0,0,0,1) || isPatternISR(0,0,0,1,1) || isPatternISR(0,0,1,1,1) || 
          isPatternISR(0,1,1,1,1) || isPatternISR(0,0,0,1,0) || isPatternISR(0,0,1,1,0)) {
    newState = ROBOT_RIGHT;
  }
  else {
    newState = ROBOT_FORWARD;
  }
  if (newState != currentState) {
    prevState = currentState;
    currentState = newState;
    executeMotorCommand(currentState);
  }
}

void executeMotorCommand(robot_state_t state) {
  switch(state) {
    case ROBOT_FORWARD: maju(); break;
    case ROBOT_LEFT: rodaKiri(); break;
    case ROBOT_RIGHT: rodaKanan(); break;
    case ROBOT_STOP: stopMotor(); break;
    case ROBOT_RECOVERY: stopMotor(); break;
  }
}

bool IRAM_ATTR isPatternISR(int s0, int s1, int s2, int s3, int s4) {
  return (sensorStatus[0] == s0 && sensorStatus[1] == s1 && sensorStatus[2] == s2 && 
          sensorStatus[3] == s3 && sensorStatus[4] == s4);
}

bool isPattern(int s0, int s1, int s2, int s3, int s4) {
  return (sensorStatus[0] == s0 && sensorStatus[1] == s1 && sensorStatus[2] == s2 && 
          sensorStatus[3] == s3 && sensorStatus[4] == s4);
}

void maju() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void mundur() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void rodaKiri() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void rodaKanan() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void initWatchdog() {
  Serial.println("Inisialisasi Watchdog Timer...");
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = STUCK_THRESHOLD,
      .idle_core_mask = 0b11,
      .trigger_panic = true
  };
  esp_err_t result = esp_task_wdt_init(&wdt_config);
  if (result == ESP_OK) {
    Serial.println("Watchdog berhasil diinisialisasi");
    esp_task_wdt_add(NULL);
    Serial.println("Task berhasil didaftarkan ke watchdog");
  } else {
    Serial.print("Gagal inisialisasi watchdog: ");
    Serial.println(result);
  }
}

void performStuckRecoveryISR() {
  static unsigned long recoveryStartTime = 0;
  static bool recoveryInProgress = false;
  if (!recoveryInProgress) {
    recoveryStartTime = millis();
    recoveryInProgress = true;
    stopMotor();
    return;
  }
  unsigned long currentTime = millis();
  if (currentTime - recoveryStartTime < 2000) {
    stopMotor();
  } else {
    isStuck = false;
    recoveryInProgress = false;
    lastMovementTime = millis();
    currentState = ROBOT_FORWARD;
    executeMotorCommand(currentState);
  }
}
