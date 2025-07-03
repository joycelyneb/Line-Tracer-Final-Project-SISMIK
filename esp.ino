#include "esp_system.h"
#include "esp_task_wdt.h"

// Definisi pin motor
#define IN1 27  // Motor kanan maju
#define IN2 14  // Motor kanan mundur
#define IN3 25  // Motor kiri maju
#define IN4 26  // Motor kiri mundur

// Definisi pin sensor (dari kiri ke kanan)
#define SENSOR_PALING_KIRI 19   // D0 sensor paling kiri
#define SENSOR_KIRI 32          // D0 sensor kiri
#define SENSOR_TENGAH 35        // D0 sensor tengah
#define SENSOR_KANAN 33         // D0 sensor kanan
#define SENSOR_PALING_KANAN 18  // D0 sensor paling kanan

// Konfigurasi Watchdog
#define WDT_TIMEOUT 5  // Timeout watchdog dalam detik
#define STUCK_THRESHOLD 5000  // Threshold untuk deteksi stuck (ms)

// Array pin sensor
int sensorPins[5] = {SENSOR_PALING_KIRI, SENSOR_KIRI, SENSOR_TENGAH, SENSOR_KANAN, SENSOR_PALING_KANAN};

// Array untuk menyimpan status sensor
int sensorStatus[5];

// Variabel untuk monitoring watchdog
unsigned long lastMovementTime = 0;
unsigned long lastWdtFeed = 0;
int lastSensorPattern = -1;
bool isStuck = false;

void setup() {
  Serial.begin(115200);
  
  // Inisialisasi Watchdog Timer
  initWatchdog();
  
  // Setup motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Setup sensor
  for(int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  Serial.println("Robot Line Follower 5 Sensor dengan Watchdog Siap!");
  Serial.print("Watchdog Timeout: ");
  Serial.print(WDT_TIMEOUT);
  Serial.println(" detik");
  
  lastMovementTime = millis();
}

void loop() {
  // Feed watchdog timer - sangat penting!
  feedWatchdog();
  
  // Baca semua sensor dan masukkan ke array
  for(int i = 0; i < 5; i++) {
    sensorStatus[i] = digitalRead(sensorPins[i]);
  }
  
  // Deteksi stuck condition
  detectStuckCondition();
  
  // Debug output - tampilkan status sensor
  Serial.print("Sensor: ");
  for(int i = 0; i < 5; i++) {
    Serial.print(sensorStatus[i]);
  }
  Serial.print(" -> ");
  
  // Jika robot stuck terlalu lama, lakukan recovery
  if(isStuck) {
    performStuckRecovery();
    return; // Skip normal line following
  }
  
  // Logika line tracer berdasarkan kombinasi sensor
  if(isPattern(0,0,1,0,0) || isPattern(0,1,1,1,0) || isPattern(1,1,1,1,1)) {
    // 00100, 01110, 11111 -> Maju
    maju();
    Serial.println("MAJU");
    updateMovementTime();
  }
  else if(isPattern(0,0,0,0,0)) {
    // 00000 -> Berhenti
    stopMotor();
    Serial.println("STOP");
  }
  else if(isPattern(1,0,0,0,0) || isPattern(1,1,0,0,0) || isPattern(1,1,1,0,0) || 
          isPattern(1,1,1,1,0) || isPattern(0,1,0,0,0) || isPattern(0,1,1,0,0)) {
    // 10000, 11000, 11100, 11110, 01000, 01100 -> Belok Kiri
    rodaKiri();
    Serial.println("BELOK KIRI");
    updateMovementTime();
  }
  else if(isPattern(0,0,0,0,1) || isPattern(0,0,0,1,1) || isPattern(0,0,1,1,1) || 
          isPattern(0,1,1,1,1) || isPattern(0,0,0,1,0) || isPattern(0,0,1,1,0)) {
    // 00001, 00011, 00111, 01111, 00010, 00110 -> Belok Kanan
    rodaKanan();
    Serial.println("BELOK KANAN");
    updateMovementTime();
  }
  else {
    // Kondisi lain -> Maju (seperti kode asli)
    maju();
    Serial.println("KONDISI TIDAK DIKENAL - MAJU");
    updateMovementTime();
  }
  
  // Small delay untuk stabilisasi
  delay(10);
}

// === FUNGSI WATCHDOG ===

void initWatchdog() {
  Serial.println("Inisialisasi Watchdog Timer...");
  
  // Konfigurasi watchdog timer
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 5000,         // 5 detik timeout
      .idle_core_mask = 0b11,     // Core 0 dan 1
      .trigger_panic = true       // Reset jika tidak di-reset
  };
  
  // Inisialisasi watchdog
  esp_err_t result = esp_task_wdt_init(&wdt_config);
  if (result == ESP_OK) {
    Serial.println("Watchdog berhasil diinisialisasi");
    
    // Subscribe task ini ke watchdog
    esp_task_wdt_add(NULL);
    Serial.println("Task berhasil didaftarkan ke watchdog");
  } else {
    Serial.print("Gagal inisialisasi watchdog: ");
    Serial.println(result);
  }
}

void feedWatchdog() {
  unsigned long currentTime = millis();
  
  // Feed watchdog setiap 100ms
  if(currentTime - lastWdtFeed >= 100) {
    esp_task_wdt_reset();
    lastWdtFeed = currentTime;
    
    // Debug info (optional - bisa dimatikan untuk mengurangi serial output)
    // Serial.println("Watchdog fed");
  }
}

void detectStuckCondition() {
  int currentPattern = calculateSensorPattern();
  unsigned long currentTime = millis();
  
  // Jika pola sensor sama dan waktu sudah melebihi threshold
  if(currentPattern == lastSensorPattern && 
     (currentTime - lastMovementTime) > STUCK_THRESHOLD) {
    
    if(!isStuck) {
      isStuck = true;
      Serial.println("WARNING: Robot terdeteksi STUCK!");
    }
  } else {
    isStuck = false;
    lastSensorPattern = currentPattern;
  }
}

int calculateSensorPattern() {
  int pattern = 0;
  for(int i = 0; i < 5; i++) {
    pattern = (pattern << 1) | sensorStatus[i];
  }
  return pattern;
}

void updateMovementTime() {
  lastMovementTime = millis();
}

void performStuckRecovery() {
  Serial.println("Melakukan recovery dari kondisi stuck...");
  
  // Strategi recovery: mundur sebentar lalu coba belok
  mundur();
  delay(300);
  
  // Coba belok kiri dulu
  rodaKiri();
  delay(200);
  
  // Reset stuck flag
  isStuck = false;
  lastMovementTime = millis();
  
  Serial.println("Recovery selesai, melanjutkan line following...");
}

// === FUNGSI MOTOR (dengan tambahan mundur) ===

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
  digitalWrite(IN1, LOW);   // Motor kanan stop
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Motor kiri maju
  digitalWrite(IN4, LOW);
}

void rodaKanan() {
  digitalWrite(IN1, HIGH);  // Motor kanan maju
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Motor kiri stop
  digitalWrite(IN4, LOW);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// === FUNGSI UTILITAS ===

// Fungsi untuk mengecek pola sensor
bool isPattern(int s0, int s1, int s2, int s3, int s4) {
  return (sensorStatus[0] == s0 && sensorStatus[1] == s1 && sensorStatus[2] == s2 && 
          sensorStatus[3] == s3 && sensorStatus[4] == s4);
}