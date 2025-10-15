/*
 * 서보 제어 + MQ-2 가스 센서
 * Arduino Uno/Nano
 * 
 * 연결:
 * - 서보 Signal → D9
 * - 서보 VCC/GND → 5V/GND (또는 외부 전원)
 * - MQ-2 VCC/GND → 5V/GND
 * - MQ-2 AOUT → A0
 * 
 * 기능:
 * 1. MQ-2 가스 센서 값을 10Hz로 시리얼 전송
 * 2. 'fire' 명령 수신 시 서보 투하 동작
 */

#include <Servo.h>

// === 핀 설정 ===
const int SERVO_PIN = 9;       // 서보 모터
const int MQ2_PIN = A0;        // MQ-2 가스 센서

// === 서보 객체 ===
Servo dropServo;

// === 서보 각도 ===
const int SERVO_CLOSE = 90;    // 닫힌 상태
const int SERVO_OPEN = 180;    // 열린 상태 (투하)

// === MQ-2 설정 ===
const float VCC = 5.0;
const float RL = 10.0;         // 부하 저항 (kΩ)
float Ro = 10.0;               // 센서 베이스 저항

// === 시리얼 명령 ===
String inputString = "";
bool commandReceived = false;

// === 타이밍 ===
unsigned long lastSensorRead = 0;
const long SENSOR_INTERVAL = 100;  // 10Hz (100ms)

void setup() {
  Serial.begin(9600);
  
  // 서보 초기화
  dropServo.attach(SERVO_PIN);
  dropServo.write(SERVO_CLOSE);
  
  // MQ-2 핀 설정
  pinMode(MQ2_PIN, INPUT);
  
  Serial.println("Servo + MQ-2 Ready!");
  Serial.println("Commands: 'fire' = drop servo");
  
  // MQ-2 캘리브레이션 (간단 버전)
  delay(2000);
  Ro = calibrateSensor();
  Serial.print("MQ-2 Ro = ");
  Serial.println(Ro);
}

void loop() {
  // === 1. 시리얼 명령 처리 ===
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      inputString.trim();
      
      if (inputString == "fire") {
        commandReceived = true;
      }
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
  
  // === 2. 서보 동작 ===
  if (commandReceived) {
    Serial.println("DROPPING!");
    moveServoOnce();
    commandReceived = false;
  }
  
  // === 3. MQ-2 센서 읽기 (10Hz) ===
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = currentMillis;
    
    float ppm = readMQ2();
    Serial.println(ppm);  // ppm 값만 전송
  }
}

// === 서보 동작 함수 ===
void moveServoOnce() {
  // 90도 → 180도 (2초 동안)
  for (int pos = SERVO_CLOSE; pos <= SERVO_OPEN; pos++) {
    dropServo.write(pos);
    delay(2000 / 90);
  }
  
  delay(2000);  // 180도에서 대기
  delay(3000);  // 추가 대기
  
  // 180도 → 90도 (2초 동안 복귀)
  for (int pos = SERVO_OPEN; pos >= SERVO_CLOSE; pos--) {
    dropServo.write(pos);
    delay(2000 / 90);
  }
  
  Serial.println("DONE");
}

// === MQ-2 읽기 함수 ===
float readMQ2() {
  int rawValue = analogRead(MQ2_PIN);
  
  // ADC → 전압
  float voltage = rawValue * (VCC / 1023.0);
  
  // 전압 → 센서 저항
  if (voltage == 0) return 0;
  float Rs = ((VCC - voltage) / voltage) * RL;
  
  // Rs/Ro 비율
  float ratio = Rs / Ro;
  if (ratio <= 0) return 0;
  
  // 비율 → ppm (근사식)
  float ppm = 613.9 * pow(ratio, -2.074);
  
  // 유효 범위 제한
  if (ppm < 0) ppm = 0;
  if (ppm > 10000) ppm = 10000;
  
  return ppm;
}

// === MQ-2 캘리브레이션 ===
float calibrateSensor() {
  const float RO_CLEAN_AIR = 9.83;
  float total = 0;
  int samples = 50;
  
  for (int i = 0; i < samples; i++) {
    int rawValue = analogRead(MQ2_PIN);
    float voltage = rawValue * (VCC / 1023.0);
    if (voltage > 0) {
      float Rs = ((VCC - voltage) / voltage) * RL;
      total += Rs;
    }
    delay(50);
  }
  
  float avgRs = total / samples;
  return avgRs / RO_CLEAN_AIR;
}

