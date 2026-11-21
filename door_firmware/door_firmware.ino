#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// // Arduino Uno(2KB SRAM) 메모리 초과 방지를 위해 큰 원형 버퍼 비활성화
// #define LD245X_USE_CIRCULAR_BUFFER 0
// #include <HLK_LD245X.h>

// ==========================================
// [ESP32 복귀 시 변경 포인트 요약]
// 1. #include <Servo.h> / SoftwareSerial -> #include <ESP32Servo.h> 및 하드웨어 Serial2로 복귀
// 2. 핀 번호를 ESP32 GPIO 핀맵으로 재설정
// 3. rpiAllowDoorOpen 상태 판단 로직을 실제 라즈베리파이 통신 수신값으로 연결
// ==========================================

Servo myservo;
// LD245X radar;

// 아두이노 우노 핀 배치
// const int radarRxPin = 2;        // 우노 D2 -> LD2450 TX 연결
// const int radarTxPin = 3;        // 우노 D3 -> LD2450 RX 연결
const int tcrtDoPin = 4;         // TCRT5000 모듈 디지털 출력(DO) 핀
const int hcTrigPin = 6;         // 초음파 Trig
const int hcEchoPin = 7;         // 초음파 Echo
const int servoPin = 9;          // 서보모터 PWM 핀

// TCRT5000 모듈 출력 기준 (일반적인 LM393 비교기 모듈은 감지 시 LOW 출력)
const int OBSTACLE_DETECTED_STATE = LOW;

// SoftwareSerial radarSerial(radarRxPin, radarTxPin);

int door = 0;                    // 0: 닫힘, 1: 열림
bool rpiAllowDoorOpen = true;    // [추후 확장] 라즈베리파이 상태 신호에 따른 문 개방 허용 플래그

// TCRT5000 장애물 감지 여부 확인
bool isTcrtObstacleDetected() {
  return (digitalRead(tcrtDoPin) == OBSTACLE_DETECTED_STATE);
}

// // 레이더 사람 감지 여부 확인 함수
// bool isRadarHumanDetected() {
//   int targets = radar.read();
//   return (targets > 0);
// }

// 초음파 거리(cm) 측정 함수
long measureDistanceCm() {
  digitalWrite(hcTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(hcTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(hcTrigPin, LOW);

  long duration = pulseIn(hcEchoPin, HIGH, 50000);
  if (duration == 0) {
    return -1; // 측정 실패 또는 범위 초과
  }
  return duration * 17 / 1000;
}

// 서보 이동 중 TCRT5000 장애물 실시간 감시 함수 (매개변수: 목표각도, 5도 당 대기 시간)
// 반환값 true: 장애물 감지됨 / false: 정상 완료
// checkObstacle이 true일 때만 이동 중 TCRT5000을 감시 (false: 역동작 중 -> 장애물 감지 시 역동작 무한루프(문 정지) 해결)
bool moveServoWithObstacleCheck(int targetAngle, int stepDelayMs, bool checkObstacle) {
  int currentAngle = myservo.read();                    // 현재 서보 각도 
  int step = (targetAngle >= currentAngle) ? 5 : -5;    // 목표 각도가 현재보다 크면 +5도, 작으면 -5도로 문을 여닫는 동작 함께 처리 

  // 실행 테스트 코드
  Serial.print("  -> moveServo 시작: ");
  Serial.print(currentAngle);
  Serial.print(" -> ");
  Serial.print(targetAngle);
  Serial.print(" (check=");
  Serial.print(checkObstacle);
  Serial.println(")");

  // 각도가 커질 때와 작아질 때(문을 열고 닫기) <= / >= 삼항연산자로 선택
  for (int angle = currentAngle;
       (step > 0) ? (angle <= targetAngle) : (angle >= targetAngle);
       angle += step) {
    myservo.write(angle);

    // 대기시간을 20으로 쪼개 장애물을 체크 (최소 1번 이상) / 현재 코드 기준 총 200ms 대기, 20ms마다 체크
    int checks = max(1, stepDelayMs / 20); 
    for (int d = 0; d < checks; d++) {
      delay(20);
      if (checkObstacle && isTcrtObstacleDetected()) {
        return true;
      }
    }
  }
  // 목표 각도가 5의 배수가 아닐 경우 남은 만큼 마저 이동할 수 있도록 맞추기 
  myservo.write(targetAngle);

  return (checkObstacle && isTcrtObstacleDetected());
}

void setup() {
  Serial.begin(115200);          // PC 시리얼 모니터 / 추후 라즈베리파이 통신용
  // radarSerial.begin(256000);     // LD2450 기본 통신 속도
  // radar.begin(radarSerial);

  myservo.attach(servoPin, 500, 2400);
  myservo.write(0);               // 초기 닫힘 각도 0도

  pinMode(hcTrigPin, OUTPUT);
  pinMode(hcEchoPin, INPUT);
  pinMode(tcrtDoPin, INPUT_PULLUP);

  Serial.println("Arduino Uno HandsFreeDoor (TCRT5000) 테스트 시작");
}

void loop() {
  // // 1단계: 레이더 센서 사람 감지
  // bool humanDetected = isRadarHumanDetected();
  // if (!humanDetected) {
  //   Serial.println("[레이더] 감지된 사람 없음");
  //   delay(300);
  //   return;
  // }
  // Serial.println("[레이더] 사람 감지됨");

  // 2단계: 초음파 센서로 일정 거리 유지 확인
  long distance = measureDistanceCm();
  Serial.print("[초음파] 거리: ");
  Serial.print(distance);
  Serial.println(" cm");

  bool distanceConditionMet = (distance >= 50);
  if (!distanceConditionMet) {
    Serial.println("[초음파] 거리 조건 미충족");
    delay(300);
    return;
  }

  // 3단계: [추후 확장] 레이더와 초음파 사이/이후 단계에서 라즈베리파이 상태 변수로 개방 여부 판단
  if (!rpiAllowDoorOpen) {
    Serial.println("[RPi 상태] 현재 문을 열 수 없는 상태입니다.");
    delay(500);
    return;
  }

  // 4단계: 서보모터 작동 및 작동 중 장애물 감지 시 되돌아가기
  if (door == 0) {
    Serial.println("[서보] 문 열기 시작...");
    bool obstructed = moveServoWithObstacleCheck(90, 200, 1);

    if (obstructed) {
      Serial.println("[경고] 문 여는 중 TCRT5000 장애물 감지! 역동작으로 다시 닫습니다.");
      moveServoWithObstacleCheck(0, 100, 0);
      door = 0;
    } else {
      Serial.println("[완료] 문이 열렸습니다.");
      door = 1;
    }
  } else {
    Serial.println("[서보] 문 닫기 시작...");
    bool obstructed = moveServoWithObstacleCheck(0, 200, 1);

    if (obstructed) {
      Serial.println("[경고] 문 닫는 중 TCRT5000 장애물 감지! 역동작으로 다시 엽니다.");
      moveServoWithObstacleCheck(90, 100, 0);
      door = 1;
    } else {
      Serial.println("[완료] 문이 닫혔습니다.");
      door = 0;
    }
  }

  delay(1000);
}