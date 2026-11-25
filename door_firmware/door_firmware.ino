#include <Arduino.h>
#include <Servo.h>

// ==========================================
// [ESP32 복귀 시 변경 포인트 요약]
// 1. #include <Servo.h> -> #include <ESP32Servo.h>, 하드웨어 Serial2로 LD2450 연결 복귀
// 2. 핀 번호를 ESP32 GPIO 핀맵으로 재설정 및 LED/부저 핀 추가
// 3. 라즈베리파이 시리얼 포트 확인 및 STATE 송신 활성화
// ==========================================

Servo myservo;

// 아두이노 우노 핀 배치
const int tcrtDoPin = 4;               // TCRT5000 모듈 디지털 출력(DO) 핀
const int hcTrigPin = 6;               // 초음파 Trig
const int hcEchoPin = 7;               // 초음파 Echo
const int servoPin = 9;                // 서보모터 PWM 핀

// TCRT5000 모듈 출력 기준 (일반적인 LM393 비교기 모듈은 감지 시 LOW 출력)
const int OBSTACLE_DETECTED_STATE = LOW;

const int DOOR_CLOSE_ANGLE = 0;
const int DOOR_OPEN_ANGLE = 90;
const long SAFE_OPEN_DISTANCE_CM = 50; // 사람이 50cm 이내면 부딪힘 방지를 위해 미개방
const unsigned long DEFAULT_HOLD_MS = 2000;

enum MoveResult {
  MOVE_COMPLETED = 0,
  MOVE_OBSTRUCTED = 1
};

// 문 및 라즈베리파이 명령 상태 변수
int door = 0;                          // 0: 닫힘, 1: 열림
int alertLevel = 0;                    // 0: 없음, 1: ALERT 1(LED), 2: ALERT 2(LED + 부저)
bool rpiAllowDoorOpen = true;          // 추가 상태 판단 플래그 (초기값 true)
String lastDoorCommand = "NONE";       // 최근 문 제어 명령 ("OPEN", "CLOSE", "HOLD", "NONE")

bool holdActive = false;               // HOLD 타이머 활성 여부
unsigned long holdDurationMs = DEFAULT_HOLD_MS;
unsigned long holdUntilMs = 0;         // HOLD 만료 시각 (millis 기준)
unsigned long lastStateReportMs = 0;   // 0.1초(100ms) 주기 STATE 송신 타이머
unsigned long lastDistanceMs = 0;      // 초음파 100ms 주기 측정 타이머

String rxLine = "";                    // 시리얼 수신 버퍼

void pollSerialCommands();

// [다음 단계 구현 예정] ALERT 1 / ALERT 2 상태에 따른 LED 및 부저 제어
void applyAlertState() {
  if (alertLevel == 1) {
    // TODO: LED 켜기
  } else if (alertLevel == 2) {
    // TODO: LED + 부저 켜기
  } else {
    // TODO: LED / 부저 끄기
  }
}

// TCRT5000 장애물 감지 여부 확인
bool isTcrtObstacleDetected() {
  return (digitalRead(tcrtDoPin) == OBSTACLE_DETECTED_STATE);
}

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

// 서보 이동 함수 (+ 장애물 감시)
// 반환값 true: 장애물 감지됨 / false: 정상 완료
// checkObstacle: true일 때만 이동 중 장애물 확인 (false: 역동작 중)
MoveResult moveServoWithObstacleCheck(int targetAngle, int stepDelayMs, bool checkObstacle) {
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

      // 이동 중 들어오는 명령을 놓치지 않도록 수신만 계속 처리 (서보 정지 없음)
      pollSerialCommands();

      if (checkObstacle && isTcrtObstacleDetected()) {
        return MOVE_OBSTRUCTED;
      }
    }
  }
  // 목표 각도가 5의 배수가 아닐 경우 최종 각도 보정 
  myservo.write(targetAngle);

  if (checkObstacle && isTcrtObstacleDetected()) {
    return MOVE_OBSTRUCTED;
  }
  return MOVE_COMPLETED;
}

void openDoor() {
  if (door == 1 && myservo.read() >= DOOR_OPEN_ANGLE - 2) {
    door = 1;
    return;
  }

  Serial.println("[서보] 문 열기 시작...");
  MoveResult res = moveServoWithObstacleCheck(DOOR_OPEN_ANGLE, 200, true);

  if (res == MOVE_OBSTRUCTED) {
    Serial.println("[경고] 문 여는 중 TCRT5000 장애물 감지! 역동작으로 다시 닫습니다.");
    moveServoWithObstacleCheck(DOOR_CLOSE_ANGLE, 100, false);
    door = 0;
    holdActive = false;
    lastDoorCommand = "NONE"; // 즉시 다시 열기를 반복하지 않도록 초기화
  } else {
    Serial.println("[완료] 문이 열렸습니다.");
    door = 1;

    // 문이 열리는 도중 HOLD가 미리 들어온 경우, 완전히 열린 시점부터 대기 시간 시작
    if (holdActive) {
      holdUntilMs = millis() + holdDurationMs;
    }
  }
}

void closeDoorWithAntiPinch() {
  if (door == 0 && myservo.read() <= DOOR_CLOSE_ANGLE + 2) {
    door = 0;
    return;
  }

  Serial.println("[서보] 문 닫기 시작...");
  MoveResult res = moveServoWithObstacleCheck(DOOR_CLOSE_ANGLE, 200, true);

  if (res == MOVE_OBSTRUCTED) {
    Serial.println("[경고] 문 닫는 중 TCRT5000 장애물 감지! 역동작으로 다시 엽니다.");
    moveServoWithObstacleCheck(DOOR_OPEN_ANGLE, 100, false);
    door = 1;

    holdDurationMs = DEFAULT_HOLD_MS;
    holdActive = true;
    holdUntilMs = millis() + holdDurationMs;
    lastDoorCommand = "HOLD";
  } else {
    Serial.println("[완료] 문이 닫혔습니다.");
    door = 0;
    holdActive = false;
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "OPEN") {
    lastDoorCommand = "OPEN";
    holdActive = false;
  } else if (cmd.startsWith("HOLD")) {
    // OPEN 이후 열린 상태를 유지(대기)하는 역할
    lastDoorCommand = "HOLD";
    holdDurationMs = DEFAULT_HOLD_MS;

    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx != -1) {
      // "HOLD 2000"으로 명령을 받았을 때 "2000"을 대기시간으로 설정 / "HOLD"만 올 경우 기본값 사용
      long parsedMs = cmd.substring(spaceIdx + 1).toInt();
      if (parsedMs > 0) {
        holdDurationMs = (unsigned long)parsedMs;
      }
    }
    holdActive = true;
    holdUntilMs = millis() + holdDurationMs;
  } else if (cmd == "CLOSE") {
    lastDoorCommand = "CLOSE";
    holdActive = false;
  } else if (cmd == "ALERT 0") {
    alertLevel = 0;
    applyAlertState();
  } else if (cmd == "ALERT 1") {
    alertLevel = 1;
    applyAlertState();
  } else if (cmd == "ALERT 2") {
    alertLevel = 2;
    applyAlertState();
  }
}

// 시리얼 수신 받기 
void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      // 명령어 수신 확인
      Serial.print("라즈베리파이 명령: ");
      Serial.println(rxLine);

      handleCommand(rxLine);
      rxLine = "";
    } else if (c != '\r') {
      if (rxLine.length() < 64) { // 우노 SRAM 보호용 길이 제한 (64bit 이상일 때 비우기)
        rxLine += c;
      } else {
        rxLine = "";
      }
    }
  }
}

// 문 상태 송신 
void sendDoorStateIfDue() {
  unsigned long now = millis();
  if (now - lastStateReportMs >= 100) { // 0.1초마다 한 줄씩 송신
    lastStateReportMs = now;
    int reed = (door == 0) ? 1 : 0;
    Serial.print("STATE angle=");
    Serial.print(myservo.read());
    Serial.print(" tof1=-1 tof2=-1 reed=");
    Serial.println(reed);
  }
}

void setup() {
  Serial.begin(115200);
  myservo.attach(servoPin, 500, 2400);
  myservo.write(DOOR_CLOSE_ANGLE);

  pinMode(hcTrigPin, OUTPUT);
  pinMode(hcEchoPin, INPUT);
  pinMode(tcrtDoPin, INPUT_PULLUP);

  Serial.println("Arduino Uno HandsFreeDoor 테스트 시작");
}

void loop() {
  // 1. 시리얼 명령 수신
  pollSerialCommands();

  // [추후 확장] 1단계: 레이더 센서 사람 감지 -> 웹캠 활성화 트리거 자리

  // 2단계: 초음파 센서로 일정 거리 유지 확인 (100ms 주기로 측정)
  if (millis() - lastDistanceMs >= 100) {
    lastDistanceMs = millis();
    long distance = measureDistanceCm();
    Serial.print("[초음파] 거리: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  // bool safeDistanceForOpen = (distance >= SAFE_OPEN_DISTANCE_CM);
  // if (distance >= 0 && distance < SAFE_OPEN_DISTANCE_CM) {
  //   Serial.println("[초음파] 50cm 이내 감지 -> 충돌 방지를 위해 문을 열지 않습니다.");
  // }

  // 3단계: HOLD 타이머 만료 시 새 명령이 없으면 보드가 스스로 닫기
  if (holdActive && door == 1 && (long)(millis() - holdUntilMs) >= 0) {
    holdActive = false;
    lastDoorCommand = "CLOSE";
    closeDoorWithAntiPinch();
  }

  // 4단계: 라즈베리파이 명령 + 초음파 거리 + 추가 상태 플래그를 종합해 문 제어
  if (lastDoorCommand == "CLOSE") {
    if (!holdActive && door == 1) {
      closeDoorWithAntiPinch();
    }
  } else if (lastDoorCommand == "OPEN") {
    if (!rpiAllowDoorOpen) {
      Serial.println("[상태] 추가 판단 조건 미충족으로 문을 열지 않습니다.");
    // } else if (!safeDistanceForOpen) {
    //   Serial.println("[판단] OPEN 명령 수신했으나 초음파 조건(50cm 이상) 미충족으로 대기합니다.");
    } else if (door == 0) {
      openDoor();
    }
  }

  // 5단계: 0.1초 주기 문 상태 송신 (라즈베리파이 연동 시 필요에 따라 활용)
  // sendDoorStateIfDue();
  // delay(100);
}