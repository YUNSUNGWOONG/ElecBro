#include <Servo.h>

Servo myservo;              // Servo 객체 생성
int currentAngle = 0;       // 서보모터의 현재 각도

const int rotationTimePerDegree = 5; // 1도 회전에 필요한 시간 (ms)

void setup() {
  Serial.begin(9600);       // 시리얼 모니터 통신 시작
  myservo.attach(10);       // 서보모터 제어 핀 설정
  myservo.write(90);        // 초기화 시 중립 위치 설정
  currentAngle = 0;         // 초기 각도 설정
  delay(500);               // 신호 안정화를 위해 초기 대기 시간
  Serial.println("Type 'R' to rotate the servo by 30 degrees.");
}

// 목표 위치로 회전하는 데 필요한 시간을 계산
int getRotationTime(int targetAngle) {
  int rotationTime = abs(targetAngle - currentAngle) * rotationTimePerDegree;
  return rotationTime;
}

// 부드러운 서보모터 이동을 위한 함수
void moveServo(int targetAngle) {
  if (targetAngle > 180) targetAngle = 180; // 각도 제한
  if (targetAngle < 0) targetAngle = 0;     // 각도 제한
  
  int step = (targetAngle > currentAngle) ? 1 : -1; // 증가 또는 감소 설정
  
  for (int angle = currentAngle; angle != targetAngle; angle += step) {
    myservo.write(angle);          // 한 단계씩 이동
    delay(rotationTimePerDegree);  // 각 단계마다 대기
  }
  currentAngle = targetAngle;      // 현재 각도 업데이트
}

void processInput(char input) {
  if (input == 'R' || input == 'r') { // 'R' 또는 'r' 입력 확인
    int targetAngle = currentAngle + 30; // 목표 각도 계산

    if (targetAngle > 180) { // 각도 제한 (180도를 초과하지 않도록 설정)
      Serial.println("Cannot rotate further. Maximum angle is 180 degrees.");
      return;
    }

    Serial.print("Rotating to: ");
    Serial.print(targetAngle);
    Serial.println(" degrees");

    moveServo(targetAngle); // 부드러운 이동

    Serial.print("Current Angle: ");
    Serial.println(currentAngle);
  } else {
    Serial.println("Invalid input. Type 'R' to rotate.");
  }
}

void loop() {
  if (Serial.available() > 0) { // 시리얼 입력이 있는지 확인
    char input = Serial.read(); // 입력된 문자 읽기
    processInput(input);        // 입력 처리
  }
}
