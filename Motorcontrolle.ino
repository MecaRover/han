// 모터 제어 핀 설정
const int leftMotorPWM1 = 3;
const int leftMotorDir1 = 2;
const int rightMotorPWM1 = 8;
const int rightMotorDir1 = 9;

const int leftMotorPWM2 = 5;
const int leftMotorDir2 = 4;
const int rightMotorPWM2 = 10;
const int rightMotorDir2 = 11;

const int leftMotorPWM3 = 7;
const int leftMotorDir3 = 6;
const int rightMotorPWM3 = 12;
const int rightMotorDir3 = 13;

// IR 센서 핀 설정
int IRpin1 = A0;  // 왼쪽 IR 센서 핀
int IRpin2 = A1;  // 오른쪽 IR 센서 핀

const unsigned long irDelay = 100; // IR 센서 값 읽기 및 출력 딜레이 (200ms)
unsigned long previousIRMillis = 0; // 마지막 IR 센서 읽기 시간

const unsigned long serialDelay = 0; // 시리얼 데이터 처리 딜레이 (100ms)
unsigned long previousSerialMillis = 0; // 마지막 시리얼 데이터 처리 시간

// 장애물 회피 관련 상수
const float obstacleDistanceThreshold = 50; // 장애물 감지 거리 (30 cm)
const unsigned long obstacleDetectionTime = 1000; // 장애물 감지 유지 시간 (2초)
const unsigned long turnDuration = 1500;      // 회전 시간 (2초)
const unsigned long moveDuration = 000;      // 직진 시간 (1초)
const float stopDistanceThreshold = 40; // 정지 거리 임계값 (50 cm)

// 장애물 감지 상태
bool isAvoidingObstacle = false;
unsigned long obstacleDetectionStartTime = 0;
bool obstacleDetected = false;

void setup() {
  // 시리얼 통신 시작
  Serial.begin(115200);
  
  // 모터 제어 핀을 출력으로 설정
  pinMode(leftMotorPWM1, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(rightMotorPWM1, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  
  pinMode(leftMotorPWM2, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorPWM2, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
  
  pinMode(leftMotorPWM3, OUTPUT);
  pinMode(leftMotorDir3, OUTPUT);
  pinMode(rightMotorPWM3, OUTPUT);
  pinMode(rightMotorDir3, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis(); // 현재 시간 가져오기

  // IR 센서 값 읽기 및 출력
  if (currentMillis - previousIRMillis >= irDelay) {
    previousIRMillis = currentMillis; // 마지막 IR 센서 읽기 시간 업데이트
    readAndPrintIRDistances();
  }

  // 시리얼 데이터가 있는지 확인
  if (currentMillis - previousSerialMillis >= serialDelay) {
    previousSerialMillis = currentMillis; // 마지막 시리얼 데이터 처리 시간 업데이트

    if (Serial.available() > 0) {
      // 시리얼 데이터 읽기
      String data = Serial.readStringUntil('\n');
      // 데이터 파싱
      parseAndSetMotorSpeeds(data);
    }
  }

  // 장애물 회피 로직
  if (!isAvoidingObstacle) {
    float distance1 = getIRDistance(IRpin1); // 왼쪽 IR 센서
    float distance2 = getIRDistance(IRpin2); // 오른쪽 IR 센서

    // 두 센서 모두 50cm 이하로 감지될 경우 정지
    if (distance1 < stopDistanceThreshold && distance2 < stopDistanceThreshold) {
      stopMotors();
      return; // 루프를 빠져나가서 더 이상의 동작을 수행하지 않음
    }

    if (distance1 < obstacleDistanceThreshold || distance2 < obstacleDistanceThreshold) {
      if (!obstacleDetected) {
        obstacleDetected = true;
        obstacleDetectionStartTime = currentMillis;
      } else if (currentMillis - obstacleDetectionStartTime >= obstacleDetectionTime) {
        isAvoidingObstacle = true;
        obstacleDetectionStartTime = currentMillis; // 회피 동작 시작 시간 업데이트

        if (distance1 < obstacleDistanceThreshold) {
          rotateRight(); // 1번 센서 감지: 오른쪽 회전
        } else if (distance2 < obstacleDistanceThreshold) {
          rotateLeft(); // 2번 센서 감지: 왼쪽 회전
        }
      }
    } else {
      obstacleDetected = false;
    }
  } else {
    unsigned long elapsedTime = currentMillis - obstacleDetectionStartTime;

    if (elapsedTime < turnDuration) {
      float distance1 = getIRDistance(IRpin1); // 다시 IR 센서 값을 읽기
      float distance2 = getIRDistance(IRpin2); // 다시 IR 센서 값을 읽기

      if (distance2 < obstacleDistanceThreshold) {
        rotateRight(); // 1번 센서 감지 시 오른쪽 회전
      } else if (distance1 < obstacleDistanceThreshold) {
        rotateLeft(); // 2번 센서 감지 시 왼쪽 회전
      }
    } else if (elapsedTime < turnDuration + moveDuration) {
      moveForward();
    } else {
      stopMotors();
      isAvoidingObstacle = false;
    }
  }
}

void readAndPrintIRDistances() {
  float distance1 = getIRDistance(IRpin1);
  float distance2 = getIRDistance(IRpin2);
  
  Serial.print("왼쪽 Distance: ");
  Serial.print(distance1);
  Serial.print(" cm, 오른쪽 Distance: ");
  Serial.print(distance2);
  Serial.println(" cm");
}

void parseAndSetMotorSpeeds(String data) {
  int startIndex = 0;
  int commaIndex = data.indexOf(',');

  float speeds[6];
  for (int i = 0; i < 6; i++) {
    if (commaIndex == -1) {
      speeds[i] = atof(data.substring(startIndex).c_str());
      break;
    }
    speeds[i] = atof(data.substring(startIndex, commaIndex).c_str());
    startIndex = commaIndex + 1;
    commaIndex = data.indexOf(',', startIndex);
  }

  setMotorSpeed(leftMotorPWM1, leftMotorDir1, speeds[0]);
  setMotorSpeed(rightMotorPWM1, rightMotorDir1, speeds[1]);
  setMotorSpeed(leftMotorPWM2, leftMotorDir2, speeds[2]);
  setMotorSpeed(rightMotorPWM2, rightMotorDir2, speeds[3]);
  setMotorSpeed(leftMotorPWM3, leftMotorDir3, speeds[4]);
  setMotorSpeed(rightMotorPWM3, rightMotorDir3, speeds[5]);
}

void setMotorSpeed(int pwmPin, int dirPin, float speed) {
  int maxPWMValue = isAvoidingObstacle ? 100 : 200; // 회피 주행 시 30, 일반 주행 시 100
  
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);  // 전진
  } else {
    digitalWrite(dirPin, LOW);   // 후진
    speed = -speed;              
  }
  
  int pwmValue = int(speed * maxPWMValue);
  pwmValue = constrain(pwmValue, 0, maxPWMValue);
  analogWrite(pwmPin, pwmValue);
}


void stopMotors() {
  analogWrite(leftMotorPWM1, 0);
  analogWrite(rightMotorPWM1, 0);
  analogWrite(leftMotorPWM2, 0);
  analogWrite(rightMotorPWM2, 0);
  analogWrite(leftMotorPWM3, 0);
  analogWrite(rightMotorPWM3, 0);

  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(leftMotorDir2, LOW);
  digitalWrite(rightMotorDir2, LOW);
  digitalWrite(leftMotorDir3, LOW);
  digitalWrite(rightMotorDir3, LOW);

  Serial.println("Emergency Stop!");
}

float getIRDistance(int pin) {
  int sensorValue = analogRead(pin);  // 센서 값 읽기

  // 센서 값이 유효한지 확인
  if (sensorValue > 0) {
    float distance = (67870.0 / (sensorValue - 4.0)) - 4.0;  // 거리 계산 (cm)
    
    // 새로운 보정 계수 적용
    float correctionFactor = 0.28; // 보정 계수
    distance *= correctionFactor;

    return distance;
  } else {
    Serial.println("Sensor not connected or invalid value.");
    return -1;  // 유효하지 않은 값 리턴
  }
}

void rotateRight() {
  setMotorSpeed(leftMotorPWM1, leftMotorDir1, 100);
  setMotorSpeed(rightMotorPWM1, rightMotorDir1, -100);
  setMotorSpeed(leftMotorPWM2, leftMotorDir2, 100);
  setMotorSpeed(rightMotorPWM2, rightMotorDir2, -100);
  setMotorSpeed(leftMotorPWM3, leftMotorDir3, 100);
  setMotorSpeed(rightMotorPWM3, rightMotorDir3, -100);
}

void rotateLeft() {
  setMotorSpeed(leftMotorPWM1, leftMotorDir1, -100);
  setMotorSpeed(rightMotorPWM1, rightMotorDir1, 100);
  setMotorSpeed(leftMotorPWM2, leftMotorDir2, -100);
  setMotorSpeed(rightMotorPWM2, rightMotorDir2, 100);
  setMotorSpeed(leftMotorPWM3, leftMotorDir3, -100);
  setMotorSpeed(rightMotorPWM3, rightMotorDir3, 100);
}

void moveForward() {
  setMotorSpeed(leftMotorPWM1, leftMotorDir1, 100);
  setMotorSpeed(rightMotorPWM1, rightMotorDir1, 100);
  setMotorSpeed(leftMotorPWM2, leftMotorDir2, 100);
  setMotorSpeed(rightMotorPWM2, rightMotorDir2, 100);
  setMotorSpeed(leftMotorPWM3, leftMotorDir3, 100);
  setMotorSpeed(rightMotorPWM3, rightMotorDir3, 100);
}
