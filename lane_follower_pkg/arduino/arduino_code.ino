/*
 * ROS 자율주행 차량용 아두이노 제어 코드
 * RS390SP 조향 모터 + RS550PH 구동 모터
 * L298N 듀얼 모터 드라이버 사용
 * 가변저항(A0)으로 조향 각도 피드백
 */

#include <ros.h>
#include <std_msgs/Float32.h>

// 조향 모터 핀 정의
const int steeringPotPin = A0;   // 가변 저항 (조향각 피드백)
const int motorIN3 = 7;
const int motorIN4 = 8;
const int motorENB = 10;

// 구동 모터 핀 정의
const int driveIN1 = 5;
const int driveIN2 = 6;
const int driveENA = 9;

int potValue = 0;
int centerPosition = 512;       // 중립 위치 (가변저항 기준)
int deadband = 20;              // 디바이스 허용 오차 범위
float steeringValue = 0.0;
float driveValue = 0.0;

ros::NodeHandle nh;

std_msgs::Float32 steering_feedback_msg;
ros::Publisher steering_feedback_pub("steering_feedback", &steering_feedback_msg);

void steeringCallback(const std_msgs::Float32& msg) {
  steeringValue = msg.data;
  controlSteering(steeringValue);
}

ros::Subscriber<std_msgs::Float32> sub("input_steering", steeringCallback);

void driveCallback(const std_msgs::Float32& msg) {
  driveValue = msg.data;
  controlDriving(driveValue);
}

ros::Subscriber<std_msgs::Float32> drive_sub("drive_command", driveCallback);

void setup() {
  // 시리얼 통신 시작
  Serial.begin(57600);
  while (!Serial) {}

  delay(1000); // 안정화 대기

  // 모터 드라이버 핀 설정
  pinMode(motorIN3, OUTPUT);
  pinMode(motorIN4, OUTPUT);
  pinMode(motorENB, OUTPUT);

  pinMode(driveIN1, OUTPUT);
  pinMode(driveIN2, OUTPUT);
  pinMode(driveENA, OUTPUT);

  digitalWrite(motorIN3, LOW);
  digitalWrite(motorIN4, LOW);
  analogWrite(motorENB, 0);

  controlDriving(0.0); // 기본값: 정지

  // ROS 노드 초기화
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(drive_sub);
  nh.advertise(steering_feedback_pub);

  centerPosition = analogRead(steeringPotPin); // 중립값 캘리브레이션
  Serial.println("Arduino initialized!");
}

void loop() {
  // 조향 각도 피드백 읽기
  potValue = analogRead(steeringPotPin);

  // 정규화된 조향 위치 계산 (-1.0 ~ 1.0)
  float normalizedPosition = (float)(potValue - centerPosition) / 512.0;
  normalizedPosition = constrain(normalizedPosition, -1.0, 1.0);

  // 조향 피드백 발행
  steering_feedback_msg.data = normalizedPosition;
  steering_feedback_pub.publish(&steering_feedback_msg);

  // ROS 통신 처리
  nh.spinOnce();

  // 주기 조절
  delay(10);
}

/**
 * 조향 모터 제어 함수
 * @param value -1.0(좌) ~ 0.0 ~ 1.0(우)
 */
void controlSteering(float value) {
  int speed = abs(value) * 255;
  if (value > 0.05) {
    steerRight(speed);
  } else if (value < -0.05) {
    steerLeft(speed);
  } else {
    steeringStop();
  }
}

/**
 * 구동 모터 제어 함수
 * @param value -1.0(후진) ~ 0.0(정지) ~ 1.0(전진)
 */
void controlDriving(float value) {
  int speed = abs(value) * 255;
  if (value > 0.1) {
    digitalWrite(driveIN1, HIGH);
    digitalWrite(driveIN2, LOW);
    analogWrite(driveENA, speed);
  } else if (value < -0.1) {
    digitalWrite(driveIN1, LOW);
    digitalWrite(driveIN2, HIGH);
    analogWrite(driveENA, speed);
  } else {
    digitalWrite(driveIN1, LOW);
    digitalWrite(driveIN2, LOW);
    analogWrite(driveENA, 0);
  }
}

/**
 * 조향 모터 우회전
 * @param speed 0~255
 */
void steerRight(int speed) {
  digitalWrite(motorIN3, HIGH);
  digitalWrite(motorIN4, LOW);
  analogWrite(motorENB, speed);
}

/**
 * 조향 모터 좌회전
 * @param speed 0~255
 */
void steerLeft(int speed) {
  digitalWrite(motorIN3, LOW);
  digitalWrite(motorIN4, HIGH);
  analogWrite(motorENB, speed);
}

/**
 * 조향 모터 정지
 */
void steeringStop() {
  digitalWrite(motorIN3, LOW);
  digitalWrite(motorIN4, LOW);
  analogWrite(motorENB, 0);
}