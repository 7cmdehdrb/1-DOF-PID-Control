#include <Servo.h>
#include "HX711.h"


// Object Define

HX711 scale(A1, A0);  // Loadcell
Servo myservo;  // Servo Motor

// Pin Define

const int servo_pin = 6;  // Servo Motor Pin
const int Dir1Pin_A = 2;  // DC Motor Pin 2
const int Dir2Pin_A = 3;  // DC Motor Pin 2
const int SpeedPin_A = 11;  // DC Motor PWM

// Serial Messafe Define

const int BUFFER_SIZE = 5;
char buf[BUFFER_SIZE];


int pos = 120;    // variable to store the servo position
const int maxDC = 120;
int dc = maxDC;

void loadCellInitialize() {
  scale.set_scale(2280.f);
  scale.tare(); // reset the scale to 0
}


float getLoadCellData() {
  return scale.get_units();
}

int readSerial() {
  /*
      Code

     990: Start DC Motor
     991: Stop DC Motor
     992: Reset Loadcell
     993: Reset Servo Motor
  */

  int rlen = Serial.readBytesUntil('\n', buf, BUFFER_SIZE);
  int value = 0;

  // Char Array to Int
  for (int i = 0; i < rlen; i++) {
    int temp = int(buf[i]) - int('0');
    value += temp * ceil(pow(10, rlen - i - 1));
  }

  return value;
}


void setup() {
  Serial.begin(9600);
  loadCellInitialize();

  pinMode(Dir1Pin_A, OUTPUT);             // 제어 1번핀 출력모드 설정
  pinMode(Dir2Pin_A, OUTPUT);             // 제어 2번핀 출력모드 설정
  pinMode(SpeedPin_A, OUTPUT);            // PWM제어핀 출력모드 설정

  digitalWrite(Dir1Pin_A, HIGH);         //모터가 시계 방향으로 회전
  digitalWrite(Dir2Pin_A, LOW);

  myservo.attach(servo_pin);
}

void loop() {
  if (Serial.available()) {
    int temp = readSerial();

    if (temp == 990) {
      // 990: Start DC Motor
      //      Serial.println("START DC");
      //      dc = maxDC;
    }
    else if (temp == 991) {
      // 991: Stop DC Motor
      //      Serial.println("STOP DC");
      //      dc = 0;
    }
    else if (temp == 992) {
      // 992: Reset Loadcell
      Serial.println("RESET LOADCELL");
      loadCellInitialize();
      pos = 120;
    }
    else if (temp == 993) {
      // 993: Reset Servo Motor
      Serial.println("RESET SERVO");
      pos = 120;
    }
    else {
      pos = readSerial();
    }
  }

  myservo.write(pos);

  analogWrite(SpeedPin_A, 150);

  float data = getLoadCellData();
  Serial.println(String(data));

  //  delay(10);

}
