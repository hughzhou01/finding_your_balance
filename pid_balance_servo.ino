#include <Wire.h>
#include <Servo.h>

int Analog_in = A0;
Servo myservo;

const int STABLE_THRESHOLD = 5; // 例如连续5次循环
int stableCount = 0;

float distance = 0.0;
float elapsedTime, time, timePrev;
float distance_previous_error, distance_error;
int period = 50;

float kp=6;
float ki=0.2;
float kd=3300;
float distance_setpoint = 14;
float PID_p, PID_i, PID_d, PID_total;

#define WINDOW_SIZE 5
int medianWindow[WINDOW_SIZE];

void setup() {
  Serial.begin(9600);  
  myservo.attach(9); 
  myservo.write(90);
  pinMode(Analog_in, INPUT);  
  delay(2000);
  time = millis();
}

void loop() {
  if (millis() > time + period) {
    time = millis();    
    distance = readBallPosition();
    distance_error = distance_setpoint - distance;

    // 检查连续满足条件的次数
    if (abs(distance_error) < 1) {
      stableCount++;
    } else {
      stableCount = 0; // 重置计数器
    }

    if (stableCount >= STABLE_THRESHOLD) {
      // 如果连续满足条件，不做任何调整
      Serial.print(PID_total);
      Serial.print(" || ");
      Serial.println("Stable for a while. No adjustment needed.");
      // PID_total = 90;
      // myservo.write(PID_total);
      return;
    }

    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd * ((distance_error - distance_previous_error) / period);
      
    if(-4 < distance_error && distance_error <4) {
      PID_i = PID_i + (ki * distance_error);
    } else {
      PID_i = 0;
    }

    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -180, 180, 140, 30);
  
    if(PID_total < 30) {PID_total = 35;}
    if(PID_total > 140) {PID_total = 140;}

    myservo.write(PID_total);
    Serial.print("PID Value = ");
    Serial.print(PID_total);
    Serial.print(" || ");
    Serial.print("Distance = ");
    Serial.println(distance);  
    distance_previous_error = distance_error;
  }
}

double linearInterpolation(double y, double x1, double y1, double x2, double y2) {
  return x1 + (y - y1) * (x2 - x1) / (y2 - y1);
}

double readBallPosition() {
    int rawReading = analogRead(Analog_in);

    // 使用中位数滤波器
    int filteredReading = medianFilter(rawReading);
    
    double Vo = (5.0 * filteredReading) / 1024.0;
    return getValue(Vo);
}

int medianFilter(int rawValue) {
    medianWindow[2] = medianWindow[1];
    medianWindow[1] = medianWindow[0];
    medianWindow[0] = rawValue;

    int sorted[WINDOW_SIZE] = {medianWindow[0], medianWindow[1], medianWindow[2]};

    // 对数组进行排序
    for(int i = 0; i < WINDOW_SIZE - 1; i++) {
        for(int j = i + 1; j < WINDOW_SIZE; j++) {
            if(sorted[j] < sorted[i]) {
                int temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }
    // 返回中位数
    return sorted[WINDOW_SIZE / 2];
}

double getValue(double y) {
    if (y > 2.75) return 0;
    else if (y <= 2.75 && y > 2.35) return linearInterpolation(y, 4, 2.75, 5, 2.35);
    else if (y <= 2.35 && y > 2.02) return linearInterpolation(y, 5, 2.35, 6, 2.02);
    else if (y <= 2.02 && y > 1.77) return linearInterpolation(y, 6, 2.02, 7, 1.77);
    else if (y <= 1.77 && y > 1.56) return linearInterpolation(y, 7, 1.77, 8, 1.56);
    else if (y <= 1.56 && y > 1.4) return linearInterpolation(y, 8, 1.56, 9, 1.4);
    else if (y <= 1.4 && y > 1.26) return linearInterpolation(y, 9, 1.4, 10, 1.26);
    else if (y <= 1.26 && y > 1.06) return linearInterpolation(y, 10, 1.26, 12, 1.06);
    else if (y <= 1.06 && y > 0.92) return linearInterpolation(y, 12, 1.06, 14, 0.92);
    else if (y <= 0.92 && y > 0.81) return linearInterpolation(y, 14, 0.92, 16, 0.81);
    else if (y <= 0.81 && y > 0.75) return linearInterpolation(y, 16, 0.81, 18, 0.75);
    else if (y <= 0.75 && y > 0.65) return linearInterpolation(y, 18, 0.75, 20, 0.65);
    else if (y <= 0.65 && y > 0.52) return linearInterpolation(y, 20, 0.65, 25, 0.52);
    else if (y <= 0.53 && y > 0.44) return linearInterpolation(y, 25, 0.53, 30, 0.44);
    else if (y <= 0.44) return 30;  // y值小于0.44时，返回x=30
}

