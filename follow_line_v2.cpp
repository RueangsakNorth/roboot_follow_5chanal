// มอเตอร์ L298N
#define IN1 5
#define IN2 4
#define IN3 19
#define IN4 18
#define enA 26 // PWM ซ้าย
#define enB 27 // PWM ขวา
// PWM channels
#define CH_A 0
#define CH_B 1

// เซ็นเซอร์ 5 ตัว
int sensorPins[5] = {32, 33, 34, 35, 25};
int position[5] = {-2, -1, 0, 1, 2}; // ตำแหน่งสัมพัทธ์
int set_point = 0;                   // จุดกึ่งกลาง

// ตัวแปร PID และการติดตามเส้น
int s[5], total, sensor_position;
float avg = 0;
int threshold = 2000; // ✅ แก้ไขค่าที่ผิด (เดิมพิมพ์เป็นตัวอักษรภาษาไทยผิด)

// ไม่มีปุ่มแล้ว
int led = 13;

void setup()
{
  Serial.begin(115200);

  // ตั้งค่า GPIO
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(led, OUTPUT);

  // PWM สำหรับ ESP32
  ledcSetup(CH_A, 1000, 8);
  ledcAttachPin(enA, CH_A);
  ledcSetup(CH_B, 1000, 8);
  ledcAttachPin(enB, CH_B);

  Serial.println
      delay(1000);
}

void loop()
{
  display_value();   // ✅ เพิ่ม debug print
  PID_LINE_FOLLOW(); // ทำงานทันที
}

void Sensor_reading()
{
  sensor_position = 0;
  total = 0;

  for (int i = 0; i < 5; i++)
  {
    int val = analogRead(sensorPins[i]);
    if (val < threshold)
      s[i] = 0;
    else
      s[i] = 1;

    sensor_position += s[i] * position[i];
    total += s[i];
  }

  if (total > 0)
    avg = (float)sensor_position / total;
  else
    avg = set_point;
}

void PID_LINE_FOLLOW()
{
  float kp = 35, kd = 350;
  float error, previous_error = 0, P, D, PID_Value;
  int base_speed = 100;
  int left_motor_speed, right_motor_speed;
  char t = 's';

  while (true)
  {
    Sensor_reading();
    error = set_point - avg;
    P = kp * error;
    D = kd * (error - previous_error);
    PID_Value = P + D;
    previous_error = error;

    left_motor_speed = base_speed + PID_Value;
    right_motor_speed = base_speed - PID_Value;

    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    motor(left_motor_speed, right_motor_speed);

    if (total == 0)
    {
      digitalWrite(led, HIGH);
      if (t != 's')
      {
        if (t == 'r')
          motor(-150, -150);
        else
          motor(-150, 150);
        while (!s[2])
          Sensor_reading();
        digitalWrite(led, LOW);
      }
    }

    if (s[4] == 1 && s[0] == 0)
      t = 'l';
    if (s[0] == 1 && s[4] == 0)
      t = 'r';

    if (total == 5)
    {
      Sensor_reading();
      if (total == 5)
      {
        motor(0, 0);
        while (total == 5)
          Sensor_reading();
      }
      else if (total == 0)
        t = 's';
    }
  }
}

void display_value()
{
  Serial.print("Raw: ");
  for (int i = 0; i < 5; i++)
  {
    int val = analogRead(sensorPins[i]);
    Serial.print(val);
    Serial.print(" ");
  }

  Serial.print("  Bin: ");
  for (int i = 0; i < 5; i++)
  {
    Serial.print(s[i]);
  }

  Serial.print("  avg = ");
  Serial.println(avg, 2);

  delay(50);
}

void motor(int a, int b)
{
  if (a >= 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    a = -a;
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  if (b >= 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    b = -b;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  a = constrain(a, 0, 255);
  b = constrain(b, 0, 255);

  ledcWrite(CH_A, a); // PWM ซ้าย
  ledcWrite(CH_B, b); // PWM ขวา
}
