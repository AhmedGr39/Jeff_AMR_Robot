#include <WiFi.h>
#include <WebServer.h>

// === Wi-Fi AP Settings ===
const char* ssid = "RobotCar";
const char* password = "123456789";
WebServer server(80);

// === Motor Pins ===
#define IN1 26  // Left forward
#define IN2 27  // Left reverse
#define IN3 18  // Right forward
#define IN4 19  // Right reverse

// === Encoder Pins ===
#define LEFT_ENC_A  4
#define RIGHT_ENC_A 33

volatile long leftTicks = 0;
volatile long rightTicks = 0;

#define TICKS_PER_REV 104
int baseSpeed = 130;

// PID parameters (modifiable via serial)
float kp = 1.0, ki = 0.01, kd = 0.6;
float error = 0, prev_error = 0, integral = 0;

// === Encoder ISRs ===
void IRAM_ATTR leftEncoderISR()  { leftTicks++; }
void IRAM_ATTR rightEncoderISR() { rightTicks++; }

// === Motor Control ===
void moveMotors(int left_pwm, int right_pwm) {
  ledcWrite(0, left_pwm > 0 ? left_pwm : 0);
  ledcWrite(1, left_pwm < 0 ? -left_pwm : 0);
  ledcWrite(2, right_pwm > 0 ? right_pwm : 0);
  ledcWrite(3, right_pwm < 0 ? -right_pwm : 0);
}

void resetEncoders() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

// === PID Motion (1 rev) ===
void moveWithPID(bool forward = true) {
  resetEncoders();
  prev_error = 0;
  integral = 0;

  Serial.println("🚗 Starting PID-controlled motion...");

  while ((leftTicks + rightTicks) / 2 < TICKS_PER_REV) {
    long l, r;
    noInterrupts();
    l = leftTicks;
    r = rightTicks;
    interrupts();

    error = l - r;
    integral += error;
    float derivative = error - prev_error;

    float correction = kp * error + ki * integral + kd * derivative;
    int L_PWM = baseSpeed - correction;
    int R_PWM = baseSpeed + correction;

    L_PWM = constrain(L_PWM, 0, 255);
    R_PWM = constrain(R_PWM, 0, 255);

    if (!forward) {
      L_PWM = -L_PWM;
      R_PWM = -R_PWM;
    }

    moveMotors(L_PWM, R_PWM);
    prev_error = error;

    Serial.print("L: "); Serial.print(l);
    Serial.print(" | R: "); Serial.print(r);
    Serial.print(" | L_PWM: "); Serial.print(L_PWM);
    Serial.print(" | R_PWM: "); Serial.print(R_PWM);
    Serial.print(" | Err: "); Serial.print(error);
    Serial.print(" | Kp: "); Serial.print(kp);
    Serial.print(" | Ki: "); Serial.print(ki);
    Serial.print(" | Kd: "); Serial.println(kd);

    delay(20);
  }

  moveMotors(0, 0);
  Serial.println("✅ Motion complete");
}

// === Web Handlers ===
void handleRoot() {
  server.send(200, "text/html", R"html(
    <!DOCTYPE html>
    <html lang="en">
      <head>
        <meta charset="UTF-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
        <title>ESP32 Robot Car</title>
        <style>
          body {
            background: #111;
            color: #fff;
            text-align: center;
            font-family: 'Segoe UI', Arial, sans-serif;
            margin: 0;
            padding: 20px;
          }
          h1 {
            font-size: 1.8em;
            margin-bottom: 20px;
          }
          button {
            background: #444;
            color: white;
            border: none;
            padding: 15px 25px;
            margin: 5px;
            font-size: 1.2em;
            border-radius: 8px;
            cursor: pointer;
            transition: background 0.3s;
          }
          button:hover {
            background: #666;
          }
          button:active {
            background: #888;
          }
          .controls {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            grid-template-rows: repeat(3, 1fr);
            gap: 10px;
            max-width: 300px;
            margin: 20px auto;
          }
          .center { grid-area: 2 / 2; }
          .top    { grid-area: 1 / 2; }
          .left   { grid-area: 2 / 1; }
          .right  { grid-area: 2 / 3; }
          .bottom { grid-area: 3 / 2; }
        </style>
      </head>
      <body>
        <h1>ESP32 Robot Car</h1>
        <div class="controls">
          <button class="top"    onclick="send('/forward')">Forward</button>
          <button class="left"   onclick="send('/left')">Left</button>
          <button class="center" onclick="send('/stop')">Stop</button>
          <button class="right"  onclick="send('/right')">Right</button>
          <button class="bottom" onclick="send('/backward')">Backward</button>
        </div>
        <script>
          function send(cmd) {
            fetch(cmd)
              .then(res => console.log('Sent: ' + cmd))
              .catch(err => console.error('Error: ', err));
          }
        </script>
      </body>
    </html>
  )html");
}

void handleForward()  { moveWithPID(true);  server.send(200, "text/plain", "Forward 1 rev"); }
void handleBackward() { moveWithPID(false); server.send(200, "text/plain", "Backward 1 rev"); }

void handleLeft() {
  resetEncoders();
  moveMotors(-baseSpeed, baseSpeed);
  delay(200); // Adjust turn time
  moveMotors(0, 0);
  server.send(200, "text/plain", "Left Turn");
}

void handleRight() {
  resetEncoders();
  moveMotors(baseSpeed, -baseSpeed);
  delay(200); // Adjust turn time
  moveMotors(0, 0);
  server.send(200, "text/plain", "Right Turn");
}

void handleStop() {
  moveMotors(0, 0);
  server.send(200, "text/plain", "Stopped");
}

// === Setup ===
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcSetup(0, 100, 8); ledcAttachPin(IN1, 0);
  ledcSetup(1, 100, 8); ledcAttachPin(IN2, 1);
  ledcSetup(2, 100, 8); ledcAttachPin(IN3, 2);
  ledcSetup(3, 100, 8); ledcAttachPin(IN4, 3);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/forward", handleForward);
  server.on("/backward", handleBackward);
  server.on("/left", handleLeft);
  server.on("/right", handleRight);
  server.on("/stop", handleStop);
  server.begin();
}

// === Loop for PID tuning via Serial ===
void loop() {
  server.handleClient();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("kp=")) kp = input.substring(3).toFloat();
    else if (input.startsWith("ki=")) ki = input.substring(3).toFloat();
    else if (input.startsWith("kd=")) kd = input.substring(3).toFloat();
    else if (input.startsWith("spd=")) baseSpeed = input.substring(4).toInt();

    Serial.print("✅ Updated → Kp: "); Serial.print(kp);
    Serial.print(" | Ki: "); Serial.print(ki);
    Serial.print(" | Kd: "); Serial.print(kd);
    Serial.print(" | Base Speed: "); Serial.println(baseSpeed);
  }
}
