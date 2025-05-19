#include <Stepper.h>
#include <math.h>
#include <FastLED.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "LED.h"
#include "MoveSensor.h"
#include "PressureSensor.h"

// 步进电机参数
#define STEPS_PER_REVOLUTION 2048
#define IN1 27
#define IN2 14
#define IN3 12
#define IN4 13

// 副板串口连接引脚（用于 Serial2）
#define TXD2 17  // 接副板的 RX 引脚
#define RXD2 16  // 接副板的 TX 引脚（即使暂时不接收，也需要定义）

Stepper stepper(STEPS_PER_REVOLUTION, IN1, IN2, IN3, IN4);

// 参与者角度定义
const int PARTICIPANT_1_START = 0;
const int PARTICIPANT_1_END   = 119;
const int PARTICIPANT_2_START = 120;
const int PARTICIPANT_2_END   = 239;
const int PARTICIPANT_3_START = 240;
const int PARTICIPANT_3_END   = 359;

// ----------- LED 模块用到的实际变量 ------------
CRGB leds1[LEDS_PER_STRIP];
CRGB leds2[LEDS_PER_STRIP];
CRGB leds3[LEDS_PER_STRIP];

int activeStrip = -1;
int activeLedCount = LEDS_PER_STRIP;
unsigned long currentDelay = 10000;
unsigned long timePerLed;
unsigned long countdownStartTime = 0;
unsigned long lastCountdownUpdate = 0;

// ----------- MPU6050 用到的实际变量 ------------
MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

unsigned long lastSuccess = 0;
unsigned long lastReconnectAttempt = 0;
int failedReadCount = 0;

// 状态变量
int previousParticipant = 0;
int roundNumber = 1;
long currentSteps = 0;
int currentAngle = 0;
int chosenParticipant = 0;
bool running = true;
bool motorRunning = false;
bool systemJustRestarted = false;
bool startedFirstRound = false;

// 调用模块函数原型
void selectParticipant();
int getCurrentParticipant();
void moveToAngle(int targetAngle);
void restartSystem();
void stopAndReset();
unsigned long adjustDelay();


// 选取参与者：
// 第一轮：全范围随机生成目标角度；后续轮次：在排除上一轮选中者的两个区间中随机生成目标角度
void selectParticipant() {
  int targetAngle;
  
  // 设置电机运行标志，关闭所有LED
  motorRunning = true;
  resetAllStrips();
  FastLED.show();
  
  if (roundNumber == 1) {
    targetAngle = random(0, 360);
    // Serial.print("Target random angle (full range): ");
    // Serial.println(targetAngle);
  } else {
    int allowedParticipants[2];
    int idx = 0;
    for (int i = 1; i <= 3; i++) {
      if (i != previousParticipant) {
        allowedParticipants[idx] = i;
        idx++;
      }
    }
    chosenParticipant = allowedParticipants[random(0, 2)];  // 在允许的两个中随机选取
    Serial.print("Allowed participants: ");
    Serial.print(allowedParticipants[0]);
    Serial.print(" and ");
    Serial.println(allowedParticipants[1]);
    Serial.print("Chosen participant zone: ");
    Serial.println(chosenParticipant);
    
    int startAngle, endAngle;
    switch(chosenParticipant) {
      case 1: startAngle = PARTICIPANT_1_START; endAngle = PARTICIPANT_1_END; break;
      case 2: startAngle = PARTICIPANT_2_START; endAngle = PARTICIPANT_2_END; break;
      case 3: startAngle = PARTICIPANT_3_START; endAngle = PARTICIPANT_3_END; break;
    }
    targetAngle = random(startAngle, endAngle + 1);
    // Serial.print("Target random angle (restricted): ");
    // Serial.println(targetAngle);
  }
  
  // 旋转电机到目标角度
  moveToAngle(targetAngle);
  
  // 根据当前角度判断选中了哪个参与者区间
  int selectedParticipant = getCurrentParticipant();
  chosenParticipant = selectedParticipant;  // 更新全局变量
  
  Serial.println("\n----- RESULT -----");
  Serial.print("Selected Participant: ");
  Serial.println(selectedParticipant);
  Serial.print("Final Angle: ");
  Serial.print(currentAngle);
  Serial.println(" degrees");
  Serial.println("------------------\n");
  
  // 更新上一轮选中的参与者（供下一轮排除使用）
  previousParticipant = selectedParticipant;
  
  // 电机运行结束，重置标志
  motorRunning = false;
  
  // 触发对应参与者的LED灯带倒计时
  startCountdown(selectedParticipant - 1, currentDelay); // 参与者编号从1开始，灯带索引从0开始
  
  roundNumber++;
}

// 根据 currentAngle 判断当前电机处于哪个参与者区间
int getCurrentParticipant() {
  if (currentAngle >= PARTICIPANT_1_START && currentAngle <= PARTICIPANT_1_END)
    return 1;
  else if (currentAngle >= PARTICIPANT_2_START && currentAngle <= PARTICIPANT_2_END)
    return 2;
  else
    return 3;
}

void moveToAngle(int targetAngle) {
  float stepsPerDegree = (float)STEPS_PER_REVOLUTION / 360.0;

  // 计算从当前角度到目标角度的差值，归一化到 -180 ~ +180（选择最短路径）
  int angleDifference = targetAngle - currentAngle;
  while (angleDifference > 180) angleDifference -= 360;
  while (angleDifference < -180) angleDifference += 360;

  // 计算精确转动所需步数
  int stepsToMove = round(angleDifference * stepsPerDegree);
  // Serial.print("Calculated angle difference: ");
  // Serial.print(angleDifference);
  // Serial.print(" degrees, steps to move: ");
  // Serial.println(stepsToMove);

  // 直接精确旋转到目标角度（不再旋转一整圈）
  stepper.setSpeed(5);  // 使用较慢转速精确移动
  stepper.step(stepsToMove);

  // 更新 currentSteps 与 currentAngle
  currentSteps += stepsToMove;
  currentSteps = (((currentSteps % STEPS_PER_REVOLUTION) + STEPS_PER_REVOLUTION) % STEPS_PER_REVOLUTION);
  currentAngle = (currentSteps * 360L) / STEPS_PER_REVOLUTION;
  // Serial.print("Now at angle: ");
  // Serial.print(currentAngle);
  // Serial.println(" degrees");

  delay(500);  // 等待旋转完成

  // 关闭电机线圈，节能且防止过热
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


// adjustDelay()：在发言等待期间，允许通过加时/减时按钮调整延时时间；同时检测 BUTTON_PIN（停止键）的按下
// 基础等待时间 10 秒，初始加/减操作时间为 3000 ms，每次操作后增量递减
unsigned long adjustDelay() {
  const unsigned long baseDelay = 10000;  // 基础等待 10秒
  currentDelay = baseDelay;               // 重置currentDelay为基础延迟时间

  float addIncrement = 3000.0;
  float subIncrement = 3000.0;

  unsigned long startTime = millis();
  unsigned long targetDelay = currentDelay;

  while (millis() - startTime < targetDelay) {
    // if (digitalRead(BUTTON_PIN) == LOW) { // 停止按钮被按下，系统停止
    //   stopAndReset();
    //   running = false;
    //   Serial.println("Stop triggered during waiting period. System stopped.");
    //   return targetDelay;
    // }

    static bool lastPressureStateInDelay = false;
    bool currentPressureStateInDelay = allPressureSensorsPressed();

    if (currentPressureStateInDelay && !lastPressureStateInDelay) {
      Serial.println("Stop triggered during waiting period by pressure sensors. System stopped.");
      stopAndReset();
      restartSystem();
      systemJustRestarted = true;
      //running = false;
      return targetDelay;
    }
    lastPressureStateInDelay = currentPressureStateInDelay;

    // 更新姿态数据并设置 trigger 标志
    updateMotionSensor(); // 你要确保此函数执行包含 roll/pitch trigger 判断的代码块

    if (leftRightTriggered) {  // ADD 替代：左右运动增加时间
      Serial.print("LEFT/RIGHT triggered, increasing delay. New delay: ");
      currentDelay += (unsigned long)addIncrement;
      targetDelay = currentDelay;
      Serial.println(currentDelay);
      addIncrement /= 2.0;
      leftRightTriggered = false;  // 重置
      delay(50);

      if (activeStrip >= 0) {
        adjustCurrentDelay(currentDelay);
      }
    }

    if (forwardBackTriggered) {  // SUB 替代：前后运动减少时间
      Serial.print("FORWARD/BACK triggered, decreasing delay. New delay: ");
      if (currentDelay > subIncrement)
        currentDelay -= (unsigned long)subIncrement;
      else
        currentDelay = 0;
      targetDelay = currentDelay;
      Serial.println(currentDelay);
      subIncrement /= 2.0;
      forwardBackTriggered = false;  // 重置
      delay(50);

      if (activeStrip >= 0) {
        adjustCurrentDelay(currentDelay);
      }
    }

    updateLedCountdown();
  }

  unsigned long elapsed = millis() - startTime;
  if (elapsed < currentDelay) {
    delay(currentDelay - elapsed);
  }
  return currentDelay;
}

// stopAndReset()：当检测到停止指令后，立即将电机从当前角度归位至 0°
void stopAndReset() {
  Serial.println("Stop command received. Returning motor to 0° position immediately.");
  Serial.print("EVENT:SYSTEM_END");
  float stepsPerDegree = (float)STEPS_PER_REVOLUTION / 360.0;
  int tolerance = 1;  // 允许的角度误差（单位：度）
  
  // 停止所有灯带的倒计时
  activeStrip = -1;
  resetAllStrips();
  FastLED.show();
  
  // 设置电机运行标志
  motorRunning = true;
  
  // 计算当前角度与 0° 之间的差值，并归一化到 -180 ~ +180
  int angleDifference = 0 - currentAngle;
  while (angleDifference > 180) angleDifference -= 360;
  while (angleDifference < -180) angleDifference += 360;
  
  int iteration = 0;
  // 使用循环分段校正，每次以极低速校正，最多尝试20次
  while (abs(angleDifference) > tolerance && iteration < 20) {
    int stepsToMove = round(angleDifference * stepsPerDegree);
    // 使用更低速（例如2 RPM）进行精确归位
    stepper.setSpeed(2);
    stepper.step(stepsToMove);
    
    delay(200);  // 给电机一些时间稳定
    
    // 更新累计步数和当前角度
    currentSteps += stepsToMove;
    currentSteps = (((currentSteps % STEPS_PER_REVOLUTION) + STEPS_PER_REVOLUTION) % STEPS_PER_REVOLUTION);
    currentAngle = (currentSteps * 360L) / STEPS_PER_REVOLUTION;
    
    // Serial.print("Iteration ");
    // Serial.print(iteration);
    // Serial.print(" - Current angle: ");
    // Serial.println(currentAngle);
    
    angleDifference = 0 - currentAngle;
    while (angleDifference > 180) angleDifference -= 360;
    while (angleDifference < -180) angleDifference += 360;
    
    iteration++;
  }
  
  // if (abs(angleDifference) <= tolerance) {
  //   // Serial.println("Motor successfully reset to approximately 0°.");
  // } else {
  //   Serial.println("Motor reset attempted, but error remains. Consider checking mechanical factors.");
  // }
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // 重置电机运行标志
  motorRunning = false;
}

// restartSystem()：当系统停止后按下 BUTTON_PIN，重置系统状态并复位电机至 0°，然后允许重新开始
void restartSystem() {
  Serial.println("Restart command received. Restarting system...");
  roundNumber = 1;
  previousParticipant = 0;
  chosenParticipant = 0;
  currentSteps = 0;
  currentAngle = 0;
  running = false;  // 加上：系统进入“等待启动”状态
  startedFirstRound = false;  // 重要！清除已启动标记
  Serial.println("System restarted. Waiting for all sensors press to start Round 1.");
}


void setup() {
  // Set up step motor speed
  stepper.setSpeed(15);
  Serial.begin(9600);
  Serial.println("Random Participant Selector Ready");
  delay(1000);

  // Initialise random seed
  randomSeed(analogRead(A0));

  // Initialise I2C
  Wire.begin();
  Wire.setClock(50000);

  setupMPU();
  setupLED();
  setupPressurePins();

  // set up sub-board
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  currentSteps = 0;
  currentAngle = 0;

  Serial.println("System initialisation complete.");
}


void loop() {
  checkPressureStatus();
  reconnectIfNeeded();
  updateMotionSensor();
  updateLedCountdown();

  // 三个压力传感器全部按下，且第一轮尚未开始
  if (allPressureSensorsPressed() && !startedFirstRound) {
    Serial.println("All sensors pressed. Starting Round 1.");
    resetAllStrips();
    FastLED.show();
    selectParticipant();
    startedFirstRound = true;  // 标记第一轮已经开始
    systemJustRestarted = false;
    startedFirstRound = true;
    running = true;  // 开始运行
  }

  if (roundNumber > 1 && running) {
    unsigned long adjustedDelay = adjustDelay();
    if (!running) return;
    Serial.print("Adjusted delay = ");
    Serial.print(adjustedDelay);
    Serial.println(" ms. Starting next round automatically.");
    resetAllStrips();
    FastLED.show();
    Serial.print("VOICE_OFF");
    selectParticipant();
  }

  delay(10);
}