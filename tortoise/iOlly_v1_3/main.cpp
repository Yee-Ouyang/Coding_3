#include <Arduino.h>
#include <Servo.h>
#include "octosnake.h"
#include "minikame.h"

MiniKame robot;  // 机器人对象
int leftLDRPin = A0;  // 左侧LDR传感器接口
int rightLDRPin = A1;  // 右侧LDR传感器接口

float qTable[3][3];  // 状态 x 动作
float learningRate = 0.1;
float discountFactor = 0.9;
int epsilon = 10;  // 探索率10%

// Function prototypes
void initializeQTable();
int getState(int leftIntensity, int rightIntensity);
int chooseAction(int state, int explorationRate);
void executeAction(int action);
void updateQTable(int currentState, int action, int reward, int nextState);
int calculateReward(int previousIntensity, int currentIntensity);

void setup() {
  //pinMode(BUILTIN_LED, OUTPUT);  // 设置内置LED引脚为输出模式
  Serial.begin(115200);          // 初始化串口通信,波特率为115200
  Serial.println("Starting robot...");
  
  pinMode(leftLDRPin, INPUT);
  pinMode(rightLDRPin, INPUT);
  robot.init();
  initializeQTable();
}

void loop() {


  int leftLightIntensity = analogRead(leftLDRPin);
  int rightLightIntensity = analogRead(rightLDRPin);

  int currentState = getState(leftLightIntensity, rightLightIntensity);
  int action = chooseAction(currentState, epsilon);

  executeAction(action);

  int reward = calculateReward(leftLightIntensity, rightLightIntensity);
  int newState = getState(analogRead(leftLDRPin), analogRead(rightLDRPin));

  updateQTable(currentState, action, reward, newState);

  delay(1000);  // 每个行动之间的延时
}


void initializeQTable() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      qTable[i][j] = 0.0; // 初始化Q表值
    }
  }
}

int getState(int leftIntensity, int rightIntensity) {
  // 定义状态：0 = 左亮，1 = 右亮，2 = 均衡
  if (leftIntensity > rightIntensity + 50) return 0; //当左侧光强度比右侧高出50以上时
  if (rightIntensity > leftIntensity + 50) return 1; //当右侧光强度比左侧高出50以上时
  return 2;
}

int chooseAction(int state, int explorationRate) {
  if (random(100) < explorationRate) {
    return random(3);  // 探索：随机选择一个动作
  } else {
    // 利用：选择最优动作
    int bestAction = 0;
    for (int i = 1; i < 3; i++) {
      if (qTable[state][i] > qTable[state][bestAction]) {
        bestAction = i;
      }
    }
    return bestAction;
  }
}

void executeAction(int action) {
  // 执行动作：0 = 向左转，1 = 向右转，2 = 直行
  switch (action) {
    case 0: robot.turnL(1, 500); break;
    case 1: robot.turnR(1, 500); break;
    case 2: robot.walk(1, 2000); break;
  }
}

void updateQTable(int currentState, int action, int reward, int nextState) {
  float bestNextQ = qTable[nextState][0];
  for (int i = 1; i < 3; i++) {
    if (qTable[nextState][i] > bestNextQ) {
      bestNextQ = qTable[nextState][i];
    }
  }
  qTable[currentState][action] += learningRate * (reward + discountFactor * bestNextQ - qTable[currentState][action]);
}

int calculateReward(int previousIntensity, int currentIntensity) {
  // 如果光照强度增加，则给予正奖励
  return (currentIntensity > previousIntensity) ? 1 : -1;
}
