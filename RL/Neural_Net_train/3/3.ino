// 机器学习爬行机器人
// 使用神经网络的强化学习
// 作者：Jim Demello，2018年11月在商洛大学
// 适配的神经网络来源：http://www.the-diy-life.com/running-an-artifical-...
// 舵机设置：舵机必须这样摆放：如果机械臂向舵机左侧逆时针旋转，则上方为0度，下方为160度（servoMax），对于两个舵机都适用。
// 当机械臂处于最高位置时，舵机1（靠近机器人的舵机）和舵机2都应该在0度。
// 超声波：超声波模块应面向机器人的后方，因为它测量机器人远离某个固体结构（如墙壁）的运动。
// 目标：在不使用数组存储结果的情况下，只使用神经网络在产生最大距离的两个机械臂位置之间移动。
// 算法：该机器人使用神经网络对训练数据进行训练，然后用神经网络进行练习移动，最后重复最成功的运动（学到的行为）。
// 该算法比我之前的强化学习算法更好，因为一旦神经网络训练完成，就可以使用0到servoMax之间的任何随机舵机位置。
// 要使这个神经网络工作，关键在于设置输入和训练数组。可能存在比我这里用的方法更好的方法。
// 你也可以尝试调整各种神经网络设置。
// 注意：虽然这个算法很有趣，但简单的强化学习算法在没有神经网络的复杂性下同样准确。不过，它是一个有趣的神经网络应用。
// 也许有人能找到更好的方法将该应用适应于神经网络——或许可以通过将距离读数应用于反向传播而不是作为神经网络的输入。


#include <Servo.h>  // 引入伺服电机库
Servo servo1, servo2;  // 定义两个伺服电机对象

//可能需要定义的所有东西
//
//extern const int PatternCount;  // 假设模式数量已经在其他地方定义
//extern const int InputNodes;  // 输入节点数
//extern const int HiddenNodes;  // 隐藏节点数
//extern const int OutputNodes;  // 输出节点数
//extern float Input[][InputNodes];  // 输入数组
//extern float Target[][OutputNodes];  // 目标数组
//extern float Hidden[];  // 隐藏层
//extern float Output[];  // 输出层
//extern float HiddenWeights[][HiddenNodes];  // 隐藏层权重
//extern float OutputWeights[][OutputNodes];  // 输出层权重
//
//extern Servo servo1, servo2;  // 假定伺服电机已在其他地方定义
//extern float distPrevious, distCurrent, distDifference;  // 假定距离变量已在其他地方定义
//extern float Output[];  // 假定神经网络输出数组已在其他地方定义
//extern float highOutput;  // 存储最高输出的变量
//extern int spos1High, spos2High, spos3High, spos4High;  // 存储最优位置的变量
//extern void myServo(Servo servo, float position, int delayBefore, int speed, int delayAfter);  // 控制伺服的函数
//extern float getDistance();  // 获取距离的函数
//extern void InputToOutput(float pos1, float pos2, float pos3, float pos4, float pos5);  // 神经网络处理的函数
//
//unsigned long previousMillis = 0;  // 用于时间控制的变量
//const long loopTimer = 10;  // 循环时间间隔
//const int servoMax = 160;  // 伺服电机的最大角度
//
//extern int freeMemory();
//extern void drive_nn();
//extern void doLearnedBehavior();
//
//extern int spos1High;
//extern int spos2High;
//extern int spos3High;
//extern int spos4High;
//
//extern const int InputNodes;  // 输入节点数
//extern const int HiddenNodes;  // 隐藏节点数
//extern const int OutputNodes;  // 输出节点数
//extern float Hidden[];  // 隐藏层节点
//extern float Output[];  // 输出层节点
//extern float HiddenWeights[][HiddenNodes];  // 隐藏层权重数组
//extern float OutputWeights[][OutputNodes];  // 输出层权重数组
//
//extern const int InputNodes;
//extern const int HiddenNodes;
//extern const int OutputNodes;
//extern const float InitialWeightMax;
//extern const float LearningRate;
//extern const float Momentum;
//extern const float Success;
//extern long TrainingCycle;
//extern float Error;
//extern float Input[][InputNodes];
//extern float Target[][OutputNodes];
//extern float Hidden[];
//extern float Output[];
//extern float HiddenWeights[][HiddenNodes];
//extern float OutputWeights[][OutputNodes];
//extern float ChangeHiddenWeights[][HiddenNodes];
//extern float ChangeOutputWeights[][OutputNodes];
//extern int RandomizedIndex[];
//extern float HiddenDelta[];
//extern float OutputDelta[];
//extern int PatternCount;
//

/******************************************************************
 * 
 ******************************************************************/




/******************************************************************
 * 
 ******************************************************************/

#ifdef __arm__
// ARM架构特有的设置
extern "C" char* sbrk(int incr);
#else  // 非ARM架构的设置
extern char *__brkval;
#endif

int freeMemory() {  // 此函数用于报告可用RAM空间
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif
}

float distance;  // 存储距离测量值
float sonarTime;  // 超声波传感器的时间
int TRIGGER = 7, ECHO = 8;  // 超声波传感器的引脚
int spos1 = 0, spos2 = 0, spos3 = 0, spos4 = 0;  // 四个伺服电机的位置servo1、2 position
int spos1High = 0, spos2High = 0, spos3High = 0, spos4High = 0;  // 记录达到最远距离时的伺服位置
int distanceHigh = 0;  // 记录最远距离
float highOutput = 0.0;
int numberTrainingCycles = 100;  //
int servoMin = 0;  // 伺服电机的最小角度
int servoMax = 160;  // 伺服电机的最大角度
float distDifference = 0, distPrevious = 0, distCurrent = 0;  // 距离差异，前一距离和当前距离

#include "math.h"  // 引入数学库

/******************************************************************
 * 神经网络配置 - 根据网络定制
 ******************************************************************/
const int PatternCount = 16;  // 模式数量
const int InputNodes = 5;  // 输入节点数
const int HiddenNodes = 7;  // 隐藏节点数
const int OutputNodes = 1;  // 输出节点数
const float LearningRate = 0.2;  // 学习率
const float Momentum = 0.9;  // 动量
const float InitialWeightMax = 0.5;  // 初始最大权重
const float Success = 0.0015;  // 成功阈值

// 目标数组，对应于输入数组中的臂位置的成功输出
float Target[PatternCount][OutputNodes] = {
    { 0 }, { 0 }, { 0 }, { 1 }, { 0 }, { 0 }, { 0 }, { 1 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 1 }
};

// 输入数组，第1和第2列保存伺服1位置，第3和第4列保存伺服2位置，第5列保存距离
const float Input[PatternCount][InputNodes] = {
    { 0, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 0 },
    { 0, 0, 1, 0, 0 },
    { 0, 0, 1, 1, 1 },
    { 1, 0, 0, 0, 0 },
    { 1, 0, 0, 1, 0 },
    { 1, 0, 1, 0, 0 },
    { 1, 0, 1, 1, 1 },
    { 1, 1, 0, 0, 0 },
    { 1, 1, 0, 1, 0 },
    { 1, 1, 1, 0, 0 },
    { 1, 1, 1, 1, 0 },
    { 0, 1, 0, 0, 0 },
    { 0, 1, 0, 1, 0 },
    { 0, 1, 1, 0, 0 },
    { 0, 1, 1, 1, 1 }
};

/******************************************************************
 * 网络配置结束
 ******************************************************************/

// 以下是相关的变量定义和计算逻辑部分
int i, j, p, q, r;  // 循环变量
int ReportEvery1000;  // 每1000次报告一次
int RandomizedIndex[PatternCount];  // 随机索引数组
long TrainingCycle;  // 训练周期
float Rando;  // 随机数
float Error = 2;  // 错误值
float Accum;  // 累加器
float Hidden[HiddenNodes];  // 隐藏层节点
float Output[OutputNodes];  // 输出层节点
float HiddenWeights[InputNodes + 1][HiddenNodes];  // 隐藏层权重
float OutputWeights[HiddenNodes + 1][OutputNodes];  // 输出层权重
float HiddenDelta[HiddenNodes];  // 隐藏层delta值
float OutputDelta[OutputNodes];  // 输出层delta值
float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];  // 改变的隐藏层权重
float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];  // 改变的输出层权重

long previousMillis = 0;  // 上一次的时间
unsigned long currentMillis;  // 当前时间
long loopTimer = 10;  // 主处理循环的时间间隔（10毫秒）

void setup() {
  Serial.begin(115200);  // 初始化串口通讯，波特率为115200
  Serial.println("Starting program");  // 串口输出，表明程序开始运行

  randomSeed(analogRead(A1));  // 从A1口读取一个模拟值用于随机种子

  ReportEvery1000 = 1;  // 设置每1000次报告一次状态

  for (int p = 0; p < PatternCount; p++) {  // 初始化随机索引数组
    RandomizedIndex[p] = p;
  }

  Serial.println("do train_nn");  // 串口输出，表明开始训练神经网络
  train_nn();  // 调用训练神经网络的函数
  delay(1000);  // 延迟1秒

  servo1.attach(9, 600, 2400);  // 将servo1附加到引脚9，设定脉冲宽度范围
  servo2.attach(6, 600, 2400);  // 将servo2附加到引脚6，设定脉冲宽度范围

  myServo(servo1, 0, 1, 8, 1);  // 将servo1设定到初始位置
  delay(1000);  // 延迟1秒
  myServo(servo2, 0, 1, 8, 1);  // 将servo2设定到初始位置
  delay(1000);  // 延迟1秒

  pinMode(TRIGGER, OUTPUT);  // 设置超声波传感器的TRIGGER引脚为输出
  pinMode(ECHO, INPUT);  // 设置超声波传感器的ECHO引脚为输入

  distPrevious = getDistance();  // 获取初始距离
  Serial.print("Initial distance= ");  // 串口输出初始距离
  Serial.println(distPrevious);

  delay(1000);  // 延迟1秒

  // exit(0);  // 如果只是测试超声波传感器，则在此退出
}  // 结束 setup 函数


float getDistance() {  // 测量距离的函数，调用5次并计算平均值
  int numberTriggers = 5;  // 测量次数
  float average = 0;  // 用于存储距离的平均值

  for (int i = 0; i < numberTriggers; i++) {
    digitalWrite(TRIGGER, LOW);  // 先将TRIGGER引脚设置为低电平
    delayMicroseconds(5);  // 保持低电平5微秒
    digitalWrite(TRIGGER, HIGH);  // 然后将TRIGGER引脚设置为高电平
    delayMicroseconds(10);  // 保持高电平10微秒
    digitalWrite(TRIGGER, LOW);  // 再将TRIGGER引脚设置回低电平

    sonarTime = pulseIn(ECHO, HIGH);  // 读取ECHO引脚的高电平持续时间，即声波往返时间
    distance = sonarTime / 58.00;  // 计算距离（单位为厘米），声速在空气中的速度约为 340 m/s，对应58 us/cm
    average += distance;  // 将测得的距离加到总和中

    delay(100);  // 每次测量后延迟100毫秒
  }

  average /= numberTriggers;  // 计算所有测量的平均距离
  Serial.print("Distance = ");  // 串口输出平均距离
  Serial.println(average);

  return average;  // 返回平均距离
}  // 结束 getDistance 函数


void doLearnedBehavior() {
    Serial.println("Do Learned behavior... ");  // 串口输出，提示开始执行学习到的行为

    // 将两个伺服电机都初始化到0位置
    myServo(servo1, 0, 1, 8, 1);
    myServo(servo2, 0, 1, 8, 1);

    delay(2000);  // 在开始执行动作之前延迟2000毫秒

    // 循环执行30次学习到的动作
    for (int i = 0; i < 30; i++) {
        // 通过串口输出当前的伺服电机位置
        Serial.print("spos1High= "); Serial.print(spos1High);
        Serial.print(" spos2High = "); Serial.print(spos2High);
        Serial.print(" spos3High = "); Serial.print(spos3High);
        Serial.print(" spos4High = "); Serial.println(spos4High);

        // 根据记录的最高位置控制伺服电机移动
        myServo(servo1, spos1High, 1, 7, 1);
        myServo(servo2, spos2High, 1, 7, 1);
        myServo(servo1, spos3High, 1, 7, 1);
        myServo(servo2, spos4High, 1, 7, 1);
    }
}  // 结束 doLearnedBehavior 函数


void loop() {  // 主循环函数，读取成功表格并执行相应的行动
    int freespace = freeMemory();  // 获取当前自由内存空间
    Serial.print("free memory= ");  // 通过串口输出当前内存空间
    Serial.println(freespace);

    drive_nn();  // 驱动神经网络进行处理

    freespace = freeMemory();  // 再次获取内存空间以检查内存使用情况
    Serial.print("free memory= ");  // 输出内存使用情况
    Serial.println(freespace);

    doLearnedBehavior();  // 执行已学习的行为

    myServo(servo1, 0, 1, 8, 1);  // 将伺服1重置到初始位置
    myServo(servo2, 0, 1, 8, 1);  // 将伺服2重置到初始位置

    Serial.print("end program ");  // 通过串口输出程序结束的信息

    delay(2000);  // 程序结束前延迟2000毫秒

    exit(0);  // 退出程序
}  // 结束主循环函数


void myServo(Servo servo, int newAngle, int angleInc, int incDelay, int servoNum) {
    // 控制伺服电机平滑地移动到新的角度
    int curAngle = servo.read();  // 读取当前伺服电机的角度

    // Serial.print("curAngle = "); Serial.println(curAngle);  // 可以取消注释以通过串口监控当前角度

    if (curAngle < newAngle) {  // 如果当前角度小于新角度，则递增角度
        for (int angle = curAngle; angle < newAngle; angle += angleInc) {
            servo.write(angle);  // 设置伺服电机的角度
            delay(incDelay);  // 延迟一段时间以控制移动的速度
        }
    } else if (curAngle > newAngle) {  // 如果当前角度大于新角度，则递减角度
        for (int angle = curAngle; angle > newAngle; angle -= angleInc) {
            servo.write(angle);  // 设置伺服电机的角度
            delay(incDelay);  // 延迟一段时间以控制移动的速度
        }
    }
}  // 结束 myServo 函数

/******************************************************************
 * 训练完神经网络后，现在在神经网络上驱动机器人，并存储产生伺服位置的最大距离
 ******************************************************************/
 void drive_nn() {
  Serial.println("Running NN Drive ");  // 通过串口输出开始驱动神经网络
  int numberTrainingCycles = 20;  // 训练周期数

  for (int x = 0; x < numberTrainingCycles; x++) {
    unsigned long currentMillis = millis();  // 获取当前时间毫秒数

    if (currentMillis - previousMillis > loopTimer) {  // 每5毫秒执行一次计算
      // 随机设置伺服电机的位置，并通过神经网络评估效果
      float pos1 = random(servoMax) / 100.0 * servoMax;
      float pos2 = random(servoMax) / 100.0 * servoMax;
      float pos3 = random(servoMax) / 100.0 * servoMax;
      float pos4 = random(servoMax) / 100.0 * servoMax;

      // 控制伺服电机移动到新位置
      myServo(servo1, pos1, 1, 7, 1);
      myServo(servo2, pos2, 1, 7, 1);
      myServo(servo1, pos3, 1, 7, 1);
      myServo(servo2, pos4, 1, 7, 1);

      distCurrent = getDistance();  // 获取当前距离
      distDifference = distCurrent - distPrevious;  // 计算距离差
      distPrevious = distCurrent;

      Serial.print("===> distDifference = "); Serial.println(distDifference);  // 输出距离差

      // 通过神经网络处理输入并获取输出
      float pos5 = map(distDifference, 0, 10, 0, 100) / 100.0;
      InputToOutput(pos1, pos2, pos3, pos4, pos5);

      Serial.print("Output from NN = "); Serial.println(Output[0]);  // 输出神经网络的结果

      // 如果输出大于某个阈值，则记录这个位置为最优位置
      if (Output[0] > .10) {
        if (Output[0] > highOutput) {
          highOutput = Output[0];
          spos1High = pos1;
          spos2High = pos2;
          spos3High = pos3;
          spos4High = pos4;

          Serial.print(" --------> spos1High= "); Serial.print(spos1High);
          Serial.print(" spos2High= "); Serial.print(spos2High);
          Serial.print(" spos3High= "); Serial.print(spos3High);
          Serial.print(" spos4High= "); Serial.println(spos4High);
          Serial.print(" Output= "); Serial.println(Output[0]);
        }
      }
      previousMillis = currentMillis;  // 更新时间，以便下次计算时间间隔
    }  // 结束 if 检查时间间隔的 block
  }  // 结束 for 循环
}  // 结束 drive_nn() 函数

//训练时显示信息


void toTerminal() {
  // 通过串口输出每个训练模式的详细信息
  for (int p = 0; p < PatternCount; p++) {
    Serial.println();  // 输出空行以便区分
    Serial.print("Training Pattern: ");
    Serial.println(p);  // 输出当前的训练模式编号

    Serial.print("Input ");  // 输出输入
    for (int i = 0; i < InputNodes; i++) {
      Serial.print(Input[p][i], DEC);
      Serial.print(" ");
    }

    Serial.print("Target ");  // 输出目标
    for (int i = 0; i < OutputNodes; i++) {
      Serial.print(Target[p][i], DEC);
      Serial.print(" ");
    }

    /******************************************************************
    计算隐藏层激活值
    ******************************************************************/
    for (int i = 0; i < HiddenNodes; i++) {
      float Accum = HiddenWeights[InputNodes][i];  // 累加器初始化
      for (int j = 0; j < InputNodes; j++) {
        Accum += Input[p][j] * HiddenWeights[j][i];  // 计算权重和输入的乘积并累加
      }
      Hidden[i] = 1.0 / (1.0 + exp(-Accum));  // 应用激活函数
    }

    /******************************************************************
    计算输出层激活值并计算误差
    ******************************************************************/
    for (int i = 0; i < OutputNodes; i++) {
      float Accum = OutputWeights[HiddenNodes][i];  // 重新初始化累加器
      for (int j = 0; j < HiddenNodes; j++) {
        Accum += Hidden[j] * OutputWeights[j][i];  // 计算隐藏层的输出和权重的乘积
      }
      Output[i] = 1.0 / (1.0 + exp(-Accum));  // 应用激活函数
    }

    Serial.print("Output ");  // 输出最终的输出层结果
    for (int i = 0; i < OutputNodes; i++) {
      Serial.print(Output[i], 5);  // 使用5位精度输出浮点数
      Serial.print(" ");
    }
  }
}

void InputToOutput(float In1, float In2, float In3, float In4, float In5) {
    float TestInput[5] = {0, 0, 0, 0, 0};  // 定义测试输入数组

    // 赋值输入数据到测试数组
    TestInput[0] = In1;  // 第一个伺服电机的第一个位置
    TestInput[1] = In2;  // 第二个伺服电机的第一个位置
    TestInput[2] = In3;  // 第一个伺服电机的第二个位置
    TestInput[3] = In4;  // 第二个伺服电机的第二个位置
    TestInput[4] = In5;  // 测量到的距离

    /******************************************************************
     计算隐藏层激活值
    ******************************************************************/
    for (int i = 0; i < HiddenNodes; i++) {
        float Accum = HiddenWeights[InputNodes][i];  // 初始化累加器
        for (int j = 0; j < InputNodes; j++) {
            Accum += TestInput[j] * HiddenWeights[j][i];  // 计算输入和权重的乘积并累加
        }
        Hidden[i] = 1.0 / (1.0 + exp(-Accum));  // 应用sigmoid激活函数
    }

    /******************************************************************
     计算输出层激活值并计算错误
    ******************************************************************/
    for (int i = 0; i < OutputNodes; i++) {
        float Accum = OutputWeights[HiddenNodes][i];  // 重新初始化累加器
        for (int j = 0; j < HiddenNodes; j++) {
            Accum += Hidden[j] * OutputWeights[j][i];  // 计算隐藏层的输出和权重的乘积
        }
        Output[i] = 1.0 / (1.0 + exp(-Accum));  // 应用sigmoid激活函数
    }

    // 调试输出，如果定义了DEBUG则输出网络的输出值
    //#ifdef DEBUG
    Serial.print("Output ");
    for (int i = 0; i < OutputNodes; i++) {
        Serial.print(Output[i], 5);  // 以5位精度输出
        Serial.print(" ");
    }
    //#endif
}

//TRAINS THE NEURAL NETWORK

void train_nn() {
    Serial.println("start training...");  // 开始训练的提示

    // 初始化隐藏层权重和变化量
    for (int i = 0; i < HiddenNodes; i++) {
        for (int j = 0; j <= InputNodes; j++) {
            ChangeHiddenWeights[j][i] = 0.0;
            float Rando = float(random(100)) / 100;
            HiddenWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
        }
    }
    //digitalWrite(LEDYEL, HIGH);
    // 初始化输出层权重和变化量
    //digitalWrite(LEDRED, LOW);
    for (int i = 0; i < OutputNodes; i++) {
        for (int j = 0; j <= HiddenNodes; j++) {
            ChangeOutputWeights[j][i] = 0.0;
            float Rando = float(random(100)) / 100;
            OutputWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
        }
    }
    
    //digitalWrite(LEDRED, HIGH);
    //SerialUSB.println("Initial/Untrained Outputs: ");
     //toTerminal();
     /******************************************************************
    Begin training  
    ******************************************************************/
    // 开始无限训练循环，直到达到错误率阈值
    for (long TrainingCycle = 1; TrainingCycle < 2147483647; TrainingCycle++) {
        // 随机化训练模式的顺序
        for (int p = 0; p < PatternCount; p++) {
            int q = random(PatternCount);
            int r = RandomizedIndex[p];
            RandomizedIndex[p] = RandomizedIndex[q];
            RandomizedIndex[q] = r;
        }

        float Error = 0.0;  // 初始化误差

        // 按随机顺序训练每个模式
        for (int q = 0; q < PatternCount; q++) {
            int p = RandomizedIndex[q];

            // 计算隐藏层激活值
            for (int i = 0; i < HiddenNodes; i++) {
                float Accum = HiddenWeights[InputNodes][i];
                for (int j = 0; j < InputNodes; j++) {
                    Accum += Input[p][j] * HiddenWeights[j][i];
                }
                Hidden[i] = 1.0 / (1.0 + exp(-Accum));  // 使用sigmoid激活函数
            }
            //digitalWrite(LEDYEL, HIGH);
            
            // 计算输出层激活值并计算误差
            for (int i = 0; i < OutputNodes; i++) {
                float Accum = OutputWeights[HiddenNodes][i];
                for (int j = 0; j < HiddenNodes; j++) {
                    Accum += Hidden[j] * OutputWeights[j][i];
                }
                Output[i] = 1.0 / (1.0 + exp(-Accum));
                OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1 - Output[i]);
                Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]);
            }

            // Serial.println(Output[0]*100);
            //digitalWrite(LEDRED, HIGH);

            // 反向传播误差到隐藏层
            for (int i = 0; i < HiddenNodes; i++) {
                float Accum = 0.0;
                for (int j = 0; j < OutputNodes; j++) {
                    Accum += OutputWeights[i][j] * OutputDelta[j];
                }
                HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]);
            }
            //digitalWrite(LEDYEL, HIGH);

            // 更新输入到隐藏层的权重

            //digitalWrite(LEDRED, LOW);
            
            for (int i = 0; i < HiddenNodes; i++) {
                ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i];
                HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i];
                for (int j = 0; j < InputNodes; j++) {
                    ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
                    HiddenWeights[j][i] += ChangeHiddenWeights[j][i];
                }
            }
            //digitalWrite(LEDRED, HIGH);

            // 更新隐藏层到输出层的权重
            //digitalWrite(LEDYEL, LOW);
            for (int i = 0; i < OutputNodes; i++) {
                ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i];
                OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i];
                for (int j = 0; j < HiddenNodes; j++) {
                    ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i];
                    OutputWeights[j][i] += ChangeOutputWeights[j][i];
                }
            }
        }
        //digitalWrite(LEDYEL, HIGH);

        // 每1000次循环后输出信息到终端并绘制图形
        if (TrainingCycle % 1000 == 0) {
            Serial.print("TrainingCycle: ");
            Serial.print(TrainingCycle);
            Serial.print(" Error = ");
            Serial.println(Error, 5);
            toTerminal();  // 输出训练数据
        }

        // 如果误差低于预定阈值，则结束训练
        if (Error < Success) {
            Serial.println("End training.");
            break;
        }
    }
}
