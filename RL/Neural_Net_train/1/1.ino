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



float getDistance() { // 测量距离的例程，调用并平均5次
  int numberTriggers = 5;
  int average = 0;
  for (int i = 0; i < numberTriggers; i++) {
    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER, LOW);
    sonarTime = pulseIn(ECHO, HIGH);
    distance = sonarTime / 58.00;
    average = average + distance;
    delay(100);
  } // 结束for循环
  average = average / numberTriggers;
  Serial.print("距离 = ");
  Serial.println(average);
  return average;
} // 结束getDistance函数

void doLearnedBehavior() {
  Serial.println("执行学习到的行为...");
  myServo(servo1, 0, 1, 8, 1);
  myServo(servo2, 0, 1, 8, 1);
  delay(2000);
  for (int i = 0; i < 30; i++) {
    Serial.print(" spos1High= ");
    Serial.print(spos1High);
    Serial.print(" spos2High = ");
    Serial.print(spos2High);
    Serial.print(" spos3High = ");
    Serial.print(spos3High);
    Serial.print(" spos4High = ");
    Serial.println(spos4High);
    
    myServo(servo1, spos1High, 1, 7, 1);
    myServo(servo2, spos2High, 1, 7, 1);
    myServo(servo1, spos3High, 1, 7, 1);
    myServo(servo2, spos4High, 1, 7, 1);
    }
}

void loop() {
    // 主循环读取成功表并执行操作

    // 检查空闲内存
    int freespace = freeMemory(); 
    Serial.print("free memory= "); 
    Serial.println(freespace); // 只是想看看内存是否成为问题

    // 调用drive_nn函数
    drive_nn();

    // 再次检查空闲内存
    freespace = freeMemory(); 
    Serial.print("free memory= "); 
    Serial.println(freespace);

    // 执行学习到的行为
    doLearnedBehavior();

    // 控制舵机1和舵机2
    myServo(servo1, 0, 1, 8, 1);
    myServo(servo2, 0, 1, 8, 1);

    // 打印程序结束信息
    Serial.print("end program ");

    // 延时2秒
    delay(2000);

    // 退出程序
    exit(0); 
} // 主循环结束

// 函数myServo，用于读取当前舵机角度，将其调整到newAngle并控制速度
void myServo(Servo servo, int newAngle, int angleInc, int incDelay, int servoNum) {
    int curAngle = 0;

    // 读取当前舵机角度
    curAngle = servo.read();

    // 如果当前角度小于目标角度
    if (curAngle < newAngle) {
        for(int angle = curAngle; angle < newAngle; angle += angleInc) {
            servo.write(angle);
            delay(incDelay);
        }
    }
    // 如果当前角度大于目标角度
    else if (curAngle > newAngle) {
        for(int angle = curAngle; angle > newAngle; angle -= angleInc) {
            servo.write(angle);
            delay(incDelay);
        }
    }
} // myServo函数结束
//******************************************************************
//******************************************************************
// 在训练完神经网络后，现在基于神经网络驱动机器人，并存储产生最大距离的舵机位置

void drive_nn() {
    Serial.println("Running NN Drive ");

    numberTrainingCycles = 20; // 尝试随机舵机位置并获取距离的次数，然后存储用于最终行走的最高位置

    for (int x = 0; x < numberTrainingCycles; x++) {
        currentMillis = millis();
        float TestInput[] = {0, 0};

        if (currentMillis - previousMillis > loopTimer) { // 每5毫秒或更长时间执行一次计算
            // 生成随机舵机位置
            int randomNo = random(servoMax);
            float pos1 = map(randomNo, 0, servoMax, 0, 100) / 100;

            randomNo = random(servoMax);
            float pos2 = map(randomNo, 0, servoMax, 0, 100) / 100;

            randomNo = random(servoMax);
            float pos3 = map(randomNo, 0, servoMax, 0, 100) / 100;

            randomNo = random(servoMax);
            float pos4 = map(randomNo, 0, servoMax, 0, 100) / 100;

            // 使用新的随机位置移动机器人
            myServo(servo1, pos1 * servoMax, 1, 7, 1);
            myServo(servo2, pos2 * servoMax, 1, 7, 1);
            myServo(servo1, pos3 * servoMax, 1, 7, 1);
            myServo(servo2, pos4 * servoMax, 1, 7, 1);

            // 获取距离
            distCurrent = getDistance();
            distDifference = distCurrent - distPrevious;
            distPrevious = distCurrent;

            Serial.print("===> distDifference = ");
            Serial.println(distDifference);

            // 将距离差异映射到一个0到100的范围
            float temp = map(distDifference, 0, 10, 0, 100) / 100;
            float pos5 = temp;

            // 将输入传递给神经网络以获得输出
            InputToOutput(pos1, pos2, pos3, pos4, pos5);

            // 调整舵机位置以适应最大值范围
            pos1 *= servoMax;
            pos2 *= servoMax;
            pos3 *= servoMax;
            pos4 *= servoMax;

            // 输出当前舵机位置和神经网络输出
            Serial.print(" pos1= ");
            Serial.print(pos1);
            Serial.print(" pos2= ");
            Serial.print(pos2);
            Serial.print(" pos3= ");
            Serial.print(pos3);
            Serial.print(" pos4= ");
            Serial.print(pos4);
            Serial.print(" pos5= ");
            Serial.print(pos5);
            Serial.print(" Output from NN =");
            Serial.println(Output[0]);

            // 如果输出大于0.10，则使用这些位置移动机器人并存储达到的最高输出位置
            if (Output[0] > .10) {
                if (Output[0] > highOutput) {
                    highOutput = Output[0];
                    spos1High = pos1;
                    spos2High = pos2;
                    spos3High = pos3;
                    spos4High = pos4;

                    Serial.print(" --------> spos1High= ");
                    Serial.print(spos1High);
                    Serial.print(" spos2High= ");
                    Serial.print(spos2High);
                    Serial.print(" spos3High= ");
                    Serial.print(spos3High);
                    Serial.print(" spos4High= ");
                    Serial.print(spos4High);
                    Serial.print(" Output= ");
                    Serial.println(Output[0]);
                }
            }

            previousMillis = currentMillis;
        } // millis循环结束
    }
} // drive_nn()函数结束

// 显示训练过程中的信息
void toTerminal() {
    for (p = 0; p < PatternCount; p++) {
        Serial.println();

        Serial.print(" Training Pattern: ");
        Serial.println(p);

        Serial.print(" Input ");
        for (i = 0; i < InputNodes; i++) {
            Serial.print(Input[p][i], DEC);
            Serial.print(" ");
        }

        Serial.print(" Target ");
        for (i = 0; i < OutputNodes; i++) {
            Serial.print(Target[p][i], DEC);
            Serial.print(" ");
        }
    }
}

/******************************************************************
计算隐藏层激活
******************************************************************/

for (i = 0; i < HiddenNodes; i++) {
    Accum = HiddenWeights[InputNodes][i];

    for (j = 0; j < InputNodes; j++) {
        Accum += Input[p][j] * HiddenWeights[j][i];
    }

    Hidden[i] = 1.0 / (1.0 + exp(-Accum)); // 激活函数
}

/******************************************************************
计算输出层激活并计算误差
******************************************************************/

for (i = 0; i < OutputNodes; i++) {
    Accum = OutputWeights[HiddenNodes][i];

    for (j = 0; j < HiddenNodes; j++) {
        Accum += Hidden[j] * OutputWeights[j][i];
    }

    Output[i] = 1.0 / (1.0 + exp(-Accum));
}

Serial.print(" Output ");
for (i = 0; i < OutputNodes; i++) {
    Serial.print(Output[i], 5);
    Serial.print(" ");
}
}

/******************************************************************
将输入传递到输出
******************************************************************/

void InputToOutput(float In1, float In2, float In3, float In4, float In5) {
    float TestInput[] = {0, 0, 0, 0, 0};

    // Serial.print("In1 = "); Serial.println(In1);

    TestInput[0] = In1; // 第一个舵机位置 - 舵机1
    TestInput[1] = In2; // 第一个舵机位置 - 舵机2
    TestInput[2] = In3; // 第二个舵机位置 - 舵机1
    TestInput[3] = In4; // 第二个舵机位置 - 舵机2
    TestInput[4] = In5; // 距离

    /******************************************************************
    计算隐藏层激活
    ******************************************************************/

    for (i = 0; i < HiddenNodes; i++) {
        Accum = HiddenWeights[InputNodes][i];

        for (j = 0; j < InputNodes; j++) {
            Accum += TestInput[j] * HiddenWeights[j][i];
        }

        Hidden[i] = 1.0 / (1.0 + exp(-Accum));
    }

    /******************************************************************
    计算输出层激活并计算误差
    ******************************************************************/

    for (i = 0; i < OutputNodes; i++) {
        Accum = OutputWeights[HiddenNodes][i];

        for (j = 0; j < HiddenNodes; j++) {
            Accum += Hidden[j] * OutputWeights[j][i];
        }

        Output[i] = 1.0 / (1.0 + exp(-Accum));
    }

    // #ifdef DEBUG
    Serial.print(" Output ");
    for (i = 0; i < OutputNodes; i++) {
        Serial.print(Output[i], 5);
        Serial.print(" ");
    }
    // #endif
}

// 训练神经网络的代码段

void train_nn() {
    /******************************************************************
    初始化隐藏层权重和变化量
    ******************************************************************/
    int prog_start = 0;
    Serial.println("start training...");

    for (int i = 0; i < HiddenNodes; i++) {
        for (int j = 0; j <= InputNodes; j++) {
            ChangeHiddenWeights[j][i] = 0.0;
            Rando = float(random(100)) / 100;
            HiddenWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
        }
    }

    /******************************************************************
    初始化输出层权重和变化量
    ******************************************************************/
    for (int i = 0; i < OutputNodes; i++) {
        for (int j = 0; j <= HiddenNodes; j++) {
            ChangeOutputWeights[j][i] = 0.0;
            Rando = float(random(100)) / 100;
            OutputWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
        }
    }

    /******************************************************************
    开始训练
    ******************************************************************/
    for (int TrainingCycle = 1; TrainingCycle < 2147483647; TrainingCycle++) {
        /******************************************************************
        随机化训练模式的顺序
        ******************************************************************/
        for (int p = 0; p < PatternCount; p++) {
            int q = random(PatternCount);
            int r = RandomizedIndex[p];
            RandomizedIndex[p] = RandomizedIndex[q];
            RandomizedIndex[q] = r;
        }

        float Error = 0.0;

        /******************************************************************
        按随机顺序循环遍历每个训练模式
        ******************************************************************/
        for (int q = 0; q < PatternCount; q++) {
            int p = RandomizedIndex[q];

            /******************************************************************
            计算隐藏层激活
            ******************************************************************/
            for (int i = 0; i < HiddenNodes; i++) {
                Accum = HiddenWeights[InputNodes][i];
                for (int j = 0; j < InputNodes; j++) {
                    Accum += Input[p][j] * HiddenWeights[j][i];
                }
                Hidden[i] = 1.0 / (1.0 + exp(-Accum)); // 激活函数
            }

            /******************************************************************
            计算输出层激活并计算误差
            ******************************************************************/
            for (int i = 0; i < OutputNodes; i++) {
                Accum = OutputWeights[HiddenNodes][i];
                for (int j = 0; j < HiddenNodes; j++) {
                    Accum += Hidden[j] * OutputWeights[j][i];
                }
                Output[i] = 1.0 / (1.0 + exp(-Accum));
                OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]);
                Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]);
            }

            /******************************************************************
            将误差反向传播到隐藏层
            ******************************************************************/
            for (int i = 0; i < HiddenNodes; i++) {
                Accum = 0.0;
                for (int j = 0; j < OutputNodes; j++) {
                    Accum += OutputWeights[i][j] * OutputDelta[j];
                }
                HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]);
            }

            /******************************************************************
            更新输入层到隐藏层的权重
            ******************************************************************/
            for (int i = 0; i < HiddenNodes; i++) {
                ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i];
                HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i];
                for (int j = 0; j < InputNodes; j++) {
                    ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
                    HiddenWeights[j][i] += ChangeHiddenWeights[j][i];
                }
            }

            /******************************************************************
            更新隐藏层到输出层的权重
            ******************************************************************/
            for (int i = 0; i < OutputNodes; i++) {
                ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i];
                OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i];
                for (int j = 0; j < HiddenNodes; j++) {
                    ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i];
                    OutputWeights[j][i] += ChangeOutputWeights[j][i];
                }
            }
        }

        /******************************************************************
        每1000个周期发送数据到终端显示，并在OLED上绘制图形
        ******************************************************************/
        ReportEvery1000 = ReportEvery1000 - 1;
        if (ReportEvery1000 == 0) {
            int graphNum = TrainingCycle / 100;
            int graphE1 = Error * 1000;
            int graphE = map(graphE1, 3, 80, 47, 0);

            Serial.print("TrainingCycle: ");
            Serial.print(TrainingCycle);
            Serial.print(" Error = ");
            Serial.println(Error, 5);

            toTerminal();

            if (TrainingCycle == 1) {
                ReportEvery1000 = 99;
            } else {
                ReportEvery1000 = 100;
            }
        }

        /******************************************************************
        如果错误率低于预定阈值，则结束训练
        ******************************************************************/
        if (Error < Success) break;
    }

    Serial.println("End training.");
}
