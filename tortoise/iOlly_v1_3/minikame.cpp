#include <Arduino.h>   // 引入Arduino核心库
#include "minikame.h"  // 引入MiniKame机器人的定义

#define MAX_PULSE_WIDTH 2400  // 舵机最大脉冲宽度
#define MIN_PULSE_WIDTH 500   // 舵机最小脉冲宽度

// 将角度转换为微秒值的函数
int angToUsec(float value) {
  return map(value, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);  // 将0-180度映射到脉冲宽度范围
}

// 声明
float lerp(float start, float end, float t);

// MiniKame机器人的初始化函数
void MiniKame::init() {
    // Initialize servo pins for Arduino Uno
    board_pins[0] = 2;  // 前左内侧
    board_pins[1] = 3;  // 前右内侧
    board_pins[2] = 4;  // 前左外侧
    board_pins[3] = 5;  // 前右外侧
    board_pins[4] = 6;  // 后左内侧
    board_pins[5] = 7;  // 后右内侧
    board_pins[6] = 8;  // 后左外侧
    board_pins[7] = 9;  // 后右外侧
    board_pins[8] = 10; // 机械头部


    // Initialize servo calibration values - adjust based on actual servo initial positions
    trim[0] = 8;
    trim[1] = -8;
    trim[2] = -2;
    trim[3] = 8;
    trim[4] = 5;
    trim[5] = 15;
    trim[6] = 5;
    trim[7] = -1;
    trim[8] = 0;

    // Initialize servo direction
    for (int i = 0; i < 9; i++) reverse[i] = 0;

    // Apply servo calibration values and connect servos
    for (int i = 0; i < 9; i++) {
        oscillator[i].setTrim(trim[i]);
        servo[i].attach(board_pins[i], MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); // Corrected to use the valid attach method
    }
}



// MiniKame类中的右转动作
// MiniKame class method to turn right
void MiniKame::turnR(float steps, float T = 600) {
  int x_amp = 15;                                                                                      // X轴振幅
  int z_amp = 15;                                                                                      // Z轴振幅
  int ap = 15;                                                                                         // 角度偏移
  int hi = 23;                                                                                         // 高度参数
  float period[] = { T, T, T, T, T, T, T, T, T };                                                      // 周期数组
  int amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0 };                     // 振幅数组
  int offset[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 45 };  // 偏移数组
  int phase[] = { 0, 180, 90, 90, 180, 0, 90, 90, 0 };                                                 // 相位数组

  execute(steps, period, amplitude, offset, phase);  // 执行动作
}

// MiniKame类中的左转动作
// MiniKame class method to turn left
void MiniKame::turnL(float steps, float T = 600) {
  int x_amp = 15;                                                                                      // X轴振幅
  int z_amp = 15;                                                                                      // Z轴振幅
  int ap = 15;                                                                                         // 角度偏移
  int hi = 23;                                                                                         // 高度参数
  float period[] = { T, T, T, T, T, T, T, T, T };                                                      // 周期数组
  int amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0 };                     // 振幅数组
  int offset[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 135 };  // 偏移数组
  int phase[] = { 180, 0, 90, 90, 0, 180, 90, 90, 0 };                                                 // 相位数组

  execute(steps, period, amplitude, offset, phase);  // 执行动作
}


// MiniKame类中的跳舞动作
// MiniKame class method to dance
void MiniKame::dance(float steps, float T = 600) {
  int x_amp = 0;                                                                                  // X轴振幅，此处设置为0，意味着在X轴上不进行振动
  int z_amp = 40;                                                                                 // Z轴振幅
  int ap = 30;                                                                                    // 角度偏移
  int hi = 20;                                                                                    // 高度参数
  float period[] = { T, T, T, T, T, T, T, T, T };                                                 // 周期数组，用于设置每个动作的时间长度
  int amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0  };               // 振幅数组，定义了每个伺服电机的振动幅度
  int offset[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 };  // 偏移数组，定义了每个伺服电机的中心位置
  int phase[] = { 0, 0, 0, 270, 0, 0, 90, 180, 0 };                                               // 相位数组，定义了每个伺服电机动作的起始相位

  execute(steps, period, amplitude, offset, phase);  // 执行动作
}

// MiniKame类中的前后摇摆动作
// MiniKame class method to swing front and back
void MiniKame::frontBack(float steps, float T = 600) {
  int x_amp = 30;                                                                                 // X轴振幅
  int z_amp = 25;                                                                                 // Z轴振幅
  int ap = 20;                                                                                    // 角度偏移
  int hi = 30;                                                                                    // 高度参数
  float period[] = { T, T, T, T, T, T, T, T, T };                                                 // 周期数组，用于设置每个动作的时间长度
  int amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0  };               // 振幅数组，定义了每个伺服电机的振动幅度
  int offset[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 };  // 偏移数组，定义了每个伺服电机的中心位置
  int phase[] = { 0, 180, 270, 90, 0, 180, 90, 270, 0 };                                          // 相位数组，定义了每个伺服电机动作的起始相位

  execute(steps, period, amplitude, offset, phase);  // 执行动作
}


// MiniKame类中的奔跑动作
// MiniKame class method to run
void MiniKame::run(float steps, float T = 5000) {
  int x_amp = 25;                                                                    // X轴振幅
  int z_amp = 15;                                                                    // Z轴振幅
  int ap = 15;                                                                       // 角度偏移
  int hi = 15;                                                                       // 高度参数
  int front_x = 9;                                                                   // 前足X轴偏移参数
  float period[] = { T, T, T, T, T, T, T, T, T };                                   // 周期数组，用于设置每个动作的时间长度
  int amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0 };  // 振幅数组，定义了每个伺服电机的振动幅度
  int offset[] = { 90 + ap - front_x, 90 - ap + front_x, 90 - hi, 90 + hi, 90 - ap - front_x, 90 + ap + front_x, 90 + hi, 90, 90 }; // 偏移数组，定义了每个伺服电机的中心位置
  int phase[] = { 0, 0, 90, 90, 180, 180, 90, 90 };  // 相位数组，定义了每个伺服电机动作的起始相位

// MiniKame类中的奔跑动作
// MiniKame class method to perform walking
  execute(steps, period, amplitude, offset, phase);  // 执行动作
}

// MiniKame类中的后退动作
// MiniKame class method to run backward
void MiniKame::runback(float steps, float T = 5000) {
  int x_amp = 25;                                                                                                               // X轴振幅
  int z_amp = 15;                                                                                                               // Z轴振幅
  int ap = 15;                                                                                                                  // 角度偏移
  int hi = 15;                                                                                                                  // 高度参数
  int front_x = 9;                                                                                                              // 前足X轴偏移参数
  float period[] = { T, T, T, T, T, T, T, T, T };                                                                               // 周期数组，用于设置每个动作的时间长度
  int amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0 };                                              // 振幅数组，定义了每个伺服电机的振动幅度
  int offset[] = { 90 + ap - front_x, 90 - ap + front_x, 90 - hi, 90 + hi, 90 - ap - front_x, 90 + ap + front_x, 90 + hi, 90 }; // 偏移数组，定义了每个伺服电机的中心位置
  int phase[] = { 0, 0, 90, 90, 180, 180, 90, 90 };                                                                             // 相位数组，定义了每个伺服电机动作的起始相位

  execute(steps, period, amplitude, offset, phase);                                                                             // 执行动作
}


// MiniKame类中的左侧月球漫步动作
// MiniKame class method to perform moonwalk to the left
void MiniKame::moonwalkL(float steps, float T = 5000) {
  int z_amp = 45;                                                   // Z轴振幅，控制机器人的前后摆动幅度
  float period[] = { T, T, T, T, T, T, T, T, T };                   // 周期数组，设置每个动作的时间长度
  int amplitude[] = { 0, 0, z_amp, z_amp, 0, 0, z_amp, z_amp, 0 };  // 振幅数组，定义了每个伺服电机的振动幅度，这里只有Z轴伺服有振幅
  int offset[] = { 90, 90, 90, 90, 90, 90, 90, 90, 90 - 96 };       // 偏移数组，设置每个伺服电机的中心位置，所有位置都设为90，但最后一个有所偏移
  int phase[] = { 0, 0, 0, 120, 0, 0, 180, 290, 0 };                // 相位数组，定义了每个伺服电机动作的起始相位，创建不同的运动序列以模拟月球漫步的步态

  execute(steps, period, amplitude, offset, phase);  // 调用execute方法执行动作
}


// 四足机器人类中的前进动作
// MiniKame class method to perform walking
void MiniKame::walk(float steps, float T = 5000) {
  volatile int x_amp = 20;                                                                                                                         // 横向振幅
  volatile int z_amp = 35;                                                                                                                         // 纵向振幅  
  volatile int ap = 20;                                                                                                                            // 角度偏移
  volatile int hi = 15;                                                                                                                            // 提高振幅
  volatile int front_x = 12;                                                                                                                       // 前腿横向振幅调整值
  volatile float period[] = {T, T, T/2, T/2, T, T, T/2, T/2, T};                                                                            // 周期数组
  volatile int amplitude[] = {x_amp,x_amp,z_amp,z_amp,x_amp,x_amp,z_amp,z_amp,5};                                                           // 振幅数组，纵向振幅设为0使前后腿同步
  volatile int offset[] = {90-ap-front_x,90+ap+front_x,90-hi+10,90+hi-10,90+ap-front_x,90-ap+front_x,90+hi-5,90-hi+5,90};      // 偏移数组
  volatile int  phase[] = {90, 90, 270, 90, 270, 270, 90, 270, 0};                                                                        // 相位数组，定义了每个伺服电机动作的起始相位

  // 初始化所有振荡器并设置它们的周期、振幅、相位和偏移
  // Initialize all oscillators and set their period, amplitude, phase, and offset
  for (int i = 0; i < 9; i++) {
    oscillator[i].reset();
    oscillator[i].setPeriod(period[i]);
    oscillator[i].setAmplitude(amplitude[i]);
    oscillator[i].setPhase(phase[i]);
    oscillator[i].setOffset(offset[i]);
  }

  _final_time = millis() + period[0] * steps;                     // 计算后退动作的结束时间
  _init_time = millis();                                          // 获取初始时间
  bool side;                                                      // 用于记录当前步行的是哪一边
  while (millis() < _final_time) {                                // 在结束时间之前，循环执行后退动作
    side = (int)((millis() - _init_time) / (period[0] / 2)) % 2;  // 根据时间判断后退的是左脚还是右脚
    setServo(0, oscillator[0].refresh());                         // 刷新伺服电机的位置
    setServo(1, oscillator[1].refresh());
    setServo(4, oscillator[4].refresh());
    setServo(5, oscillator[5].refresh());

    if (side == 0) {  // 如果是右边的步子，刷新右边的腿部伺服电机
      setServo(3, oscillator[3].refresh());
      setServo(6, oscillator[6].refresh());
    } else {  // 如果是左边的步子，刷新左边的腿部伺服电机
      setServo(2, oscillator[2].refresh());
      setServo(7, oscillator[7].refresh());
    }
    //delay(1);  // 暂停1毫秒后继续，保持运动的连续性
  }
}

// MiniKame类中的后退动作-原始版本未调整
// MiniKame class method to perform walking backwards
void MiniKame::walkback(float steps, float T=5000){
  volatile int x_amp = 20;                                                                                                                     // 横向振幅，控制左右摆动的幅度
  volatile int z_amp = 15;                                                                                                                     // 纵向振幅，控制前后摆动的幅度
  volatile int ap = 20;                                                                                                                        // 角度偏移，用于调整行走时腿部的位置
  volatile int hi = 15;                                                                                                                        // 提高振幅，用于控制步行时抬腿的高度
  volatile int back_x = 12;                                                                                                                    // 后腿横向振幅的调整值
  volatile float period[] = { T, T, T / 2, T / 2, T, T, T / 2, T / 2 };                                                                     // 周期数组，设置每个动作的时间长度
  volatile int amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp };                                                    // 振幅数组，定义了每个伺服电机的振动幅度
  volatile int offset[] = { 90 + ap + back_x, 90 - ap - back_x, 90 - hi, 90 + hi, 90 - ap + back_x, 90 + ap - back_x, 90 + hi, 90 - hi };  // 偏移数组，设置每个伺服电机的中心位置
  volatile int  phase[] = {270, 270, 270, 90, 90, 90, 90, 270, 0};                                                                      // 相位数组，定义了每个伺服电机动作的起始相位

  // 初始化所有振荡器并设置它们的周期、振幅、相位和偏移
  // Initialize all oscillators and set their period, amplitude, phase, and offset
  for (int i = 0; i < 9; i++) {
    oscillator[i].reset();
    oscillator[i].setPeriod(period[i]);
    oscillator[i].setAmplitude(amplitude[i]);
    oscillator[i].setPhase(phase[i]);
    oscillator[i].setOffset(offset[i]);
  }

  _final_time = millis() + period[0] * steps;                     // 计算后退动作的结束时间
  _init_time = millis();                                          // 获取初始时间
  bool side;                                                      // 用于记录当前步行的是哪一边
  while (millis() < _final_time) {                                // 在结束时间之前，循环执行后退动作
    side = (int)((millis() - _init_time) / (period[0] / 2)) % 2;  // 根据时间判断后退的是左脚还是右脚
    setServo(0, oscillator[0].refresh());                         // 刷新伺服电机的位置
    setServo(1, oscillator[1].refresh());
    setServo(4, oscillator[4].refresh());
    setServo(5, oscillator[5].refresh());

    if (side == 0) {  // 如果是右边的步子，刷新右边的腿部伺服电机
      setServo(3, oscillator[3].refresh());
      setServo(6, oscillator[6].refresh());
    } else {  // 如果是左边的步子，刷新左边的腿部伺服电机
      setServo(2, oscillator[2].refresh());
      setServo(7, oscillator[7].refresh());
    }
    delay(1);  // 暂停1毫秒后继续，保持运动的连续性
  }
}

// MiniKame类中的蹦跶动作
// MiniKame class method to perform an up and down motion
void MiniKame::upDown(float steps, float T = 500) {
  int x_amp = -20;                                                                 // 横向振幅设置为0，表示不左右摆动
  int z_amp = 70;                                                                // 纵向振幅，控制机器人上下运动的幅度
  int ap = -20;                                                                   // 角度偏移，用于调整腿部的位置
  int hi = 0;                                                                   // 提高振幅，用于控制腿部抬高的幅度
  int front_x = 0;                                                               // 前腿横向振幅的调整值，这里设置为0
  float period[] = { T, T, T, T, T, T, T, T };                                // 周期数组，设置每个动作的时间长度
  int amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp };  // 振幅数组，定义了每个伺服电机的振动幅度
  int offset[] = { 90 + ap - front_x,  90 - ap + front_x, 90 - hi, 90 + hi, 90 - ap - front_x, 90 + ap + front_x, 90 + hi, 90 - hi, 90 }; // 偏移数组，设置每个伺服电机的中心位置
  int phase[] = { 90, 90, 90, 270, 180, 180, 270, 90, 0 };  // 相位数组，定义了每个伺服电机动作的起始相位

  // 使用execute方法来执行上下动作
  // Use the execute method to perform the up and down motion
  execute(steps, period, amplitude, offset, phase);
}



// MiniKame类中的俯卧撑动作动作
// MiniKame class method to perform push-ups
void MiniKame::pushUp(float steps, float T = 600) {
  int z_amp = 40;                                                                             // 纵向振幅，控制上下运动的幅度
  int x_amp = 65;                                                                             // 横向振幅，控制左右运动的幅度
  int hi = 30;                                                                                // 提高振幅，用于控制腿部抬高的幅度
  float period[] = { T, T, T, T, T, T, T, T };                                             // 周期数组，设置每个动作的时间长度
  int amplitude[] = { 0, 0, z_amp, z_amp, 0, 0, 0, 0 };                                       // 振幅数组，定义了每个伺服电机的振动幅度
  int offset[] = { 90, 90, 90 - hi, 90 + hi, 90 - x_amp, 90 + x_amp, 90 + hi, 90 - hi, 90 };  // 偏移数组，设置每个伺服电机的中心位置
  int phase[] = { 0, 0, 0, 180, 0, 0, 0, 0, 0 };                                              // 相位数组，定义了每个伺服电机动作的起始相位

  // 使用execute方法来执行俯卧撑动作
  // Use the execute method to perform the push-up motion
  execute(steps, period, amplitude, offset, phase);
}

// MiniKame类中的打招呼动作方法
// MiniKame class method to perform a greeting (hello) motion
void MiniKame::hello() {
  int T = 800;                                                                          // 原本是350，现在设置为800，控制周期时间长度
  float period[] = { T, T, T, T, T, T, T, T, T };                                       // 同上，周期数组
  int amplitude[] = { 0, 40, 0, 40, 0, 0, 0, 0, 2 };                                    // 振幅数组，部分振幅设置为40，使得部分腿部进行振动
  int offset[] = { 90 + 15, 40, 90 - 65, 90, 90 + 10, 90 - 10, 90 + 20, 90 - 20, 90 };  // 偏移数组，调整中心位置
  int phase[] = { 0, 0, 0, 90, 0, 0, 0, 0, 0 };                                         // 相位数组，设置动作的起始相位

  // 执行打招呼动作，重复4次
  // Execute the greeting motion, repeating it 4 times
  execute(4, period, amplitude, offset, phase);
}

// MiniKame类中的再见动作方法
// MiniKame class method to perform a goodbye (bye) motion
void MiniKame::bye() {
  int T = 800;                                                                                    // 同上，设置周期时间长度为800
  float period[] = { T, T, T, T, T, T, T, T };                                                 // 周期数组
  int amplitude[] = { 40, 40, 90, 90, 0, 0, 0, 2 };                                               // 振幅数组，设置振幅值
  int offset[] = { 90 + 50, 90 - 50, 90 - 15, 90 + 15, 90 + 20, 90 - 20, 90 + 10, 90 - 10, 90 };  // 偏移数组
  int phase[] = { 0, 0, 90, 90, 0, 0, 0, 0, 0 };                                                  // 相位数组

  // 执行再见动作，重复4次
  // Execute the goodbye motion, repeating it 4 times
  execute(4, period, amplitude, offset, phase);
}

// MiniKame类中的害怕动作方法
// MiniKame class method to perform an afraid motion
void MiniKame::afraid() {
  float afraidPositionLeft[] = { 90 + 15, 90 - 15, 90 - 65, 90 + 65, 90 + 20, 90 - 20, 90 + 10, 90 - 10, 90 - 3 };   // 左侧害怕位置
  moveServos(150, afraidPositionLeft);                                                                               // 移动伺服电机至左侧害怕位置
  float afraidPositionRight[] = { 90 + 15, 90 - 15, 90 - 65, 90 + 65, 90 + 20, 90 - 20, 90 + 10, 90 - 10, 90 + 3 };  // 右侧害怕位置
  moveServos(150, afraidPositionRight);                                                                              // 移动伺服电机至右侧害怕位置
}

// MiniKame类中的困惑动作方法
// MiniKame class method to perform a confused motion
void MiniKame::confused() {  // 向上的困惑动作
  int ap = 20;
  int hi = 35;
  int up = 30 + random(5, 20);                                                                                                             // 向上动作的随机增量
  int down = 5 + random(5, 20);                                                                                                            // 向下动作的随机增量
  float confusedPositionA[] = { 90 + ap, 90 - ap, 90 - hi + up, 90 + hi + down, 90 - ap, 90 + ap, 90 + hi - up, 90 - hi - down, 90 + 5 };  // 困惑位置A
  moveServos(random(200, 700), confusedPositionA);                                                                                         // 移动伺服电机至困惑位置A
  delay(random(800, 1400));                                                                                                                // 随机延迟一段时间
  float confusedPositionB[] = { 90 + ap, 90 - ap, 90 - hi - down, 90 + hi - up, 90 - ap, 90 + ap, 90 + hi + down, 90 - hi + up, 90 - 5 };  // 困惑位置B
  moveServos(random(200, 700), confusedPositionB);                                                                                         // 移动伺服电机至困惑位置B
  delay(random(800, 1400));                                                                                                                // 再次随机延迟
}

// MiniKame类中的点头（表示“是”）动作方法
// MiniKame class method to perform a nodding (saying "yes") motion
void MiniKame::shakeYes(float steps, float T = 400) {                                             // 表示“是”的点头动作
  int z_amp = 20;                                                                                 // 纵向振幅，控制上下运动的幅度
  int x_amp = 30;                                                                                 // 横向振幅，控制左右运动的幅度
  int ap = 25;                                                                                    // 角度偏移
  int hi = 30;                                                                                    // 提高振幅
  float period[] = { T, T, T, T, T, T, T, T };                                                 // 周期数组
  int amplitude[] = { 0, 0, z_amp, z_amp, 0, 0, 0, 0 };                                           // 振幅数组
  int offset[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 };  // 偏移数组
  int phase[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };                                                  // 相位数组

  // 使用execute方法来执行点头动作
  // Use the execute method to perform the nodding motion
  execute(steps, period, amplitude, offset, phase);
}

// MiniKame类中的头摇（表示“不”）动作方法
// MiniKame class method to perform a head shaking (saying "no") motion
void MiniKame::shakeNo() {                                                                                            // 表示“不”的头摇动作
  int ap = 20;                                                                                                        // 角度偏移
  int hi = 35;                                                                                                        // 提高振幅
  float shakeNoPositionLeft[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 - 45 };  // 左侧“不”位置
  moveServos(100, shakeNoPositionLeft);                                                                               // 移动伺服电机至左侧“不”位置

  float shakeNoPositionRight[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 + 45 };  // 右侧“不”位置
  moveServos(100, shakeNoPositionRight);                                                                               // 移动伺服电机至右侧“不”位置
}

// MiniKame类中的摇头动作方法（头部前后摆动）
// MiniKame class method to perform a headbanging motion (head moving back and forth)
void MiniKame::headbang(float steps, float T = 400) {                                             // 摇头动作
  int z_amp = 30;                                                                                 // 纵向振幅
  int x_amp = 30;                                                                                 // 横向振幅
  int ap = 25;                                                                                    // 角度偏移
  int hi = 30;                                                                                    // 提高振幅
  float period[] = { T, T, T, T, T, T, T, T };                                                 // 周期数组
  int amplitude[] = { 0, 0, z_amp, z_amp, 0, 0, z_amp, z_amp };                                   // 振幅数组
  int offset[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 };  // 偏移数组
  int phase[] = { 0, 0, 0, 180, 0, 0, 0, 180, 0 };                                                // 相位数组

  // 使用execute方法来执行摇头动作
  // Use the execute method to perform the headbanging motion
  execute(steps, period, amplitude, offset, phase);
}


// MiniKame类中的向左看动作方法
// MiniKame class method to perform a look to the left action
void MiniKame::lookLeft() {  // 向左看
  int ap = 20;
  int hi = 35;
  float lookLeftPosition[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 + 45 };
  moveServos(150, lookLeftPosition);  // 将伺服电机移动到向左看的位置
}

// MiniKame类中的向右看动作方法
// MiniKame class method to perform a look to the right action
void MiniKame::lookRight() {  // 向右看
  int ap = 20;
  int hi = 35;
  float lookRightPosition[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 - 45 };
  moveServos(150, lookRightPosition);  // 将伺服电机移动到向右看的位置
}

// MiniKame类中的向上看动作方法
// MiniKame class method to perform a look up action
void MiniKame::lookUp() {  // 向上看
  int ap = 20;
  int hi = 35;
  float lookUpPosition[] = { 90 + ap, 90 - ap, 90 - hi - 35, 90 + hi + 35, 90 - ap, 90 + ap, 90 + hi - 35, 90 - hi + 35, 90 };
  moveServos(150, lookUpPosition);  // 将伺服电机移动到向上看的位置
}

// MiniKame类中的向左上方看动作方法
// MiniKame class method to perform a look up-left action
void MiniKame::lookUpLeft() {  // 向左上方看
  int ap = 20;
  int hi = 35;
  float lookUpLeftPosition[] = { 90 + ap, 90 - ap, 90 - hi - 35, 90 + hi + 35, 90 - ap, 90 + ap, 90 + hi - 35, 90 - hi + 35, 90 - 45 };
  moveServos(150, lookUpLeftPosition);  // 将伺服电机移动到向左上方看的位置
}

// MiniKame类中的向右上方看动作方法
// MiniKame class method to perform a look up-right action
void MiniKame::lookUpRight() {  // 向右上方看
  int ap = 20;
  int hi = 35;
  float lookUpRightPosition[] = { 90 + ap, 90 - ap, 90 - hi - 35, 90 + hi + 35, 90 - ap, 90 + ap, 90 + hi - 35, 90 - hi + 35, 90 + 45 };
  moveServos(150, lookUpRightPosition);  // 将伺服电机移动到向右上方看的位置
}

// MiniKame类中的向下看动作方法
// MiniKame class method to perform a look down action
void MiniKame::lookDown() {  // 向下看
  int ap = 20;
  int hi = 35;
  float lookDownPosition[] = { 90 + ap, 90 - ap, 90 - hi + 35, 90 + hi - 35, 90 - ap, 90 + ap, 90 + hi + 35, 90 - hi - 35, 90 };
  moveServos(150, lookDownPosition);  // 将伺服电机移动到向下看的位置
}

// MiniKame类中的向左下方看动作方法
// MiniKame class method to perform a look down-left action
void MiniKame::lookDownLeft() {  // 向左下方看
  int ap = 20;
  int hi = 35;
  float lookDownLeftPosition[] = { 90 + ap, 90 - ap, 90 - hi + 35, 90 + hi - 35, 90 - ap, 90 + ap, 90 + hi + 35, 90 - hi - 35, 90 - 45 };
  moveServos(150, lookDownLeftPosition);  // 将伺服电机移动到向左下方看的位置
}

// MiniKame类中的向右下方看动作方法
// MiniKame class method to perform a look down-right action
void MiniKame::lookDownRight() {  // 向右下方看
  int ap = 20;
  int hi = 35;
  float lookDownRightPosition[] = { 90 + ap, 90 - ap, 90 - hi + 35, 90 + hi - 35, 90 - ap, 90 + ap, 90 + hi + 35, 90 - hi - 35, 90 + 45 };
  moveServos(150, lookDownRightPosition);  // 将伺服电机移动到向右下方看的位置
}

// MiniKame类中的将头部归位到中心的动作方法
// MiniKame class method to center the head position
void MiniKame::center() {  // 头部居中
  int ap = 20;
  int hi = 35;
  float centerPosition[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 };
  moveServos(150, centerPosition);  // 将伺服电机移动到中心位置
}

// MiniKame类中的休息动作方法，使机器人进入休息状态
// MiniKame class method to perform a rest action, putting the robot in a rest state
void MiniKame::rest() {
  int ap = 20;
  int hi = -15;
  float restPosition[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 };
  moveServos(150, restPosition);  // 将伺服电机移动到休息位置
}

// MiniKame类中的将机器人各部位回归到初始位置的方法
// MiniKame class method to return the robot to its home (default) position
void MiniKame::home() {
  int ap = 0;  // 角度偏移
  int hi = 0;  // 提高振幅
  int position[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 };
  for (int i = 0; i < 9; i++) setServo(i, position[i]);  // 循环设置每个伺服电机到初始位置
}

void MiniKame::imitate(int s0,int s1,int s2,int s3,int s4,int s5,int s6,int s7,int s8) {

  int position[] = { s0, s1, s2, s3, s4, s5, s6, s7, s8 };
  for (int i = 0; i < 9; i++) setServo(i, position[i]);  // 循环设置每个伺服电机到初始位置
}

// MiniKame类中的撒尿动作方法，可能与特定动作相关
// MiniKame class method for a specific action, possibly related to a 'pie' motion
void MiniKame::pie() {
  int ap = 20;
  int hi = 35;
  float piePositionA[] = { 90 + ap, 90 - ap + 15, 90 - hi + 60, 90 + hi + 30, 90 - ap - 45, 90 + ap - 30, 90 + hi + 10, 90 - hi + 90, 90 - 5 };
  moveServos(150, piePositionA);  // 移动到某个特定的位置A
  float piePositionB[] = { 90 + ap, 90 - ap + 15, 90 - hi + 60, 90 + hi + 30, 90 - ap - 45, 90 + ap - 15, 90 + hi + 10, 90 - hi + 90, 90 - 5 };
  moveServos(150, piePositionB);  // 移动到另一个特定的位置B
}

// MiniKame类中的校准动作方法，将机器人的部位调整到校准位置
// MiniKame class method to calibrate the robot, adjusting its parts to a calibration position
void MiniKame::calibrate() {
  int ap = 0;    // 角度偏移设为0
  int hi = -15;  // 提高振幅设为-15
  int position[] = { 90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi, 90 };
  for (int i = 0; i < 9; i++) setServo(i, position[i]);  // 循环设置每个伺服电机到校准位置
}

// MiniKame类中的将所有伺服电机角度设为90度的方法
// MiniKame class method to set all servo angles to 90 degrees
void MiniKame::zero() {
  for (int i = 0; i < 9; i++) setServo(i, 90);  // 将所有伺服电机设置为90度，即中心位置
}

// MiniKame类中的反转伺服电机方向的方法
// MiniKame class method to reverse the direction of a servo motor
void MiniKame::reverseServo(int id) {
  if (reverse[id])
    reverse[id] = 0;  // 如果已经反转，则取消反转
  else
    reverse[id] = 1;  // 如果未反转，则进行反转
}

// MiniKame类中的设置伺服电机目标角度的方法
// MiniKame class method to set the target angle for a servo motor
void MiniKame::setServo(int id, float target) {
  if (!reverse[id])
    servo[id].writeMicroseconds(angToUsec(target + trim[id]));  // 设置伺服电机到目标角度（未反转）
  else
    servo[id].writeMicroseconds(angToUsec(180 - (target + trim[id])));  // 设置伺服电机到目标角度（已反转）
  _servo_position[id] = target;                                         // 更新当前伺服电机的位置
}

// MiniKame类中的获取伺服电机当前角度的方法
// MiniKame class method to get the current angle of a servo motor
float MiniKame::getServo(int id) {
  return _servo_position[id];  // 返回指定伺服电机的当前角度
}

// MiniKame类中的移动伺服电机到目标位置的方法
// MiniKame class method to move the servos to target positions
void MiniKame::moveServos(int time, float target[9]) {
  if (time > 10) {
    for (int i = 0; i < 9; i++) _increment[i] = (target[i] - _servo_position[i]) / (time / 10.0);
    _final_time = millis() + time;

    while (millis() < _final_time) {
      _partial_time = millis() + 10;
      for (int i = 0; i < 9; i++) setServo(i, _servo_position[i] + _increment[i]);
      while (millis() < _partial_time)
        ;  // 暂停一段时间
    }
  } else {
    for (int i = 0; i < 9; i++) setServo(i, target[i]);
  }
  for (int i = 0; i < 9; i++) _servo_position[i] = target[i];  // 更新所有伺服电机的位置
}

// MiniKame类中的"执行"特定动作的方法
// MiniKame class method to execute specific actions
void MiniKame::execute(float steps, float period[9], int amplitude[9], int offset[9], int phase[9]) {

  for (int i = 0; i < 9; i++) {
    oscillator[i].setPeriod(period[i]);        // 设置振荡器的周期
    oscillator[i].setAmplitude(amplitude[i]);  // 设置振荡器的振幅
    oscillator[i].setPhase(phase[i]);          // 设置振荡器的相位
    oscillator[i].setOffset(offset[i]);        // 设置振荡器的偏移
  }

  unsigned long global_time = millis();

  for (int i = 0; i < 9; i++) oscillator[i].setTime(global_time);  // 设置全局时间

  _final_time = millis() + period[0] * steps;
  while (millis() < _final_time) {
    for (int i = 0; i < 9; i++) {
      setServo(i, oscillator[i].refresh());  // 更新伺服电机的位置
    }
    yield();  // 让出CPU时间
  }
}
