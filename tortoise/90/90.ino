#include <Servo.h>

// 创建Servo对象来控制舵机
Servo leg1Servo1;  // 第一条腿的第一个舵机
Servo leg1Servo2;  // 第一条腿的第二个舵机
Servo leg2Servo1;
Servo leg2Servo2;
Servo leg3Servo1;
Servo leg3Servo2;
Servo leg4Servo1;
Servo leg4Servo2;

void setup() {
  // 将舵机对象与Arduino的PWM引脚关联
  leg1Servo1.attach(9);  // 例如，连接到数字引脚2
  leg1Servo2.attach(7);
  
  leg2Servo1.attach(5);
  leg2Servo2.attach(3);
  
  leg3Servo1.attach(4);
  leg3Servo2.attach(2);
  
  leg4Servo1.attach(8);
  leg4Servo2.attach(6);
 

  // 初始化舵机位置
  initializePosition();
}

void initializePosition() {
  // 所有舵机设置到起始位置（例如，所有舵机角度设置为90度）
  leg1Servo1.write(45);//水平位置
  leg1Servo2.write(90);
  leg2Servo1.write(100);//水平位置
  leg2Servo2.write(90);
  leg3Servo1.write(70);//
  leg3Servo2.write(90);
  leg4Servo1.write(125);
  leg4Servo2.write(70); //水平位置
  delay(5000); // 等待2秒，让舵机到达初始位置
//  leg1Servo1.write(90);
//  leg1Servo2.write(90);
//  leg2Servo1.write(90);
//  leg2Servo2.write(90);
//  leg3Servo1.write(90);
//  leg3Servo2.write(90);
//  leg4Servo1.write(90);
//  leg4Servo2.write(90);
//  delay(5000); // 等待2秒，让舵机到达初始位置
}

void loop() {
  moveForward();
  
  
  }


void moveForward() {
  // 示例：轮流移动每个腿的舵机来模拟行走
  // 注意：以下角度和延时需要根据实际机器人结构调整
  leg1Servo1.write(90);
  delay(300);
  leg1Servo2.write(60);
  delay(300);
  leg1Servo1.write(45);
  delay(300);
  leg1Servo2.write(90);
  delay(300);

 
  leg2Servo1.write(45);
  delay(300);
  leg2Servo2.write(60);
  delay(300);
  leg2Servo1.write(100);
  delay(300);
  leg2Servo2.write(100);
  delay(300);

  leg3Servo1.write(110);
  delay(300);
  leg3Servo2.write(125);
  delay(300);
  leg3Servo1.write(70);
  delay(300);
  leg3Servo2.write(90);
  delay(300);

  leg4Servo1.write(90);
  delay(300);
  leg4Servo2.write(90);
  delay(300);
  leg4Servo1.write(125);
  delay(300);
  leg4Servo2.write(70);
  delay(300);

}
