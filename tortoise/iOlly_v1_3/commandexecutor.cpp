#include "commandexecutor.h"

void CommandExecutor::init(MiniKame* kame) {
  robot = kame;
  running = false;
  // Initialize robot and running state
  // 初始化机器人和运行状态
}

void CommandExecutor::parseCommand(const String& commands) {
  String command = commands.substring(0, 2);
  Serial.println(commands);
  Serial.println(command);
  if (command == "01") {  // walk / 向前
    robot->walk(1, 450);  // Step size 450 / 步长450
    running = true;
  } else if (command == "02") {  // CENTER / 中心位置
    robot->center();
    robot->home();
  } else if (command == "03") {  // LEFT / 向左
    robot->turnL(1, 450);        // Step size 450 / 步长450
    running = true;
  } else if (command == "04") {  // RIGHT / 向右
    robot->turnR(1, 450);        // Step size 450 / 步长450
    running = true;
  } else if (command == "05") {  // home / 重置
    running = false;
    robot->home();
  } else if (command == "06") {  // lookUp / 俯瞰
    robot->lookUp();
  } else if (command == "07") {  // lookDown / 仰视
    robot->lookDown();
  } else if (command == "08") {  // lookLeft / 左看
    robot->lookLeft();
  } else if (command == "09") {  // lookRight / 右看
    robot->lookRight();
  } else if (command == "10") {  // shakeYes / 点头
    robot->shakeYes(2, 200);
  } else if (command == "11") {  // shakeNo / 摇头
    robot->shakeNo();
  } else if (command == "12") {  // PUSHUP / 俯卧撑
    robot->pushUp(2, 1600);
  } else if (command == "13") {  // upDown / 蹦跶
    robot->upDown(8, 200);       // Move up and down 8 times, speed 100 / 上下移动8次，速度100
  } else if (command == "14") {  // walback / 向后
    robot->walkback(1, 450);     // Step size 450 / 步长450
    running = true;
  } else if (command == "15") {  // HELLO / 打招呼
    robot->hello();
  } else if (command == "16") {  // SKATE / 滑冰
    robot->frontBack(4, 300);    // Move front and back 4 times, speed 300 / 前后移动4次，速度300
  } else if (command == "17") {  // DANCE / 跳舞
    robot->dance(2, 800);
  } else if (command == "18") {  // MOONWALK / 月球漫步
    robot->moonwalkL(2, 1600);
  } else if (command == "19") {  // REST / 休息
    robot->rest();
  } else if (command == "20") {  // PIE / 撒尿
    robot->pie();
  } else if (command == "21") {  // LookUpLeft / 抬头向左看
    robot->lookUpLeft();
  } else if (command == "22") {  // LookUpRight / 抬头向右看
    robot->lookUpRight();
  } else if (command == "23") {  // LookDownLeft / 低头向左看
    robot->lookDownLeft();
  } else if (command == "24") {  // LookDownRight / 低头向右看
    robot->lookDownRight();
  } else if (command == "25") {  // calibrate / 校准
    robot->calibrate();
  } else if (command == "26") {  // afraid / 害怕
    robot->afraid();
  } else if (command == "27") {  // bye / 再见
    robot->bye();
  } else if (command == "28") {  // confused / 困惑
    robot->confused();
  } else if (command == "29") {  // run / 奔跑
    robot->run(1, 300);          // Step size 300 / 步长300
    running = true;
  } else if (command == "30") {  // Headbang / 摇头晃脑
    robot->headbang(2, 250);     // Step size 300 / 步长300
    running = true;
  } else if (command == "31") {  // runback / 回撤
    robot->runback(1, 300);
    running = true;
  } else if (command == "32") {
    int s[9];
    for (int i = 0; i < 9; i++) {
      if (i % 2 == 0)
        s[i] = (commands.charAt(i * 2 + 2) - 48) * 10 + (commands.charAt(i * 2 + 3) - 48) + 45;
      else
        s[i] = 135 - ((commands.charAt(i * 2 + 2) - 48) * 10 + (commands.charAt(i * 2 + 3) - 48));
    }
    robot->imitate(s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8]);
  } else {
    robot->home();
  }
}