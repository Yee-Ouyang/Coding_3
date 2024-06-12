#ifndef commandexecutor_h // 如果未定义commandexecutor_h，则定义它，确保头文件只被包含一次
#define commandexecutor_h

#include <Arduino.h> // 引入Arduino基础库
#include "minikame.h" // 引入MiniKame机器人的定义
//#include <ESP8266mDNS.h> //包含mDNS,可以在局域网内输入http://robot.local找到机器人，省得进路由器找IP麻烦。
// CommandExecutor类，用于执行来自输入的命令
class CommandExecutor {
  public:
    void init(MiniKame* kame); // 初始化方法，设置机器人实例
    void parseCommand(const String& command); // 解析并执行命令的方法
  private:
    MiniKame * robot; // 指向MiniKame机器人实例的指针
    bool running; // 标志位，表示机器人是否在执行命令
};

#endif // 结束条件编译，确保头文件定义只被包含一次
