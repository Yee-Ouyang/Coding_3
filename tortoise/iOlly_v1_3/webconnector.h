#ifndef webconnector_h // 如果未定义 webconnector_h，则定义它
#define webconnector_h // 定义 webconnector_h 以防止头文件重复包含

//#include <ESP8266WiFi.h> // 包含 ESP8266WiFi 库，用于 WiFi 功能
#include <WiFiClient.h>  // 包含 WiFiClient 库，用于创建 WiFi 客户端
//#include <ESP8266mDNS.h> //包含mDNS,可以在局域网内输入http://robot.local找到机器人，省得进路由器找IP麻烦。
//#include <Ticker.h> //定时器库，用于“异步”处理wifi链接问题
class WebConnector { // 定义 WebConnector 类
  public:
    // Initialize the server
    // 初始化服务器的函数
    void init();

    // Handle incoming connections and commands
    // 处理进来的连接和命令
    void handleConnection();

    // Get the most recent command
    // 获取最新的命令
    String getActiveCommand();

  private:
    String input; // 用于存储输入的字符串
    String activeCommand; // 用于存储活跃命令的字符串
    String getHtmlContent();
};

#endif // 结束 ifndef
