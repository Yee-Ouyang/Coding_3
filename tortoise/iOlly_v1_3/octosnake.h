#ifndef octosnake_h // 如果没有定义 octosnake_h，那么定义它
#define octosnake_h // 定义 octosnake_h 用于防止头文件重复包含

//-- Macro for converting from degrees to radians
// 将角度转换为弧度的宏定义
#ifndef DEG2RAD
  #define DEG2RAD(g) ((g)*M_PI)/180 // DEG2RAD 宏，用于将角度 g 转换成弧度
#endif

class Oscillator{ // 定义 Oscillator 类

    public:
        Oscillator(); // 构造函数
        float refresh(); // 刷新并返回当前位置的函数
        void reset(); // 重置参考时间的函数
        float time_to_radians(double time); // 时间转换为弧度的函数
        float degrees_to_radians(float degrees); // 角度转换为弧度的函数
        float degrees_to_time(float degrees); // 角度转换为时间的函数
        void setPeriod(int period); // 设置周期的函数
        void setAmplitude(int amplitude); // 设置振幅的函数
        void setPhase(int phase); // 设置相位的函数
        void setOffset(int offset); // 设置偏移量的函数
        void setTrim(int trim); // 设置微调量的函数
        void setTime(unsigned long ref); // 设置参考时间的函数
        unsigned long getTime(); // 获取参考时间的函数

    private:
        int _period; // 周期变量
        int _amplitude; // 振幅变量
        int _phase; // 相位变量
        int _offset; // 偏移量变量
        int _trim; // 微调量变量
        bool _stop; // 停止标志变量
        unsigned long _ref_time = 0; // 参考时间变量，初始值为 0
        float _delta_time = 0; // 时间差变量，初始值为 0
};

#endif // 结束 ifdef