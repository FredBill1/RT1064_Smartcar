#ifndef _Encoder_hpp
#define _Encoder_hpp

#include "QTimer.hpp"
#include "Systick.hpp"
#include "parameters.hpp"

class Encoder {
    const QTimer& _qtimer;
    using qtimer_data_t = decltype(_qtimer.get());
    qtimer_data_t _last;
    uint64_t _last_time;
    double mps;  // m/s 米每秒

    // 换算成轮子转轴的转速
    void convert(double delta) {
        constexpr double scalefactor =        // 比例系数
            3.14159265358979323846 / 1024. *  // 编码器转1周产生1024个脉冲
            30. / 70. *                       // 车轮齿数为70, 编码器齿数为30
            6.3 / 100. *                      // 车轮直径6.3cm
            1e6 *                             // 时间戳单位是us
            441. / 450.;                      // 实际测量得到的系数
        uint64_t cur_time = systick.get_us();
        int64_t dt = systick.get_diff_us(_last_time, cur_time);
        _last_time = cur_time;
        mps = delta * scalefactor / dt;
    }

 public:
    Encoder(const QTimer& qtimer) : _qtimer(qtimer) {}
    void init() {
        _last = _qtimer.get();
        _last_time = systick.get_us();
        mps = 0;
    }
    void update() {
        qtimer_data_t cur = _qtimer.get();
        qtimer_data_t delta = cur - _last;
        convert(delta);
        _last = cur;
    }
    // m/s 米每秒
    double get() const { return mps; }
};

#endif  // _Encoder_hpp