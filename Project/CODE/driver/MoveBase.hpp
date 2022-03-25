#ifndef _MoveBase_hpp
#define _MoveBase_hpp

#include "utils/FakeAtomic.hpp"

class MoveBase {
 public:
    struct WheelSpeed {
        float L1, L2, R1, R2;
    };
    struct BaseSpeed {
        float x, y, yaw;
    };
    class ControlState {
        int flag;

     public:
        ControlState() : flag(0) {}
        ControlState(int state) { set(state); }
        ControlState(bool L1, bool L2, bool R1, bool R2) { set(L1, L2, R1, R2); }
        inline void set(int state) { flag = state; }
        inline void set(bool L1, bool L2, bool R1, bool R2) {
            flag = ((int)L1) | ((int)L2 << 1) | ((int)R1 << 2) | ((int)R2 << 3);
        }
        ControlState& operator^=(const ControlState& other) {
            flag ^= other.flag;
            return *this;
        }
        ControlState operator^(const ControlState& other) { return ControlState(flag ^ other.flag); }
        inline bool L1() const { return flag & 1; }
        inline bool L2() const { return flag & 2; }
        inline bool R1() const { return flag & 4; }
        inline bool R2() const { return flag & 8; }
    };

 protected:
    FakeAtomicLoader<WheelSpeed> _wheelSpeedLoader;
    FakeAtomicLoader<BaseSpeed> _baseSpeedLoader;
    FakeAtomicLoader<ControlState> _controlStateLoader;
    WheelSpeed _wheelSpeed;
    BaseSpeed _baseSpeed;
    ControlState _controlState;

 public:
    // 车底盘中心到轮子中心的距离的分量, 单位是m
    static constexpr float r_x = 0.198 / 2, r_y = 0.120 / 2 + 0.03;

    inline void cmd_vel(float x, float y, float yaw) {
        WheelSpeed res;
        res.L1 = x - y - yaw * (r_x + r_y);
        res.L2 = x + y - yaw * (r_x + r_y);
        res.R1 = x + y + yaw * (r_x + r_y);
        res.R2 = x - y + yaw * (r_x + r_y);
        _wheelSpeedLoader.store(res);
    }
    inline void cmd_vel(const BaseSpeed& base) { cmd_vel(base.x, base.y, base.yaw); }
    inline void cmd_vel(float L1, float L2, float R1, float R2) { _wheelSpeedLoader.store({L1, L2, R1, R2}); }
    inline void cmd_vel(const WheelSpeed& wheel) { _wheelSpeedLoader.store(wheel); }

    inline void get_vel(float L1, float L2, float R1, float R2) {
        BaseSpeed res;
        res.x = (L1 + L2 + R1 + R2) / 4;
        res.y = (-L1 + L2 + R1 - R2) / 4;
        res.yaw = (-L1 - L2 + R1 + R2) / (4 * (r_x + r_y));
        _baseSpeedLoader.store(res);
    }
    inline void get_vel(WheelSpeed& wheel) { get_vel(wheel.L1, wheel.L2, wheel.R1, wheel.R2); }

    inline void setControlState(bool L1, bool L2, bool R1, bool R2) { _controlStateLoader.emplace(L1, L2, R1, R2); }
    inline void setControlState(int state) { _controlStateLoader.emplace(state); }
    inline void setControlState(const ControlState& state) { _controlStateLoader.store(state); }

    inline bool loadWheelSpeed() { return _wheelSpeedLoader.load(_wheelSpeed); }
    inline bool loadBaseSpeed() { return _baseSpeedLoader.load(_baseSpeed); }
    inline bool loadControlState() { return _controlStateLoader.load(_controlState); }

    inline const WheelSpeed& getWheelSpeed() const { return _wheelSpeed; }
    inline const BaseSpeed& getBaseSpeed() const { return _baseSpeed; }
    inline const ControlState& getControlState() const { return _controlState; }

    MoveBase() {
        cmd_vel(0, 0, 0);
        get_vel(0, 0, 0, 0);
        setControlState(0);
    }
};

#endif  // _MoveBase_hpp