#ifndef _driver_ArmDrv_hpp
#define _driver_ArmDrv_hpp

class ArmDrv {
 public:
    void initial_pose() const;
    void pick() const;
    void before_place() const;
    void place(int index) const;
    void drop(int index) const;
};

#endif  // _driver_ArmDrv_hpp