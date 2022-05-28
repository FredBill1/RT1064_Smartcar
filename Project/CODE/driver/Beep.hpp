#ifndef _driver_Beep_hpp
#define _driver_Beep_hpp

class Beep {
    static int cnt;
    bool enabled = false;
    void check_cnt();

 public:
    void set(bool enabled = true);
};

#endif  // _driver_Beep_hpp