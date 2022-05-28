#ifndef _driver_Camera_hpp
#define _driver_Camera_hpp

#include <cstdint>

class Camera {
 public:
    static constexpr int W = 752;
    static constexpr int H = 480;

    void init();
    uint8_t* snapshot();
    void release();

    void read_config();
    void write_config();

    int16_t get_auto_exposure();
    void set_auto_exposure(int16_t value);

    int16_t get_exposure_time();
    void set_exposure_time(int16_t value);

    int16_t get_fps();
    void set_fps(int16_t value);

    int16_t get_gain();
    void set_gain(int16_t value);
};

extern Camera camera;

#endif  // _driver_Camera_hpp