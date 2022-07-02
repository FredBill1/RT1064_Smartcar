#ifndef _artResult_ResultCatgory_hpp
#define _artResult_ResultCatgory_hpp

#include <cstdint>

namespace ResultCatgory {

enum class Major : uint8_t {
    animal,
    fruit,
    vehicle,
    None,
};

enum class Minor : uint8_t {
    apple,
    banana,
    bus,
    car,
    cat,
    cow,
    dog,
    durian,
    grape,
    horse,
    orange,
    pig,
    plane,
    ship,
    train,
    None,
};

constexpr Minor id_to_minor(uint8_t id) { return static_cast<Minor>(id); }

constexpr Major minor_to_major(Minor minor) {
    switch (minor) {
    // 动物
    case Minor::dog:
    case Minor::horse:
    case Minor::cat:
    case Minor::cow:
    case Minor::pig: return Major::animal;

    // 水果
    case Minor::orange:
    case Minor::apple:
    case Minor::durian:
    case Minor::grape:
    case Minor::banana: return Major::fruit;

    // 交通工具
    case Minor::train:
    case Minor::ship:
    case Minor::plane:
    case Minor::car:
    case Minor::bus: return Major::vehicle;

    // 没有
    default: return Major::None;
    }
}

constexpr uint8_t minor_to_index(Minor minor) {
    switch (minor) {
    // 动物
    case Minor::dog: return 1;
    case Minor::horse: return 2;
    case Minor::cat: return 3;
    case Minor::cow: return 4;
    case Minor::pig: return 5;

    // 水果
    case Minor::orange: return 1;
    case Minor::apple: return 2;
    case Minor::durian: return 3;
    case Minor::grape: return 4;
    case Minor::banana: return 5;

    // 交通工具
    case Minor::train: return 1;
    case Minor::ship: return 2;
    case Minor::plane: return 3;
    case Minor::car: return 4;
    case Minor::bus: return 5;

    // 没有
    default: return 0;
    }
}

constexpr uint8_t major_to_index(Major major) {
    switch (major) {
    case Major::animal: return 1;
    case Major::fruit: return 2;
    case Major::vehicle: return 3;

    // 没有
    default: return 0;
    }
}

}  // namespace ResultCatgory

#endif