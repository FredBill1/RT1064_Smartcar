#ifndef _ICM20948CFG_hpp
#define _ICM20948CFG_hpp

#include <cstdint>
// Default = +/- 4g. Valid ranges: 2, 4, 8, 16
enum AccelerometerFSR {
    AccelFSR2g = 2,
    AccelFSR4g = 4,  // default
    AccelFSR8g = 8,
    AccelFSR16g = 16,
};

// Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000
enum GyroscopeFSR {
    GyroFSR250dps = 250,
    GyroFSR500dps = 500,
    GyroFSR1000dps = 1000,
    GyroFSR2000dps = 2000,  // default
};

static int32_t cfg_acc_fsr = AccelFSR4g;
static int32_t cfg_gyr_fsr = GyroFSR2000dps;

// clang-format off
// ��������
static constexpr bool use_uncal_mag        = 0; // δУ׼�ų�
static constexpr bool use_uncal_gyro       = 0; // δУ׼������
static constexpr bool use_accel            = 0; // ���ٶȼ�
static constexpr bool use_gyro             = 0; // ������
static constexpr bool use_mag              = 0; // �ų�
static constexpr bool use_gravity          = 0; // ����
static constexpr bool use_linear_accel     = 1; // �߼��ٶ�
static constexpr bool use_rpy_orientation  = 0; // 9DOF RPY
static constexpr bool use_6DOF_orientation = 1; // 6DOFλ��
static constexpr bool use_9DOF_orientation = 0; // 9DOFλ��
static constexpr bool use_mag_orientation  = 0; // �ش�λ��

// ����/ms
static constexpr uint32_t period_uncal_mag        = 100; // δУ׼�ų� 1~255Hz
static constexpr uint32_t period_uncal_gyro       = 100; // δУ׼������ 1~255Hz
static constexpr uint32_t period_accel            = 100; // ���ٶȼ� 1~225Hz
static constexpr uint32_t period_gyro             = 20 ; // ������ 1~225Hz
static constexpr uint32_t period_mag              = 100; // �ų� 1~70Hz
static constexpr uint32_t period_gravity          = 20 ; // ���� 50~255Hz (����6DOFλ��)
static constexpr uint32_t period_linear_accel     = 20 ; // �߼��ٶ� 50~255Hz (����6DOFλ�˺ͼ��ٶȼ�)
static constexpr uint32_t period_rpy_orientation  = 20 ; // 9DOF RPY 50~255Hz (����9DOFλ��)
static constexpr uint32_t period_6DOF_orientation = 20 ; // 6DOFλ�� 50~255Hz
static constexpr uint32_t period_9DOF_orientation = 20 ; // 9DOFλ�� 50~255Hz
static constexpr uint32_t period_mag_orientation  = 20 ; // �ش�λ�� 1~255Hz
// clang-format on

#endif  // _ICM20948CFG_hpp