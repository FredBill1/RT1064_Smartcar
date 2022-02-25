#include "SerialIO.hpp"
#include "devices.hpp"
#include "rosRT/Topic.hpp"
#include "rosRT/msgs/geometry_msgs.hpp"

using namespace rosRT;
using namespace rosRT::msgs;

void imu6DOFCB(const QuaternionStamped& data) {
    static SerialIO::TxUtil<float, 4, true> txUtil("imu6DOF", 0);
    if (txUtil.txFinished()) {
        txUtil.setAll(data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w);
        wireless.send(txUtil);
    }
}

void imuLinAccCB(const Vector3Stamped& data) {
    static SerialIO::TxUtil<float, 3, true> txUtil("imuLinAcc", 1);
    if (txUtil.txFinished()) {
        txUtil.setAll(data.vector.x, data.vector.y, data.vector.z);
        wireless.send(txUtil);
    }
}

auto imu6DOFSub = Subscriber::create<QuaternionStamped>("imu/6DOF_orientation", 1, imu6DOFCB);
auto imuLinAccSub = Subscriber::create<Vector3Stamped>("imu/linear_accel", 1, imuLinAccCB);