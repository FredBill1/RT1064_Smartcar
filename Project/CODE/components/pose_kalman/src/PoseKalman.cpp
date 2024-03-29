#include "pose_kalman/PoseKalman.hpp"

#include <utility>

#include "map-macro/map.h"
#define MAP_P(x) x
//
#include <crt.h>
#include <rtthread.h>
//
#include "Systick.hpp"
#include "pose_kalman/KFType.hpp"
#include "pose_kalman/SystemModel.hpp"
#include "pose_kalman/measurementTypes.hpp"
#include "utils/FakeAtomic.hpp"
#include "utils/InterruptGuard.hpp"
#include "utils/PeekQueue.hpp"
#include "utils/Stamped.hpp"

namespace pose_kalman {
using namespace Kalman;
using namespace Eigen;

template <class U> using MapAs = Eigen::Map<Eigen::Matrix<T, U::RowsAtCompileTime, U::ColsAtCompileTime>>;
template <class U> using MapAsConst = Eigen::Map<const Eigen::Matrix<T, U::RowsAtCompileTime, U::ColsAtCompileTime>>;

#define MeasurementMemberName(type) type##_m

struct PoseKalman::Impl {
    bool enabled, isFirst;
    uint64_t lastPredict_us;
    SystemModel sys;

#define MeasurementMemberUtil(type) type::Model MeasurementMemberName(type);
    MAP(MeasurementMemberUtil, MeasurementModelTypes);
#undef MeasurementMemberUtil

    pose_kalman_KF<State> kf;
    PeekQueue measurementQueue[(int)MeasurementType::NUM_TYPES];
    void initQueue();
};

void PoseKalman::Impl::initQueue() {
#define initQueueUtil(type)                                                                        \
    measurementQueue[(int)MeasurementType::type].init(MeasurementQueueSize(MeasurementType::type), \
                                                      sizeof(Stamped<T[type::SIZE]>));
    MAP(initQueueUtil, MeasurementModelTypes);
#undef initQueueUtil
}

PoseKalman::PoseKalman() {
    pimpl = (Impl*)rt_malloc(sizeof(Impl));
    pimpl = new (pimpl) Impl();
    RT_ASSERT(pimpl);
    pimpl->enabled = false;
    pimpl->initQueue();
}

PoseKalman::~PoseKalman() {
    pimpl->~Impl();
    rt_free(pimpl);
}
void PoseKalman::setEnabled(bool enabled) {
    InterruptGuard guard;
    if (pimpl->enabled == enabled) return;
    if (enabled) {
        pimpl->isFirst = true;
        for (auto& queue : pimpl->measurementQueue) queue.clear();
    }
    pimpl->enabled = enabled;
}

bool PoseKalman::getEnabled() const {
    InterruptGuard guard;
    return pimpl->enabled;
}

void PoseKalman::setState(const T stateData[], uint64_t timestamp_us) {
    static const Covariance<State> SMALL_COV = Covariance<State>::Identity() * 1e-6;
    if (!getEnabled()) {
        State x = MapAsConst<State>(stateData);
        pimpl->kf.init(x);
        pimpl->kf.setCovariance(SMALL_COV);
    } else {
        enqueMeasurement(MeasurementType::SetState, stateData, timestamp_us);
    }
}
void PoseKalman::setSystemCovariance(const T systemCovariance[]) {
    Covariance<State> cov = MapAsConst<Covariance<State>>(systemCovariance);
    pimpl->sys.setCovariance(cov);
}

void PoseKalman::setPredictionCovariance(const T predictionCovariance[]) {
    Covariance<State> cov = MapAsConst<Covariance<State>>(predictionCovariance);
    pimpl->kf.setCovariance(cov);
}

void PoseKalman::setMeasurementCovariance(MeasurementType measurementType, const T measurementCovariance[]) {
    switch (measurementType) {
#define setMeasurementCovarianceUtil(type)                                                      \
    case MeasurementType::type: {                                                               \
        Covariance<type::Data> cov = MapAsConst<Covariance<type::Data>>(measurementCovariance); \
        pimpl->MeasurementMemberName(type).setCovariance(cov);                                  \
    } break;
        MAP(setMeasurementCovarianceUtil, MeasurementModelTypes);
#undef setMeasurementCovarianceUtil
    default: break;
    }
}

void PoseKalman::enqueMeasurement(MeasurementType measurementType, const T measurementData[], uint64_t timestamp_us) {
    switch (measurementType) {
#define enqueMeasurementUtil(type)                                       \
    case MeasurementType::type: {                                        \
        Stamped<T[type::SIZE]> data;                                     \
        rt_memcpy(data.data, measurementData, sizeof(T) * type::SIZE);   \
        data.timestamp_us = timestamp_us;                                \
        pimpl->measurementQueue[(int)MeasurementType::type].push(&data); \
    } break;
        MAP(enqueMeasurementUtil, MeasurementModelTypes);
#undef enqueMeasurementUtil
    default: break;
    }
}

void PoseKalman::update(uint64_t timestamp_us) {
    if (!getEnabled()) return;
    if (pimpl->isFirst) pimpl->isFirst = false, pimpl->lastPredict_us = timestamp_us;
    Control u;
    int64_t dt;
    for (;;) {
        int64_t max_dt = 0;
        MeasurementType res = MeasurementType::NUM_TYPES;
#define update_max_dt_util(type)                                               \
    {                                                                          \
        auto& queue = pimpl->measurementQueue[(int)MeasurementType::type];     \
        auto& m_time = ((Stamped<T[type::SIZE]>*)queue.front())->timestamp_us; \
        while (queue.peek()) {                                                 \
            dt = systick.get_diff_us(m_time, timestamp_us);                    \
            if (dt < timeout_us) {                                             \
                if (dt > max_dt) max_dt = dt, res = MeasurementType::type;     \
                break;                                                         \
            }                                                                  \
            queue.pop();                                                       \
        }                                                                      \
    }
        MAP(update_max_dt_util, MeasurementModelTypes)
#undef update_max_dt_util

        if (res == MeasurementType::NUM_TYPES) break;

        switch (res) {
#define update_predict_update_util(type)                                       \
    case MeasurementType::type: {                                              \
        auto& queue = pimpl->measurementQueue[(int)MeasurementType::type];     \
        auto& m_time = ((Stamped<T[type::SIZE]>*)queue.front())->timestamp_us; \
        auto& m_data = ((Stamped<T[type::SIZE]>*)queue.front())->data;         \
        dt = systick.get_diff_us(pimpl->lastPredict_us, m_time);               \
        if (dt > 0) {                                                          \
            u.dt() = dt * 1e-6;                                                \
            pimpl->kf.predict(pimpl->sys, u);                                  \
            pimpl->lastPredict_us = m_time;                                    \
        }                                                                      \
        type::Data data;                                                       \
        for (int i = 0; i < type::SIZE; ++i) data[i] = m_data[i];              \
        T& x_yaw = const_cast<State&>(pimpl->kf.getState()).yaw();             \
        if constexpr (type::YAW) {                                             \
            T& m_yaw = data.yaw();                                             \
            m_yaw = x_yaw + wrapAngle(m_yaw - x_yaw);                          \
        }                                                                      \
        /* 对矩形矫正的特殊判断 */                                   \
        if constexpr (std::is_same<type, Rect>::value) data.yaw() = x_yaw;     \
        pimpl->kf.update(pimpl->MeasurementMemberName(type), data);            \
        if constexpr (std::is_same<type, Rect>::value) x_yaw = data.yaw();     \
        x_yaw = wrapAngle(x_yaw);                                              \
        queue.pop();                                                           \
    } break;
            MAP(update_predict_update_util, MeasurementModelTypes)
#undef update_predict_update_util
        default: break;
        }
    }
    dt = systick.get_diff_us(pimpl->lastPredict_us, timestamp_us);
    if (dt > 0) {
        u.dt() = dt * 1e-6;
        pimpl->kf.predict(pimpl->sys, u);
        pimpl->lastPredict_us = timestamp_us;
    }
}

const T* PoseKalman::getState() const { return pimpl->kf.getState().data(); }
}  // namespace pose_kalman