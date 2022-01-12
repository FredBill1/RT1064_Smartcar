#ifndef _MQueue_hpp
#define _MQueue_hpp

// 感觉RTT官方库里的实现有点问题，所以自己写一个；大部分代码应该差不多
#include <rtthread.h>

template <typename T, int N> class MQueue {
 private:
    rt_messagequeue mID;
    char mPool[(sizeof(void*) + RT_ALIGN(sizeof(T), RT_ALIGN_SIZE)) * N];

 public:
    MQueue(const char* name = "mq") { rt_mq_init(&mID, name, mPool, sizeof(T), sizeof(mPool), RT_IPC_FLAG_FIFO); };
    ~MQueue() { rt_mq_detach(&mID); };

    /**
     * @brief 向队尾插入数据
     *
     * @param data          要放入的数据
     * @param timeout_tick  超时时间，为0则不阻塞
     * @return true         失败，例如队列已满或超时
     * @return false        成功
     */
    bool pushback(const T& data, int32_t timeout_tick = 0) {
        return rt_mq_send_wait(&mID, &data, sizeof(data), timeout_tick) != RT_EOK;
    };

    /**
     * @brief 向队首插入数据（不知道为啥官方库里不也弄个设定延时的）
     *
     * @param data      要放入的数据
     * @return true     失败，如队列已满
     * @return false    成功
     */
    bool pushfront(const T& data) { return rt_mq_urgent(&mID, &data, sizeof(data)) != RT_EOK; };

    /**
     * @brief 取出队首数据
     *
     * @param data          按引用原地取出数据
     * @param timeout_tick  超时时间，小于0则永久等待，等于0则立即返回
     * @return true         失败，如超时
     * @return false        成功
     */
    bool popfront(T& data, int32_t timeout_tick = -1) {
        return rt_mq_recv(&mID, &data, sizeof(data), (timeout_tick < 0) ? -1 : timeout_tick) != RT_EOK;
    };

    /**
     * @brief 非阻塞地放入数据，如果队列已满则弹出最先放入的数据
     *
     * @param data 要放入的数据
     */
    void put(const T& data) {
        while (pushback(data)) {
            T tmp;
            popfront(tmp, 0);
        }
    };

    /**
     * @brief 阻塞地获取数据
     *
     * @param data 按引用原地取出数据
     */
    void get(T& data) { popfront(data); };
};

template <typename T> class SensorDataBuf {
 private:
    MQueue<T, 1> mq;

 public:
    void put(const T& data) {
        while (mq.pushback(data)) {
            T tmp;
            mq.popfront(tmp, 0);
        }
    }
    void get(T& data) { mq.popfront(data); }
};

#endif  // _MQueue_hpp