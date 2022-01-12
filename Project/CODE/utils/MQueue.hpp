#ifndef _MQueue_hpp
#define _MQueue_hpp

// 感觉RTT官方库里的实现有点问题，所以自己写一个；大部分代码应该差不多
#include <rtthread.h>

class MQueue {
 private:
    rt_messagequeue mID;
    void* Pool;

 protected:
    void* tmp;
    const size_t Data_sz;

 public:
    MQueue(size_t data_sz, size_t data_cnt, const char* name = "mq") : Data_sz(data_sz) {
        size_t pool_sz = (sizeof(void*) + RT_ALIGN(data_sz, RT_ALIGN_SIZE)) * data_cnt;
        Pool = rt_malloc(pool_sz);
        tmp = rt_malloc(data_sz);
        rt_mq_init(&mID, name, Pool, Data_sz, pool_sz, RT_IPC_FLAG_FIFO);
    }
    virtual ~MQueue() {
        rt_free(Pool);
        rt_free(tmp);
        rt_mq_detach(&mID);
    }

    /**
     * @brief 向队尾插入数据
     *
     * @param data          要放入的数据
     * @param timeout_tick  超时时间，为0则不阻塞
     * @return true         失败，例如队列已满或超时
     * @return false        成功
     */
    bool pushback(const void* data, int32_t timeout_tick = 0) {
        return rt_mq_send_wait(&mID, data, Data_sz, timeout_tick) != RT_EOK;
    };

    /**
     * @brief 向队首插入数据（不知道为啥官方库里不也弄个设定延时的）
     *
     * @param data      要放入的数据
     * @return true     失败，如队列已满
     * @return false    成功
     */
    bool pushfront(const void* data) { return rt_mq_urgent(&mID, data, Data_sz) != RT_EOK; };

    /**
     * @brief 取出队首数据
     *
     * @param data          按引用原地取出数据
     * @param timeout_tick  超时时间，小于0则永久等待，等于0则立即返回
     * @return true         失败，如超时
     * @return false        成功
     */
    bool popfront(void* data, int32_t timeout_tick = -1) {
        return rt_mq_recv(&mID, data, Data_sz, (timeout_tick < 0) ? -1 : timeout_tick) != RT_EOK;
    };

    /**
     * @brief 非阻塞地放入数据，如果队列已满则弹出最先放入的数据
     *
     * @param data 要放入的数据
     */
    void put(const void* data) {
        while (pushback(data)) popfront(tmp, 0);
    };

    /**
     * @brief 阻塞地获取数据
     *
     * @param data 按引用原地取出数据
     */
    void get(void* data) { popfront(data); };

    /**
     * @brief 获取数据所占字节数
     *
     * @return size_t 数据所占字节数
     */
    size_t data_size() const { return Data_sz; }
};

#endif  // _MQueue_hpp