/*
 *  Copyright (c) 2014-2015 InvenSense Inc.
 *  Portions Copyright (c) 2014-2015 Movea.
 *  All rights reserved.
 *
 *  This software, related documentation and any modifications thereto
 *  (collectively "Software") is subject to InvenSense and its licensors'
 *  intellectual property rights under U.S. and international copyright and
 *  other intellectual property rights laws.
 *
 *  InvenSense and its licensors retain all intellectual property and
 *  proprietary rights in and to the Software and any use, reproduction,
 *  disclosure or distribution of the Software without an express license
 *  agreement from InvenSense is strictly prohibited.
 */


/** @defgroup InvScheduler InvScheduler
 *  @brief Simple cooperative scheduler
 *
 *  @ingroup EmbUtils
 *  @{
 */

#ifndef _INV_SCHEDULER_H_
#define _INV_SCHEDULER_H_

#include <stdint.h>

/* Import config if any */
#ifdef     INVSCHEDULER_CFG_INC
  #include INVSCHEDULER_CFG_INC
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* InvScheduler definitions ***************************************************/

/** @brief InvScheduler task state definition
 */
enum InvSchedulerTaskState {
	INVSCHEDULER_TASK_STATE_STOP = 0,        /**< task is stopped             */
	INVSCHEDULER_TASK_STATE_STARTED,         /**< task was started and is not
	                                              scheduled yet               */
	INVSCHEDULER_TASK_STATE_READY,           /**< task is scheduled and waiting
	                                              its time                    */
	INVSCHEDULER_TASK_STATE_RUNNING          /**< task is currently running   */
};

/** @brief Helper constant for priority value
 *  Priority is higher when value is greater
 */
#define INVSCHEDULER_TASK_PRIO_MIN 		1				/**< minimal priority */
#define INVSCHEDULER_TASK_PRIO_NORMAL 	(UINT8_MAX/2)	/**< normal priority  */
#define INVSCHEDULER_TASK_PRIO_MAX 		(UINT8_MAX-1)   /**< maximum priority */

/* Forward declarations */
struct InvScheduler;

/** @brief 	InvSchedulerTask objct states definition
 */
typedef struct InvSchedulerTask {
	void (*func)(void *);               /**< task main function          */
	void *arg;                          /**< task main function          */
#ifdef INVSCHEDULER_TASK_NAME
	const char *name;                   /**< optional name for the task  */
#endif
	uint8_t priority;                   /**< task priority value         */
	enum InvSchedulerTaskState state;   /**< task state                  */
	uint32_t lasttime;                  /**< last time task was executed */
	uint32_t period;                    /**< task period value           */
	uint32_t delay;                     /**< task delay value            */
	struct InvScheduler * scheduler;    /**< reference to scheduler the
	                                         task is attach to           */
	struct InvSchedulerTask *next;      /**< next task in list           */
	struct InvSchedulerTask *prev;      /**< previous task in list       */
} InvSchedulerTask;

/** @brief 	InvScheduler object states definition
 */
typedef struct InvScheduler {
	volatile uint32_t 	currentTime;	/** current time value                    */
	struct InvSchedulerTask *queue;	    /** list of task awaiting to be scheduled */
	void * contextLock;                 /** reference to some context passed to
	                                        lock/unlock macro to protect critical section */
} InvScheduler;

/** @brief Overloadable constant use for tick/time conversion
 *  Tick resolution is in us. Default value corresponds to 1 ms resolution.
 */
#ifndef INVSCHEDULER_PERIOD_US
  #define INVSCHEDULER_PERIOD_US   (1000)
#endif

/** @brief Macros to convert from/to tick/us/ms
 */
#define INVSCHEDULER_TO_MS(tick) (((tick)*(INVSCHEDULER_PERIOD_US))/1000)
#define INVSCHEDULER_TO_US(tick) ((tick)*(INVSCHEDULER_PERIOD_US))
#define INVSCHEDULER_FROM_MS(ms) (((ms)*1000)/(INVSCHEDULER_PERIOD_US))
#define INVSCHEDULER_FROM_US(us) ((us)/(INVSCHEDULER_PERIOD_US))


/* Overloadable locks **********************************************************/

/** @brief Overloadable macro to start a critical section
 *
 *  Although macro is variadic, it expects one or zero argument
 *  If defined with one argument, argument passed to the macros corresponds to
 *  the context lock refernce that can be set with InvScheduler_setLockContext()
 */
#ifndef InvScheduler_lock
  #define InvScheduler_lock(...) (void)0
#endif

/** @brief Overloadable macro to start a critical section
 *
 *  Although macro is variadic, it expects one or zero argument
 *  If defined with one argument, argument passed to the macros corresponds to
 *  the context lock refernce that can be set with InvScheduler_setLockContext()
 */
#ifndef InvScheduler_unlock
  #define InvScheduler_unlock(...) (void)0
#endif


/* Scheduler methods **********************************************************/

/** @brief Reset scheduler states
 *  @param[in] scheduler    handle to scheduler
 */
static inline void InvScheduler_init(InvScheduler *scheduler)
{
	scheduler->currentTime  = 0;
	scheduler->queue        = 0;
	scheduler->contextLock  = 0;
}

/** @brief Set context reference for crtitical section
 *
 *  If InvScheduler_lock/unlock() macros are defined and expect one argument,
 *  this context reference will be passed to those macros
 *
 *  @param[in] context 	pointer to some context passed lock/unlock() macros
 */
static inline void InvScheduler_setLockContext(InvScheduler *scheduler,
		void * context)
{
	scheduler->contextLock  = context;
}

/** @brief Update the current time of one tick
 *  @param[in] scheduler    handle to scheduler
 */
static inline void InvScheduler_updateTime(InvScheduler *scheduler)
{
	++scheduler->currentTime;
}

/** @brief Update the current time of delta tick
 *  @param[in] scheduler    handle to scheduler
 *  @param[in] delta 		number of tick to add to current time
 */
static inline void InvScheduler_updateTimeDelta(InvScheduler *scheduler,
		uint32_t delta)
{
	scheduler->currentTime += delta;
}

/** @brief Return the current time in tick
 *  @param[in] scheduler    handle to scheduler
 *  @return Current tick value
 */
static inline uint32_t InvScheduler_getTime(const InvScheduler *scheduler)
{
	return scheduler->currentTime;
}

/** @brief Get the time in tick of the next task to be executed
 *
 *  This function can be useful to know when to call the dispatcher
 *
 *  @warning If a task is currently running it will be ignored.
 *           This function must not be called from within a task main.
 *
 *  @param[in] scheduler    handle to scheduler
 *  @return number of tick in which next task will be executed
 *          0          indicating imediately
 *          UINT32_MAX indicating nevever (no task to be exectuted)
 */
uint32_t InvScheduler_getNextTimeU(const InvScheduler *scheduler);

/** @brief Identical to InvScheduler_getNextTime() but with locks
 */
static inline uint32_t InvScheduler_getNextTime(const InvScheduler *scheduler)
{
	uint32_t next_time;

	InvScheduler_lock(scheduler->contextLock);
	next_time = InvScheduler_getNextTimeU(scheduler);
	InvScheduler_unlock(scheduler->contextLock);

	return next_time;
}

/** @brief Get the minimum period in tick of all active tasks
 *
 *  This function can be useful to update the call rate of
 *  InvScheduler_updateTime()
 *
 *  @warning If a task is currently running it will be ignored.
 *           This function must not be called from within a task main.
 *
 *  @param[in] scheduler    handle to scheduler
 *  @return minimum period in tick of all tasks
 *          0 meaning there is something to do imdiatly
 *          UINT32_MAX indicating there is nothing to do
 */
uint32_t InvScheduler_getMinPeriodU(const InvScheduler *scheduler);

/** @brief Identical to InvScheduler_getMinPeriodU() but with locks
 */
static inline uint32_t InvScheduler_getMinPeriod(const InvScheduler *scheduler)
{
	uint32_t next_time;

	InvScheduler_lock(scheduler->contextLock);
	next_time = InvScheduler_getMinPeriodU(scheduler);
	InvScheduler_unlock(scheduler->contextLock);

	return next_time;
}

/** @brief The scheduler dispather
 *
 *  Task is executed outside the critical section.
 *  Tasks will be executed in the same context of caller
 *
 *  @warning Should not be called from within a task
 *  @param[in] scheduler    handle to scheduler
 *  @return number of executed tasks
 */
int InvScheduler_dispatchTasks(InvScheduler *scheduler);

/** @brief Run the first task of the ready queue (if timeout reached)
 *  @param[in] scheduler    handle to scheduler
 *  @warning Should not be called from within a task
 *  @return 1 if a task was executed, 0 otherwise
 */
int InvScheduler_dispatchOneTask(InvScheduler *scheduler);

/** @brief Return number of active tasks
 *  @warning Should not be called from within a task
 *  @param[in] scheduler    handle to scheduler
 *  @return Return number of active task
 */
int InvScheduler_getActiveTaskCountU(const InvScheduler *scheduler);

/** @brief Identical to InvScheduler_getActiveTaskCountU() but with locks
 */
static inline int InvScheduler_getActiveTaskCount(const InvScheduler *scheduler)
{
	int count;

	InvScheduler_lock(scheduler->contextLock);
	count = InvScheduler_getActiveTaskCountU(scheduler);
	InvScheduler_unlock(scheduler->contextLock);

	return count;
}

/** @brief Initialize a new task
 *  @param[in] scheduler     handle to the scheduler the task is linked to
 *  @param[in] task          task states
 *  @param[in] cb 		     function to call periodicaly
 *  @param[in] arg 		     task function parameter
 *  @param[in] priority      priority of the task
 *  @param[in] period	     period of the task in tick
 */
void InvScheduler_initTaskDo(InvScheduler *scheduler, InvSchedulerTask *task,
		void (*func)(void *), void *arg, uint8_t prio, uint32_t period);

/** @brief Simitlar to InvScheduler_initTaskDo() but allow to set a name
 */
static inline void InvScheduler_initTask(InvScheduler *scheduler,
		InvSchedulerTask *task, const char *name, void (*func)(void *),
		void *arg, uint8_t prio, uint32_t period)
{
	InvScheduler_initTaskDo(scheduler, task, func, arg, prio, period);
#ifdef INVSCHEDULER_TASK_NAME
	task->name = name;
#else
	(void)name;
#endif
}

/** @brief Start a task after a delay
 *  @param[in] task     task to start
 *  @param[in] delay    delay in tick before executing the task
 */
void InvScheduler_startTaskU(InvSchedulerTask *task, uint32_t delay);

/** @brief Identical to InvScheduler_startTaskU() but with locks
 */
void InvScheduler_startTask(InvSchedulerTask *task, uint32_t delay);

/** @brief 	Stop a task
 *  @param[in] task    handle to task
 */
void InvScheduler_stopTaskU(InvSchedulerTask *task);

/** @brief Identical to InvScheduler_stopTaskU() but with locks
 */
void InvScheduler_stopTask(InvSchedulerTask *task);

/** @brief Change period of a task
 *  @param[in] task    handle to task
 */
static inline void InvScheduler_setTaskPeriodU(InvSchedulerTask *task,
		uint32_t period)
{
	task->period = period;
}

/** @brief Change priority of a task
 *  @param[in] task    handle to task
 */
static inline void InvScheduler_setTaskPrioU(InvSchedulerTask *task,
		uint8_t prio)
{
	task->priority = prio;
}

/* Optionnal hooks called before/after exectuting a task **********************/

#ifndef   InvScheduler_onTaskEnterHook
  #define InvScheduler_onTaskEnterHook(task, time) (void)0
#else
/** @brief Hook to be called before running a task
 *  @param[in] task    task about to be exectuted
 *  @param[in] time    current scheduler tick
 */
extern void InvScheduler_onTaskEnterHook(const InvSchedulerTask *task,
		uint32_t time);
#endif

#ifndef   InvScheduler_onTaskExitHook
  #define InvScheduler_onTaskExitHook(task, time) (void)0
#else
/** @brief Hook to be called after running a task
 *  @param[in] task    task that just got exectute
 *  @param[in] time    current scheduler tick
 */
extern void InvScheduler_onTaskExitHook(const InvSchedulerTask *task,
		uint32_t time);
#endif

/* Debugging functions ********************************************************/

#ifndef NDEBUG

/** @brief Print task states to string
 */
void InvScheduler_printTask(const InvSchedulerTask *task,
		int (*printf_cb)(const char *format, ...));

/** @brief Dumps all tasks from a scheduler to string
 */
void InvScheduler_dumpTasks(const InvSchedulerTask *queue,
		int (*printf_cb)(const char *format, ...));

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
