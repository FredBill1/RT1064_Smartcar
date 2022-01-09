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

#include "InvScheduler.h"

static void InvScheduler_insertTask(InvScheduler * scheduler,
		InvSchedulerTask *task)
{
	if(scheduler->queue == 0) {
		task->prev = task;
		task->next = 0;
		scheduler->queue = task;
	} else {
		task->prev = scheduler->queue->prev;
		task->next = 0;
		scheduler->queue->prev->next = task;
		scheduler->queue->prev       = task;
	}
}

static inline void InvScheduler_removeTask(InvScheduler * scheduler,
		InvSchedulerTask *task)
{
	if(scheduler->queue == task) {
		scheduler->queue = task->next;
	} else {
		if(scheduler->queue->prev == task)
			scheduler->queue->prev = task->prev;
		task->prev->next = task->next;
	}
	if(task->next)
		task->next->prev = task->prev;
}

static InvSchedulerTask * InvScheduler_getTaskToSchedule(InvScheduler * scheduler,
		uint32_t now)
{
	InvSchedulerTask * cur  = scheduler->queue;
	InvSchedulerTask * task = 0;
	uint32_t           max_diff = 0;

	while(cur) {
		const uint32_t timeout = (cur->delay != 0) ? cur->delay : cur->period;
		uint32_t elpased;

		/* initalize task states after it was started */
		if(cur->state == INVSCHEDULER_TASK_STATE_STARTED) {
			cur->state    = INVSCHEDULER_TASK_STATE_READY;
			cur->lasttime = now;

			if(cur->delay == 0) {
				cur->lasttime -= cur->period; /* ensure task is run ASAP */
			}
		}

		elpased = (now - cur->lasttime);

		/* check timeout against elpased time */
		if(elpased >= timeout) {
			const uint32_t diff = (elpased - timeout);

			if(task == 0 || diff > max_diff ||
					(diff == max_diff && cur->priority > task->priority)) {
				task = cur;
				max_diff = diff;
			}
		}

		cur = cur->next;
	}

	return task;
}

int InvScheduler_getActiveTaskCountU(const InvScheduler *scheduler)
{
	int count = 0;
	const InvSchedulerTask *cur;

	/* /!\ RUNNING task is not in the queue hence ignored */
	for(cur = scheduler->queue; cur != 0; cur = cur->next)
		++count;

	return count;
}

uint32_t InvScheduler_getNextTimeU(const InvScheduler *scheduler)
{
	const uint32_t           now = scheduler->currentTime;
	const InvSchedulerTask * cur = scheduler->queue;
	uint32_t                 max_diff = UINT32_MAX;

	for(; cur != 0; cur = cur->next ) {
		if(cur->state == INVSCHEDULER_TASK_STATE_STARTED) {
			return 0;
		} else {
			const uint32_t timeout = (cur->delay != 0) ? cur->delay : cur->period;
			const uint32_t elpased = (now - cur->lasttime);

			if(elpased >= timeout) {
				return 0;
			} else {
				const uint32_t diff = timeout - elpased;

				if(diff < max_diff) {
					max_diff = diff;
				}
			}
		}
	}

	return max_diff;
}

uint32_t InvScheduler_getMinPeriodU(const InvScheduler *scheduler)
{
	if(scheduler->queue) {
		InvSchedulerTask *cur = scheduler->queue->next;
		uint32_t min = scheduler->queue->period;

		/* /!\ RUNNING task is not in the queue hence ignored */
		/* /!\ delay is not taken into account */

		for(; cur != 0; cur = cur->next ) {
			if(cur->period < min) {
				min = cur->period;
			}
		}

		return min;
	}

	return UINT32_MAX;
}

int InvScheduler_dispatchOneTask(InvScheduler *scheduler)
{
	int run = 0;
	const uint32_t now = scheduler->currentTime;
	InvSchedulerTask * task;

	InvScheduler_lock(scheduler->contextLock);

	task = InvScheduler_getTaskToSchedule(scheduler, now);

	if(task) {
		/* update lastime and task state */
		task->delay    = 0; /* clear delay */
		task->lasttime = now;
		task->state    = INVSCHEDULER_TASK_STATE_RUNNING;

		InvScheduler_removeTask(scheduler, task);

		InvScheduler_unlock(scheduler->contextLock);
		InvScheduler_onTaskEnterHook(task, scheduler->currentTime);
		task->func(task->arg); /* execute the task */
		InvScheduler_onTaskExitHook(task, scheduler->currentTime);
		InvScheduler_lock(scheduler->contextLock);

		/* schedule task for next period */
		if(task->state == INVSCHEDULER_TASK_STATE_RUNNING) {
			task->state = INVSCHEDULER_TASK_STATE_READY;
			InvScheduler_insertTask(scheduler, task);
		}

		run = 1;
	}

	InvScheduler_unlock(scheduler->contextLock);

	return run;
}

int InvScheduler_dispatchTasks(InvScheduler *scheduler)
{
	int count = 0;

	while(InvScheduler_dispatchOneTask(scheduler))
		++count;

	return count;
}

void InvScheduler_initTaskDo(InvScheduler *scheduler, InvSchedulerTask *task,
		void (*func)(void *), void *arg, uint8_t prio, uint32_t period)
{
	task->scheduler = scheduler;
	task->func 		= func;
	task->arg 		= arg;
	task->priority 	= prio;
	task->period 	= period;
	task->state		= INVSCHEDULER_TASK_STATE_STOP;
#ifdef INVSCHEDULER_TASK_NAME
	task->name 		= "";
#endif
}

void InvScheduler_startTaskU(InvSchedulerTask *task, uint32_t delay)
{
	if(task->state == INVSCHEDULER_TASK_STATE_STARTED ||
			task->state == INVSCHEDULER_TASK_STATE_READY) {
		InvScheduler_removeTask(task->scheduler, task);
	}
	InvScheduler_insertTask(task->scheduler, task);
	task->delay = delay;
	task->state = INVSCHEDULER_TASK_STATE_STARTED;
}

void InvScheduler_startTask(InvSchedulerTask *task, uint32_t delay)
{
	InvScheduler_lock(task->scheduler->contextLock);
	InvScheduler_startTaskU(task, delay);
	InvScheduler_unlock(task->scheduler->contextLock);
}

void InvScheduler_stopTaskU(InvSchedulerTask *task)
{
	if(task->state == INVSCHEDULER_TASK_STATE_STARTED ||
			task->state == INVSCHEDULER_TASK_STATE_READY) {
		InvScheduler_removeTask(task->scheduler, task);
	}
	task->state = INVSCHEDULER_TASK_STATE_STOP;
}

void InvScheduler_stopTask(InvSchedulerTask *task)
{
	InvScheduler_lock(task->scheduler->contextLock);
	InvScheduler_stopTaskU(task);
	InvScheduler_unlock(task->scheduler->contextLock);
}

/* Debugging functions ********************************************************/

#ifndef NDEBUG

static const char *InvScheduler_taskState2Str(enum InvSchedulerTaskState state)
{
	switch(state) {
	case INVSCHEDULER_TASK_STATE_STOP:    return "STATE_STOP   ";
	case INVSCHEDULER_TASK_STATE_STARTED: return "STATE_STARTED";
	case INVSCHEDULER_TASK_STATE_READY:   return "STATE_READY  ";
	case INVSCHEDULER_TASK_STATE_RUNNING: return "STATE_RUNNING";
	default:                              return "STATE_???    ";
	}
}

void InvScheduler_printTask(const InvSchedulerTask *task,
		int (*printf_cb)(const char *format, ...))
{
	printf_cb("[%p:%s]\n"
			"    prio   = %-12u state  = %s \n"
			"    period = %-12u delay = %-12u\n"
			"    time   = %-12lu\n"
			"    next   = %p prev   = %p\n",
			(void *)task,
#ifdef INVSCHEDULER_TASK_NAME
			task->name,
#else
			"",
#endif
			(unsigned int)task->priority,
			InvScheduler_taskState2Str(task->state), (unsigned int)task->period,
			(unsigned int)task->delay, (unsigned long)task->lasttime,
			(void *)task->next, (void *)task->prev
	);
}

void InvScheduler_dumpTasks(const InvSchedulerTask * queue,
		int (*printf_cb)(const char *format, ...))
{
	const InvSchedulerTask *cur = queue;

	for(; cur != 0; cur = cur->next) {
		InvScheduler_printTask(cur, printf_cb);
	}
}

#endif
