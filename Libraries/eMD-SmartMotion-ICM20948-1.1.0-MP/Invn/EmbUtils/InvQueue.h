/*
    Copyright (c) 2014-2015 InvenSense Inc. Portions Copyright (c) 2014-2015 Movea. All rights reserved.

    This software, related documentation and any modifications thereto (collectively "Software") is subject
    to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
    other intellectual property rights laws.

    InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
    and any use, reproduction, disclosure or distribution of the Software without an express license
    agreement from InvenSense is strictly prohibited.
*/

/**
 * \defgroup InvQueue InvQueue
 * \brief Provides means of enqueueing objects
 * \ingroup EmbUtils
 * \{
 */

/*
This service provides two types, which the client instantiates as desired:
	- QueueT, which describes a queue of items, and
	- QueueItemT, which describes an item that can be handled discretely,
		placed into, or removed from, a queue

And, it provides various functions for manipulation of objects of these types

The usage model is to include an instance of QueueItemT into a client's object's
declaration, thus making instances thereof queueable via this service.

Optimally, the QueueItemT is the first item in the client's declaration, so that the same pointer value
can interchangably refer to both a) an object instansiated from the client type (for the client's own
application purposes), and b) the queue item object that is contained therewithin (for this queueing
service's purposes). However, the queue item can also be located elsewhere in the client declaration,
if one is willing to go to the extra effort in coding the client, e.g. to regenerate a pointer to a
client object from the queue item pointer that is returned by e.g. QueueGet().

To better support the above optimal placement of queue item objects within client objects, this
service's function declarations use "void *" where they would nominally use "QueueItemT *".

The following methods are available; see the individual function declarations for details.

means of primordially initializing service objects:

QueueInit			- required to initialize the given Queue object prior to first use
QueueItemInit		- required to initialize the given QueueItem object prior to first use

means of adding / removing Items from a Queue:

QueueInsertBefore	- inserts Item into Queue as the headward neighbor of NextItem
QueueInsertAfter	- inserts Item into Queue as the tailward neighbor of PrevItem
QueueRemoveBefore	- removes from Queue the Item that is the headward neighbor of NextItem
QueueRemoveAfter	- removes from Queue the Item that is the tailward neighbor of PrevItem
QueueRemove			- removes Item from whatever Queue it may be enqueued upon

shorthand versions of the above:

QueuePut			- inserts Item into Queue at its tail
QueuePutLifo		- inserts Item into Queue at its head
QueueGet			- removes from Queue the Item that is at its head

means of incrementally traversing a Queue's Items:

QueuePrev			- returns the Item that is the headward neighbor on Queue of NextItem
QueueNext			- returns the Item that is the tailward neighbor on Queue of PrevItem

means of querying the status of a Queue:

QueueEmpty			- returns True it the given Queue is empty
QueueHeadOf			- returns the Item at the head of Queue
QueueTailOf			- returns the Item at the tail of Queue

means of querying the status of an Item:

QueueAtHead			- returns True if Item is at the head of its containing queue
QueueAtMiddle		- returns True if Item is at neither the head nor tail of its containing queue
QueueAtTail			- returns True if Item is at the tail of its containing queue
QueueOf				- returns the Queue on which Item resides, or 0 if it is not enqueued anywhere

Generally, these methods are 'well behaved' with respect to erroneous usage.
E.g., QueueGet(0) (a null / nonexistent Queue) returns a null Item (same as if an actual Queue
were empty). However, they are not foolproof, e.g. while QueuePut(ActualQueuedItem, ActualQueue)
devolves to a no-op, leaving ActualQueuedItem on whatever queue is was already enqueued upon,
QueuePut(ActualUnqueuedItem, 0), which also devolves to a no-op, has the likely application
semantic of a memory leak, i.e. forever losing access to ActualItem.
*/

#ifndef	QueueIncluded
#define	QueueIncluded

#include "InvBool.h"

/******************************************************************************/
/* types */
/******************************************************************************/

//	there are no user serviceable components in any of the types declared in this module

//	these initialize plasmid style, i.e. their item pointers point to themselves
//	therefore, the offsets of the Next and Prev members in the 2 declarations must be identical

/******************************************************************************/
//	describe an item removal callback funtion as one taking a pointer to the item, and returning void
//typedef		void	(* QueueCallbackT)(void /* QueueItemT */ * Item);

/******************************************************************************/
/** @brief describe a queue item */
typedef	struct QueueItemT
{	struct	QueueItemT	*	Next;	//	points to next item in queue, or to self when unqueued
	struct	QueueItemT	*	Prev;	//	points to previous item in queue, or to self when unqueued
	struct	QueueT		*	Queue;	//	points to the Queue that Item is enqueued upon, if any
}	QueueItemT;

/******************************************************************************/
/** @brief describe a queue */
typedef	struct QueueT
{	struct	QueueItemT	*	Next;	//	head of queue; points to self when queue is empty
	struct	QueueItemT	*	Prev;	//	tail of queue; points to self when queue is empty
}	QueueT;

/******************************************************************************/
/* consts */
/******************************************************************************/
//	describe the head and tail of a queue, for use with certain service members
//	forces selection of Item at head or tail of Queue

//	use QueueSelectHead as a seed value for the NextItem parameter
//	use QueueSelectTail as a seed value for the PrevItem parameter
#define	QueueSelectHead	((void *)0)
#define	QueueSelectTail	((void *)-1)

/******************************************************************************/
/* globals */
/******************************************************************************/

/******************************************************************************/
/* service routines */
/******************************************************************************/

/******************************************************************************/
//	the following stuff initializes queues and items - require prior to first use of object
/******************************************************************************/

void		QueueInit		(QueueT	*	Queue);
void		QueueItemInit	(void	*	Item);

/******************************************************************************/
//	the following stuff enqueues or dequeues items
/******************************************************************************/

//	inserts Item into Queue as the headward neighbor of NextItem
void		QueueInsertBefore	(QueueT	*	Queue,	void	*	NextItem,	void	*	Item);

//	inserts Item into Queue as the tailward neighbor of PrevItem
void		QueueInsertAfter	(QueueT	*	Queue,	void	*	PrevItem,	void	*	Item);

//	removes from Queue the Item that is the headward neighbor of NextItem
void	*	QueueRemoveBefore	(QueueT	*	Queue,	void	*	NextItem);

//	removes from Queue the Item that is the tailward neighbor of PrevItem
void	*	QueueRemoveAfter	(QueueT	*	Queue,	void	*	PrevItem);

//	remove Item from whatever queue, if any, it is presently in
//	returns Item regardless of whether it is null or not enqueued.
void	*	QueueRemove			(void	*	Item);

/******************************************************************************/
//	specialized variants of the above

//	places an unqueued Item onto the tail of Queue; with QueueGet(), effects FIFO semantics
//void		QueuePut					(QueueT	*	Queue,	void	*	Item);
#define		QueuePut(Queue, Item)		(QueueInsertAfter	((Queue), QueueSelectTail, (Item)))

//	for LIFO lovers, use this instead of QueuePut()
//void		QueuePutLifo				(QueueT	*	Queue,	void	*	Item);
#define		QueuePutLifo(Queue, Item)	(QueueInsertBefore	((Queue), QueueSelectHead, (Item)))

//	returns the Item that is at the head of Queue; return 0 if Queue is empty
//void	*	QueueGet					(QueueT	*	Queue);
#define		QueueGet(Queue)				(QueueRemoveAfter	((Queue), QueueSelectHead))

/******************************************************************************/
//	the following stuff does not enqueue or dequeue anything
/******************************************************************************/

/******************************************************************************/
/*	traverse the Items on Queue - Items are _not_ removed from Queue
	- initially:
		For QueueNext(), use PrevItem = QueueSelectHead to start at head of Queue
		For QueuePrev(), use NextItem = QueueSelectTail to start at tail of Queue
	- subsequently, use the Item pointer returned by the previos invocation
		to traverse to the next/prev Item
	-	returns 0 if Queue is empty, no more Items, etc.
*/
void	*	QueuePrev			(QueueT	*	Queue,	void	*	NextItem);
void	*	QueueNext			(QueueT	*	Queue,	void	*	PrevItem);

/******************************************************************************/
//	means of querying the status of a queue

//bool		QueueEmpty			(QueueT	*	Queue);	//	returns True if Queue is empty
#define		QueueEmpty(Queue)	((Queue)->Next == ((QueueItemT *)(Queue)))

//void	*	QueueHeadOf			(QueueT	*	Queue);	//	returns Item at head of Q
#define		QueueHeadOf(Queue)	(QueueNext	((Queue), QueueSelectHead))

//void	*	QueueTailOf			(QueueT	*	Queue);	//	returns Item at tail of Q
#define		QueueTailOf(Queue)	(QueuePrev	((Queue), QueueSelectTail))

/******************************************************************************/
//	means of querying the status of an item

inv_bool_t	QueueAtHead			(void	*	Item);	//	returns True if Item is at given point within Q
inv_bool_t	QueueAtMiddle		(void	*	Item);
inv_bool_t	QueueAtTail			(void	*	Item);
QueueT	*	QueueOf				(void	*	Item);	//	returns pointer to owning Q, or 0 if dequeued

/******************************************************************************/
/* module level administrative routines */
/******************************************************************************/

//	void	QueueModuleInit		(void);
#define	QueueModuleInit()

/******************************************************************************/
#endif	//	QueueIncluded

/** \} */
