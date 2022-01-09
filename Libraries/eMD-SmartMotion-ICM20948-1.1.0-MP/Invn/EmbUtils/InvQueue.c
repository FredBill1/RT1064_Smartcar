/*
 * Queue.c
 */
#include	"InvQueue.h"

//	code looks good with tab stops of 4 characters !!!

/******************************************************************************/
/* types */
/******************************************************************************/

/******************************************************************************/
/* consts */
/******************************************************************************/

/******************************************************************************/
/* statics */
/******************************************************************************/

/******************************************************************************/
/* externs */
/******************************************************************************/

/******************************************************************************/
/* code follows */
/******************************************************************************/

/******************************************************************************/
/* service routines */
/******************************************************************************/

/******************************************************************************/
//	initializes a queue - required prior to use
void		QueueInit			(QueueT	*	Queue)//,	QueueCallbackT		RemovalCallback)
{
	QueueItemT	*	Q	=	(QueueItemT *)	Queue;

	//	show Queue to be empty
	Queue->Next	=	Q;
	Queue->Prev	=	Q;
//	Queue->RemovalCallback	=	RemovalCallback;
}	//	end of QueueInit()

/******************************************************************************/
//	initializes a queue item - required prior to use
void		QueueItemInit		(void	*	Item)
{
	QueueItemT	*	I	=	Item;

	//	show Item to be unqueued
	I->Next		=	I;
	I->Prev		=	I;
	I->Queue	=	0;
}	//	end of QueueItemInit()

/******************************************************************************/
//	inserts Item into Queue as the headward neighbor of NextItem
void		QueueInsertBefore	(QueueT	*	Queue,	void	*	NextItem,	void	*	Item)
{
	QueueItemT	*	Q		=	(QueueItemT *)Queue;
	QueueItemT	*	NextI	=	NextItem;
	QueueItemT	*	I		=	Item;

	if	(!	Q)											return;	// can't put item onto a null queue
	if	(!	I)											return;	// can't put a null item
	if	(I->Queue)										return;	// can't put item if already enqueued

	if		(NextI == QueueSelectHead)	NextI	=	Q->Next;
	else if	(NextI == QueueSelectTail)	NextI	=	Q->Prev;


//	link being made:					prev item	Item	NextItem
	I->Prev			=	NextI->Prev;//	<-----------|
	I->Next			=	NextI;		//				|------>
	NextI->Prev		=	I;			//				<-------|
	I->Prev->Next	=	I;			//	|---------->
	I->Queue		=	Queue;
}	//	end of QueueInsertBefore()

/******************************************************************************/
//	inserts Item into Queue as the tailward neighbor of PrevItem
void		QueueInsertAfter	(QueueT	*	Queue,	void	*	PrevItem,	void	*	Item)
{
	QueueItemT	*	Q		=	(QueueItemT *)Queue;
	QueueItemT	*	PrevI	=	PrevItem;
	QueueItemT	*	I		=	Item;

	if	(!	Q)											return;	// can't put item onto a null queue
	if	(!	I)											return;	// can't put a null item
	if	(I->Queue)										return;	// can't put item if already enqueued

	if		(PrevI == QueueSelectHead)	PrevI	=	Q->Next;
	else if	(PrevI == QueueSelectTail)	PrevI	=	Q->Prev;

//	link being made:					PrevItem	Item	next item
	I->Next			=	PrevI->Next;//				|------>
	I->Prev			=	PrevI;		//	<-----------|
	PrevI->Next		=	I;			//	|---------->
	I->Next->Prev	=	I;			//				<-------|
	I->Queue		=	Queue;
}	//	end of QueueInsertAfter()

/******************************************************************************/
//	remove Item from whatever queue, if any, it is presently in
//	returns Item regardless of whether it's null or not enqueued.
void	*	QueueRemove			(void	*	Item)
{
	QueueItemT	*	I		=	Item;

	if	(!	I)				return	I;	//	null item
//	to optimize typical usage, skip next check, as function has null effect for the guarded case
//	if	(!	I->Next == I)	return	I;	//	unqueued item

//	link being made:					prev item	Item	next item
	I->Prev->Next	=	I->Next;	//	|------------------>
	I->Next->Prev	=	I->Prev;	//	<-------------------|					
	I->Next			=	I;			//				|^
	I->Prev			=	I;			//				|^

	I->Queue	=	0;

	return	I;
}	//	end of QueueRemove()

/******************************************************************************/
//	removes from Queue the Item that is the headward neighbor of NextItem
void	*	QueueRemoveBefore	(QueueT	*	Queue,	void	*	NextItem)
{
	QueueItemT	*	NextI	=	NextItem;

	if	(!	Queue)					return	0;							//	can't remove from a null queue
	if	(QueueEmpty	(Queue))		return	0;							//	can't remove from an empty queue
	if	(NextI == QueueSelectHead)	return	0;							//	no such item
	if	(NextI == QueueSelectTail)	return	QueueRemove	(Queue->Prev);	//	remove the tail item, if any
	else							return	QueueRemove	(NextI->Prev);	//	remove preceding item, if any
#ifdef	neverdef
	if	(NextI->Queue == Queue)		return	QueueRemove	(NextI->Prev);	//	remove preceding item, if any
									return	0;							//	NextItem is not on Queue
#endif
}	//	end of QueueRemoveBefore()

/******************************************************************************/
//	removes from Queue the Item that is the tailward neighbor of PrevItem
void	*	QueueRemoveAfter	(QueueT	*	Queue,	void	*	PrevItem)
{
	QueueItemT	*	PrevI	=	PrevItem;

	if	(!	Queue)					return	0;							//	can't remove from a null queue
	if	(QueueEmpty	(Queue))		return	0;							//	can't remove from an empty queue
	if	(PrevI == QueueSelectHead)	return	QueueRemove	(Queue->Next);	//	remove the head item, if any
	if	(PrevI == QueueSelectTail)	return	0;							//	no such item
	else							return	QueueRemove	(PrevI->Next);	//	remove preceding item, if any

#ifdef	neverdef
	if	(PrevI->Queue == Queue)		return	QueueRemove	(PrevI->Next);	//	remove preceding item, if any
									return	0;							//	PrevItem is not on Queue
#endif
}	//	end of QueueRemoveAfter()

/******************************************************************************/
/*	traverse the Items on Queue - Items are _not_ removed from Queue
	- initially:
		For QueueNext(), use PrevItem = QueueSelectHead to start at head of Queue
		For QueuePrev(), use NextItem = QueueSelectTail to start at tail of Queue
	- subsequently, use the Item pointer returned by the previos invocation
		to traverse to the next/prev Item
	-	returns 0 if Queue is empty, no more Items, etc.
*/

/******************************************************************************/
void	*	QueuePrev			(QueueT	*	Queue,	void	*	NextItem)
{
	QueueItemT	*	Q		=	(QueueItemT *)Queue;
	QueueItemT	*	NextI	=	NextItem;

	if	(!	Queue)					/* null value */							return	0;
	if	(NextI == QueueSelectHead)	/* QueueSelectHead is not a valid value */	return	0;
	if	(NextI == QueueSelectTail)	//	forcing return of tail
	{
		if	(QueueEmpty	(Queue))												return	0;
		else						/* return tail */							return	Queue->Prev;
	}
		//	NextItem is an actual item
	if	(NextI->Prev == Q)			/* already at head of queue */				return	0;
	else							/* return preceding Item */					return	NextI->Prev;
}	//	end of QueuePrev()

/******************************************************************************/
void	*	QueueNext			(QueueT	*	Queue,	void	*	PrevItem)
{
	QueueItemT	*	Q		=	(QueueItemT *)Queue;
	QueueItemT	*	PrevI	=	PrevItem;

	if	(!	Queue)					/* null value */							return	0;
	if	(PrevI == QueueSelectHead)	// forcing return of head
	{
		if	(QueueEmpty	(Queue))												return	0;
		else						/* return head */							return	Queue->Next;
	}
	if	(PrevI == QueueSelectTail)	/* QueueSelectTail is not a valid value */	return	0;
		//	PrevItem is an actual item
	if	(PrevI->Next == Q)			/* already at tail of queue */				return	0;
	else							/* return next Item */						return	PrevI->Next;
}	//	end of QueueNext()

/******************************************************************************/
//	means of querying the status of an item

inv_bool_t	QueueAtHead			(void	*	Item)
{
	QueueItemT	*	I	=	Item;

	if	(!	I)							return	false;
	if	(!	I->Queue)					return	false;
	if	(I->Prev == (void *)I->Queue)	return	true;
	else								return	false;
}	//	end of QueueAtHead()

inv_bool_t	QueueAtMiddle		(void	*	Item)
{
	QueueItemT	*	I	=	Item;

	if	(!	I)							return	false;
	if	(!	I->Queue)					return	false;
	if	(I->Next == (void *)I->Queue)	return	false;
	if	(I->Prev == (void *)I->Queue)	return	false;
	else								return	true;
}	//	end of QueueAtHead()

inv_bool_t	QueueAtTail			(void	*	Item)
{
	QueueItemT	*	I	=	Item;

	if	(!	I)							return	false;
	if	(!	I->Queue)					return	false;
	if	(I->Next == (void *)I->Queue)	return	true;
	else								return	false;
}	//	end of QueueAtHead()

QueueT	*	QueueOf				(void	*	Item)	//	returns pointer to owning Q, or null if dequeued
{
	QueueItemT	*	I	=	Item;

	if	(!	I)							return	false;
	if	(!	I->Queue)					return	false;
	else								return	I->Queue;
}	//	end of QueueAtHead()

/******************************************************************************/
/* administrative routines */
/******************************************************************************/

/******************************************************************************/
#ifdef	neverdef
void	QueueModuleReset	(void)
{}	/* end of QueueModuleReset */
#endif

/******************************************************************************/
#ifdef	neverdef
void	QueueModuleInit		(void)
{}	/* end of QueueModuleInit */
#endif

/******************************************************************************/
/*
 * end of Queue.c
 */
