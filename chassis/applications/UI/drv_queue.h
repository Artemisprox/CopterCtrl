#ifndef _DRV_QUEUE_H_
#define _DRV_QUEUE_H_

typedef int Status; 
typedef int SqElemType; /* QElemType类型根据实际情况而定，这里假设为int */
#define SQQUEUE_MAXSIZE 6 /* 存储空间初始分配量 */
/* 循环队列的顺序存储结构 */
typedef struct
{
	SqElemType data[SQQUEUE_MAXSIZE];
	int front;    	/* 头指针 */
	int rear;		/* 尾指针，若队列不空，指向队列尾元素的下一个位置 */
}SqQueue;

typedef int LqElemType; /* QElemType类型根据实际情况而定，这里假设为int */

typedef struct QNode /* 结点结构 */
{
	void (*func)(void *para);
	LqElemType data;
	struct QNode *next;
} QNode, *QueuePtr;

typedef struct /* 队列的链表结构 */
{
	QueuePtr front, rear; /* 队头、队尾指针 */
} LinkQueue;


Status InitSqQueue(SqQueue *Q);
Status ClearSqQueue(SqQueue *Q);
Status SqQueueEmpty(SqQueue Q);
int SqQueueLength(SqQueue Q);
Status GetSqHead(SqQueue Q,SqElemType *e);
Status EnSqQueue(SqQueue *Q,SqElemType e);
Status DeSqQueue(SqQueue *Q,SqElemType *e);
Status SqQueueSum(SqQueue Q);

Status InitLqQueue(LinkQueue *Q);
Status DestroyLqQueue(LinkQueue *Q);
Status ClearLqQueue(LinkQueue *Q);
Status LqQueueEmpty(LinkQueue Q);
int LqQueueLength(LinkQueue Q);
Status GetLqHead(LinkQueue Q, LqElemType *e);
Status EnLqQueue(LinkQueue *Q, LqElemType e);
Status DeLqQueue(LinkQueue *Q, LqElemType *e);
Status LqQueueTraverse(LinkQueue *Q);
#endif

