#include "stdio.h"    
#include "stdlib.h"    
#include "math.h"  
#include "time.h"
#include "drv_queue.h"

#define OK 1
#define ERROR 0
#define TRUE 1
#define FALSE 0



/* 初始化一个空队列Q */
Status InitSqQueue(SqQueue *Q)
{
	Q->front=0;
	Q->rear=0;
	return  OK;
}

/* 将Q清为空队列 */
Status ClearSqQueue(SqQueue *Q)
{
	Q->front=Q->rear=0;
	return OK;
}

/* 若队列Q为空队列,则返回TRUE,否则返回FALSE */
Status SqQueueEmpty(SqQueue Q)
{ 
	if(Q.front==Q.rear) /* 队列空的标志 */
		return TRUE;
	else
		return FALSE;
}

/* 返回Q的元素个数，也就是队列的当前长度 */
int SqQueueLength(SqQueue Q)
{
	return  (Q.rear-Q.front+SQQUEUE_MAXSIZE)%SQQUEUE_MAXSIZE;
}

/* 若队列不空,则用e返回Q的队头元素,并返回OK,否则返回ERROR */
Status GetSqHead(SqQueue Q,SqElemType *e)
{
	if(Q.front==Q.rear) /* 队列空 */
		return ERROR;
	*e=Q.data[Q.front];
	return OK;
}

/* 若队列未满，则插入元素e为Q新的队尾元素 */
Status EnSqQueue(SqQueue *Q,SqElemType e)
{
	if ((Q->rear+1)%SQQUEUE_MAXSIZE == Q->front)	/* 队列满的判断 */
		return ERROR;
	Q->data[Q->rear]=e;			/* 将元素e赋值给队尾 */
	Q->rear=(Q->rear+1)%SQQUEUE_MAXSIZE;/* rear指针向后移一位置， */
								/* 若到最后则转到数组头部 */
	return  OK;
}

/* 若队列不空，则删除Q中队头元素，用e返回其值 */
Status DeSqQueue(SqQueue *Q,SqElemType *e)
{
	if (Q->front == Q->rear)			/* 队列空的判断 */
		return ERROR;
	*e=Q->data[Q->front];				/* 将队头元素赋值给e */
	Q->front=(Q->front+1)%SQQUEUE_MAXSIZE;	/* front指针向后移一位置， */
									/* 若到最后则转到数组头部 */
	return  OK;
}

/* 从队头到队尾依次对队列Q中每个元素输出 */
int SqQueueSum(SqQueue Q)
{ 
	int i;
    int sum=0;
	i=Q.front;
	while(i!=Q.rear)
	{
        sum+=Q.data[i];
		i=(i+1)%SQQUEUE_MAXSIZE;
	}
	return sum;
}

