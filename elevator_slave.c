/*
David Parrott, Brett Carter, Choong Huh, Taylor Hancuff
CS 466 elevator controller
slave program
*/
#class auto
#define OS_MAX_EVENTS          4       // Maximum number of events (semaphores, queues, mailboxes)
#define OS_MAX_TASKS           11  		// Maximum number of tasks system can create (less stat and idle tasks)
#define OS_TASK_CREATE_EN		 1       // Enable normal task creation
#define OS_TASK_CREATE_EXT_EN	 1       // Enable extended task creation
#define OS_TASK_STAT_EN			 1       // Enable statistics task creation
#define OS_MBOX_EN				 1			// Enable mailboxes
#define OS_MBOX_POST_EN			 1			// Enable MboxPost
#define OS_TIME_DLY_HMSM_EN	 1			// Enable OSTimeDlyHMSM
#define STACK_CNT_512	       8       // number of 512 byte stacks (application tasks + stat task + prog stack)
#define OS_MAX_MEM_PART			 10  		// Maximum number of memory partions in system
#define OS_MEM_EN					 1 		// Enable memory manager
#define OS_Q_EN					 1  		// Enable queues
#define OS_TICKS_PER_SEC       64
#define TASK_STK_SIZE     		 512		/* Size of each task's stacks (# of bytes)       */
#define TASK_START_ID       	 0	      /* Application tasks IDs                         */
#define TASK_START_PRIO    	 10      /* Application tasks priorities */

#use "BL4S1xx.LIB"
#use "ucos2.lib"
#use "rand.lib"

#define V0 ((void*) 0)
#define LOW (0)
#define HIGH (1)
#define zSetCS(x) WrPortI(PDB6R, V0, (x)?(1<<6):0)
#define zSetSCK(x) WrPortI(PDB7R, V0, (x)?(1<<7):0)
#define zSetMOSI(x) WrPortI(PDB3R, V0, (x)?(1<<3):0)
#define zGetMISO() ((RdPortI(PEDR)&(1<<6))?1:0)

int		ISRFlag;
int		ISR_handle;
int		primeArray[1000];
int		primeCount;
int		stop_flag;
void		enQueue();
void		deQueue();
void		primes(int n);
void	 	primeQ(int n);
void 		GPIO(int* array);
void		turnMotor(int entries);
int		array[4];

typedef char uint8_t;
uint8_t  transfer(uint8_t out);
uint8_t  zReadByte(uint8_t address);
void     zWriteByte(uint8_t address, uint8_t data);
 int quehandler;
 int dequehandler;
 int primehandler;
 OS_EVENT *reading, *DQ, *PR;
int fullSeq[][4] = {{1,0,0,0},{0,0,1,0},{0,1,0,0},{0,0,0,1}};
//BEGIN CODE FROM MILLER LOWE
// This is the aggreate data structure that you wish to queue between two tasks
//
typedef struct {
    int d[4];
} mQueueEntry_t;

//
// A Queue handle points to one of these..
typedef struct {
    int elementSize;
    int elementCount;
    int inQueue;
    OS_EVENT * queue;
    OS_MEM * memPool;
} labQueue_t;

// API return Values
typedef enum {
    LABQ_OK = 0,
    LABQ_ERROR = -1,
    LABQ_TIMEOUT = -2,
    LABQ_FULL = -3,
} labQueueReturn_t;

// Number of elements in the queue
#define Q1_ELEMENTS 900

// You must allocate storage for the queue data and funny pointer array
static void * q1_pArray[Q1_ELEMENTS];  // array of pointers for uCos Queue
static char * q1_eStorage[Q1_ELEMENTS * sizeof(mQueueEntry_t)]; // element storage
labQueue_t lq;

//
// -------------------------------- labQueue Implementation
// -------------------------------- Could be in a seperate file or paste into yours..
// Requires OS Defines (Prios to the '#use "ucos2.lib"' statement
// OS_MAX_EVENTS  +1 ber Queue allocated
// OS_MEM_EN      Must be defined/enabled
// OS_Q_EN        Shoudl be 1

//
// labQueueCreate -
//
// Fills out a queue structure that you have to allocate and hand in to the create.
//
labQueueReturn_t labQueueCreate(	labQueue_t *lq, int elementSize, int elementCount, void * elementStorage, void *pointerArray[]){
    INT8U error;
    lq->inQueue = 0;
    lq->elementSize = elementSize;
    lq->elementCount = elementCount;
    lq->queue = OSQCreate(pointerArray, elementCount);
    assert(lq->queue);
    lq->memPool = OSMemCreate(elementStorage, elementCount, elementSize, &error);
    assert(error==OS_NO_ERR);
    return (LABQ_OK);
}



//
// Give a queue handle and a pointer to the element, copy the data into the
// the queue.  This will fail for the put fails, not block..  We could extend it to
// blocking with a semaphore and a little work.. Who's motovated for that?
//
labQueueReturn_t labQueuePut( labQueue_t *lq, void *element){
    INT8U error;
    void * qElement;
    labQueueReturn_t result;

    qElement = OSMemGet(lq->memPool, &error);

    switch (error)
    {
    case OS_NO_ERR:
    	  lq->inQueue++;
        memcpy(qElement, element, lq->elementSize);
        error = OSQPost( lq->queue, qElement);
        switch (error)
        {
        case OS_NO_ERR:
            result = LABQ_OK;
            break;
        case OS_Q_FULL:
            error = OSMemPut(lq->memPool, qElement);
            assert( error == OS_NO_ERR);
            result = LABQ_FULL;
            break;
        default:
            error = OSMemPut(lq->memPool, qElement);
            assert( error == OS_NO_ERR);
            result = LABQ_ERROR;
            assert(0);
            break;
        }
        break;
    case OS_MEM_NO_FREE_BLKS:
        result = LABQ_FULL;
        break;
    default:
        result = LABQ_ERROR;
        assert(0);
        break;
    }
    return result;
}

//
// get the oldest element in the queue.  Fills out the element buffer
// that you pass in.
//
labQueueReturn_t labQueueGet( labQueue_t *lq, void * element, int timeout){
    void *qElement;
    INT8U error;
    labQueueReturn_t result;

    assert(timeout <= 0x0000ffff);
    qElement = OSQPend (lq->queue, timeout, &error);
    switch (error)
    {
    case OS_NO_ERR:
        memcpy(element, qElement, lq->elementSize);
        error = OSMemPut(lq->memPool, qElement);
        assert( error == OS_NO_ERR);
        result = LABQ_OK;
        lq->inQueue--;
        break;
    case OS_TIMEOUT:
        result = LABQ_TIMEOUT;
        break;
    default:
        assert(0);
        result = LABQ_ERROR;
        break;
    }
	return result;
}

void pinInit(){
	int mask;
	WrPortI(PDFR, V0, RdPortI(PDFR) & ~((1<<3) | (1<<6) | (1<<7)));
   WrPortI(PDDCR, V0, RdPortI(PDDCR) & ~((1<<3) | (1<<6) | (1<<7)));
   WrPortI(PDCR, V0, 0);
   WrPortI(PDDDR, V0, RdPortI(PDDDR) | ((1<<1) | (1<<3) | (1<<6) | (1<<7)));

   WrPortI(PEFR, V0, RdPortI(PEFR) & ~((1<<6)));
   mask = RdPortI(PEDDR);
   mask &= ~(1<<6);
   mask |= (1<<3);
   WrPortI(PEDDR, V0, mask);
   WrPortI(PEDCR, V0, RdPortI(PEDCR) & ~(1<<3));
}

uint8_t transfer(uint8_t out){
 	uint8_t i;
   uint8_t in = 0;
   zSetSCK(LOW);
   for(i = 0; i<8; i++){
    	in <<= 1;
      zSetMOSI(out & 0x80);
      zSetSCK(HIGH);
      in += zGetMISO();
      zSetSCK(LOW);
      out <<= 1;
   }
   zSetMOSI(0);
   return(in);
}
//read from address: 0x12
uint8_t zReadByte(uint8_t address){
 	uint8_t preRead = 0x41;
   uint8_t value;
   //assert(address < 16);
   zSetCS(LOW);
   transfer(preRead);
   transfer(address);
   value = transfer(0);
   zSetCS(HIGH);
   return value;
}
//write to address: 0x13
void zWriteByte(uint8_t address, uint8_t data){
 	uint8_t preWrite = 0x40;
	//printf("%d \n", address);
  // assert(address < 16);
   zSetCS(LOW);
    //OSTimeDly(100);
   //transfer(preWrite | address);
   transfer(preWrite);
   transfer(address);
   transfer(data);
   zSetCS(HIGH);

}

void fillQueue(){
	mQueueEntry_t qEnt;
   labQueueReturn_t result;
   int i;
	for(i = 0;result != LABQ_FULL;i++){
   	result = labQueuePut(&lq, &qEnt);
      //printf("%i \n",i);
   }
}


void GPIO(int* array)
{
	uint8_t Byte;
	int i;
   Byte = array[0]*(1<<7)|array[1]*(1<<6)|array[2]*(1<<5)|array[3]*(1<<4);
   zWriteByte(0x00,0x00);
   zWriteByte(0x12,Byte);

}

void enQueue(){
   int i;
   for(i = 1;i < 5; i++){
   	array[i-1] = digIn(i);
      //printf("%i ",array[i-1]);
   }
   //printf("\n");
   digOut(0,1);
   OSTimeDlyHMSM(0,0,0,1);
   digOut(0,0);
}

/*
void enQueue(){
	mQueueEntry_t qEnt;
	int i;
   //printf("enqueue \n");
   for(i = 1; i < 5; i++){
      qEnt.d[i-1] = digIn(i);
		//printf("%i ",qEnt.d[i-1]);
   }
   //printf("\n");

	labQueuePut(&lq, &qEnt);
   digOut(0,1);
   OSTimeDlyHMSM(0,0,0,1);
   digOut(0,0);
   RSB_CLEAR_ALL_IRQ(quehandler);
}
*/

void deQueueAll(){
	int i;
   for(i = 1; i < 5; i++){
   	digOut(i,array[i-1]);
   }
   digOut(0,1);
   OSTimeDlyHMSM(0,0,0,1);
   digOut(0,0);
   //printf("dequeued!\n");
}

/*
void deQueueAll(){
	mQueueEntry_t qEnt;
	int i, t;
	//printf("dequeue\n");
   while(lq.inQueue > 0){
   	labQueueGet(&lq, &qEnt, 0x0000ffff);
   	for(i = 1; i < 5; i++){
   		t = qEnt.d[i];
      	digOut(i, t);
     		OSTimeDlyHMSM(0,0,0,9);
   	}
	}
RSB_CLEAR_ALL_IRQ(dequehandler);
}
*/
void primes(int n){
   int i, count, c;
   printf("in primes\n");
   i = 3;
   for(count = 2; count <= n;){
      for(c = 2; c <= i - 1; c++){
         if(i % c == 0)
            break;
      }
      if(c == i){
         primeArray[primeCount] = i;
         primeCount++;
         count++;
      }
      i++;
   }
}

void primeQ(int n){
   int i,j, count, c;
   mQueueEntry_t qEnt;
   OSTimeDlyHMSM(0,0,0,9);
   i = 3;
   printf("in prime function\n");
   for(count = 2; count <= n;){
   	OSTimeDlyHMSM(0,0,0,9);
   	if(primeCount == 1000){primeCount = 0;}
      for(c = 2; c <= i - 1; c++){
         if(stop_flag == 1){
         	for( j = 0; j < primeCount; j++){
             printf("%i\n",primeArray[j]);
            }
            printf("number of primes: %i\n", primeCount);
            stop_flag = 0;
            break;
         }
      	OSTimeDlyHMSM(0,0,0,9);
         if(i % c == 0)
            break;
      }
      if(c == i){
         primeArray[primeCount] = i;
         primeCount++;
         count++;
         OSTimeDlyHMSM(0,0,0,9);
         if(stop_flag == 1){
         	for( j = 0; j < primeCount; j++){
             printf("%i\n",primeArray[j]);
            }
            printf("number of primes: %i\n", primeCount);
            stop_flag = 0;
            break;
         }
      }
      i++;
   }
}

root void enQueue_handle(){
 	OSSemPost(reading);
   OSSemPost(PR);
   RSB_CLEAR_ALL_IRQ(quehandler);
}

root void deQueueAll_handle(){
 	OSSemPost(DQ);
   RSB_CLEAR_ALL_IRQ(dequehandler);
}

root void primehandle(){
 	stop_flag = 1;
   RSB_CLEAR_ALL_IRQ(primehandler);
}

void INQueue(){

    while(1){
     OSSemPend(reading,0,NULL);
     //printf("queueing\n");
     enQueue();
    }
}

void DEQueue(){
 	while(1){
    OSSemPend(DQ, 0,NULL);
    //printf("DEQueue called semaphore grabbed\n");
    deQueueAll();
   }
}

void PRimes(){
	while(1){
   	printf("in prime task\n");
   	OSSemPend(PR, 0, NULL);
      printf("got semaphore\n");
		primeQ(1000);
   }
}

void main(){

	labQueueReturn_t qr;
   mQueueEntry_t qEnt;
   mQueueEntry_t qE2;
   int i, row, col;
   int num[4];
   reading = OSSemCreate(0);
   DQ = OSSemCreate(0);
   PR = OSSemCreate(0);
   stop_flag = 0;
   OSInit();
	brdInit();
   pinInit();
   digOut(0,0);
	puts("okay");
   primeCount = 0;
   quehandler = addISRIn(0,0,&enQueue_handle);
   dequehandler = addISRIn(5,0,&deQueueAll_handle);
   primehandler = addISRIn(6,0,&primehandle);
   setExtInterrupt(5, BL_IRQ_FALL, dequehandler);
   setExtInterrupt(0, BL_IRQ_FALL, quehandler);
   setExtInterrupt(6, BL_IRQ_FALL, primehandler);
   enableISR(quehandler, 1);
   enableISR(dequehandler, 1);
   enableISR(primehandler, 1);
   qr = labQueueCreate(&lq, sizeof(qEnt.d), Q1_ELEMENTS, q1_eStorage, q1_pArray);

   OSTaskCreateExt(INQueue,
  					    (void *)0,
                   10,
                   1,
                   TASK_STK_SIZE,
                   (void *)0,
                   OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  OSTaskCreateExt(DEQueue,
  					    (void *)0,
                   11,
                   2,
                   TASK_STK_SIZE,
                   (void *)0,
                   OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
/*
 OSTaskCreateExt(PRimes,
  					    (void *)0,
                   13,
                   3,
                   TASK_STK_SIZE,
                   (void *)0,
                   OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
 */
  OSStart();
/*
   GPIO(fullSeq[0]);
   enQueue();
   printf("%i \n",lq.inQueue);

   GPIO(fullSeq[1]);
   enQueue();
   printf("%i \n",lq.inQueue);

	qr = labQueueGet(&lq, &qE2, 0x0000ffff);

   for(i = 0; i < 4; i++){
   	printf("%i ",qE2.d[i]);
   }
   printf("\n");

	qr = labQueueGet(&lq, &qE2, 0x0000ffff);

   for(i = 0; i < 4; i++){
   	printf("%i ",qE2.d[i]);
   }
   for(i = 0; i < 4; i++){
		GPIO(fullSeq[i]);
      enQueue();
   }
*/
	//fillQueue();

   /*
   ISRFlag = 0;
   ISR_handle = addISRIn(0, 0, &enQueue);			//read from GPIO
   setExtInterrupt(0, BL_IRQ_FALL, ISR_handle);
   enableISR(ISR_handle, 1);
*/
   //primeQ(49);
	//primes(49);
/*
   for(i = 0; i < 49; i++){
	   qr = labQueueGet(&lq, &qE2, 0x0000ffff);
   	printf("%i : %i : %i\n", qE2.d[0], primeArray[i], qE2.d[1]);
   }
*/



}

