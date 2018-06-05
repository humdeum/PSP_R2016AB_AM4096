/* Copyright 2013 The MathWorks, Inc. */


/* ---------------------------- */
/* RTOS-specific headers        */
/* Note: must be included first */
/* ---------------------------- */
#include "nuttxinitialize.h"
/* Added headers */
#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>
#include <systemlib/err.h>

/* ---------------------------- */
/* Required Coder Target header */
/* ---------------------------- */
#include "MW_custom_RTOS_header.h"
#include "MW_target_hardware_resources.h"

/* ---------------------------- */
/* RTOS-specific declarations   */
/* ---------------------------- */
/***********************************************
 * Added for Simulink Threads
 ************************************************/
pthread_attr_t attr;
static baseRateInfo_t g_info;
struct sched_param g_sp;
bool g_baseRateLife = false;


/***********************************************
 * Added for HRT
 ************************************************/
struct hrt_call BaseRate_HRT;


#define MW_RTOS_DEBUG 1

#define CHECK_STATUS(status, expStatus, fcn) warnx("Call to %s returned status (%d)", fcn, status);if (status != expStatus) {warnx("Call to %s returned error status (%d).\n", fcn, status); perror(fcn);}
#define CHECK_STATUS_NOT(status, errStatus, fcn) warnx("Call to %s returned status (%d)", fcn, status);if (status == errStatus) {warnx("Call to %s returned error status (%d).", fcn, status); perror(fcn);}

/******** Define external mode auxiliary vars/functions here *********/
baseRateInfo_t EXT_sig_info; 
extern pthread_t EXT_Mode_Thread;
extern void ExtModeTask(void* arg);
extern sem_t ExtModeTsk_Sem;

void setTaskPeriod(double periodInSeconds, int sigNo)
{
  timer_t timerId;
  struct sigevent sev;
  struct itimerspec its;
  long stNanoSec;
  int status;

  /* Create a timer */
  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = sigNo;
  sev.sigev_value.sival_ptr = &timerId;
  status = timer_create(CLOCK_REALTIME, &sev, &timerId);
  CHECK_STATUS(status, 0,"timer_create");

  /* Arm real-time scheduling timer */
  stNanoSec = (long)(periodInSeconds * 1e9);
  its.it_value.tv_sec = stNanoSec / 1000000000;
  its.it_value.tv_nsec = stNanoSec % 1000000000;
  its.it_interval.tv_sec = its.it_value.tv_sec;
  its.it_interval.tv_nsec = its.it_value.tv_nsec;
  status = timer_settime(timerId, 0, &its, NULL);
  CHECK_STATUS(status, 0,"timer_settime");
}
/******** End of external mode vars/auxiliary functions *********/


static void Base_HRT_Callback(void* arg)
{
    // while(g_baseRateLife) {
        sem_post(&baserateTaskSem);
     //}

}



/* ---------------------------- */
/* Externally visible functions */
/* ---------------------------- */
void myWaitForThisEvent(int sigNo)
{
    sigset_t sigMask;

    sigemptyset(&sigMask);
    sigaddset(&sigMask, sigNo);
	sigwaitinfo(&sigMask, NULL);
 //	CHECK_STATUS_NOT(status, -1, "sigwaitinfo");
}

void myAddBlockForThisEvent(int sigNo)
{
	int status;
    sigset_t sigMask;

    sigemptyset(&sigMask);
    sigaddset(&sigMask, sigNo);
    status = pthread_sigmask(SIG_BLOCK, &sigMask, NULL);
    CHECK_STATUS(status, 0, "pthread_sigmask");
}

void nuttxRTOSInit(double baseRatePeriod, int baseRatePriority, int numSubrates)
{
    int status;
    size_t stackSize;
#ifdef MW_HAS_MULTIPLE_RATES
    int i; 
    char taskName[20];
#endif
    sched_lock();
    status = sem_init(&termSem, 0, 0);
    CHECK_STATUS(status, 0,"sem_init:termSem");
    status = sem_init(&stopSem, 0, 0);
    CHECK_STATUS(status, 0,"sem_init:stopSem");
    status = sem_init(&baserateTaskSem, 0, 0);
    CHECK_STATUS(status, 0,"sem_init:baserateTaskSem");
    
    // or you can use/check: _POSIX_PRIORITY_SCHEDULING
    // _POSIX_THREAD_PRIORITY_SCHEDULING
#if !defined (_POSIX_PRIORITY_SCHEDULING)
    warnx("Priority scheduling is NOT supported by your system.");
    warnx("The generated code will not run correctly because your");
    warnx("model contains multiple rates and uses multi-tasking");
    warnx("code generation mode. You can only run the generated code");
    warnx("in single-tasking mode in your system. Open");
    warnx("Simulation -> Configuration Parameters -> Solver dialog");
    warnx("and set \"Tasking mode for periodic sample times\" parameter to SingleTasking.");
    warnx("Re-build the Simulink model with the new settings and try executing the generated code again.");
    exit(-1);
#endif
    
    /* Set scheduling policy of the main thread to SCHED_FIFO */
    g_sp.sched_priority = sched_get_priority_max(SCHED_FIFO) - 50;
    status = sched_setscheduler(0, SCHED_FIFO, &g_sp);
    CHECK_STATUS(status, 0,"sched_setscheduler");
    
    /*Added init attribute and scheduler policy */
    pthread_attr_init(&attr);
    status = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	CHECK_STATUS(status, 0,"pthread_attr_setinheritsched");
	status = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	CHECK_STATUS(status, 0,"pthread_attr_setschedpolicy");
	
    /* PTHREAD_STACK_MIN is the minimum stack size required to start a thread */
    stackSize = 2048 + PTHREAD_STACK_MIN;
    
    /*added stack attribute back in */
    status = pthread_attr_setstacksize(&attr, stackSize);
    CHECK_STATUS(status, 0,"pthread_attr_setstacksize");


#ifdef MW_RTOS_DEBUG
    warnx("   stackSize = %d sched_priority = %d", stackSize, g_sp.sched_priority);
#endif
    
    /* set up info structure */
    g_info.period = baseRatePeriod;
    g_info.sigNo = BASERATE_TIMER_SIGNAL; // SIGRTMIN;
    
#ifdef MW_RTOS_DEBUG
    warnx("   MW_BASERATE_PERIOD = %8.5f MW_BASERATE_PRIORITY = %d SIGRTMIN = 0x%08X", (double)baseRatePeriod, (int)baseRatePriority, SIGRTMIN);
    warnx("   Init info.period = %8.5f sigNo = 0x%04X", g_info.period, g_info.sigNo);
#endif

/* Create the Base Rate Task here */
#ifdef MW_RTOS_DEBUG
    warnx("**creating the Base Rate thread before calling pthread_create**");
#endif
    g_sp.sched_priority = baseRatePriority;
	status = pthread_attr_setschedparam(&attr, &g_sp);

    status = pthread_create(&baseRateThread, &attr, (void *) baseRateTask, (void *) &g_info);
#ifdef MW_RTOS_DEBUG
    warnx("** Base Rate Task ID = %d with Priority = %d", baseRateThread,g_sp.sched_priority);
#endif
     
    

#ifdef MW_HAS_MULTIPLE_RATES

	#ifdef MW_RTOS_DEBUG    
		printf("**Creating subrate task threads**\n");
		fflush(stdout);
	#endif
						
	warnx("Number of sub rate tasks: %d \n",MW_NUMBER_SUBRATES);
	
	for (i = 0; i < MW_NUMBER_SUBRATES; i++) {
		taskId[i] = i;
		status = sem_init(&subrateTaskSem[i], 0, 0);
		CHECK_STATUS(status, 0,"sem_init: subrateTaskSem");
		g_sp.sched_priority = MW_BASERATE_PRIORITY -40 + subratePriority[i] - 1;
		status = pthread_attr_setschedparam(&attr, &g_sp);
		CHECK_STATUS(status, 0,"pthread_attr_setschedparam");
		status = pthread_create(&subRateThread[i], &attr, (void *) subrateTask, (void *)&taskId[i]);
	}
			

#endif //End of "If Multiple Rates" check

    
/* Create the Terminate Task here */
#ifdef MW_RTOS_DEBUG
    warnx("**creating the terminate thread before calling pthread_create**");
#endif
    g_sp.sched_priority = baseRatePriority;
	status = pthread_attr_setschedparam(&attr, &g_sp);
    
    status = pthread_create(&terminateThread, &attr, (void *) terminateTask, (void *) &g_info);
#ifdef MW_RTOS_DEBUG
    warnx("** Terminate Task ID = %d with Prioirity = %d", terminateThread, g_sp.sched_priority);
#endif
   
    
/* Create the Scheduler Task here */
long stMicroSec;
stMicroSec = (long)(g_info.period * 1e6);
hrt_call_every(&BaseRate_HRT, stMicroSec, stMicroSec, Base_HRT_Callback, NULL); // timing callback to post base-rate semaphore


 //define background task for EXT Mode   
#if EXT_MODE == 1 
	#ifdef EXT_MODE_BKGND
	/*---------- Creating EXT-mode Background Task ------------------*/
		/* Setup signal info to block for EXT Mode */
		sem_init(&ExtModeTsk_Sem,0,0); // init semaphore here
		
		EXT_sig_info.period = 0.1; //Sample rate at which we want Simulink to update w/ ext mode
		EXT_sig_info.sigNo = SIGRTMIN; 
		sigemptyset(&EXT_sig_info.sigMask);
		sigaddset(&EXT_sig_info.sigMask, EXT_sig_info.sigNo);
		myAddBlockForThisEvent(EXT_sig_info.sigNo);

		/* Setup thread for Custom Task */
		fflush(stdout);
		g_sp.sched_priority = 40; //default is 100
		status = pthread_attr_setschedparam(&attr, &g_sp);
		status = pthread_create(&EXT_Mode_Thread, &attr, (void *) ExtModeTask, (void *) &EXT_sig_info);
		CHECK_STATUS(status, 0,"EXT Mode Background pthread_create");
	/*---------------------------------------------------------*/
	#endif
#endif
    
#ifdef MW_RTOS_DEBUG
    warnx("**DONE! creating simulink task threads**");
#endif
    sched_unlock();
}