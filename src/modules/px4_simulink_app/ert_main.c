/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
 *
 * Code generated for Simulink model 'px4_swashplateless_2016a'.
 *
 * Model version                  : 1.107
 * Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
 * C/C++ source code generated on : Mon Jun  4 14:17:10 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include <stdio.h>
#include <stdlib.h>
#include "px4_swashplateless_2016a.h"
#include "px4_swashplateless_2016a_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "rt_nonfinite.h"
#include "nuttx/config.h"
#include "nuttx/i2c.h"
#include "nuttx/mtd.h"
#include "nuttx/fs/ioctl.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "unistd.h"
#include "fcntl.h"
#include "errno.h"
#include "debug.h"
#include "time.h"
#include "math.h"
#include "poll.h"
#include "unistd.h"
#include "uORB/uORB.h"
#include "uORB/topics/sensor_combined.h"
#include "uORB/topics/vehicle_attitude.h"
#include "uORB/topics/optical_flow.h"
#include "drivers/drv_led.h"
#include "drivers/drv_pwm_output.h"
#include "drivers/drv_rc_input.h"
#include "drivers/drv_hrt.h"
#include "arch/board/board.h"
#include "sys/mount.h"
#include "sys/ioctl.h"
#include "sys/stat.h"
#include "systemlib/perf_counter.h"
#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "systemlib/param/param.h"
#include "nuttxinitialize.h"

void baseRateTask(void *arg);
void exitTask(void *arg);
void terminateTask(void *arg);
static void usage(const char *reason);
void SpawnThreads_Task(void *arg);
void ShutDown_Procedure(void);

/* For Pixhawk Firmware build-process */
__EXPORT int px4_simulink_app_main(int argc, char *argv[]);

/* Define semaphores */
sem_t stopSem;
sem_t termSem;
sem_t baserateTaskSem;
sem_t Thread_Spawner_Task_Sem;

#ifdef EXT_MODE_BKGND

sem_t ExtModeTsk_Sem;
volatile int ExtModeStarted = 0;
pthread_t EXT_Mode_Thread;

#endif

/* Base-rate HRT-callback is used to post base-rate semaphore */
extern struct hrt_call BaseRate_HRT;

/* Define pthread_t for aperiodic tasks */

/* Define pthread_t for nuttxinitialize.c */
pthread_t terminateThread;
pthread_t schedulerThread;
pthread_t baseRateThread;
int subratePriority[0];

/* Define the Base Rate Task here */
void baseRateTask(void *arg)
{
  volatile boolean_T noErr;
  noErr = (rtmGetErrorStatus(px4_swashplateless_2016a_M) == (NULL)) &&
    !rtmGetStopRequested(px4_swashplateless_2016a_M);
  sem_init(&baserateTaskSem,0,0);

#ifdef EXT_MODE_BKGND

  ExtModeStarted = 1;

#endif

  int sem_value;
  while (noErr ) {
    sem_getvalue(&baserateTaskSem, &sem_value);
    if (sem_value > 1) {

#ifdef PIXHAWK_PSP_HARD_REAL_TIME

      if (sem_value > 20) {
        printf("Hard real-time constraint violated, shutting down \n");
        fflush(stdout);
        ShutDown_Procedure();
      }

#else
#ifdef MW_RTOS_DEBUG

      printf("Task over-run, sem_value: %d \n",sem_value);
      fflush(stdout);

#endif

      sem_init(&baserateTaskSem,0,0);

#endif

    }

    sem_wait(&baserateTaskSem);

    /* External mode */
    {
      boolean_T rtmStopReq = false;
      rtExtModePauseIfNeeded(px4_swashplateless_2016a_M->extModeInfo, 2,
        &rtmStopReq);
      if (rtmStopReq) {
        rtmSetStopRequested(px4_swashplateless_2016a_M, true);
      }

      if (rtmGetStopRequested(px4_swashplateless_2016a_M) == true) {
        rtmSetErrorStatus(px4_swashplateless_2016a_M, "Simulation finished");
        break;
      }
    }

#ifndef EXT_MODE_BKGND

    /* External mode */
    {
      boolean_T rtmStopReq = false;
      rtExtModeOneStep(px4_swashplateless_2016a_M->extModeInfo, 2, &rtmStopReq);
      if (rtmStopReq) {
        rtmSetStopRequested(px4_swashplateless_2016a_M, true);
      }
    }

#endif

    px4_swashplateless_2016a_step();

    /* Get model outputs here */
    rtExtModeCheckEndTrigger();
    noErr = (rtmGetErrorStatus(px4_swashplateless_2016a_M) == (NULL)) &&
      !rtmGetStopRequested(px4_swashplateless_2016a_M);
  }                                    /* while */

  /* Wait for aperiodic threads to close */

  /* Close BaseRate thread */
  sem_post(&termSem);
}

/* Define the exit task here */
void exitTask(void *arg)
{
  sem_post(&stopSem);
}

/* Define the terminate task here */
void terminateTask(void *arg)
{
  printf("**blocking on termSem semaphore in terminateTask**\n");
  sem_wait(&termSem);
  printf("**terminating the model**\n");
  fflush(stdout);
  rtExtModeShutdown(2);

  /* Disable rt_OneStep() here */

  /* Terminate model */
  px4_swashplateless_2016a_terminate();
  sem_post(&stopSem);
}

// volatile int ExtModeStarted = 0;
void ExtModeTask(void *arg)
{

#ifdef EXT_MODE_BKGND

  volatile boolean_T noErr;
  baseRateInfo_t info = *((baseRateInfo_t *)arg);
  setTaskPeriod(info.period, info.sigNo);
  noErr = (rtmGetErrorStatus(px4_swashplateless_2016a_M) == (NULL));
  while (noErr) {
    myWaitForThisEvent(info.sigNo);
    if (ExtModeStarted==1) {
      //     sem_wait(&ExtModeTsk_Sem); To experiment with later
      /* External mode */
      {
        boolean_T rtmStopReq = false;
        rtExtModeOneStep(px4_swashplateless_2016a_M->extModeInfo, 2, &rtmStopReq);
        if (rtmStopReq) {
          rtmSetStopRequested(px4_swashplateless_2016a_M, true);
        }
      }
    }

    noErr = (rtmGetErrorStatus(px4_swashplateless_2016a_M) == (NULL));
  }

#endif

}

/* Define the nuttx function calls here */
/* Print the correct usage. */
static void usage(const char *reason)
{
  if (reason)
    warnx("%s\n", reason);
  errx(1,
       "usage: px4_simulink_app {start|stop|status} [-p <additional params>]\n\n");
}

void SpawnThreads_Task(void *arg)
{
  char *ExtModeInputArray[6];
  ExtModeInputArray[0]= "\0";
  ExtModeInputArray[1]= "-port\0";

#if defined(EXT_MODE_DESCRIPTOR) && defined(BAUD_RATE)

  ExtModeInputArray[2]= EXT_MODE_DESCRIPTOR;//This should be an option adjustable by the user
  ExtModeInputArray[4]= BAUD_RATE;     //This should be an option adjustable by the user

#else

  ExtModeInputArray[2]= "ttyS6";
  ExtModeInputArray[4]= "115200";

#endif

  ExtModeInputArray[3]= "-baud\0";
  ExtModeInputArray[5] = "-w\0";       //This will force the program to wait until it receives a packet from host machine
  rtExtModeParseArgs(6,ExtModeInputArray,NULL);
  px4_swashplateless_2016a_initialize();

  /* External mode */
  rtSetTFinalForExtMode(&rtmGetTFinal(px4_swashplateless_2016a_M));
  rtExtModeCheckInit(2);

  {
    boolean_T rtmStopReq = false;
    rtExtModeWaitForStartPkt(px4_swashplateless_2016a_M->extModeInfo, 2,
      &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(px4_swashplateless_2016a_M, true);
    }
  }

  rtERTExtModeStartMsg();
  sleep(1);
  nuttxRTOSInit(0.004, 250, 0);
  sem_wait(&Thread_Spawner_Task_Sem);
  hrt_cancel(&BaseRate_HRT);
  warnx("Received semaphore to end this task \n");
}

void ShutDown_Procedure()
{
  /*if we get here, it means we experienced a task over-run, this also implies:
     - the sem_value is > 10 and base-rate thread will still execute until the end of this current iteration
     - <model>_M structs error status will be set to ERROR
     - sub-rate semaphores states are not known - it is better to increment inside here to force exit */
  FILE* fp_taskover_run = NULL;
  char buff[20];
  struct tm *sTm;
  time_t now = time(NULL);
  sTm = gmtime (&now);
  strftime (buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", sTm);
  fp_taskover_run = fopen("/fs/microsd/log/task_overrun_log.txt","a+");
  fprintf(fp_taskover_run,"%s \n",buff);
  fclose(fp_taskover_run);
  rtmSetErrorStatus(px4_swashplateless_2016a_M, "Module finished");
  g_baseRateLife = false;
  sem_post(&Thread_Spawner_Task_Sem);
}

int px4_simulink_app_main(int argc, char *argv[]) //Px4 App
{
  int rc = 0;
  int SpawnThread_Task_PID = 0;
  if (argc < 1)
    usage("missing command");
  if (!strcmp(argv[1], "start")) {
    if (g_baseRateLife == false) {
      /* just start the Simulink Tasks here */
      printf("**starting the model**\n");
      fflush(stdout);
      g_baseRateLife = true;
      sem_init(&Thread_Spawner_Task_Sem,0,0);
      SpawnThread_Task_PID = task_create("Spawn_Thread_Tasks", 100, 2048, (void *)
        SpawnThreads_Task, (void *) &SpawnThread_Task_PID);
    } else {
      warnx("\t**model is already running**\n");
      fflush(stdout);
    }

    exit(0);
  }

  if (!strcmp(argv[1], "stop")) {
    if (g_baseRateLife == true) {
      rtmSetErrorStatus(px4_swashplateless_2016a_M, "Module finished");
      g_baseRateLife = false;
      sem_post(&baserateTaskSem);

      /* wait until the tasks completely finish */
      warnx("exiting model... waiting on stopSem...");

      /* Wait for stop semaphore */
      sem_wait(&stopSem);
      sem_post(&Thread_Spawner_Task_Sem);
    } else {
      warnx("\t**model is not running**\n");
      fflush(stdout);
    }

    exit(0);
  }

  if (!strcmp(argv[1], "status")) {
    if (g_baseRateLife) {
      warnx("\trunning\n");
    } else {
      warnx("\tnot started\n");
    }

    exit(0);
  }

  usage("unrecognized command");
  rc = 1;
  return rc;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
