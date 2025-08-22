/* FreeRTOS.org includes. */
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* task's priority */
#define MAIN_TASK_PRIO	20
#define RX_TASK_PRIO 5
#define PARSER_TASK_PRIO 4
#define GIMBAL_TASK_PRIO 3
#define DRIVE_TASK_PRIO 1
#define LASER_TASK_PRIO 2

/* The task functions. */
void MainTask( void *pvParameters );
void vIR_Recv_Pin_IRQHandler( void );
void RemoteRxTask( void *pvParameters );
void RemoteParserTask( void *pvParameters );
void GimbalControlTask( void *pvParameters );
void DriveControlTask( void *pvParameters );
void LaserControlTask( void *pvParameters );

/* ...........................................................................
 *
 * 메시지큐 & 사용자 정의 블럭 정의
 * ===================
 */
QueueHandle_t RemoteQueue, GimbalQueue, DriveQueue, LaserQueue;
TimerHandle_t GimbalTimer, DriveTimer, LaserTimer;
TaskHandle_t xHandleMain;

#define QUEUE_LENGTH	10

/*
 * 메시지 큐에 사용할 사용자 정의 블록
 * 태스크 간에 교환할 데이터의 구조 정의
 */
typedef enum {
	EVENT_NUM1,
	EVENT_NUM2,
	EVENT_NUM3
} EventType;

typedef struct {
    EventType eventType;
    // ...etc
} Message_t;

/*-----------------------------------------------------------*/

void USER_THREADS( void )
{
	/* Setup the hardware for use with the Beagleboard. */
	//prvSetupHardware();
#ifdef CMSIS_OS
	osThreadDef(defaultTask, MainTask, osPriorityNormal, 0, 256);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
#else
	/* Create one of the two tasks. */
	xTaskCreate(	(TaskFunction_t)MainTask,		/* Pointer to the function that implements the task. */
					"MainTask",	/* Text name for the task.  This is to facilitate debugging only. */
					configMINIMAL_STACK_SIZE * 2,		/* Stack depth - most small microcontrollers will use much less stack than this. */
					NULL,		/* We are not using the task parameter. */
					TASK_MAIN_PRIO,	/* This task will run at this priority */
					&xHandleMain );		/* We are not using the task handle. */
#endif
}
/*-----------------------------------------------------------*/

void MainTask( void *pvParameters )
{
	const char *pcTaskName = "MainTask";

	/* Print out the name of this task. */
	printf( "%s is running\r\n", pcTaskName );

	/* Create the other task in exactly the same way. */
//	xTaskCreate(	(TaskFunction_t)Task,		/* Pointer to the function that implements the task. */
//					"Task",	/* Text name for the task.  This is to facilitate debugging only. */
//					256,		/* Stack depth - most small microcontrollers will use much less stack than this. */
//					NULL,		/* We are not using the task parameter. */
//					TASK_PRIO,	/* This task will run at this priority */
//					&xHandleTask );		/* We are not using the task handle. */

	/* delete self task */
	/* Print out the name of this task. */
	printf( "%s is deleted\r\n", pcTaskName );
	vTaskDelete (xHandleMain);	// vTaskDelete (NULL);
}
/*-----------------------------------------------------------*/

// ISR에서 RemoteRxTask에게 알림을 줄 때 사용할 핸들
TaskHandle_t xRemoteRxTaskHandle = NULL;

// 펄스 폭/간격을 기록하기 위한 변수
volatile uint32_t ulLastPulseTime;

//================================================================================
// IR 수신할 HAL_GPIO_EXTI_Callback에서 호출
//================================================================================
void vIR_Recv_Pin_IRQHandler( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ulCurrentTime = xTaskGetTickCountFromISR(); // 현재 시간(틱)을 얻음

    // 펄스의 간격을 계산 (이 값은 RemoteRxTask에서 사용)
    uint32_t ulPulseInterval = ulCurrentTime - ulLastPulseTime;
    ulLastPulseTime = ulCurrentTime; // 다음 측정을 위해 현재 시간 업데이트

    if( xRemoteRxTaskHandle != NULL )
    {
        // RemoteRxTask에게 "새로운 펄스가 도착했다"고 알림
        // (ISR 내에서 사용 가능한 API)
        vTaskNotifyGiveFromISR( xRemoteRxTaskHandle, &xHigherPriorityTaskWoken );
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

void RemoteRxTask( void *pvParameters )
{
    const char *pcTaskName = "RemoteRxTask";

    printf( "%s is running\r\n", pcTaskName );

    RemoteRxTaskHandle = xTaskGetCurrentTaskHandle();
    RemoteQueue = xQueueCreate( QUEUE_LENGTH, sizeof(uint32_t) );

    if (RemoteQueue == NULL) {
        // 오류 처리
        printf("xQueueCreate error found(GimbalQueue)\n");

    // 리모콘 데이터를 저장할 버퍼
    uint32_t ulPulseDataBuffer[MAX_PULSE_COUNT];
    uint32_t ulReceivedCode = 0;

    // 이 Task는 ISR로부터 신호가 올 때까지 대기
    while(1)
    {
        if ( ulTaskNotifyTake( pdTRUE, portMAX_DELAY ) == 1 )
        {
            // ISR이 기록한 ulLastPulseTime 값을 사용하여 디코딩
            // 이 로직은 IR 프로토콜(NEC, RC-5 등)에 따라 달라집니다.
            // ... (디코딩 로직) ...

            // 디코딩된 최종 코드를 xRemoteSignalQueue에 전송
            ulReceivedCode = DecodeIRSignal(ulPulseDataBuffer);
            xQueueSend( RemoteQueue, &ulReceivedCode, pdMS_TO_TICKS(10) );
        }
    }
}
/*-----------------------------------------------------------*/

void RemoteParserTask( void *pvParameters )
{
    const char *pcTaskName = "RemoteParserTask";
    //Message_t msg;

    printf( "%s is running\r\n", pcTaskName );

	uint32_t ulReceivedCode;

	while(1)
	{
		// RemoteRxTask로부터 데이터가 올 때까지 대기
		if ( xQueueReceive( RemoteQueue, &ulReceivedCode, portMAX_DELAY ) == pdPASS )
		{
			// 수신된 코드를 파싱
			switch( ulReceivedCode )
			{
//				case IR_CODE_...:
//					xQueueSend( GimbalQueue, ...);
//					break;
//				case IR_CODE_...:
//					xQueueSend( DriveQueue, ...);
//					break;
			}
		}
	}
}
/*-----------------------------------------------------------*/

void GimbalControlTask( void *pvParameters )
{
    const char *pcTaskName = "GimbalControlTask";
    Message_t msg;

    printf( "%s is running\r\n", pcTaskName );

    // 큐 생성
    GimbalQueue = xQueueCreate(QUEUE_LENGTH, sizeof(GimbalMessage_t));
    if (GimbalQueue == NULL) {
        // 오류 처리
        printf("xQueueCreate error found(GimbalQueue)\n");
    }

    // 무한 루프: 큐에서 메시지를 대기하고 처리
    while (1) {
        if (xQueueReceive(GimbalQueue, &msg, portMAX_DELAY) == pdPASS) {
            switch (msg.eventType) {
                case EVENT_NUM01:
                    break;

                case EVENT_NUM02:
                    break;

                case EVENT_NUM03:
                    break;
            }
        }
    }
}
/*-----------------------------------------------------------*/

void DriveControlTask( void *pvParameters )
{
    const char *pcTaskName = "DriveControlTask";
    Message_t msg;

    printf( "%s is running\r\n", pcTaskName );

    // 큐 생성
    DriveQueue = xQueueCreate(QUEUE_LENGTH, sizeof(DriveMessage_t));
    if (DriveQueue == NULL) {
        // 오류 처리
        printf("xQueueCreate error found(DriveQueue)\n");
    }

    // 무한 루프: 큐에서 메시지를 대기하고 처리
    while (1) {
        if (xQueueReceive(DriveQueue, &msg, portMAX_DELAY) == pdPASS) {
            switch (msg.eventType) {
                case EVENT_NUM01:
                    break;

                case EVENT_NUM02:
                    break;

                case EVENT_NUM03:
                    break;
            }
        }
    }
}
/*-----------------------------------------------------------*/

void LaserControlTask( void *pvParameters )
{
    const char *pcTaskName = "LaserControlTask";
    Message_t msg;

    printf( "%s is running\r\n", pcTaskName );

    // 큐 생성
    LaserQueue = xQueueCreate(QUEUE_LENGTH, sizeof(LaserMessage_t));
    if (LaserQueue == NULL) {
        // 오류 처리
        printf("xQueueCreate error found(LaserQueue)\n");
    }

    // 무한 루프: 큐에서 메시지를 대기하고 처리
    while (1) {
        if (xQueueReceive(LaserQueue, &msg, portMAX_DELAY) == pdPASS) {
            switch (msg.eventType) {
                case EVENT_NUM01:
                    break;

                case EVENT_NUM02:
                    break;

                case EVENT_NUM03:
                    break;
            }
        }
    }
}
