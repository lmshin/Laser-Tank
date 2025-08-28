/* FreeRTOS.org includes. */
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#define MAX_LENGTH_REMOCON_CODE	31

#define BIT0_A	1000
#define BIT0_B	1500
#define BIT1_A	2000
#define BIT1_B	2500

/* task's priority */
#define MAIN_TASK_PRIO	1
#define RX_TASK_PRIO 5
#define PARSER_TASK_PRIO 4
#define GIMBAL_TASK_PRIO 3
#define DRIVE_TASK_PRIO 1
#define LASER_TASK_PRIO 2

/* The task functions. */
void MainTask( void *pvParameters );
static int getSigBitNumber( int usec );
void vRemoteRxTask( void *pvParameters );
void vRemoteParserTask( void *pvParameters );
void GimbalControlTask( void *pvParameters );
void DriveControlTask( void *pvParameters );
void LaserControlTask( void *pvParameters );

/* ...........................................................................
 *
 * 메시지큐 & 사용자 정의 블럭 정의
 * ===================
 */
QueueHandle_t GimbalQueue, DriveQueue, LaserQueue;
extern QueueHandle_t xRemoteRxQueue, xRemoteParserQueue;
TimerHandle_t GimbalTimer, DriveTimer, LaserTimer;
TaskHandle_t xHandleMain;
extern TaskHandle_t xRemoteRxTaskHandle;

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
					MAIN_TASK_PRIO,	/* This task will run at this priority */
					&xHandleMain );		/* We are not using the task handle. */
#endif

	// 펄스 너비 정보를 전달할 큐 생성 (ISR -> RxTask)
	xRemoteRxQueue = xQueueCreate( QUEUE_LENGTH, sizeof( uint32_t ) );

	// 디코딩된 코드를 전달할 큐 생성 (RxTask -> ParserTask)
	xRemoteParserQueue = xQueueCreate( QUEUE_LENGTH, sizeof( uint32_t ) );

	if (xRemoteRxQueue == NULL || xRemoteParserQueue == NULL) {
		printf("Error: 큐 생성 실패.\n");
		Error_Handler();
	}
    // 태스크 생성
    xTaskCreate( (TaskFunction_t)vRemoteRxTask, "RemoteRxTask", 256, NULL, RX_TASK_PRIO, &xRemoteRxTaskHandle );
    xTaskCreate( (TaskFunction_t)vRemoteParserTask, "RemoteParserTask", 256, NULL, PARSER_TASK_PRIO, NULL );

    vTaskStartScheduler();
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
	while(1) {
	fflush(stdout);
	vTaskDelay(1);
	//printf( "%s is deleted\r\n", pcTaskName );
	}
	//vTaskDelete (xHandleMain);	// vTaskDelete (NULL);
}
/*-----------------------------------------------------------*/

// 펄스 폭/간격을 기록하기 위한 변수
volatile uint32_t ulLastPulseTime;

// bit level of remote control signal
static int getSigBitNumber( int usec )
{
	// '0'( 1000 -> 1500 )
	if (usec >= BIT0_A && usec < BIT0_B)
	{
	  return 0;
	}
	// '1'( 2000 -> 2500 )
	if (usec >= BIT1_A && usec < BIT1_B)
	{
	  return 1;
	}
	return -1; // outbound value
}

void vRemoteRxTask( void *pvParameters )
{
    const char *pcTaskName = "vRemoteRxTask";
    uint32_t ulPulseDuration_ticks;
    static uint32_t ulPulseBuffer[MAX_LENGTH_REMOCON_CODE];
    static uint8_t ucPulseCount = 0;
    TickType_t xLastReceiveTime;

    printf( "%s is running\r\n", pcTaskName );

    // 태스크 핸들을 저장합니다. (필요한 경우 사용)
    xRemoteRxTaskHandle = xTaskGetCurrentTaskHandle();

    // 큐가 성공적으로 생성되었는지 확인합니다.
    if ( xRemoteRxQueue == NULL ) {
        printf("Error: xRemoteRxQueue not created.\n");
        vTaskDelete(NULL); // 태스크 종료
    }

    // 이 태스크는 큐에 펄스 데이터가 들어올 때마다 깨어납니다.
    while(1)
    {
    	// ISR로부터 큐에 데이터가 들어오길 기다립니다.
        // 타임아웃(100ms)을 설정하여 신호가 끊겼을 때를 감지합니다.
        if ( xQueueReceive( xRemoteRxQueue, &ulPulseDuration_ticks, pdMS_TO_TICKS(100) ) == pdPASS )
        {
            // 마지막 펄스와 현재 펄스 사이의 간격이 길다면, 새로운 신호가 시작되었다고 판단합니다.
            if ((xTaskGetTickCount() - xLastReceiveTime) > pdMS_TO_TICKS(50)) {
                ucPulseCount = 0; // 새 코드 수신을 위해 버퍼를 리셋
            }
            xLastReceiveTime = xTaskGetTickCount();

            // 펄스 너비를 버퍼에 저장합니다.
            if (ucPulseCount < MAX_LENGTH_REMOCON_CODE) {
                ulPulseBuffer[ucPulseCount++] = ulPulseDuration_ticks;
            }

            // 모든 펄스를 다 받았는지 확인합니다.
            if (ucPulseCount >= MAX_LENGTH_REMOCON_CODE) {
                // 버퍼에 저장된 펄스들을 하나의 코드로 디코딩합니다.
                uint32_t ulReceivedCode = 0;
                for(int i = 0; i < MAX_LENGTH_REMOCON_CODE; i++) {
                    int bit = getSigBitNumber(ulPulseBuffer[i]);
                    if (bit != -1) {
                        ulReceivedCode = (ulReceivedCode << 1) | bit;
                    }
                }

                // 디코딩된 코드를 파서 태스크로 보냅니다.
                xQueueSend(xRemoteParserQueue, &ulReceivedCode, pdMS_TO_TICKS(10));

                // 디버깅을 위해 결과 출력
                printf("디코딩된 IR 코드: 0x%08lX\n", ulReceivedCode); fflush(stdout);

                // 다음 코드를 받기 위해 카운터 리셋
                ucPulseCount = 0;
            }
        }
    }
}
/*-----------------------------------------------------------*/

void vRemoteParserTask( void *pvParameters )
{
    const char *pcTaskName = "RemoteParserTask";
    //Message_t msg;

    printf( "%s is running\r\n", pcTaskName );

	uint32_t ulReceivedCode;

	while(1)
	{
		// RemoteRxTask로부터 데이터가 올 때까지 대기
		if ( xQueueReceive( xRemoteParserQueue, &ulReceivedCode, portMAX_DELAY ) == pdPASS )
		{
			// 수신된 코드를 파싱
			printf("파서 태스크에서 코드 수신: 0x%08lX\n", ulReceivedCode); fflush(stdout);

			switch( ulReceivedCode )
			{
				// 예시: 리모콘 코드를 기반으로 다른 큐에 메시지 전송
//				case IR_CODE_LEFT:
//					// xQueueSend( xGimbalQueue, ...);
//					break;
//				case IR_CODE_RIGHT:
//					// xQueueSend( xDriveQueue, ...);
//					break;
				default:
					break;
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
    //GimbalQueue = xQueueCreate(QUEUE_LENGTH, sizeof(GimbalMessage_t));
    if (GimbalQueue == NULL) {
        // 오류 처리
        printf("xQueueCreate error found(GimbalQueue)\n");
    }

    // 무한 루프: 큐에서 메시지를 대기하고 처리
    while (1) {
        if (xQueueReceive(GimbalQueue, &msg, portMAX_DELAY) == pdPASS) {
            switch (msg.eventType) {
                //case EVENT_NUM01:
                    //break;

                //case EVENT_NUM02:
                    //break;

                //case EVENT_NUM03:
                    //break;
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
    //DriveQueue = xQueueCreate(QUEUE_LENGTH, sizeof(DriveMessage_t));
    if (DriveQueue == NULL) {
        // 오류 처리
        printf("xQueueCreate error found(DriveQueue)\n");
    }

    // 무한 루프: 큐에서 메시지를 대기하고 처리
    while (1) {
        if (xQueueReceive(DriveQueue, &msg, portMAX_DELAY) == pdPASS) {
            switch (msg.eventType) {
                //case EVENT_NUM01:
                    //break;

                //case EVENT_NUM02:
                    //break;

                //case EVENT_NUM03:
                    //break;
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
    //LaserQueue = xQueueCreate(QUEUE_LENGTH, sizeof(LaserMessage_t));
    if (LaserQueue == NULL) {
        // 오류 처리
        printf("xQueueCreate error found(LaserQueue)\n");
    }

    // 무한 루프: 큐에서 메시지를 대기하고 처리
    while (1) {
        if (xQueueReceive(LaserQueue, &msg, portMAX_DELAY) == pdPASS) {
            switch (msg.eventType) {
                //case EVENT_NUM01:
                    //break;

                //case EVENT_NUM02:
                   //break;

                //case EVENT_NUM03:
                    //break;
            }
        }
    }
}
