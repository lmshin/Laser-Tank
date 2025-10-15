/* FreeRTOS.org includes. */
#include <stdio.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "main.h"

#define MAX_LENGTH_REMOCON_CODE	32

#define BIT0_A	1000
#define BIT0_B	1500
#define BIT1_A	2000
#define BIT1_B	2500

// L298N 모터 드라이버 핀 정의
#define LEFT_MOTOR_IN1_PORT GPIOD
#define LEFT_MOTOR_IN1_PIN GPIO_PIN_4
#define LEFT_MOTOR_IN2_PORT GPIOD
#define LEFT_MOTOR_IN2_PIN GPIO_PIN_5

#define RIGHT_MOTOR_IN1_PORT GPIOC
#define RIGHT_MOTOR_IN1_PIN GPIO_PIN_2
#define RIGHT_MOTOR_IN2_PORT GPIOC
#define RIGHT_MOTOR_IN2_PIN GPIO_PIN_3

// L298N ENA (속도) 핀 정의 (PWM 사용)
#define LEFT_MOTOR_ENA_TIMER htim2 // 왼쪽 모터 ENA 핀에 연결된 타이머 핸들
#define LEFT_MOTOR_ENA_CHANNEL TIM_CHANNEL_1 // 해당 타이머의 채널
#define RIGHT_MOTOR_ENA_TIMER htim2 // 오른쪽 모터 ENA 핀에 연결된 타이머 핸들
#define RIGHT_MOTOR_ENA_CHANNEL TIM_CHANNEL_2 // 해당 타이머의 채널

// vDriveControlTask PWM 최대 값 (타이머 설정에 따라 변경)
#define PWM_MAX_VALUE 65535

/* task's priority */
#define MAIN_TASK_PRIO	1
#define RX_TASK_PRIO 5
#define PARSER_TASK_PRIO 4
#define GIMBAL_TASK_PRIO 3
#define DRIVE_TASK_PRIO 1
#define LASER_TASK_PRIO 2

//GimbalTask 를 위한 코드 시작
#include "stm32f4xx_hal.h"
extern TIM_HandleTypeDef htim3;

#define PULSE_INCREMENT 100 // 조정 단위
#define MAX_PULSE_VERTICAL 2000
#define MIN_PULSE_VERTICAL 1000
#define MAX_PULSE_HORIZONTAL 2000
#define MIN_PULSE_HORIZONTAL 1000


extern uint32_t current_pulse_vertical;
extern uint32_t current_pulse_horizontal;
QueueHandle_t qid;
#define Test_TASK_ID	12345

typedef enum {
	VERTICAL_DOWN,
	VERTICAL_UP,
	HORIZONTAL_LEFT,
	HORIZONTAL_RIGHT,

} MotorCommand;

typedef struct {
	char ucMessageID;;
	MotorCommand motorCommand;
    // ...etc
} Gimbal_Message_t;

TaskHandle_t xHandleGimbal;

//typedef struct tag_qBuffer {
//	char ucMessageID;
//	char ucData[10]; // bugfix. 배열의 크기에 따른 스택오버플로우 발생 위험(2021/9/5)
//}qBuffer;

#define QUEUE_ITEM_SIZE sizeof(Gimbal_Message_t)

////GimbalTask 를 위한 코드 종료


/* The task functions. */
void MainTask( void *pvParameters );
static int getSigBitNumber( int usec );
void vRemoteRxTask( void *pvParameters );
void vRemoteParserTask( void *pvParameters );
void vGimbalControlTask( void *pvParameters );
void vDriveControlTask( void *pvParameters );
void stopLaserTimerCallback( TimerHandle_t xTimer );
void vLaserControlTask( void *pvParameters );

/* ...........................................................................
 *
 * 메시지큐 & 사용자 정의 블럭 정의
 * ===================
 */
QueueHandle_t xGimbalControlQueue, xDriveQueue, xLaserQueue;
extern QueueHandle_t xRemoteRxQueue, xRemoteParserQueue;
TimerHandle_t GimbalTimer, DriveTimer, xLaserTimer;
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

typedef enum {
	DRIVE_FORWARD,
	DRIVE_BACKWARD,
	DRIVE_CW,
	DRIVE_CCW,
	DRIVE_STOP
} DriveEventType;

typedef struct {
	DriveEventType eventType;
} DriveMessage_t;

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

	// 모터 제어를 위한 PWM 타이머 시작
	HAL_TIM_PWM_Start(&LEFT_MOTOR_ENA_TIMER, LEFT_MOTOR_ENA_CHANNEL);
	HAL_TIM_PWM_Start(&RIGHT_MOTOR_ENA_TIMER, RIGHT_MOTOR_ENA_CHANNEL);

	// 펄스 너비 정보를 전달할 큐 생성 (ISR -> RxTask)
	xRemoteRxQueue = xQueueCreate( QUEUE_LENGTH, sizeof( uint32_t ) );
	// 디코딩된 코드를 전달할 큐 생성 (RxTask -> ParserTask)
	xRemoteParserQueue = xQueueCreate( QUEUE_LENGTH, sizeof( uint32_t ) );
	xDriveQueue = xQueueCreate( QUEUE_LENGTH, sizeof( uint32_t ) );
	xGimbalControlQueue =  xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
	xLaserQueue = xQueueCreate( QUEUE_LENGTH, sizeof( uint32_t ) );
	if (xRemoteRxQueue == NULL || xRemoteParserQueue == NULL || xLaserQueue == NULL || xGimbalControlQueue == NULL || xDriveQueue == NULL) {
		printf("Error: 큐 생성 실패.\n");
		Error_Handler();
	}

	// 태스크 생성
    xTaskCreate( (TaskFunction_t)vRemoteRxTask, "RemoteRxTask", 256, NULL, RX_TASK_PRIO, &xRemoteRxTaskHandle );
    xTaskCreate( (TaskFunction_t)vRemoteParserTask, "RemoteParserTask", 256, NULL, PARSER_TASK_PRIO, NULL );
    xTaskCreate( (TaskFunction_t)vDriveControlTask, "DriveControlTask", 256, NULL, DRIVE_TASK_PRIO, NULL );
    xTaskCreate(  (TaskFunction_t)vGimbalControlTask, "GimbalTask", configMINIMAL_STACK_SIZE * 2, NULL, GIMBAL_TASK_PRIO, &xHandleGimbal );
    xTaskCreate( (TaskFunction_t)vLaserControlTask, "LaserControlTask", 256, NULL, LASER_TASK_PRIO, NULL );

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
    DriveMessage_t dmsg;

    printf( "%s is running\r\n", pcTaskName );

	uint32_t ulReceivedCode;

	while(1)
	{
		// RemoteRxTask로부터 데이터가 올 때까지 대기
		if ( xQueueReceive( xRemoteParserQueue, &ulReceivedCode, portMAX_DELAY ) == pdPASS )
		{
			// 수신된 코드를 파싱
			printf("파서 태스크에서 코드 수신: 0x%08lX\n", ulReceivedCode); fflush(stdout);

			//이게
			switch( ulReceivedCode )
			{
				// 예시: 리모콘 코드를 기반으로 다른 큐에 메시지 전송
//				case IR_CODE_LEFT:
//					// xQueueSend( xGimbalQueue, ...);
//					break;
//				case IR_CODE_RIGHT:
//					// xQueueSend( xDriveQueue, ...);
//					break;
            case 0x003FC639: // 2
				dmsg.eventType = DRIVE_FORWARD;
				// 큐에 메시지 전송
				if ( xQueueSend( xDriveQueue, &dmsg, 0 ) != pdPASS ) {
					printf("xDriveQueue error\n");
				}
                break;
            case 0x003FD2AD: // 8
				dmsg.eventType = DRIVE_BACKWARD;
				// 큐에 메시지 전송
				if ( xQueueSend( xDriveQueue, &dmsg, 0 ) != pdPASS ) {
					printf("xDriveQueue error\n");
				}
                break;
            case 0x003FD6A9: // 6
				dmsg.eventType = DRIVE_CW;
				// 큐에 메시지 전송
				if ( xQueueSend( xDriveQueue, &dmsg, 0 ) != pdPASS ) {
					printf("xDriveQueue error\n");
				}
                break;
            case 0x003FC43B: // 4
				dmsg.eventType = DRIVE_CCW;
				// 큐에 메시지 전송
				if ( xQueueSend( xDriveQueue, &dmsg, 0 ) != pdPASS ) {
					printf("xDriveQueue error\n");
				}
                break;
            case 0x003FCE31: // 5
				dmsg.eventType = DRIVE_STOP;
				// 큐에 메시지 전송
				if ( xQueueSend( xDriveQueue, &dmsg, 0 ) != pdPASS ) {
					printf("xDriveQueue error\n");
				}
                break;
			default:
				break;
			}
		}
	}
}
/*-----------------------------------------------------------*/

void vGimbalControlTask( void *pvParameters )
{
	const char *pcTaskName = "GimbalControlTask";
	    Gimbal_Message_t msg;

	    printf( "%s is running\r\n", pcTaskName );

	    // 큐 생성
	//    GimbalQueue = xQueueCreate(QUEUE_LENGTH, sizeof(Gimbal_Message_t));
	//    if (GimbalQueue == NULL) {
	//        // 오류 처리
	//        printf("xQueueCreate error found(GimbalQueue)\n");
	//    }

	    // 무한 루프: 큐에서 메시지를 대기하고 처리
	    while (1) {
	        if (xQueueReceive(qid, &msg, portMAX_DELAY) == pdPASS) {
	            switch (msg.motorCommand) {
	                case VERTICAL_DOWN:
	                	current_pulse_vertical -= PULSE_INCREMENT;
	                	if(current_pulse_vertical < MIN_PULSE_VERTICAL){
	                		current_pulse_vertical = MIN_PULSE_VERTICAL;
	                	}
	                	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_pulse_vertical);
	                	printf("VERTICAL_DOWN\n");
	                    break;

	                case VERTICAL_UP:
	                	current_pulse_vertical += PULSE_INCREMENT;
	                	if(current_pulse_vertical > MAX_PULSE_VERTICAL){
	                		current_pulse_vertical = MAX_PULSE_VERTICAL;
	                	}
	                	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_pulse_vertical);
	                	printf("VERTICAL_UP\n");
	                    break;

	                case HORIZONTAL_LEFT:
	                	current_pulse_horizontal  -= PULSE_INCREMENT;
	                	if(current_pulse_horizontal < MIN_PULSE_HORIZONTAL){
	                		current_pulse_horizontal = MIN_PULSE_VERTICAL;
	                    }
	                	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, current_pulse_horizontal);
	                	printf("HORIZONTAL_LEFT\n");
	                    break;

	                case HORIZONTAL_RIGHT:
	                	current_pulse_horizontal  += PULSE_INCREMENT;
	                	 if(current_pulse_horizontal > MAX_PULSE_HORIZONTAL){
	                		 current_pulse_horizontal = MAX_PULSE_HORIZONTAL;
	                	 }
	                	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, current_pulse_horizontal);
	                	printf("HORIZONTAL_RIGHT\n");
	                    break;
	            }
	        }
	    }
}
/*-----------------------------------------------------------*/

void vDriveControlTask( void *pvParameters )
{
    const char *pcTaskName = "DriveControlTask";
    DriveMessage_t msg;

    printf( "%s is running\r\n", pcTaskName );

    // 큐 생성
    xDriveQueue = xQueueCreate(QUEUE_LENGTH, sizeof(DriveMessage_t));
    if (xDriveQueue == NULL) {
        printf("xQueueCreate error found(xDriveQueue)\n");
        vTaskDelete(NULL);
    }

    // 무한 루프: 큐에서 메시지를 대기하고 처리
    while (1) {
        if (xQueueReceive(xDriveQueue, &msg, portMAX_DELAY) == pdPASS) {
            switch (msg.eventType) {
			case DRIVE_FORWARD:
				printf("FORWARD\n");
				// 전진: 왼쪽과 오른쪽 모터 모두 정방향으로 최대 속도 회전
				HAL_GPIO_WritePin(LEFT_MOTOR_IN1_PORT, LEFT_MOTOR_IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LEFT_MOTOR_IN2_PORT, LEFT_MOTOR_IN2_PIN, GPIO_PIN_RESET);
				//HAL_TIM_SetCompare(&LEFT_MOTOR_ENA_TIMER, LEFT_MOTOR_ENA_CHANNEL, PWM_MAX_VALUE);

				HAL_GPIO_WritePin(RIGHT_MOTOR_IN1_PORT, RIGHT_MOTOR_IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(RIGHT_MOTOR_IN2_PORT, RIGHT_MOTOR_IN2_PIN, GPIO_PIN_RESET);
				//HAL_TIM_SetCompare(&RIGHT_MOTOR_ENA_TIMER, RIGHT_MOTOR_ENA_CHANNEL, PWM_MAX_VALUE);
				break;

			case DRIVE_BACKWARD:
				printf("BACKWARD\n");
				// 후진: 왼쪽과 오른쪽 모터 모두 역방향으로 최대 속도 회전
				HAL_GPIO_WritePin(LEFT_MOTOR_IN1_PORT, LEFT_MOTOR_IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LEFT_MOTOR_IN2_PORT, LEFT_MOTOR_IN2_PIN, GPIO_PIN_SET);
				//HAL_TIM_SetCompare(&LEFT_MOTOR_ENA_TIMER, LEFT_MOTOR_ENA_CHANNEL, PWM_MAX_VALUE);

				HAL_GPIO_WritePin(RIGHT_MOTOR_IN1_PORT, RIGHT_MOTOR_IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RIGHT_MOTOR_IN2_PORT, RIGHT_MOTOR_IN2_PIN, GPIO_PIN_SET);
				//HAL_TIM_SetCompare(&RIGHT_MOTOR_ENA_TIMER, RIGHT_MOTOR_ENA_CHANNEL, PWM_MAX_VALUE);
				break;

			case DRIVE_CW:
				printf("CW\n");
				// 시계방향 회전: 왼쪽(전진), 오른쪽(후진)
				HAL_GPIO_WritePin(LEFT_MOTOR_IN1_PORT, LEFT_MOTOR_IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LEFT_MOTOR_IN2_PORT, LEFT_MOTOR_IN2_PIN, GPIO_PIN_RESET);
				//HAL_TIM_SetCompare(&LEFT_MOTOR_ENA_TIMER, LEFT_MOTOR_ENA_CHANNEL, PWM_MAX_VALUE);

				HAL_GPIO_WritePin(RIGHT_MOTOR_IN1_PORT, RIGHT_MOTOR_IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RIGHT_MOTOR_IN2_PORT, RIGHT_MOTOR_IN2_PIN, GPIO_PIN_SET);
				//HAL_TIM_SetCompare(&RIGHT_MOTOR_ENA_TIMER, RIGHT_MOTOR_ENA_CHANNEL, PWM_MAX_VALUE);
				break;

			case DRIVE_CCW:
				printf("CCW\n");
				// 반시계방향 회전: 왼쪽(후진), 오른쪽(전진)
				HAL_GPIO_WritePin(LEFT_MOTOR_IN1_PORT, LEFT_MOTOR_IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LEFT_MOTOR_IN2_PORT, LEFT_MOTOR_IN2_PIN, GPIO_PIN_SET);
				//HAL_TIM_SetCompare(&LEFT_MOTOR_ENA_TIMER, LEFT_MOTOR_ENA_CHANNEL, PWM_MAX_VALUE);

				HAL_GPIO_WritePin(RIGHT_MOTOR_IN1_PORT, RIGHT_MOTOR_IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(RIGHT_MOTOR_IN2_PORT, RIGHT_MOTOR_IN2_PIN, GPIO_PIN_RESET);
				//HAL_TIM_SetCompare(&RIGHT_MOTOR_ENA_TIMER, RIGHT_MOTOR_ENA_CHANNEL, PWM_MAX_VALUE);
				break;

			case DRIVE_STOP:
				printf("STOP\n");
				// 정지: 모든 모터 정지 (IN1/IN2 모두 LOW로 설정)
				HAL_GPIO_WritePin(LEFT_MOTOR_IN1_PORT, LEFT_MOTOR_IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LEFT_MOTOR_IN2_PORT, LEFT_MOTOR_IN2_PIN, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(RIGHT_MOTOR_IN1_PORT, RIGHT_MOTOR_IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RIGHT_MOTOR_IN2_PORT, RIGHT_MOTOR_IN2_PIN, GPIO_PIN_RESET);

				// PWM 듀티 사이클을 0으로 설정
				//HAL_TIM_SetCompare(&LEFT_MOTOR_ENA_TIMER, LEFT_MOTOR_ENA_CHANNEL, 0);
				//HAL_TIM_SetCompare(&RIGHT_MOTOR_ENA_TIMER, RIGHT_MOTOR_ENA_CHANNEL, 0);
				break;
			}
		}
	}
}
/*-----------------------------------------------------------*/

void vLaserControlTask( void *pvParameters )
{
    const char *pcTaskName = "LaserControlTask";
    Message_t msg;


    printf( "%s is running\r\n", pcTaskName );

    // 큐 생성
    //xLaserQueue = xQueueCreate(QUEUE_LENGTH, sizeof(LaserMessage_t));
    if (xLaserQueue == NULL) {
		// 오류 처리
		printf("xQueueCreate error found(laserQueue)\n");
	}

    // 무한 루프: 큐에서 메시지를 대기하고 처리
    while (1) {
        if (xQueueReceive(xLaserQueue, &msg, portMAX_DELAY) == pdPASS) {
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
