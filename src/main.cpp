#include <Arduino_FreeRTOS.h>
#include <semphr.h>  
#include <Arduino.h>

SemaphoreHandle_t xSerialSemaphoreGlobal;
SemaphoreHandle_t xSerialSemaphore;

void TaskSendData( void *pvParameters );
void TaskAnalogRead1( void *pvParameters );
void TaskAnalogRead2( void *pvParameters );
void TaskAnalogRead3( void *pvParameters );
void TaskAnalogRead4( void *pvParameters );
static void filtro( void *pvParameters );


static TaskHandle_t TaskHandleAnalogRead1;
static TaskHandle_t TaskHandleAnalogRead2;
static TaskHandle_t TaskHandleAnalogRead3;
static TaskHandle_t TaskHandleAnalogRead4;
static TaskHandle_t xTaskFiltro;


QueueHandle_t xQueue1;
QueueHandle_t xQueue2;
QueueHandle_t xQueue3;
QueueHandle_t xQueue4;
QueueHandle_t xQueue5;

#define NZEROS 8
#define NPOLES 8
#define GAIN   1.067888271e+00

int next_input_value = 1;
int next_output_value = 0;

static float xv[NZEROS+1], yv[NPOLES+1];



void setup() {

Serial.begin(9600);

while (!Serial) {
  ; 
}

xQueue1 = xQueueCreate( 3, sizeof( int ) );
xQueue2 = xQueueCreate( 3, sizeof( int ) );
xQueue3 = xQueueCreate( 3, sizeof( int ) );
xQueue4 = xQueueCreate( 3, sizeof( int ) );
xQueue5 = xQueueCreate( 3, sizeof( int ) );

if ( xSerialSemaphore == NULL )  
{
  xSerialSemaphore = xSemaphoreCreateMutex();  
  if ( ( xSerialSemaphore ) != NULL )
    xSemaphoreGive( ( xSerialSemaphore ) );  
}

if ( xSerialSemaphoreGlobal == NULL )  
{
  xSerialSemaphoreGlobal = xSemaphoreCreateMutex();  
  if ( ( xSerialSemaphoreGlobal ) != NULL )
    xSemaphoreGive( ( xSerialSemaphoreGlobal ) );  
}

xTaskCreate(
  TaskSendData
  ,  (const portCHAR *)"DigitalRead"  
  ,  128  
  ,  NULL
  ,  2  
  ,  &TaskHandleAnalogRead1 );

xTaskCreate(
  TaskAnalogRead1
  ,  (const portCHAR *) "AnalogRead"
  ,  128  
  ,  NULL
  ,  2  
  ,  &TaskHandleAnalogRead1 );

	xTaskCreate(
  TaskAnalogRead2
  ,  (const portCHAR *) "AnalogRead"
  ,  128  
  ,  NULL
  ,  2  
  ,  &TaskHandleAnalogRead2 );

  xTaskCreate(
  TaskAnalogRead3
  ,  (const portCHAR *) "AnalogRead"
  ,  128  
  ,  NULL
  ,  2  
  ,  TaskHandleAnalogRead3 );

	xTaskCreate(
  TaskAnalogRead4
  ,  (const portCHAR *) "AnalogRead"
  ,  128  
  ,  NULL
  ,  2  
  ,  TaskHandleAnalogRead4 );

	xTaskCreate(
  filtro
  ,  (const portCHAR *) "AnalogRead"
  ,  128  
  ,  NULL
  ,  1  
  ,  &xTaskFiltro );






}

void loop()
{
  // Empty. 
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskSendData( void *pvParameters __attribute__((unused)) )  
{
  uint8_t pushButton = 2;

  
  pinMode(pushButton, INPUT);

  for (;;) 
  {
    int buttonState = digitalRead(pushButton);

 		int pxRxedMessage1;
 		int pxRxedMessage2;
 		int pxRxedMessage3;
 		int pxRxedMessage4;
 		int pxRxedMessage5;
    
		if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {

      if( xQueue1 != 0 )
      {
        if( xQueueReceive( xQueue1, &( pxRxedMessage1 ), 0 ) )
        {
       	  ;
        }
      }
  		if( xQueue2 != 0 )
      {
        if( xQueueReceive( xQueue2, &( pxRxedMessage2 ), 0 ) )
        {
       	  ;
        }
      }
  		if( xQueue3 != 0 )
      {
        if( xQueueReceive( xQueue3, &( pxRxedMessage3 ), 0 ) )
        {
       	  ;
        }
      }
  		
			if( xQueue4 != 0 )
      {
        if( xQueueReceive( xQueue4, &( pxRxedMessage4 ), 0 ) )
        {
       	  ;
        }
      }
			if( xQueue5 != 0 )
      {
        if( xQueueReceive( xQueue5, &( pxRxedMessage5 ), 0 ) )
        {
       	  ;
        }
      }

      Serial.print("?");
      Serial.print("44740");
      Serial.print("-");
      Serial.print("44741");
      Serial.print("-");
      Serial.print((int)pxRxedMessage1);
      Serial.print("-");
      Serial.print((int)pxRxedMessage2);
      Serial.print("-");
      Serial.print((int)pxRxedMessage3);
      Serial.print("-");
      Serial.print((int)pxRxedMessage5);
      Serial.print("-");
      Serial.print("%");
      Serial.println("");

      xSemaphoreGive( xSerialSemaphore ); 
    }
    vTaskDelay(10);  
  }
}

void TaskAnalogRead1( void *pvParameters __attribute__((unused)) )  
{

  for (;;)
  { int status;
    int sensorValue = analogRead(A0);

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      if( xQueueSend( xQueue1, ( void * )	&sensorValue, ( TickType_t ) 10 / portTICK_PERIOD_MS ) == pdPASS )
        {
          ;
        }

 			xSemaphoreGive( xSerialSemaphore ); 
    }

    vTaskDelay(10);  
  }
}

void TaskAnalogRead2( void *pvParameters __attribute__((unused)) )  
{

  for (;;)
  {
    int sensorValue = analogRead(A1);

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      if( xQueueSend( xQueue2, ( void * )	&sensorValue, ( TickType_t ) 10 / portTICK_PERIOD_MS ) == pdPASS )
        {
          ;
        }

      xSemaphoreGive( xSerialSemaphore ); 
    }

    vTaskDelay(10);  
  }
}

void TaskAnalogRead3( void *pvParameters __attribute__((unused)) )  
{

  for (;;)
  {
    int sensorValue = analogRead(A2);

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      if( xQueueSend( xQueue3, ( void * )	&sensorValue, ( TickType_t ) 10 / portTICK_PERIOD_MS ) == pdPASS )
        {
          ;
        }
      xSemaphoreGive( xSerialSemaphore ); 
    }

    vTaskDelay(10);  
  }
}

void TaskAnalogRead4( void *pvParameters __attribute__((unused)) )  
{

  for (;;)
  {
    int sensorValue = analogRead(A3);

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      if( xQueueSend( xQueue4, ( void * )	&sensorValue, ( TickType_t ) 10 / portTICK_PERIOD_MS ) == pdPASS )
        {
          ;
        }
      xSemaphoreGive( xSerialSemaphore ); 
    }

    vTaskDelay(1);  
  }
}

static void filtro( void *pvParameters )
{

 		int pxRxedMessage5;
	for(;;)
	{
	
	if( xQueue4 != 0 )
  {
    if( xQueueReceive( xQueue4, &( pxRxedMessage5 ), 0 ) )
    {
    }
  }
	xv[0] = xv[1];
	xv[1] = xv[2];
	xv[2] = xv[3];
	xv[3] = xv[4];
	xv[4] = xv[5];
	xv[5] = xv[6];
	xv[6] = xv[7];
	xv[7] = xv[8];
	xv[8] = pxRxedMessage5 / GAIN;
	yv[0] = yv[1];
	yv[1] = yv[2];
	yv[2] = yv[3];
	yv[3] = yv[4];
	yv[4] = yv[5];
	yv[5] = yv[6];
	yv[6] = yv[7];
	yv[7] = yv[8];
	yv[8] =   (xv[0] + xv[8]) 	 -   2.4729169287  * (xv[1] + xv[7]) +   6.2932443010 * (xv[2] + xv[6])
								 -   8.3639178949  * (xv[3] + xv[5]) +  10.7325710860 * xv[4]
								 + ( -0.8768965608 * yv[0]) + (  2.2040944764 * yv[1])
								 + ( -5.7004581072 * yv[2]) + (  7.7011550694 * yv[3])
								 + (-10.0446924200 * yv[4]) + (  7.9582550879 * yv[5])
								 + ( -6.0874180044 * yv[6]) + (  2.4323167850 * yv[7]);

	next_output_value = yv[8];

  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
  {
    if( xQueueSend( xQueue5, ( void * )	&next_output_value , ( TickType_t ) 10 / portTICK_PERIOD_MS ) == pdPASS )
      {
      }

    xSemaphoreGive( xSerialSemaphore ); 
  }

	vTaskDelay( 2 );
	}
}
