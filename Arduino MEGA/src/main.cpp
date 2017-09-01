/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"


#include "Arduino_FreeRTOS.h"
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).



// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define two Tasks for DigitalRead & AnalogRead
void TaskDigitalRead( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void TaskBlink( void *pvParameters );
void TaskLightControl (void *pvParameters);


int sensorValue=0;

// the setup function runs once when you press reset or power the board
void setup() {

  pinMode(6,OUTPUT);

  TCCR4A =
      (1 << COM1A1) | (1 << COM1B1) |
      // Fast PWM mode.
      (1 << WGM11);
  TCCR4B =
      // Fast PWM mode.
      (1 << WGM12) | (1 << WGM13) |
      // No clock prescaling (fastest possible
      // freq).
      (1 << CS10);
  OCR4A = 0;
  // Set the counter value that corresponds to
  // full duty cycle. For 15-bit PWM use
  // 0x7fff, etc. A lower value for ICR1 will
  // allow a faster PWM frequency.
  ICR4 = 0xffff;

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up two Tasks to run independently.
  xTaskCreate(
    TaskDigitalRead
    ,  (const portCHAR *)"DigitalRead"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskAnalogRead
    ,  (const portCHAR *) "AnalogRead" // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

    // Now set up two tasks to run independently.
    xTaskCreate(
      TaskBlink
      ,  (const portCHAR *)"Blink"   // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

      // Now set up two tasks to run independently.
      xTaskCreate(
        TaskLightControl
        ,  (const portCHAR *)"Light control"   // A name just for humans
        ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL );

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*-----    Digital Read Task          --------------*/
/*--------------------------------------------------*/
void TaskDigitalRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  /*
    DigitalReadSerial
    Reads a digital input on pin 2, prints the result to the serial monitor

    This example code is in the public domain.
  */

  // digital pin 2 has a pushbutton attached to it. Give it a name:
  uint8_t pushButton = 30;

  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT_PULLUP);


  for (;;) // A Task shall never return or exit.
  {
    // read the input pin:
    int buttonState = digitalRead(pushButton);

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
/*    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      Serial.println(buttonState);

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }*/

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

/*--------------------------------------------------*/
/*-----    Analog Read Task          ---------------*/
/*--------------------------------------------------*/
void TaskAnalogRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  for (;;)
  {
    // read the input on analog pin 0:
    sensorValue = analogRead(A0);

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    //if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
  //  {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
      //Serial.println(sensorValue);

    //  xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
  //  }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

/*--------------------------------------------------*/
/*-----    Activity LED Task         ---------------*/
/*--------------------------------------------------*/
void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN,OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 200 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 200 / portTICK_PERIOD_MS ); // wait for one second
  }
}
/*--------------------------------------------------*/
/*-----    Light Control Task         ---------------*/
/*--------------------------------------------------*/

void TaskLightControl(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
      int scale=map(sensorValue, 0, 1023, 0, 65535);
      //int scale=map(sensorValue, 0, 1023, 0, 255);
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
  //  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
  //  {
      OCR4A=scale;
      //analogWrite(6, scale); // set the lighting brightness to the set point:
      vTaskDelay(1);  // one tick delay (15ms) in between reads for stability

  //  Serial.print("Set point :");
  //  Serial.println(scale);

//    xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
//  }
  }
}
