
#include "Arduino.h"
#include "Arduino_FreeRTOS.h"
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).

#include "def.h"
//#include "../lib/DS1307.h"
#include <DS1307.h>
#include <HT16K33.h>
#include <DS1631.h>
#include <SeqButton.h>

Adafruit_7segment display = Adafruit_7segment(); //Instantiation of an Display object
/********** Adresses des composants I2C **********/
#define Def_I2C_Adr_SevenSeg			0x70          // I2C bus address of seven seg display
#define Def_I2C_Adr_DS1307				0x68          //I2C RTC address
#define Def_I2C_Adr_Temp				  DS1631_ADDR   // I2C bus address of DS1307 RTC
// ---------------------------------------------------------------------------

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;
SemaphoreHandle_t xI2cSemaphore;

// Define all Tasks
void TaskDigitalRead (void *pvParameters );
void TaskAnalogRead (void *pvParameters );
void TaskBlink (void *pvParameters );
void TaskLightControl (void *pvParameters);
void TaskDisplay (void *pvParameters);
void TaskRTC (void *pvParameters);
void TaskTempSensor (void *pvParameters);

//Define Task handle if necessarry
TaskHandle_t TaskDisplay_handle;

#define DS1307_I2C_ADDRESS 0x68  // Define the I²C address of the RTC module

DS1307 RTC; //Object TimeRTC
StructRTC srtc; //Structirte timeRTC
/*---Temp monitoring--*/
DS1631 Tsensor = DS1631(); //Instantiation of an Tsensor object

int sensorValue=0;

SeqButton	TempButton;

unsigned long time;

void setTempBut(void)
{
  Serial.println("[Event] -Suspend Display task");
  vTaskSuspend(TaskDisplay_handle);
  display.printFloat(Boitier.temp,1,10),
  display.drawColon(false);
  display.writeDisplay();
}

void relTempbut(void)
{
    Serial.println("[Event] Resume Display task");
    vTaskResume(TaskDisplay_handle);
}


// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
	Serial.println("[**Init**] Serial is enable at 115200 bps");

  // srtc.second=0;
  // srtc.minute=17;
  // srtc.hour=20;
  // srtc.day_of_week=6;
  // srtc.day_of_month=2;
  // srtc.month=9;
  // srtc.year=17;


  display.begin(Def_I2C_Adr_SevenSeg);     //Start I²C communication
  display.clear();         //Clear the display
  display.writeDisplay();
  display.setBrightness(1);

  RTC.init(0x68); //Init RTC
  //RTC.setDateDs1307(srtc);

  RTC.getDateDs1307(&srtc);
	display.printFloat((srtc.hour*100+srtc.minute),0,10);
	display.writeDisplay();

  Serial.print(srtc.hour);
  Serial.print(":");
  Serial.print(srtc.minute);
  Serial.print(":");
  Serial.println(srtc.second);

  // ----   Temp initialization       ----//
	Tsensor.writeConfig(13);  //Write to configuration registers of DS1631 -> Set to 12-bit, 1-shot mode
  //Read the temperature from the I2C device (must be each minutes at least)
  Boitier.temp=Tsensor.readTempOneShot();


  pinMode(6,OUTPUT);

  TCCR4A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Fast PWM mode.
  TCCR4B = (1 << WGM12) | (1 << WGM13) |(1 << CS10);  // No clock prescaling (fastest possible
    // freq).
  OCR4A = 0;
  // Set the counter value that corresponds to
  // full duty cycle. For 15-bit PWM use
  // 0x7fff, etc. A lower value for ICR1 will
  // allow a faster PWM frequency.
  ICR4 = 0xffff;

  // initialize the button with callback on push (no repeat)
  TempButton.init(31, &setTempBut,&relTempbut);

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


  if ( xI2cSemaphore == NULL )  // Check to confirm that the I2C has not already been created.
  {

    xI2cSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xI2cSemaphore ) != NULL )
      xSemaphoreGive( ( xI2cSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
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
      ,  5  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

      // Now set up two tasks to run independently.
    xTaskCreate(
      TaskRTC
      ,  (const portCHAR *)"RTC"   // A name just for humans
      ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

      // Now set up two tasks to run independently.
    xTaskCreate(
      TaskDisplay
      ,  (const portCHAR *)"Display"   // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  &TaskDisplay_handle );

      // Now set up two tasks to run independently.
    xTaskCreate(
      TaskTempSensor
      ,  (const portCHAR *)"TempSensor"   // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
  // Hooked to Idle Task, will run when CPU is Idle
  // time = millis();
  // Serial.print("[");
  // Serial.print(time);
  // Serial.print(F("Loop function"));
  // Serial.println("]");
  // delay(50);

}

/*--------------------------------------------------*/
/*-----    Digital Read Task          --------------*/
/*--------------------------------------------------*/
void TaskDigitalRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  // digital pin 2 has a pushbutton attached to it. Give it a name:
  uint8_t pushButton = 30;
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);

  while (1) // A Task shall never return or exit.
  {
    // read the input pin:
    int buttonState = digitalRead(pushButton);
    int buttonTemp = digitalRead(31);
    TempButton.handler();
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) //Means that the ressource is available
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      if(!buttonState){
        Serial.print(srtc.hour);
        Serial.print(":");
        Serial.print(srtc.minute);
        Serial.print(":");
        Serial.println(srtc.second);
        Serial.print("Temp:");
        Serial.println(Boitier.temp);
      }

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

/*--------------------------------------------------*/
/*-----    Analog Read Task - 15ms   ---------------*/
/*--------------------------------------------------*/
void TaskAnalogRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  while (1)
  {
    // read the input on analog pin 0:
    sensorValue = analogRead(A0);
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

  while (1) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 150 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 150 / portTICK_PERIOD_MS ); // wait for one second
  }
}

/*--------------------------------------------------*/
/*-----    Light Control Task   15ms ---------------*/
/*--------------------------------------------------*/
void TaskLightControl(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  while (1) // A Task shall never return or exit.
  {
      int scale=map(sensorValue, 0, 1023, 0, 65535);
      //int scale=map(sensorValue, 0, 1023, 0, 255);
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
  //  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
  //  {
      OCR4A=scale; // Update PWM Output

      //Serial.println(RTC.TimeRTC.second);
      //analogWrite(6, scale); // set the lighting brightness to the set point:
      vTaskDelay(1);  // one tick delay (15ms) in between reads for stability

  //  Serial.print("Set point :");
  //  Serial.println(scale);

//    xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
//  }
  }
}

/*--------------------------------------------------*/
/*-----       RTC I2C Task           ---------------*/
/*--------------------------------------------------*/
void TaskRTC(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  while (1) // A Task shall never return or exit.
  {
      //int scale=map(sensorValue, 0, 1023, 0, 255);
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
      if ( xSemaphoreTake( xI2cSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {

        RTC.getDateDs1307(&srtc);
        //analogWrite(6, scale); // set the lighting brightness to the set point:
        vTaskDelay(66.66);  // one tick delay (15ms) in between reads for stability

    //  Serial.print("Set point :");
    //  Serial.println(scale);

      xSemaphoreGive(xI2cSemaphore ); // Now free or "Give" the I2C Port for others.
    }
  }
}

/*--------------------------------------------------*/
/*-----       Display Task               ---------------*/
/*--------------------------------------------------*/
void TaskDisplay(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

static bool dot=false;
    // //Display initialization
    // 	display.begin(0x70);     //Start I²C communication
    // 	display.clear();         //Clear the display
    // 	display.writeDisplay();
    // 	display.setBrightness(1);
    // //	display.printError();

// ----   Display initialization       ----//
	// display.begin(Def_I2C_Adr_SevenSeg);     //Start I²C communication
	// display.clear();         //Clear the display
	// display.writeDisplay();
	// display.setBrightness(1);
  //	display.blinkRate(HT16K33_BLINK_2HZ);
//-------------------------------------------------//
      // 	/*Display 'Init' on the display*/
  // display.writeDigitRaw(0,48);
  // display.writeDigitRaw(1,84);
  // display.writeDigitRaw(3,48);
  // display.writeDigitRaw(4,112);
  // display.writeDisplay();
  /*****************************/

  while (1) // A Task shall never return or exit.
  {
    // display.printFloat(100,0,10);
		// display.writeDisplay();

    // // 	/*Print'Init' on the display*/
    	// display.writeDigitRaw(0,48);
    	// display.writeDigitRaw(1,84);
    	// display.writeDigitRaw(3,48);
    	// display.writeDigitRaw(4,112);
    	// display.writeDisplay();

    if ( xSemaphoreTake( xI2cSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // RTC.getDateDs1307(&srtc);
			 dot=!dot;
			 display.printFloat((srtc.hour*100+srtc.minute),0,10);
			 display.writeDisplay();
			 display.drawColon(dot);//Serial3.println("1s");
			 display.writeDisplay();


    }
      xSemaphoreGive(xI2cSemaphore ); // Now free or "Give" the I2C Port for others.

    vTaskDelay(66.66);
  }
}

/*--------------------------------------------------*/
/*-----       Temperature Task  15 sec ---------------*/
/*--------------------------------------------------*/
void TaskTempSensor(void *pvParameters)  // This is a task.
{
    (void) pvParameters;
    while (1) // A Task shall never return or exit.
    {

      if ( xSemaphoreTake( xI2cSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        Boitier.temp=Tsensor.readTempOneShot();
        xSemaphoreGive(xI2cSemaphore ); // Now free or "Give" the I2C Port for others.
      vTaskDelay(1000);
    }
  }
}
