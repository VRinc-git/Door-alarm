#include "ESP32_SPIFFS.h"
#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* RTOS Macros */
#define mainAuto_RELOAD_TIMER_PERIOD  pdMS_TO_TICKS(1000)

/* Macros */
#define LED 2
#define RLED  14
#define GLED  12
#define BLED  13
#define DOOR 36
#define BUTTON 39


/* Hardware serial pins */
#define RXD2 16
#define TXD2 17


/* SPIFFS variables */
String SPIFF_dataItem;
//String number_data;


/* flags */
char doorFlag = -1;
int counter = 0;
bool movement_flag = 0;
bool BlockFlag = 1;


/* Millis */
unsigned long previousTime = 0;
unsigned long currentTime;


/* SIM800L variables */
char* send_data;
String message;
String phonenumber;
char pnumber[14];


/* MPU6050 */
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
int accX, accY, accZ;


/* Functions */
void led(bool R, bool G, bool B);
void Send_Data(char* number, char* message);
void ReceiveMode();


/************ Rtos varisbles *************/

/* task handle variables */
TaskHandle_t task1_handle;
TaskHandle_t task2_handle;
TaskHandle_t task3_handle;
TaskHandle_t task4_handle;
TaskHandle_t task5_handle;

/* queue handle variables */
QueueHandle_t send_queue;

/* Timer handle variable */
TimerHandle_t xAutoReloadTimer;

/* task handlers*/
void door_Task(void *pvParameters);
void send_Task(void *pvParameters);
void button_Task(void *pvParameters);
void recv_Task(void *pvParameters);
void move_Task(void *pvParameters);




/**************** Setup *******************/

void setup()
{

  /* GPIO setup */
  pinMode(DOOR, INPUT);
  pinMode(BUTTON, INPUT);
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT);

  /* UART Setup*/
  Serial.begin(115200);                                         //Serial monitor
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);                  //Hardware serial for sim800l

  /* LED */
  led(0, 0, 0);
  delay(100);
  led(0, 0, 1);

  /* SPIFFS Setup */
  if (!SPIFFS.begin())
  {
    Serial.println("SPIFFS Mounting Failed");
    while (1)
    {
      Serial.print("*");
      delay(1000);
    }
  }

  /* MPU6050 Setup */
  if (!mpu.begin()) {
    Serial.println(" MPU6050 Failed ");
    while (1)
    {
      Serial.print("*");
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);                // set accelerometer range to +-8G
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);                  // set filter bandwidth to 21 Hz

  Serial.println("Starting ...");

  mpu.getEvent(&a, &g, &temp);                                 // Storing the initial values
  accX = (int)(a.acceleration.x);
  Serial.println(accX);
  accY = (int)(a.acceleration.y);
  Serial.println(accY);
  accZ = (int)(a.acceleration.z);
  Serial.println(accZ);
  delay(100);

  /* Fetching Data from SPIFFS */
  SPIFF_dataItem = readFile(SPIFFS, "/data.txt");
  SPIFF_dataItem.trim();
  int strLen = SPIFF_dataItem.length() + 1;
  pnumber[strLen];
  SPIFF_dataItem.toCharArray(pnumber, strLen);
  Serial.println(pnumber);

  /* Queue */
  send_queue = xQueueCreate(10, sizeof(char*));

  if (send_queue == NULL)
  {
    Serial.println("Queue Creation Failed");
    while (1)
    {
      Serial.print("*");
      delay(1000);
    }
  }

  /* Timer */
  xAutoReloadTimer = xTimerCreate("Auto Reload",
                                  mainAuto_RELOAD_TIMER_PERIOD,
                                  pdTRUE,
                                  0,
                                  prvAutoReloadTimerCallback );
  if (xAutoReloadTimer == NULL)
  {
    Serial.println("Timer initialization failed");
    while (1)
    {
      Serial.print("*");
      delay(1000);
    }
  }

  /* Task */

  xTaskCreatePinnedToCore(
    door_Task,
    "Door Task",
    4096,
    NULL,
    1,
    &task1_handle,
    0);

  xTaskCreatePinnedToCore(
    send_Task,
    "Send Task",
    4096,
    NULL,
    1,
    &task2_handle,
    0);

  xTaskCreatePinnedToCore(
    button_Task,
    "Button Task",
    4096,
    NULL,
    1,
    &task3_handle,
    0);

  xTaskCreatePinnedToCore(
    move_Task,
    "Movement Task",
    4096,
    NULL,
    1,
    &task5_handle,
    1);

  delay(10000);
}

void loop()
{
  // Idle task
}


/* Task One*/
void door_Task(void *pvParameters)
{
  bool flag1 = 0;
  bool flag2 = 1;

  while (1)
  {
    if (digitalRead(DOOR) == LOW)
    {
      delay(500);
      if (flag1 == 0)
      {

        char msg[] = "Vault is Open";
        Serial.println(msg);
        char *pchar = msg;
        xQueueSend(send_queue, &pchar, portMAX_DELAY);

        doorFlag = 1;
        flag1 = 1;
        flag2 = 1;
      }
    }
    else if (digitalRead(DOOR) == HIGH)
    {
      delay(500);
      if (flag2 == 1)
      {
        char msg[] = "Vault is Closed";
        Serial.println(msg);
        char *pchar = msg;
        xQueueSend(send_queue, &pchar, portMAX_DELAY);

        doorFlag = 0;
        flag1 = 0;
        flag2 = 0;
      }
    }


  }
}


/* Task Two*/
void send_Task(void *pvParameters)
{
  bool flag1 = 0;
  bool flag2 = 1;
  char *Qdata;

  while (1)
  {
    if (xQueueReceive(send_queue, &Qdata, portMAX_DELAY) == pdPASS)
    {
      send_data = Qdata;
      Serial.println(send_data);
      Send_Data(pnumber, send_data);
    }
    delay(1000);
  }

  vTaskDelete( NULL );
}



/*Task Three*/
void button_Task(void *pvParameters)
{
  while (1)
  {
    if (digitalRead(BUTTON) == HIGH)
    {
      currentTime = xTaskGetTickCount();
      Serial.println("Press detected");

      if (currentTime - previousTime >= 1000)
      {
        delay(3000);
        while (digitalRead(BUTTON) == HIGH)
        {
          Serial.println("Long press detected");

          /* Suspending Other Tasks */
          vTaskSuspend(task1_handle);
          vTaskSuspend(task2_handle);
          vTaskSuspend(task5_handle);

          Serial.println("Other tasks suspended");
          delay(500);

          /* Task Four creation*/                                 //Creating New Task for receiving data
          xTaskCreatePinnedToCore(
            recv_Task,
            "Recv Task",
            4096,
            NULL,
            1,
            &task4_handle,
            0);

          Serial.println("New task created");
          previousTime = currentTime;
          break;
        }

      }
    }
    delay(1000);
  }
  vTaskDelete( NULL );
}

/* Task Four*/
void recv_Task(void* pvParameters)
{

  vTaskSuspend(task3_handle);                                     //Suspending task 3

  while (1)
  {
    delay(1000);
    while (Serial2.available() > 0)
    {
      message = Serial2.readString();                             //Reveiving data
      message.trim();
      Serial.println(message);

      if (message.indexOf("REGISTER*") > 0)                       //Registring phone number
      {
        phonenumber = message.substring(message.indexOf("\"") + 1, message.indexOf("\",\""));
        Serial.println(phonenumber);
        phonenumber.trim();
        Serial.println("message recieved");
        delay(500);

        if (SPIFF_dataItem != phonenumber)
        {
          writeFile(SPIFFS, "/data.txt", phonenumber.c_str());      //Saving Number in SPIFFS
          Serial.println("SPIFFS successfully updated");

          delay(5000);
          ESP.restart();
        }
        else
        {
          Serial.println("Same number");
          delay(5000);
          ESP.restart();
        }

      }
      else
      {
        Serial.println("No message");
        delay(5000);
        ESP.restart();
      }

    }

  }
  vTaskDelete( NULL );

}



/* Task Five */
void move_Task(void *pvParameter)
{

  while (1)
  {
    mpu.getEvent(&a, &g, &temp);
    Serial.print("Acceleration X: ");
    Serial.print((int)(a.acceleration.x));

    Serial.print(", Y: ");
    Serial.print((int)(a.acceleration.y));

    Serial.print(", Z: ");
    Serial.print((int)(a.acceleration.z));

    Serial.println(" m/s^2");
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");


    if (accX != (int)(a.acceleration.x) || accY != (int)(a.acceleration.y) || accZ != (int)(a.acceleration.z))
    {
      movement_flag = 1;
    }
    else
    {
      movement_flag = 0;
    }

    if (BlockFlag == 1)
    {
      if (movement_flag == 1)
      {
        char msg[] = "Movement Detected";
        Serial.println(msg);
        char *pchar = msg;
        xQueueSend(send_queue, &pchar, portMAX_DELAY);

        BlockFlag = 0;
        if (xTimerStart( xAutoReloadTimer, portMAX_DELAY) == pdPASS)
          Serial.println("Timer start");
      }
    }

    if (counter == 30)
    {
      BlockFlag = 1;
      counter = 0;
      if (xTimerStop( xAutoReloadTimer, portMAX_DELAY) == pdPASS)
        Serial.println("Timer stop");
    }

    Serial.println("");
    delay(1000);
  }
  vTaskDelete( NULL );
}

/***************** Timer Callback function ***************/

static void prvAutoReloadTimerCallback( TimerHandle_t xTimer )
{
  counter = counter + 1;
  Serial.println(counter);
}

/****************** prv functions *****************/

/* Serial Communication */
void Serialcom()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());
  }
}


/* Receive function */
void ReceiveMode()
{
  Serial.println("Recv mode");

  Serial2.println("AT");
  Serial2.println("AT+CMGF=1");
  Serial2.println("AT+CNMI=1,2,0,0,0");

  led(0, 0, 0);
  delay(1000);
  led(0, 0, 1);
}


/* Send function */
void Send_Data(char* number, char* message)
{
  Serial.println("Sending Data...");

  Serial2.print(F("AT+CMGF=1\r"));
  delay(100);

  Serial2.print(F("AT+CMGS=\""));
  Serial2.print(number);
  Serial2.print(F("\"\r"));
  delay(500);

  Serial2.print(message);
  Serial2.print("\r");
  delay(500);

  Serial2.print((char)26);
  delay(500);

  Serial2.println();

  Serial.println("Data Sent.");

  led(0, 0, 0);
  delay(100);
  led(0, 1, 0);
  delay(100);
  led(0, 0, 0);
  delay(100);
  led(0, 1, 0);
  delay(5000);
}


/* Led Function */

void led(bool R, bool G, bool B)
{
  digitalWrite(RLED, !R);
  digitalWrite(GLED, !G);
  digitalWrite(BLED, !B);
}
