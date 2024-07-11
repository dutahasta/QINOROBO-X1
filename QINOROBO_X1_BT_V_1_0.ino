#define DEBUGGING_SERIAL      1

#include <FastLED.h>
#include <ESP32Servo.h>
#include <NimBLEDevice.h>

static NimBLEServer* pServer;

#define NUM_LEDS        4
#define DATA_PIN_LED   15  //10

Servo myservo;    //Define an instance of a servo
CRGB leds[NUM_LEDS];

 
/*******Servo pin************************************/
#define servo_pin     18   //9 v


/*******Define the pin of Line Tracking Sensor**********/
#define SensorLeft    35  //A0 v  //input pin of left sensor
#define SensorMiddle  34  //A1 v  //input pin of middle sensor
#define SensorRight   32  //A2 v  //input pin of right sensor

#define  LED          32  //A3 v

/*******Ultrasonic Sensor interface*****/
#define EchoPin       19  //13 v//ECHO to D13
#define TrigPin       5   //12 v//TRIG to D12

  /* [Motor 1] ---- [Motor 2]
        |               |
        |               |
        |               |
     [Motor 3] ---- [Motor 4]
*/

/****JM1Motor controls pins***///Right front motor
#define PIN_A_PWM1    25
#define PIN_A_M1      26
/****JM4Motor controls pins***///Right rear motor
#define PIN_D_PWM4    23
#define PIN_D_M4      12

/****JM2Motor controls pins***///Left front motor
#define PIN_B_PWM2    16
#define PIN_B_M2      17
/****JM3Motor controls pins***///left rear motor
#define PIN_C_PWM3    27
#define PIN_C_M3      14

/********默认速度补偿30***********/
uint8_t speed_motor_1 = 100;      //Right front motor speed
uint8_t speed_motor_4 = 100;      //Right rear motor speed

uint8_t speed_motor_2 = 130;      //Left front motor speed
uint8_t speed_motor_3 = 130;      //Left rear motor speed

String speed_motor_1_str, speed_motor_4_str, speed_motor_2_str, speed_motor_3_str; ///string type variable

float distance_M, distance_L, distance_R;
char  ble_val;
int   color_num = 0;
bool  new_incoming_command = false;
bool  got_specific_value = false;
String value_specific = "";

class ServerCallbacks: public NimBLEServerCallbacks 
{
    void onConnect(NimBLEServer* pServer) 
    {
        Serial.println("Client connected");
        //Serial.println("Multi-connect support: start advertising");
        //NimBLEDevice::startAdvertising();
    };
    /** Alternative onConnect() method to extract details of the connection. 
     *  See: src/ble_gap.h for the details of the ble_gap_conn_desc struct.
     */  
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc)
    {
        Serial.print("Client address: ");
        Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
        /** We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments, try for 5x interval time for best results.  
         */
        pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
    };
    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("Client disconnected - start advertising");
        NimBLEDevice::startAdvertising();
    };
};

class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue().c_str();

      #if(DEBUGGING_SERIAL == 1)
      Serial.print("cmd: ");
      for(uint8_t x = 0; x < rxValue.length();x++)
      { 
        Serial.print(rxValue[x]);
      } 
      Serial.println("");      
      #endif
      

      if((rxValue[0] >= 0x41) && (rxValue[0] <= 0x7A))  //value: A - z
      { 
        ble_val = (char)rxValue[0];
        got_specific_value = false;

        Serial.print("ble_val1: ");
        Serial.println(ble_val); 
        new_incoming_command = true;
      } 
      else if((rxValue[0] >= 0x30) && (rxValue[0] <= 0x39))
      {
        value_specific = "";
        for(uint8_t y = 0; y < rxValue.length(); y++)
        {
          if(rxValue[y] != '#')
          {
            value_specific = (String)value_specific + String(rxValue[y]);
          }
        }
        Serial.print("specs val: ");
        Serial.println(value_specific);
        got_specific_value = true;
        new_incoming_command = true;
      }
      else
      {
        
      }
  };
  
};

void ble_setup()
{
/** sets device name */
    NimBLEDevice::init("BT24");

    /** Optional: set the transmit power, default is 3db */
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
    
    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_DISPLAY_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // use passkey
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

    /** 2 different ways to set security - both calls achieve the same result.
     *  no bonding, no man in the middle protection, secure connections.
     *   
     *  These are the default values, only shown here for demonstration.   
     */ 
    //NimBLEDevice::setSecurityAuth(false, false, true); 
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pFFE0Service = pServer->createService("FFE0");
    NimBLECharacteristic* pE1Characteristic = pFFE0Service->createCharacteristic(
                                               "FFE1",
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE |
                                               NIMBLE_PROPERTY::NOTIFY
                                               );
  
    pE1Characteristic->setCallbacks(new CharacteristicCallbacks);

    /** 2904 descriptors are a special case, when createDescriptor is called with
     *  0x2904 a NimBLE2904 class is created with the correct properties and sizes.
     *  However we must cast the returned reference to the correct type as the method
     *  only returns a pointer to the base NimBLEDescriptor class.
     */
//    NimBLE2904* pBeef2904 = (NimBLE2904*)pBeefCharacteristic->createDescriptor("2904"); 
//    pBeef2904->setFormat(NimBLE2904::FORMAT_UTF8);
//    pBeef2904->setCallbacks(&dscCallbacks);
  
    NimBLECharacteristic* pE2Characteristic = pFFE0Service->createCharacteristic(
                                               "FFE2",
                                               NIMBLE_PROPERTY::WRITE
                                              );

    pE2Characteristic->setCallbacks(new CharacteristicCallbacks);


    /** Start the services when finished creating all Characteristics and Descriptors */  
    pFFE0Service->start();
    
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    /** Add the services to the advertisment data **/
    pAdvertising->addServiceUUID(pFFE0Service->getUUID());
    /** If your device is battery powered you may consider setting scan response
     *  to false as it will extend battery life at the expense of less data sent.
     */
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("Advertising Started");  
}

void setup() 
{
  Serial.begin(115200);//Start the serial monitor and set baud rate to 9600

  FastLED.addLeds<WS2812B, DATA_PIN_LED, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);  //Initial brightness(0~255)
  FastLED.show();   //Initialize all NeoPixels to the closed state
  
  /***set servo configuration */
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(servo_pin, 500, 2400);  // attaches the servo on pin 18 to the servo object
                                         // using SG90 servo min/max of 500us and 2400us
                                         // for MG995 large servo, use 1000us and 2000us,
                                         // which are the defaults, so this line could be
                                         // "myservo.attach(servoPin);"
  
  pinMode (LED, OUTPUT);//Configure 7-color LED pins mode for output
  
  /****All motor control pins are set to output mode***/
  pinMode (PIN_A_M1,    OUTPUT);
  pinMode (PIN_A_PWM1,  OUTPUT);
  pinMode (PIN_B_M2,    OUTPUT);
  pinMode (PIN_B_PWM2,  OUTPUT);
  pinMode (PIN_C_M3,    OUTPUT);
  pinMode (PIN_C_PWM3,  OUTPUT);
  pinMode (PIN_D_M4,    OUTPUT);
  pinMode (PIN_D_PWM4,  OUTPUT);

  /****All interfaces of the Line Tracking Sensor are set to input mode***/
  pinMode (SensorLeft,    INPUT);
  pinMode (SensorMiddle,  INPUT);
  pinMode (SensorRight,   INPUT);

  pinMode (EchoPin,       INPUT);    //The ECHO pin is set to input mode
  pinMode (TrigPin,       OUTPUT);   //The TRIG pin is set to output mode

  ble_setup();
}


/**************Main Loop***************/
void loop() {
  if (new_incoming_command)//If the receiving area is not empty
  {
    #if(DEBUGGING_SERIAL == 1)
      Serial.printf("ble_val2: %c \r\n", ble_val);//The serial prints the read data
    #endif
    
    switch (ble_val)
    {
      /***************Switch Mode******************/
      case 's': Stop();         break;  //Stop
      case 'a': Advance();      break;  //Go forward
      case 'c': Back();         break;  //Backward
      case 'b': Turn_Left();    break;  //Turn left
      case 'd': Turn_Right();   break;  //Turn right
      case 'k': L_Move();       break;  //Left shift
      case 'h': R_Move();       break;  //Right shift
      case 'l': LU_Move();      break;  //Upper left move
      case 'j': LD_Move();      break;  //lower left move
      case 'g': RU_Move();      break;  //Upper right move
      case 'i': RD_Move();      break;  //lower right move
      case 'e': drift_left();   break;  //Drift
      case 'f': drift_right();  break; //Drift

      case 'p': Tracking();     break;  //Tracking
      case 'q': Follow();       break;  //Follow
      case 'r': Avoidance();    break;  //Obstacle Avoidance


      /*********************The car changes speed*************************/
      case 't': police();                     break;  //Open the 7-color LED
      //case 'u': digitalWrite(LED, LOW);   break;  //Close the 7-color LED
      case 'm': /*color_num++;*/ showColor(); break;  //Switching the color of 2812 LED
      case 'o': closeColor  ();               break;  //Close the 2812 LED
      case 'n': /*color_num--;*/ showColor(); break;  //Switching the color of 2812 LED

      /*********************The car changes speed*************************/
      case 'v':    /*Read left front motor M2 speed*/
        if(got_specific_value)
        {
          speed_motor_2_str = value_specific;  //Failed to read data the first time
          speed_motor_2 = String(speed_motor_2_str).toInt();     //The speed value is a string and needs to be converted to an integer
          speed_motor_2 = map(speed_motor_2, 0, 100, 0, 255);  //Mapping from 0 to 100 to 0 to 255
  
          #if(DEBUGGING_SERIAL == 1)
            Serial.printf("spd2: %d \r\n", speed_motor_2);  //Serial debugging
          #endif
          
          delay(100); 
        }
        break;
      case 'w':   /*Read left rear M3 motor speed*/
        if(got_specific_value)
        {
          speed_motor_3_str = value_specific;
          speed_motor_3 = String(speed_motor_3_str).toInt();
          speed_motor_3 = map(speed_motor_3, 0, 100, 0, 255);
          
          #if(DEBUGGING_SERIAL == 1)
            Serial.printf("spd3: %d \r\n", speed_motor_3);  //Serial debugging
          #endif
          
          delay(100);
        }
        break;
      case 'x':   /*Read right front M1 motor speed*/
        if(got_specific_value)
        {
          speed_motor_1_str = value_specific;
          speed_motor_1 = String(speed_motor_1_str).toInt();
          speed_motor_1 = map(speed_motor_1, 0, 100, 0, 255);
          
          #if(DEBUGGING_SERIAL == 1)
            Serial.printf("spd1: %d \r\n", speed_motor_1);  //Serial debugging
          #endif
          
          delay(100);
        }
        break;
      case 'y':   /*Read right rear M4 motor speed*/
        if(got_specific_value)
        {
          speed_motor_4_str = value_specific;
          speed_motor_4 = String(speed_motor_4_str).toInt();
          speed_motor_4 = map(speed_motor_4, 0, 100, 0, 255);
          
          #if(DEBUGGING_SERIAL == 1)
            Serial.printf("spd4: %d \r\n", speed_motor_4);  //Serial debugging
          #endif
          
          delay(100);
        }
        break;

      default: break;
    }
    new_incoming_command = false;
  }
}


/*********************Obstacle avoidance*******************************/
void Avoidance(void)
{
  while (1)
  {
    distance_M = Get_Distance();   //Get the distance and save in the distance variable  
    if (distance_M < 20) 
    {//When the distance in front is less than 20cm  
      Stop();  //Robot stop
      delay(500); //Delay 500ms
      myservo.write(180);  //Ultrasonic cradle head turns left
      delay(500); //延时500ms
      distance_L = Get_Distance();  //Assign the left ultrasonic distance to variable a1
      delay(100); //Read values when stable
      myservo.write(0); //Ultrasonic cradle head turns right
      delay(500); //Delay 500ms
      distance_R = Get_Distance(); //Assign the right ultrasonic distance to variable a2
      delay(100);  //Read values when stable

      myservo.write(90);  //Return to the 90 degree position
      delay(500);
      
      if (distance_L > distance_R) 
      { //When the distance on the left is greater than the distance on the right
        Turn_Left();  //Robot turns left 
        delay(300);  //Turn left 700 ms
      } 
      else 
      {
        Turn_Right(); //Robot turns right
        delay(300);
      }
    }
    else { //If the distance in front is >=20cm, the robot will go ahead
      Advance(); //Go ahead
    }

    if (new_incoming_command)//If the receiving area is not empty
    {
      new_incoming_command = false;
      if (ble_val == 's') 
      {
        Stop();
        break;
      }
    }
  }
}

/*****************************Follow*******************************/
void Follow(void)
{
  while (1)
  {
    float distance = Get_Distance();  //Get the distance and save in the distance variable 
    if (distance >= 20 && distance <= 40) //Range of advance  
    {
      Advance();
    }
    else if (distance > 10 && distance < 20)  //Range of stop
    {
      Stop();
    }
    else if (distance <= 10)  //Range of fall back
    {
      Back();
    }
    else  //Other cases stop
    {
      Stop();
    }


    if (new_incoming_command)//If the receiving area is not empty
    {
      new_incoming_command = false;
      if (ble_val == 's') 
      {
        Stop();
        break;
      }
    }
  }
}

/****************************Line Tracking*******************************/
void Tracking(void) 
{   //Tracking black line
  while (1)
  {
    uint8_t SL = digitalRead  (SensorLeft);   //Read the value of the left Line Tracking Sensor
    uint8_t SM = digitalRead  (SensorMiddle); //Read the value of the intermediate Line Tracking Sensor
    uint8_t SR = digitalRead  (SensorRight);  //Read the value of the right Line Tracking Sensor
    if (SM == HIGH) 
    {
      if (SL == LOW && SR == HIGH) 
      {  // black on right, white on left, turn right
        Turn_Right();
      }
      else if (SR == LOW && SL == HIGH) 
      {  // black on left, white on right, turn left
        Turn_Left();
      }
      else 
      {  // white on both sides, going forward
        Advance();
      }
    }
    else {
      if (SL == LOW && SR == HIGH) 
      { // black on right, white on left, turn right
        Turn_Right();
      }
      else if (SR == LOW && SL == HIGH) 
      {  // white on right, black on left, turn left
        Turn_Left();
      }
      else 
      { // all white, stop
        Stop();
      }
    }


    if (new_incoming_command)//If the receiving area is not empty
    {
      new_incoming_command = false;
      if (ble_val == 's') 
      {
        Stop();
        break;
      }
    }
  }
}

/*********************Ultrasonic detects the distance *******************************/
float Get_Distance(void) 
{    //Altrasonic detects the distance 
  float dis;
  digitalWrite  (TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite  (TrigPin, HIGH); //Give the TRIG a high level at least 10 µ s to trigger
  delayMicroseconds(10);
  digitalWrite  (TrigPin, LOW);
  dis = pulseIn (EchoPin, HIGH) / 58.2; //Work out the distance
  delay(50);
  return dis;
}

/*********************RGB2812 display*******************************/
void showColor(void) 
{
  //  Serial.print("color num:"); //Serial debugging
  //  Serial.println(color_num);
  //  There are only 7 colors, you can add them yourself
  if (color_num > 3)color_num = 0;
  if (color_num < 0)color_num = 3;
  switch (color_num) {
    case  0:
      for (int i = 0; i < 4; i++) 
      {
        leds[i] = CRGB(255, 0, 0);  //The iLED is red
      }
      FastLED.show();  //Display
      break;
    case  1:
      for (int i = 0; i < 4; i++) 
      {
         leds[i] = CRGB(255, 80, 0); 
      }
      FastLED.show();  //Display
      break;
    case  2:
      for (int i = 0; i < 4; i++) 
      {
        leds[i] = CRGB(255, 255, 0); 
      }
      FastLED.show();  //Display
      break;
    case  3:
      for (int i = 0; i < 4; i++) 
      {
        leds[i] = CRGB(0, 255, 0); 
      }
      FastLED.show(); //Display
      break;
    case  4:
      for (int i = 0; i < 4; i++) 
      {
        leds[i] = CRGB(0, 0, 255); 
      }
      FastLED.show(); //Display
      break;
    case  5:
      for (int i = 0; i < 4; i++) 
      {
        leds[i] = CRGB(0, 255, 255); 
      }
      FastLED.show(); //Display
      break;
    case  6:
      for (int i = 0; i < 4; i++) 
      {
        leds[i] = CRGB(160, 32, 240); 
      }
      FastLED.show(); //Display
      break;
    default : break;
  }
}

/***********************Close RGB2812***************************/
void closeColor() 
{                  //Close aii 2818 LEDs  
  for (int i = 0; i < 4; i++) 
  {
    leds[i] = CRGB(0, 0, 0); 
    FastLED.show();//Execute display
  }
}

void police(void)
{
    for (int i = 0; i <= 4; i++) 
  {
    leds[i] = CRGB ( 0, 0, 255);
  }
    FastLED.show();
    delay(40);

    for (int i = 4; i >= 0; i--) 
    {
    leds[i] = CRGB ( 255, 0, 0);
    }
    FastLED.show();
    delay(40);
}

/**********The car advance***********/
void Advance(void) 
{
  digitalWrite (PIN_A_M1, LOW);
  analogWrite  (PIN_A_PWM1, speed_motor_1);
  digitalWrite (PIN_D_M4, LOW);
  analogWrite  (PIN_D_PWM4, speed_motor_4);

  digitalWrite (PIN_B_M2, LOW);
  analogWrite  (PIN_B_PWM2, speed_motor_2);
  digitalWrite (PIN_C_M3, LOW);
  analogWrite  (PIN_C_PWM3, speed_motor_3);
}

/**********The car back***********/
void Back(void) 
{
  digitalWrite (PIN_A_M1, HIGH);
  analogWrite  (PIN_A_PWM1, 255 - speed_motor_1);
  digitalWrite (PIN_D_M4, HIGH);
  analogWrite  (PIN_D_PWM4, 255 - speed_motor_4);

  digitalWrite (PIN_B_M2, HIGH);
  analogWrite  (PIN_B_PWM2, 255 - speed_motor_2);
  digitalWrite (PIN_C_M3, HIGH);
  analogWrite  (PIN_C_PWM3, 255 - speed_motor_3);
}

/**********The car rotates on the left***********/
void Turn_Left(void) 
{
  digitalWrite (PIN_A_M1, LOW);
  analogWrite  (PIN_A_PWM1, speed_motor_1);
  digitalWrite (PIN_D_M4, LOW);
  analogWrite  (PIN_D_PWM4, speed_motor_4);

  digitalWrite (PIN_B_M2, HIGH);
  analogWrite  (PIN_B_PWM2, 255 - speed_motor_2);
  digitalWrite (PIN_C_M3, HIGH);
  analogWrite  (PIN_C_PWM3, 255 - speed_motor_3);
}

/**********The car rotates on the right***********/
void Turn_Right(void) 
{
  digitalWrite (PIN_A_M1, HIGH);
  analogWrite  (PIN_A_PWM1, 255 - speed_motor_1);
  digitalWrite (PIN_D_M4, HIGH);
  analogWrite  (PIN_D_PWM4, 255 - speed_motor_4);

  digitalWrite (PIN_B_M2, LOW);
  analogWrite  (PIN_B_PWM2, speed_motor_2);
  digitalWrite (PIN_C_M3, LOW);
  analogWrite  (PIN_C_PWM3, speed_motor_3);
}

/**********The car stops***********/
void Stop(void) 
{
  digitalWrite (PIN_A_M1, LOW);
  analogWrite  (PIN_A_PWM1, 0);
  digitalWrite (PIN_B_M2, LOW);
  analogWrite  (PIN_B_PWM2, 0);
  digitalWrite (PIN_C_M3, LOW);
  analogWrite  (PIN_C_PWM3, 0);
  digitalWrite (PIN_D_M4, LOW);
  analogWrite  (PIN_D_PWM4, 0);
}

/**********The car moves to the left***********/
void L_Move(void) 
{
  digitalWrite (PIN_A_M1, LOW);
  analogWrite  (PIN_A_PWM1, speed_motor_1);
  digitalWrite (PIN_D_M4, HIGH);
  analogWrite  (PIN_D_PWM4, 255 - speed_motor_4);

  digitalWrite (PIN_B_M2, HIGH);
  analogWrite  (PIN_B_PWM2, 255 - speed_motor_2);
  digitalWrite (PIN_C_M3, LOW);
  analogWrite  (PIN_C_PWM3, speed_motor_3);
}


/**********The car moves to the right***********/
void R_Move(void) 
{
  digitalWrite (PIN_A_M1, HIGH);
  analogWrite  (PIN_A_PWM1, 255 - speed_motor_1);
  digitalWrite (PIN_D_M4, LOW);
  analogWrite  (PIN_D_PWM4, speed_motor_4);

  digitalWrite (PIN_B_M2, LOW);
  analogWrite  (PIN_B_PWM2, speed_motor_2);
  digitalWrite (PIN_C_M3, HIGH);
  analogWrite  (PIN_C_PWM3, 255 - speed_motor_3);
}

/*************The car moves to the front*************/
void LU_Move(void) 
{
  digitalWrite (PIN_A_M1, LOW);
  analogWrite  (PIN_A_PWM1, speed_motor_1);
  digitalWrite (PIN_D_M4, LOW);
  analogWrite  (PIN_D_PWM4, 0);

  digitalWrite (PIN_B_M2, LOW);
  analogWrite  (PIN_B_PWM2, 0);
  digitalWrite (PIN_C_M3, LOW);
  analogWrite  (PIN_C_PWM3, speed_motor_3);
}


/***************The car moves back****************/
void LD_Move(void) 
{
  digitalWrite (PIN_A_M1, LOW);
  analogWrite  (PIN_A_PWM1, 0);
  digitalWrite (PIN_D_M4, HIGH);
  analogWrite  (PIN_D_PWM4, 255 - speed_motor_4);

  digitalWrite (PIN_B_M2, HIGH);
  analogWrite  (PIN_B_PWM2, 255 - speed_motor_2);
  digitalWrite (PIN_C_M3, LOW);
  analogWrite  (PIN_C_PWM3, 0);
}


/***************The car moves to the right front**************/
void RU_Move(void) 
{
  digitalWrite (PIN_A_M1, LOW);
  analogWrite  (PIN_A_PWM1, 0);
  digitalWrite (PIN_D_M4, LOW);
  analogWrite  (PIN_D_PWM4, speed_motor_4);

  digitalWrite (PIN_B_M2, LOW);
  analogWrite  (PIN_B_PWM2, speed_motor_2);
  digitalWrite (PIN_C_M3, LOW);
  analogWrite  (PIN_C_PWM3, 0);
}


/***************The car moves to the right rear**************/
void RD_Move(void) 
{
  digitalWrite (PIN_A_M1, HIGH);
  analogWrite  (PIN_A_PWM1, 255 - speed_motor_1);
  digitalWrite (PIN_D_M4, LOW);
  analogWrite  (PIN_D_PWM4, 0);

  digitalWrite (PIN_B_M2, LOW);
  analogWrite  (PIN_B_PWM2, 0);
  digitalWrite (PIN_C_M3, HIGH);
  analogWrite  (PIN_C_PWM3, 255 - speed_motor_3);
}

/********************The car drifts left*********************/
void drift_left(void) 
{
  digitalWrite (PIN_A_M1, LOW);
  analogWrite  (PIN_A_PWM1, 0);
  digitalWrite (PIN_D_M4, LOW);
  analogWrite  (PIN_D_PWM4, speed_motor_4);

  digitalWrite (PIN_B_M2, LOW);
  analogWrite  (PIN_B_PWM2, 0);
  digitalWrite (PIN_C_M3, HIGH);
  analogWrite  (PIN_C_PWM3, 255 - speed_motor_3);
}


/********************The car drifts right*********************/
void drift_right(void) 
{
  digitalWrite (PIN_A_M1, LOW);
  analogWrite  (PIN_A_PWM1, 0);
  digitalWrite (PIN_D_M4, HIGH);
  analogWrite  (PIN_D_PWM4, 255 - speed_motor_4);

  digitalWrite (PIN_B_M2, LOW);
  analogWrite  (PIN_B_PWM2, 0);
  digitalWrite (PIN_C_M3, LOW);
  analogWrite  (PIN_C_PWM3, speed_motor_3);
}
