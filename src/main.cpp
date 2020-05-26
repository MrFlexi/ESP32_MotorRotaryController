#include <Arduino.h>
#include <driver/gpio.h>
#include "driver/pcnt.h"
#include <Servo.h>
#include <NTPClient.h> // Internet Time Server
#include <WiFiUdp.h>
#include <Ticker.h>
#include <WiFi.h>
#include <Wire.h>
#include "jsutilities.h"


//------------------------------------------------------
//               PINOUT
//        GPIO        Function
//        21          SDA
//        22          SCL
//        32          Lichtschranke   Richtung
//        33          Lichtschranke   Puls
//        34          Servo


#define mqtt_on	 0
#define USE_INA  1

#if (mqtt_on)
#include <PubSubClient.h>

PubSubClient client(espClient);
long lastMsgAlive = 0;
long lastMsgDist = 0;
#endif


//--------------------------------------------------------------------------
// ESP Sleep Mode
//--------------------------------------------------------------------------

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60       // sleep for 1 minute

//--------------------------------------------------------------------------
// Ticker
//--------------------------------------------------------------------------
Ticker tickerSaveUptime;

//--------------------------------------------------------------------------
// Sensors
//--------------------------------------------------------------------------
SDL_Arduino_INA3221 ina3221;    // I2C 


//--------------------------------------------------------------------------
// get time from internet
//--------------------------------------------------------------------------
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 7200); // 7200 = + 2h


//----------------------------------------------------------------------
// Servo
//----------------------------------------------------------------------
Servo servo1;
static const int servoPin =  GPIO_NUM_34;

//----------------------------------------------------------------------
// Lichtschranke Pins
//----------------------------------------------------------------------
gpio_num_t aPinNumber = GPIO_NUM_33;
gpio_num_t bPinNumber = GPIO_NUM_32;


//----------------------------------------------------------------------
// Puls Counter
//----------------------------------------------------------------------
unsigned long durationA;
unsigned long durationB;
volatile int32_t 	count=0;
pcnt_unit_t 		unit = PCNT_UNIT_0;
pcnt_config_t r_enc_config;


//--------------------------------------------------------------------------
// Wifi Setup + MQTT
//--------------------------------------------------------------------------
const char *ssid = "MrFlexi";
const char *password = "Linde-123";
//const char *mqtt_server = "192.168.1.144"; // Laptop
const char *mqtt_server = "test.mosquitto.org"; // Laptop
const char *mqtt_topic = "mrflexi/solarserver/";

WiFiClient espClient;

#if	(mqtt_on)
PubSubClient client(espClient);
long lastMsgAlive = 0;
long lastMsgDist = 0;
#endif
void setup_wifi()
{

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(String(WiFi.localIP()));
  timeClient.begin();
}

#if (mqtt_on)
void callback(char *mqtt_topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(mqtt_topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
 // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == 's')
  {
    //find_the_sun();
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(mqtt_topic, "connected");
      // ... and resubscribe
      client.subscribe("mrflexi/solarserver/#");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

#endif


void print_ina()
{
  Serial.println("");
  float shuntvoltage1 = 0;
  float busvoltage1 = 0;
  float current_mA1 = 0;
  float loadvoltage1 = 0;

  busvoltage1 = ina3221.getBusVoltage_V(1);
  shuntvoltage1 = ina3221.getShuntVoltage_mV(1);
  current_mA1 = -ina3221.getCurrent_mA(1); // minus is to get the "sense" right.   - means the battery is charging, + that it is discharging
  loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);

  Serial.print("Bus Voltage:");
  Serial.print(busvoltage1);
  Serial.println(" V");
  Serial.print("Shunt Voltage:");
  Serial.print(shuntvoltage1);
  Serial.println(" mV");
  Serial.print("Battery Load Voltage:");
  Serial.print(loadvoltage1);
  Serial.println(" V");
  Serial.print("Battery Current 1:");
  Serial.print(current_mA1);
  Serial.println(" mA");
  Serial.println("");
}


void setup_servo() {
    servo1.attach(	servoPin, Servo::CHANNEL_NOT_ATTACHED, 20,	180	);
}

void servo_sweep() {
    for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
        servo1.write(posDegrees);
        Serial.println(posDegrees);
        delay(50);
    }

	delay(1000);

	for(int posDegrees = 180; posDegrees > 0; posDegrees--) {
        servo1.write(posDegrees);
        Serial.println(posDegrees);
        delay(50);
    }
}


int16_t getCountRaw() {
	int16_t c;
	pcnt_get_counter_value(unit, &c);
	return c;
}


void setup_pulsecounter(){

	// Puls Counter Setup
	gpio_pad_select_gpio(aPinNumber);
	gpio_set_direction(aPinNumber, GPIO_MODE_INPUT);
	gpio_pulldown_en(aPinNumber);
	
	r_enc_config.pulse_gpio_num = aPinNumber; 	//Rotary Encoder Chan A 
	r_enc_config.ctrl_gpio_num = bPinNumber;    //Rotary Encoder Chan B

	r_enc_config.unit = unit;
	r_enc_config.channel = PCNT_CHANNEL_0;

	r_enc_config.pos_mode = PCNT_COUNT_INC; 	//Count Only On Rising-Edges
	r_enc_config.neg_mode = PCNT_COUNT_DIS;   	// Discard Falling-Edge

	r_enc_config.lctrl_mode = PCNT_MODE_KEEP;    // Rising A on HIGH B = CW Step
	r_enc_config.hctrl_mode = PCNT_MODE_REVERSE; // Rising A on LOW B = CCW Step

	r_enc_config		.counter_h_lim = INT16_MAX;
	r_enc_config		.counter_l_lim = INT16_MIN ;
	
	pcnt_unit_config(&r_enc_config);

	// Filter out bounces and noise
	pcnt_set_filter_value(unit, 250);  // Filter Runt Pulses
	pcnt_filter_enable(unit);
  
  /* Enable events on  maximum and minimum limit values */
	//pcnt_event_enable(unit, PCNT_EVT_H_LIM);
	//pcnt_event_enable(unit, PCNT_EVT_L_LIM);

	pcnt_counter_pause(unit); // Initial PCNT init
	pcnt_counter_clear(unit);
	pcnt_intr_enable(unit);
	pcnt_counter_resume(unit);
}

void setup()
{
	Serial.begin(115200);
  i2c_scan();

  #if (USE_INA)
  ina3221.begin();
  Serial.print("Manufact. ID=0x");
  int MID;
  MID = ina3221.getManufID();
  Serial.println(MID, HEX);
  print_ina();
#endif
  
  
	setup_servo();
	setup_pulsecounter();
  servo_sweep();
}


void loop() {

Serial.print(digitalRead(aPinNumber));Serial.print(" ");Serial.println(getCountRaw());

 delay(100);
}

