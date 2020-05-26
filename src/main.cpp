#include <Arduino.h>
#include <driver/gpio.h>
#include "driver/pcnt.h"


gpio_num_t aPinNumber = GPIO_NUM_33;
gpio_num_t bPinNumber = GPIO_NUM_32;

unsigned long durationA;
unsigned long durationB;

volatile int32_t 	count=0;
pcnt_unit_t 		unit = PCNT_UNIT_0;

pcnt_config_t r_enc_config;


int16_t getCountRaw() {
	int16_t c;
	pcnt_get_counter_value(unit, &c);
	return c;
}


void setup(){

	Serial.begin(115200);

	gpio_pad_select_gpio(aPinNumber);
	gpio_set_direction(aPinNumber, GPIO_MODE_INPUT);
	gpio_pulldown_en(aPinNumber);
	
	// Set up encoder PCNT configuration
	r_enc_config.pulse_gpio_num = aPinNumber; //Rotary Encoder Chan A
	r_enc_config.ctrl_gpio_num = bPinNumber;    //Rotary Encoder Chan B

	r_enc_config.unit = unit;
	r_enc_config.channel = PCNT_CHANNEL_0;

	r_enc_config.pos_mode = PCNT_COUNT_INC; //Count Only On Rising-Edges
	r_enc_config.neg_mode = PCNT_COUNT_DIS;   // Discard Falling-Edge

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




void loop() {

Serial.print(digitalRead(aPinNumber));Serial.print(" ");Serial.println(getCountRaw());

 delay(100);
}

