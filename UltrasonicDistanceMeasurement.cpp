#include "RoboticsFramework.h"

#define ESP_INTR_FLAG_DEFAULT 0

void ultrasonic_isr_handler(void *parameters){
	ultra_data_t *data = (ultra_data_t*)parameters;
	data->end_ms = esp_timer_get_time();
	data->differential = data->end_ms - data->start_ms;
	data->currently_reading = false; 
}

void UltrasonicDistanceMeasurement::begin(uint64_t input_pin, uint64_t output_pin, uint32_t timeout_us){
	
	// Saving Values for future reference. 
	this->input_pin = input_pin; 
	this->output_pin = output_pin; 
	this->timeout_us = timeout_us; 
	
	this->setup_interrupts();
	this->setup_outputs();
}

void UltrasonicDistanceMeasurement::start_measurement(void){
	gpio_set_level(gpio_num_t(this->output_pin), 1);
	ets_delay_us(10);
	gpio_set_level(gpio_num_t(this->output_pin), 0);
	this->ultra_data.currently_reading = true; 
} 

double UltrasonicDistanceMeasurement::get_cm(void){
	double data = (double)this->ultra_data.differential *0.0343;
	return data; 
}

void UltrasonicDistanceMeasurement::install_isr_service(void){
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
}

void UltrasonicDistanceMeasurement::setup_esp_timer(void){
	esp_timer_init();
}

void UltrasonicDistanceMeasurement::setup_interrupts(void){
	// GPIO configuration struct that will allow us to setup 
	// output_pins
	gpio_config_t io_conf;
	io_conf.intr_type = gpio_int_type_t(GPIO_PIN_INTR_ANYEDGE);
	io_conf.pin_bit_mask = (1ULL << this->input_pin);
	io_conf.mode = GPIO_MODE_INPUT; 
	io_conf.pull_up_en = gpio_pullup_t(0); 
	io_conf.pull_down_en = gpio_pulldown_t(0);

	// Configures the GPIO based off our parameters. 
	gpio_config(&io_conf);
	// Setting up ISR Handler, passes off the ultra_data. 
	gpio_isr_handler_add(gpio_num_t(this->input_pin), ultrasonic_isr_handler,  (void*)(&this->ultra_data));

}

void UltrasonicDistanceMeasurement::setup_outputs(void){
	// GPIO configuration struct that will allow us to setup 
	// interrupts 
	gpio_config_t io_conf; 
	io_conf.intr_type = gpio_int_type_t(GPIO_PIN_INTR_DISABLE);
	io_conf.pin_bit_mask = (1ULL << this->output_pin);
	io_conf.mode = GPIO_MODE_OUTPUT; 
	io_conf.pull_up_en = gpio_pullup_t(0); 
	io_conf.pull_down_en = gpio_pulldown_t(0);

	// Configures the GPIO based off our parameters. 
	gpio_config(&io_conf);
}

