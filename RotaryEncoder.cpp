#include "RoboticsFramework.h"

void rotary_encoder_task(void *parameters){
	RotaryEncoder *obj = (RotaryEncoder*)(parameters);
	obj->run_task();
	vTaskDelete(NULL);
}

void  IRAM_ATTR rotary_encoder_interrupt_task(void *parameters){
	RotaryEncoder *obj = (RotaryEncoder*)(parameters);
	obj->pulse_count++; 
}

void RotaryEncoder::run_task(void){
	if(this->gpio_semaphore == NULL){
		this->gpio_semaphore  = xSemaphoreCreateBinary();
	}
	
	for(;;){
		xSemaphoreTake(this->gpio_semaphore, portMAX_DELAY );
		// Compute rpm based off latest data. 
		this->latest_rpm = (this->pulse_count/this->update_rate) * 1000 * 60; 
		// Reset the pulse count. 
		this->pulse_count = 0; 
		xSemaphoreGive(this->gpio_semaphore);
		vTaskDelay(periodic_update_delay);
	}
}

void RotaryEncoder::begin(uint64_t pin_mask, uint16_t steps_per_rotation, uint16_t update_rate){
	this->input_gpio.begin(pin_mask, 0, 0);	
	xTaskCreate(rotary_encoder_task,  "rotary encoder reading task", 10000, (void*)this, 40, NULL);  
	
	// Generating the Semaphore so we can prevent other devices using the GPIO/reading the gpio value. 
	if(this->gpio_semaphore == NULL){
		this->gpio_semaphore  = xSemaphoreCreateBinary();
	}
	this->periodic_update_delay =  update_rate / portTICK_PERIOD_MS;
}

void AverageRPMSpeed::add_rotary_encoder(RotaryEncoder *rotary){
	this->rotary_list.push_back(rotary);
}