#include "RoboticsFramework.h"

#define ESP_INTR_FLAG_DEFAULT 0

void DiscreteInput::begin(uint64_t pin_mask, uint8_t pullup, uint8_t pulldown){
	
	// Setting values. 
	this->pin_mask = pin_mask; 
	this->pullup = pullup; 
	this->pulldown = pulldown; 
	
	// GPIO configuration struct that will allow us to setup 
	// output_pins
	gpio_config_t io_conf;
	io_conf.intr_type = gpio_int_type_t(GPIO_PIN_INTR_ANYEDGE);
	io_conf.pin_bit_mask = (1ULL << this->pin_mask);
	io_conf.mode = GPIO_MODE_INPUT; 
	io_conf.pull_up_en = gpio_pullup_t(pullup); 
	io_conf.pull_down_en = gpio_pulldown_t(pulldown);
	
	// Configures the GPIO based off our parameters. 
	gpio_config(&io_conf);
	
}

void DiscreteInput::install_isr_service(void){
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
}

uint8_t DiscreteInput::read(void){
	this->latest_val = gpio_get_level(gpio_num_t(this->pin_mask));
	return this->latest_val;
}

uint8_t DiscreteInput::get(void){
	uint8_t val = this->latest_val; 
	latest_val = 0; 
	return val;
}

void DiscreteInput::attach_interrupt(gpio_isr_t isr_handler, gpio_int_type_t interrupt_type, void *args){
	// GPIO configuration struct that will allow us to setup 
	// output_pins
	gpio_config_t io_conf;
	io_conf.intr_type = interrupt_type;
	io_conf.pin_bit_mask = (1ULL << this->pin_mask);
	io_conf.mode = GPIO_MODE_INPUT; 
	io_conf.pull_up_en = gpio_pullup_t(pullup); 
	io_conf.pull_down_en = gpio_pulldown_t(pulldown);
	
	// Configures the GPIO based off our parameters. 
	gpio_config(&io_conf);
	
	gpio_isr_handler_add(gpio_num_t(this->pin_mask), isr_handler,  args);
}

void read_gpio_task(void *parameters){
	DiscreteInputPeriodicTask *obj = (DiscreteInputPeriodicTask *)(parameters);
	
	for(;;){
		xSemaphoreTake(obj->gpio_semaphore, portMAX_DELAY );
		for(uint16_t i = 0; i < obj->discrete_inputs.size(); i++){
			obj->discrete_inputs[i]->read();
		}
		xSemaphoreGive(obj->gpio_semaphore);
		vTaskDelay(obj->time_interval_ms/portTICK_RATE_MS);
	}
	
	vTaskDelete(NULL);
}

void DiscreteInputPeriodicTask::setup(uint32_t stack_space, char *task_name, uint16_t time_interval_ms){
	// Reading a task that passes over self object, and a task handler so we can read and manipulate the task 
	// separatly. 
	this->time_interval_ms = time_interval_ms; 
	
	// Generating the Semaphore so we can prevent other devices using the GPIO/reading the gpio value. 
	if(this->gpio_semaphore == NULL){
		this->gpio_semaphore  = xSemaphoreCreateBinary();
	}
	
	// Creates the period gpio read task. 
	xTaskCreate(read_gpio_task, task_name, stack_space, (void*)this, 10,  &this->gpio_read_task_handler);
}

void DiscreteInputPeriodicTask::add_discrete_input(DiscreteInput *input){
	// Wait until we can add more gpio readers. 
	if(this->gpio_semaphore == NULL){
		this->gpio_semaphore  = xSemaphoreCreateBinary();
	}
	xSemaphoreTake(this->gpio_semaphore, portMAX_DELAY);
	this->discrete_inputs.push_back(input);
	xSemaphoreGive(this->gpio_semaphore);
}

void DiscreteInputPeriodicTask::wait_gpio_semaphore(void){
	xSemaphoreTake(this->gpio_semaphore, portMAX_DELAY);
}

void DiscreteInputPeriodicTask::release_gpio_semaphore(void){
	xSemaphoreGive(this->gpio_semaphore);
}