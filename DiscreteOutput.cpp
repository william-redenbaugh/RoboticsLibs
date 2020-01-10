#include "RoboticsFramework.h"

void DiscreteOutput::setup(uint64_t pin_mask){
	// GPIO pin mask 
	this->pin_mask = pin_mask;
	
	// IO configuration profile. 
	gpio_config_t io_conf;
    
	//disable interrupt
    io_conf.intr_type = gpio_int_type_t(GPIO_PIN_INTR_DISABLE);
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL<< pin_mask));
    //disable pull-down mode
    io_conf.pull_down_en = gpio_pulldown_t(0);
    //disable pull-up mode
    io_conf.pull_up_en = gpio_pullup_t(0);
    //configure GPIO with the given settings
	
	// Installs gpio settings. 
	gpio_config(&io_conf);
	gpio_set_level(gpio_num_t(pin_mask), 0);	
}

void DiscreteOutput::set(void){
	gpio_set_level(gpio_num_t(this->pin_mask), 1);
	this->pin_state = 1; 
}

void DiscreteOutput::clear(void){
	gpio_set_level(gpio_num_t(this->pin_mask), 0);	
	this->pin_state = 0; 
}