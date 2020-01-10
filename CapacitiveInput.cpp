#include "RoboticsFramework.h"

#define TOUCH_THRESH_NO_USE   (0)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

// Touch Pad enumerated values. 
//TOUCH_PAD_NUM0 = 0,
//TOUCH_PAD_NUM1 = 1,    
//TOUCH_PAD_NUM2 = 2,    
//TOUCH_PAD_NUM3 = 3,    
//TOUCH_PAD_NUM4 = 4,    
//TOUCH_PAD_NUM5 = 5,    
//TOUCH_PAD_NUM6 = 6,    
//TOUCH_PAD_NUM7 = 7,    
//TOUCH_PAD_NUM8 = 8,    
//TOUCH_PAD_NUM9 = 9,    

// Not an actual touch pad, but if your running through a for loop use this. 
//TOUCH_PAD_MAX = 10

void read_capacitance_task(void *parameters){
	CapacitiveInput *obj  = (CapacitiveInput*)(parameters);
	uint16_t latest_touch_val = 0x0000; 
	for(;;){
		for(uint8_t i = 0; i < TOUCH_PAD_MAX; i++){
			uint8_t is_active = (1 << i) & obj->touch_pad_activated;
			if(is_active){
				touch_pad_read_filtered(touch_pad_t(i), &latest_touch_val); 
				obj->latest_touch_vals[i] = latest_touch_val; 
			}
		}
		// Delay every amount of system ticks. 
		vTaskDelay(obj->periodic_touch_delay);
	}
	
	vTaskDelete(NULL);
}

void CapacitiveInput::begin(void){
	
	// Setting up the touchpad subsystems. 
	touch_pad_init();
	
	// If use interrupt trigger mode, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
	touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

}

void CapacitiveInput::complete_setup(void){
	// Initialize and start a software filter to detect slight change of capacitance.
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
	
}

void CapacitiveInput::setup_touchpad(touch_pad_t touch_pad){
	touch_pad_config(touch_pad, TOUCH_THRESH_NO_USE);
	
	// Set the write bit in the bitmask so we know it's activated. 
	int touch_bitmask_shift = touch_pad_t(touch_pad);
	this->touch_pad_activated |= 1 << touch_bitmask_shift;	
}

void CapacitiveInput::calibrate_touchpads(void){
	uint16_t latest_touch_val = 0x0000; 
	for(uint8_t i = 0; i < TOUCH_PAD_MAX; i++){
		uint8_t is_active = (1 << i) & this->touch_pad_activated;
		if(is_active){
			touch_pad_read_filtered(touch_pad_t(i), &latest_touch_val); 
			ESP_ERROR_CHECK(touch_pad_set_thresh(touch_pad_t(i), latest_touch_val * 2 / 3));
		}
	}
}

void CapacitiveInput::set_periodic_read_task(void){
	xTaskCreate(read_capacitance_task,  "capacitance reader task", 10000, (void*)this, 40, NULL);  
}