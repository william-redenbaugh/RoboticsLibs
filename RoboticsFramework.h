#ifndef _ROBOTICSFRAMEWORK_H
#define _ROBOTICSFRAMEWORK_H

//#include <iostream>
//#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector> 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "esp_log.h"
#include "driver/touch_pad.h"

// Nice cute little struct that makes dealing with Ultrasonic sensor
// interrupts a little easier. 

typedef struct{
	volatile uint64_t differential; 
	volatile uint64_t start_ms; 
	volatile uint64_t end_ms; 
	volatile bool currently_reading; 
}ultra_data_t;

class UltrasonicDistanceMeasurement{
	public: 
		void begin(uint64_t input_pin, uint64_t output_pin, uint32_t timeout_us);
		
		// If we haven't installed the ISR service already, we do it here. 
		void install_isr_service(void);
		void setup_esp_timer(void);
		void start_measurement(void);
		
		// get's data in centimeters back. 
		double get_cm(void);
		
	private: 
		void setup_interrupts(void);
		void setup_outputs(void);
		
		uint64_t input_pin; 
		uint64_t output_pin; 
		uint64_t timeout_us;
		
		ultra_data_t ultra_data;  
};

class DiscreteInput{
	public: 
		// Set's up GPIO pin
		void begin(uint64_t pin_mask, uint8_t pullup, uint8_t pulldown); 
		
		// Attachs an interrupt to the gpio system 
		void attach_interrupt(gpio_isr_t isr_handler, gpio_int_type_t interrupt_type,  void *args);
		
		// Just in case you haven't done it already. 
		void install_isr_service(void);
		
		// Read's discrete value. 
		uint8_t read(void);
		
		// Gets the latest discrete value!
		uint8_t get(void);
	
	private:
		volatile uint8_t latest_val; 
		uint64_t pin_mask; 
		uint8_t pullup; 
		uint8_t pulldown; 
};

class DiscreteInputPeriodicTask{
	public: 
		void setup(uint32_t stack_space, char *task_name, uint16_t time_interval_ms);
		void add_discrete_input(DiscreteInput *input);
		
		// Allows us to get and release the gpio semaphore. 
		void wait_gpio_semaphore(void);
		void release_gpio_semaphore(void);
		
		TaskHandle_t gpio_read_task_handler; 
		std::vector<DiscreteInput*> discrete_inputs; 
		uint16_t time_interval_ms = 0x0000;
		SemaphoreHandle_t gpio_semaphore = NULL;
		
};

class DiscreteOutput{
	public: 
		// sets up a gpio for output. 
		void setup(uint64_t pin_mask);
		
		// sets a value
		void set(void);
		
		// clears a value. 
		void clear(void);	
		
	private: 
		uint64_t pin_mask; 
		uint8_t pin_state; 
};

class CapacitiveInput{
	public: 
	
		// Touchpad setup.  Initializes everything we need to do. 
		void begin(void); 
		
		// completes setup, must be called after setting up all the touchpads. 
		void complete_setup(void);
		
		// Setting up each touchpad. 
		void setup_touchpad(touch_pad_t touch_pad);
		
		// Calibrating the touchpads, should be the last thing you call after completing setup. 
		// Will calibrate all previously configered touchpads. 
		void calibrate_touchpads(void);
		
		// Allows us to setup a periodic reading task so we can get the sensor data whenever. 
		void set_periodic_read_task(void);
		
		// Let's us know if the touch pad is already active through the bitmask. 
		uint16_t touch_pad_activated = 0x0000;
		uint16_t latest_touch_vals[16];
		// How much time we are delaying the forloop. 
		TickType_t periodic_touch_delay = 1000 / portTICK_PERIOD_MS;
		
	private: 
};

class RotaryEncoder{
	public: 
		void begin(uint64_t pin_mask, uint16_t steps_per_rotation, uint16_t update_rate);
		
		// Just allows the task to be run off a method inside the thread. 
		void run_task(void);
		
		// How often is the data updated, will also affect how other calculations are complete. 
		uint16_t update_rate; 
		TickType_t periodic_update_delay = 1000 / portTICK_PERIOD_MS;
		
		// How many pulses have passed
		volatile uint32_t pulse_count; 
		
		// What is the latest RPM measurement based off last update. 
		volatile uint32_t latest_rpm; 
		DiscreteInput input_gpio; 
		
		// Semaphore for us to toy with :)
		SemaphoreHandle_t gpio_semaphore = NULL;
	private: 
	
};

class AverageRPMSpeed{
	public:
	
		// Allows us to have a list of rotary encoders to read through. 
		void add_rotary_encoder(RotaryEncoder *rotary);
		
		// US CUSTOMARY BEGIN// 
		
		// Starts the conversion system, so we can convert RPM to real distance. Unit in miles. 
		void set_conversion_miles(uint32_t rotations_per_mile); 
		
		// Starts the conversion system, so we can convert RPM to real distance. Unit in miles. 
		void set_conversion_feet(uint32_t rotations_per_mile); 
		
		// Starts the conversion system, so we can convert RPM to real distance. Unit in miles. 
		void set_conversion_inches(uint32_t rotations_per_mile); 
	
		// US CUSTOMARY END //
		
		// THE REST OF THE WORLD'S METRICS BEGIN //

		// Starts the conversion system, so we can convert RPM to real distance. Unit in centimeters
		void set_conversion_cm(uint32_t rotations_per_mile); 
		
		// Starts the conversion system, so we can convert RPM to real distance. Unit in decimeters
		void set_conversion_dm(uint32_t rotations_per_mile); 
		
		// Starts the conversion system, so we can convert RPM to real distance. Unit in decimeters
		void set_conversion_m(uint32_t rotations_per_mile); 
		
		// Starts the conversion system, so we can convert RPM to real distance. Unit in kilometers
		void set_conversion_km(uint32_t rotations_per_mile); 
		
		// THE REST OF THE WORLD'S METRICS END //	
		
		// Reading the rotary lists. 
		std::vector<RotaryEncoder*>rotary_list; 
};



#endif