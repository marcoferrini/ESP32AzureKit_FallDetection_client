#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "xi2c.h"
#include "fonts.h"
#include "ssd1306.h"
#include "iot_button.h"

const static char *TAG1 = "FALL_DETECTION:";
extern QueueHandle_t data_queue;

//Button parameters
#define BUTTON_IO_NUM				0
#define BUTTON_ACTIVE_LEVEL         BUTTON_ACTIVE_LOW


//I2C parameters 
#define I2C_MASTER_SCL_IO           26
#define I2C_MASTER_SDA_IO           25
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000


static button_handle_t btn_handle = NULL; 

/////////////////////////////////////////////////
//I2C CONFIG FUCNCTIONS
//
///////////////////////////////////////////////////
static i2c_config_t i2c_conf;

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t* conf = &i2c_conf;

    conf->mode = I2C_MODE_MASTER;
	conf->sda_io_num = I2C_MASTER_SDA_IO;
	conf->scl_io_num = I2C_MASTER_SCL_IO;
	conf->sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf->scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf->master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(i2c_master_port, conf);

    return i2c_driver_install(i2c_master_port, conf->mode, 0, 0, 0);
}




/////////////////////////////////////////////////
//DISPLAY TASK
//
///////////////////////////////////////////////////


void display_task(){
	uint8_t fall_detected = 0;
	while (1)
	{
		if(!uxQueueSpacesAvailable(data_queue)){
			xQueuePeek(data_queue,&(fall_detected),0);
		} 
		if (fall_detected) 
		{
			SSD1306_GotoXY(8, 5);
			SSD1306_Puts("       ", &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(8, 5);
			SSD1306_Puts("fall detected", &Font_7x10, SSD1306_COLOR_WHITE);
		} else
		{
			SSD1306_GotoXY(8, 5);
			SSD1306_Puts("               ", &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(8, 5);
			SSD1306_Puts("NO fall", &Font_7x10, SSD1306_COLOR_WHITE);
		}
		SSD1306_UpdateScreen();
    	vTaskDelay(1000 / portTICK_PERIOD_MS);  
	}
	
	
	
	    
}



