#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "xi2c.h"
#include "fonts.h"
#include "ssd1306.h"
#include "iot_button.h"
#include "sensors.c"


//TASk
#define TASK_PRIORITY				4

#define REMOTE_SERVICE_UUID  	0x00FF
#define REMOTE_CHAR_UUID    	0xFF01
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define IGNORE_HANDLE   0

#define ATTR_VALUE_SIZE 1

const static char *CLIENT_TAG = "BLE CLIENT:";
QueueHandle_t data_queue;

static esp_gattc_descr_elem_t *descr_elem_result = NULL;

static void gattc_A_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
//static esp_gattc_char_elem_t *char_elem_result   = NULL;

static const char remote_device_name[] = "IOT_SERVER";
static bool connect    = false;
static bool get_server = false;

static esp_bt_uuid_t remote_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {										//Parametri per la scansione.
  .scan_type              = BLE_SCAN_TYPE_ACTIVE,										//Devono essere definiti globalmente per essere
  .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,										//accessibili durante tutto il processo di scan.
  .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
  .scan_interval          = 0x50,
  .scan_window            = 0x30
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_A_cb,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/**
 * @brief GATT client callback function for handling various BLE events.
 * 
 * This function handles registration events and routes other GATT client events
 * to their respective profile-specific callback functions based on the GATT interface provided.
 * 
 * @param event Type of GATT client event.
 * @param gattc_if GATT client interface, identifying the client instance.
 * @param param Pointer to parameter union holding event-specific data.
 */
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    for (int i = 0; i < PROFILE_NUM; i++) {
    	if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab[i].gattc_if) {	/* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
        	if (gl_profile_tab[i].gattc_cb) {
            	gl_profile_tab[i].gattc_cb(event, gattc_if, param);
          	}
      	}
    }
}

/**
 * @brief GAP callback function for handling GAP-related BLE events.
 * 
 * This function handles events related to BLE scanning, such as setting scan parameters,
 * starting scanning, and processing scan results.
 * 
 * @param event Type of GAP event.
 * @param param Pointer to parameter union holding event-specific data.
 */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){	//Funzione chiamata dal driver bluetooth ogni volta che si verifica un evento

	switch (event) {

		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:

			ESP_LOGI(CLIENT_TAG,"ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT\n");

			if(param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				ESP_ERROR_CHECK(esp_ble_gap_start_scanning(5));
			}
			else ESP_LOGI(CLIENT_TAG,"Unable to set scan parameters, error code %d\n\n", param->scan_param_cmpl.status);
		break;

		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:

			ESP_LOGI(CLIENT_TAG,"ESP_GAP_BLE_SCAN_START_COMPLETE_EVT\n");

			if(param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				ESP_LOGI(CLIENT_TAG,"Scan started\n\n");
			}
			else ESP_LOGI(CLIENT_TAG,"Unable to start scan process, error code %d\n\n", param->scan_start_cmpl.status);
		break;

		case ESP_GAP_BLE_SCAN_RESULT_EVT:

			//printf("ESP_GAP_BLE_SCAN_RESULT_EVT\n");

			if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {

				// try to read the complete name
				uint8_t *adv_name = NULL;
				uint8_t adv_name_len = 0;
				adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
				if (adv_name) {
					if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0 && connect == false) {
				    	connect = true;
				    	ESP_LOGI(CLIENT_TAG,"Starting Connection\n");
				    	esp_ble_gap_stop_scanning();
				    	esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, param->scan_rst.bda, param->scan_rst.ble_addr_type, true);
					}
				}
			}
			else if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT)  {
				ESP_LOGI(CLIENT_TAG,"ESP_GAP_SEARCH_INQ_CMPL_EVT\n");
				if (connect == false){
					vTaskDelay(10000 / portTICK_PERIOD_MS);
					ESP_ERROR_CHECK(esp_ble_gap_start_scanning(5));
				}
			}

		break;

		default:

			ESP_LOGW(CLIENT_TAG,"GAP Event %d unhandled\n\n", event);

		break;
	}

}

/**
 * @brief Profile A (fall detection) specific GATT client callback.
 * 
 * Handles specific events for Profile A (fall detection), such as registration, opening connections,
 * service discovery, characteristic operations, and notifications.
 * 
 * @param event GATT client event type.
 * @param gattc_if GATT client interface for the event.
 * @param param Pointer to parameter union for the event.
 */
static void gattc_A_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param){
	switch (event) {

		case ESP_GATTC_REG_EVT:

	    	ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&ble_scan_params));

	  	break;

	    case ESP_GATTC_OPEN_EVT:

	    	ESP_LOGI(CLIENT_TAG,"ESP_GATTC_OPEN_EVT\n");
	    		if (param->open.status == ESP_GATT_OK){
	    	    	ESP_LOGI(CLIENT_TAG,"ESP_GATT_OK\n");
	    	  	}

	 	break;

	    case ESP_GATTC_CONNECT_EVT:

	    	ESP_LOGI(CLIENT_TAG,"ESP_GATTC_CONNECT_EVT \n"); 
	        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
	        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

	 	break;

	    case ESP_GATTC_DIS_SRVC_CMPL_EVT:

	        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
	            ESP_LOGI(CLIENT_TAG,"ESP_GATTC_DIS_SRVC_CMPL_EVT failed\n");
	            break;
	        }
	        ESP_ERROR_CHECK(esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_service_uuid));

	 	break;

	    case ESP_GATTC_SEARCH_RES_EVT:

	    	ESP_LOGI(CLIENT_TAG,"ESP_GATTC_SEARCH_RES_EVT \n");

	            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && param->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
	                get_server = true;
	                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = param->search_res.start_handle;
	                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = param->search_res.end_handle;
	            }

		break;

	 	case ESP_GATTC_SEARCH_CMPL_EVT:{

	 		uint16_t count = 0;
	 		esp_gattc_char_elem_t *char_elem_result = NULL;

	 		ESP_LOGI(CLIENT_TAG,"ESP_GATTC_SEARCH_CMPL_EVT\n");
			if(get_server == true){
				ESP_ERROR_CHECK(esp_ble_gattc_get_attr_count( gattc_if, param->search_cmpl.conn_id, ESP_GATT_DB_CHARACTERISTIC,
															gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
															gl_profile_tab[PROFILE_A_APP_ID].service_end_handle, IGNORE_HANDLE, &count));

				if (count > 0){
					char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
					if (!char_elem_result){
                    	ESP_LOGE(CLIENT_TAG, "gattc no mem");
                    	break;
					}else{
						 esp_gatt_status_t status = esp_ble_gattc_get_char_by_uuid(gattc_if, param->search_cmpl.conn_id, gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
																		gl_profile_tab[PROFILE_A_APP_ID].service_end_handle, remote_char_uuid, char_elem_result,
																		&count);
						if (status != ESP_GATT_OK){
                        	ESP_LOGE(CLIENT_TAG, "esp_ble_gattc_get_char_by_uuid error");
                        	free(char_elem_result);
                        	char_elem_result = NULL;
                        	break;
                    	}
						gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;

						ESP_ERROR_CHECK(esp_ble_gattc_read_char(gattc_if, param->search_cmpl.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,ESP_GATT_AUTH_REQ_NONE));
						if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                       		ESP_ERROR_CHECK(esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle));
                    	}
					}
					free(char_elem_result);
				} else
				{
					ESP_LOGE(CLIENT_TAG,"No attr found");
				}
				
			}
			else ESP_LOGI(CLIENT_TAG,"Service Error \n");

		break;
	 	}

		case ESP_GATTC_REG_FOR_NOTIFY_EVT: 
			
			ESP_LOGI(CLIENT_TAG , "ESP_GATTC_REG_FOR_NOTIFY_EVT");
			if (param->reg_for_notify.status != ESP_GATT_OK){
            	ESP_LOGE(CLIENT_TAG, "REG FOR NOTIFY failed: error status = %d", param->reg_for_notify.status);
			}else{
				uint16_t count = 0;
            	uint16_t notify_en = 1;
            	esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         	gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         	ESP_GATT_DB_DESCRIPTOR,
                                                                        	gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         	gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                         	gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                         	&count);
				if (ret_status != ESP_GATT_OK){
                	ESP_LOGE(CLIENT_TAG, "esp_ble_gattc_get_attr_count error");
                	break;
            	}
				if (count > 0){
                	descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
               		if (!descr_elem_result){
                    	ESP_LOGE(CLIENT_TAG, "malloc error, gattc no mem");
                   		break;
                	}else{
                    	ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         param->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    	if (ret_status != ESP_GATT_OK){
                        	ESP_LOGE(CLIENT_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                        	free(descr_elem_result);
                        	descr_elem_result = NULL;
                        	break;
                    	}
                    	/* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    	if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        	ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     	gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     	descr_elem_result[0].handle,
                                                                     	sizeof(notify_en),
                                                                     	(uint8_t *)&notify_en,
                                                                    	ESP_GATT_WRITE_TYPE_RSP,
                                                                     	ESP_GATT_AUTH_REQ_NONE);
                    	}

                   		if (ret_status != ESP_GATT_OK){
                        	ESP_LOGE(CLIENT_TAG, "esp_ble_gattc_write_char_descr error");
                    	}

                    	/* free descr_elem_result */
                    	free(descr_elem_result);
                	}
            	}
            	else{
                	ESP_LOGE(CLIENT_TAG, "decsr not found");
            	}


			}
			
		break;
		case ESP_GATTC_NOTIFY_EVT:
			if (param->notify.is_notify){
        	    ESP_LOGI(CLIENT_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        	}else{
        	    ESP_LOGI(CLIENT_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        	}
        	esp_log_buffer_hex(CLIENT_TAG, param->notify.value, param->notify.value_len);
			ESP_LOGI(CLIENT_TAG, "notify value = %02x, notify lenght = %u", param->notify.value[0], param->notify.value_len);
			uint8_t tmp;
			if(!uxQueueSpacesAvailable(data_queue)){
				xQueuePeek(data_queue,&(tmp),0);
				printf("%02x",tmp);
				xQueueReceive(data_queue,&tmp, 0);
				xQueueSend(data_queue, (param->notify.value), 0);
			
			}else{
				xQueueSend(data_queue, (param->notify.value), 0);
			}
        break;

		case ESP_GATTC_WRITE_DESCR_EVT:
        	if (param->write.status != ESP_GATT_OK){
        	    ESP_LOGE(CLIENT_TAG, "write descr failed, error status = %x", param->write.status);
        	    break;
        	}
        	ESP_LOGI(CLIENT_TAG, "write descr success ");
        break;


	 	case ESP_GATTC_READ_CHAR_EVT:{
			uint8_t tmp;
	 		if (param->read.status != ESP_GATT_OK){
	 			ESP_LOGI(CLIENT_TAG,"Char read Error\n");
	 			break;
	 		}

	 		ESP_LOGI(CLIENT_TAG,"data = ");
	 		for (int i = param->read.value_len -1; i >= 0; i--){
	 			ESP_LOGI(CLIENT_TAG,"%02x",param->read.value[i]);
	 		}
	 		ESP_LOGI(CLIENT_TAG,"\n\n");

	 	break;
	 	}

		case ESP_GATTC_WRITE_CHAR_EVT:
        	if (param->write.status != ESP_GATT_OK){
        	    ESP_LOGE(CLIENT_TAG, "write char failed, error status = %x", param->write.status);
        	    break;
        	}
        	ESP_LOGI(CLIENT_TAG, "write char success ");
        	break;


	 	case ESP_GATTC_DISCONNECT_EVT:

	 		connect = false;
	 		get_server = false;

	 		ESP_LOGI(CLIENT_TAG,"Server Disconnected \n");

	 		ESP_ERROR_CHECK(esp_ble_gap_start_scanning(5));

		break;

	 	default:
	 		ESP_LOGE(CLIENT_TAG,"GATT Event %d unhandled\n\n", event);
	 	break;
	}
}

/////////////////////////////////////////////////
//BUTTON SETUP 
//
///////////////////////////////////////////////////

/**
 * @brief Callback function for a button press.
 *
 * This function is called when a user-defined button is pressed. It checks if a connection is established
 * and writes a characteristic to reset the fall_detection.
 */
void btn_cb() {
	uint8_t fall_detected= 0; 
	uint8_t tmp; 

	ESP_LOGI(CLIENT_TAG,"User defines button was pushed");
	if(connect) {
    	ESP_ERROR_CHECK(esp_ble_gattc_write_char(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id,gl_profile_tab[PROFILE_A_APP_ID].char_handle,ATTR_VALUE_SIZE,&fall_detected, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE));	
		ESP_LOGI(CLIENT_TAG, "fall_detected = %d", fall_detected);
	} else {
		ESP_LOGE(CLIENT_TAG, "No connection");
	}	
	
	
	return; 		
}

/**
 * @brief Initializes the button used in the application.
 *
 * This function sets up a button on a specified GPIO pin using the iot_button.h library. 
 * It logs the start and completion of the initialization process, and sets a callback function
 * to handle button tap events.
 */
void button_init(){
	ESP_LOGI(CLIENT_TAG, "STARTED: button initialization"); 
	btn_handle = iot_button_create((gpio_num_t)BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
	//set event callback
	ESP_ERROR_CHECK(iot_button_set_evt_cb(btn_handle, BUTTON_CB_TAP, btn_cb, NULL));
    ESP_LOGI(CLIENT_TAG, "COMPLETED: Button initialization");
	return;
}


/////////////////////////////////////////

/**
 * @brief Main application function to setup sensors, display, Bluetooth Low Energy (BLE) and tasks.
 *
 * This function is responsible for initializing the hardware and software components
 * used in the application. It sets up a data queue, button, I2C for sensor communication,
 * an OLED display, and the Bluetooth Low Energy (BLE) subsystem. It also creates the display task.
 */
void app_main(void)
{

	/////////////////////////////////////////////
	//SENSORS SETUP
	//
	/////////////////////////////////////////////
	data_queue = xQueueCreate(1, sizeof(uint8_t)); 
	ESP_LOGI(CLIENT_TAG, "data queue created");
	button_init();
	ESP_LOGI(CLIENT_TAG, "botton configured");

	//i2c master setup
	ESP_ERROR_CHECK(i2c_master_init());
	
	//Display setup 
    SSD1306_Init();
	vTaskDelay(1000/portTICK_PERIOD_MS);
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	/////////////////////////////////////////////
	//BLE SETUP
	//
	/////////////////////////////////////////////

	ESP_ERROR_CHECK(nvs_flash_init());											//Partizione NVS nella memoria flash necessaria al driver del BLE
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	//Inizializzazione controller Bluetooth (HCI, LL and PHY)
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
	ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

	//Inizializzazione Stack Software (Creazione thread gestore del bluetooth)
	ESP_ERROR_CHECK(esp_bluedroid_init());
	ESP_ERROR_CHECK(esp_bluedroid_enable());

	ESP_LOGI(CLIENT_TAG,"Stack Initialized\n\n");

	//Configurazione della funzione di callback del driver
	ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
	ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_gattc_cb));
	ESP_ERROR_CHECK(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,ESP_PWR_LVL_P9));
	ESP_ERROR_CHECK(esp_ble_gattc_app_register(PROFILE_A_APP_ID));

	/////////////////////////////////////////////
	//TASK CREATION
	//
	/////////////////////////////////////////////

	xTaskCreate(&display_task, "display_task", 1024*10, NULL, TASK_PRIORITY, NULL);
	ESP_LOGI(CLIENT_TAG, "SCHEDULER: starting");

}
