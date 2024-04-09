#include "nvs_flash.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_light.h"
#include "string.h"
#include "driver/gpio.h"

#include <stdio.h>
#include "driver/ledc.h"

static const char *TAG = "CCT";
#define H_LED_IO        (3) 
#define C_LED_IO        (5) 
#define H_CHANNEL       LEDC_CHANNEL_0
#define C_CHANNEL       LEDC_CHANNEL_1
#define PWM_MODE        LEDC_LOW_SPEED_MODE

const uint16_t MAX_DUTY = 8190;

const uint16_t MIN_TEMP = 200;
const uint16_t MAX_TEMP = 400;
const uint16_t MID_TEMP = MIN_TEMP + (MAX_TEMP - MIN_TEMP)/2;

static uint16_t COLOR_TEMP = MID_TEMP;
static uint8_t PWR = 50;
static bool ISON = true;

void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0001,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

void SaveToNVS(){
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    err = nvs_set_i32(my_handle, "saved_color", (int32_t) COLOR_TEMP);
    switch (err) {
        case ESP_OK: ESP_LOGI("SAVE", "Saved color temperature value: %i", (int)COLOR_TEMP); break;       
        default :    ESP_LOGW("SAVE", "Saving error!!!");
    }

    int32_t pwr_toSave = (PWR >= 100) ? pwr_toSave = (int32_t)PWR : 100;    
    err = nvs_set_i32(my_handle, "saved_pwr", pwr_toSave);
    switch (err) {
        case ESP_OK: ESP_LOGI("SAVE", "Saved pwr value: %i", (int)pwr_toSave); break;       
        default :    ESP_LOGW("SAVE", "Saving error!!!");
    }
}


double GetPWR_Pcnt(){
    if(PWR >= 254) return 100.0;
    return (PWR * 100.0) / 255.0;
}

uint16_t GetPWM_Pcnt(bool hot_pwm){
    uint16_t pwm = 100;

    int16_t delta = COLOR_TEMP - MID_TEMP;

    if(delta > 100){ delta = 100; }
    if(delta <-100){ delta =-100; }

    if(delta >= 0){
        if(hot_pwm){ return pwm; }
        return 100 - delta;
    }
    
    if(!hot_pwm){ return pwm; }
    return 100 + delta;

}


void PWM_task(void *pvParameters)
{
    uint16_t color_temp = 0;
    uint8_t pwr = 50;
    uint16_t cycles_without_changes = 0;
    bool saved = false;
    while (1)
    {        
        if(ISON){
            if((pwr != PWR) || (color_temp != COLOR_TEMP)){
                cycles_without_changes = 0;
                saved = false;

                uint16_t pwm1_pcnt = GetPWM_Pcnt(false);
                uint16_t pwm2_pcnt = GetPWM_Pcnt(true);
                uint16_t pwr_pcnt = GetPWR_Pcnt();

                double pwm1_mult = (pow(5, (double)pwm1_pcnt/35) - 1)/100.0;
                double pwm2_mult = (pow(5, (double)pwm2_pcnt/35) - 1)/100.0;
                double pwr_mult = pwr_pcnt/100.0;

                uint32_t duty1 = MAX_DUTY * pwm1_mult * pwr_mult;
                uint32_t duty2 = MAX_DUTY * pwm2_mult * pwr_mult;
               
                ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, H_CHANNEL, duty1));
                ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, C_CHANNEL, duty2));                
                ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, H_CHANNEL));
                ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, C_CHANNEL));

                //ESP_LOGI("PWM", "pwm1(%i) m(%f), pwm2(%i) m(%f), pwr(%i) m(%f)", (int)pwm1_pcnt, pwm1_mult, (int)pwm2_pcnt, pwm2_mult, (int)pwr_pcnt, pwr_mult); 
                //ESP_LOGI("PWM", "d1(%i), d2(%i)", (int)duty1, (int)duty2);
                pwr = PWR;
                color_temp = COLOR_TEMP; 
                ESP_LOGI("PWM", "pwm1(%i) m(%f), pwm2(%i) m(%f), pwr(%i) m(%f), d1(%i), d2(%i)", (int)pwm1_pcnt, pwm1_mult, (int)pwm2_pcnt, pwm2_mult, (int)pwr_pcnt, pwr_mult, (int)duty1, (int)duty2); 
            }
            else{ cycles_without_changes++;  }

            if((cycles_without_changes >= 50) && (!saved)){ SaveToNVS(); saved = true; }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);    
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(0x%x), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);


    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT)
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
            {
                ISON = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;              
                ESP_LOGI(TAG, "Light sets to %s", ISON ? "On" : "Off");
            }
        }
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
            {
                COLOR_TEMP = *(uint16_t *)message->attribute.data.value;
               // SetColor(light_color);
              //  ESP_LOGI(TAG, "Color sets to %i", (int)COLOR_TEMP);
            }
        }

        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8)
            {
                PWR = *(uint8_t *)message->attribute.data.value;
               // SetColor(light_color);
                ESP_LOGI(TAG, "PWR sets to %i", (int)PWR);
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            //xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
            
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}
/* initialize Zigbee stack with Zigbee end-device config */

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // ------------------------------ Cluster BASIC ------------------------------
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion = 0x0002;
    uint32_t HWVersion = 0x0002;
    uint8_t ManufacturerName[] = {8, 'D', 't', 'k', 'a', 'H', 'o', 't', 'e'}; // warning: this is in format {length, 'string'} :
    uint8_t ModelIdentifier[] = {5, 'A', 'L', 'a', 'm', 'p'};
    uint8_t DateCode[] = {8, '2', '0', '2', '4', '0', '8', '2', '6'};
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, DateCode);

    // ------------------------------ Cluster IDENTIFY ------------------------------
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

    // ------------------------------ Cluster LIGHT ------------------------------
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = 0,
    };
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    // ===============================Color Cluster for test==============================
    // It works with HA!!!
    // Use esp_zb_color_control_cluster_add_attr for this
    //esp_zb_color_control_cluster_create
    esp_zb_color_cluster_cfg_t esp_zb_color_cluster_cfg = { 
        .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value x */
        .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value y */ 
        .color_mode = 0x0002,                                                               /*!<  The mode which attribute determines the color of the device */ 
        .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,                          /*!<  The bitmap determines behavior of some cluster commands */ 
        .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE,  /*!<  The enhanced-mode which attribute determines the color of the device */ 
        .color_capabilities = 0x0010,                                                       /*!<  Specifying the color capabilities of the device support the color control cluster */ 
    };
    esp_zb_attribute_list_t *esp_zb_color_cluster = esp_zb_color_control_cluster_create(&esp_zb_color_cluster_cfg);
    
    uint16_t color_attr = MID_TEMP;
    uint16_t min_temp = MIN_TEMP;//ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_DEFAULT_VALUE;
    uint16_t max_temp = MAX_TEMP;//ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_DEFAULT_VALUE;
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &color_attr);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID, &min_temp);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID, &max_temp);
    
    // ============================= Level Cluster for test =============================
    esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);
    uint8_t level = 50;
    esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &level);
    

    // ------------------------------ Cluster Temperature ------------------------------
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value = -50,
        .max_value = 100,
    };
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);


    // ------------------------------ Create cluster list ------------------------------
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    
    esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
   // esp_zb_cluster_list_update_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_cluster_list_add_color_control_cluster(esp_zb_cluster_list, esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_update_color_control_cluster(esp_zb_cluster_list, esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);


    // ------------------------------ Create endpoint list ------------------------------
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    //Create struct for esp_zb_ep_list_add_ep function from newest library
    //Warning! BitFields?!
    esp_zb_endpoint_config_t zb_endpoint_config = {
        .endpoint =  HA_ESP_LIGHT_ENDPOINT,                       /*!< Endpoint */
        .app_profile_id =  ESP_ZB_AF_HA_PROFILE_ID,               /*!< Application profile identifier */
        .app_device_id =  ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,       /*!< Application device identifier */
        .app_device_version = 4,                                  /*!< Application device version */
    };
    //For old library
    //esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, HA_ESP_LIGHT_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID);

    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, zb_endpoint_config);

    // ------------------------------ Register Device ------------------------------
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

void ConfigGPIOs(){
    gpio_reset_pin(C_LED_IO);
    gpio_reset_pin(H_LED_IO);
    gpio_set_direction(C_LED_IO, GPIO_MODE_OUTPUT);
    gpio_set_direction(H_LED_IO, GPIO_MODE_OUTPUT);
    
    gpio_set_level(C_LED_IO, 0);
    gpio_set_level(H_LED_IO, 0);
}

void ConfigPWM(){
    ConfigGPIOs();

    ledc_timer_t timer = LEDC_TIMER_3;
    ledc_timer_bit_t timer_bits = LEDC_TIMER_13_BIT;

     ledc_timer_config_t ledc_timer = {
        .speed_mode       = PWM_MODE,
        .timer_num        = timer,
        .duty_resolution  = timer_bits,
        .freq_hz          = 4000,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t H_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = H_CHANNEL,
        .timer_sel      = timer,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = H_LED_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };

    ledc_channel_config_t C_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = C_CHANNEL,
        .timer_sel      = timer,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = C_LED_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&H_channel));
    ESP_ERROR_CHECK(ledc_channel_config(&C_channel));
}


void AntiZB_task(void *pvParameters){
    int delta = 1;
    int counter = 0;
    while (true)
    {
        if(counter < 11){
            COLOR_TEMP += delta;

            if((COLOR_TEMP >= 400) || (COLOR_TEMP <= 200)){ delta = delta * (-1); }
            counter++;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void LoadVals(){
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    
    int32_t saved_color = MID_TEMP; 
    err = nvs_get_i32(my_handle, "saved_color", &saved_color);

    switch (err) {
        case ESP_OK:                    ESP_LOGI("LOAD", "Found color temperature value: %i", (int)saved_color); break;
        case ESP_ERR_NVS_NOT_FOUND:     ESP_LOGW("LOAD", "Color temperature value is not found. Default color temperature value (%i) used", (int)MID_TEMP); break;
        default :                       ESP_LOGW("LOAD", "Reading error!!! Default color temperature value (%i) used", (int)MID_TEMP);
    }

    COLOR_TEMP = saved_color;

    int32_t saved_pwr = PWR; 
    err = nvs_get_i32(my_handle, "saved_pwr", &saved_pwr);

    switch (err) {
        case ESP_OK:                    ESP_LOGI("LOAD", "Found pwr value: %i", (int)saved_pwr); break;
        case ESP_ERR_NVS_NOT_FOUND:     ESP_LOGW("LOAD", "Pwr value is not found. Default pwr value (%i) used", (int)PWR); break;
        default :                       ESP_LOGW("LOAD", "Reading error!!! Default pwr  value (%i) used", (int)PWR);
    }

    PWR = saved_pwr;
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    LoadVals();
    ConfigPWM();

    
    xTaskCreate(PWM_task, "PWM_task", 4096, NULL, 5, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    xTaskCreate(AntiZB_task, "AntiZB_task", 4096, NULL, 5, NULL);

}