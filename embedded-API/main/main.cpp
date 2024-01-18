//std includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

//esp-idf includes
#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/i2c.h>
#include <driver/gptimer.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_log.h>
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_eth.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "cJSON.h"


//freeRTOS includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

//extra includes
#include <tp_includes/UniversalLCD/universallcd.hpp>


//i2c defines
#define LCD_ADDR 0x27
#define I2C_MASTER I2C_NUM_0
#define TIMER_HZ (1000*1000)

//wifi defines
#define WIFI_CONNECTED_BIT BIT0
#define WiFi_CONNECTING_BIT BIT1
#define WIFI_FAIL_BIT BIT2

struct LcdDrive : public universalLCD::BusInterface{
    void send(uint8_t config, uint8_t data){
        uint8_t package = (config & 0b00001111) | (data & 0xF0);
        i2c_master_write_to_device(I2C_MASTER, LCD_ADDR,&package,1,pdMS_TO_TICKS(100));
    }
};

//LCD functions
static bool timer_delay_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
void delay(uint32_t us);
void i2c_init(void);
void lcd_timer_init(void);

//WiFi functions
void wifi_init_sta();
static void wifi_event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data);

//http server functions
void init_server();
static esp_err_t lcd_request_hander(httpd_req_t *req);
static esp_err_t not_found(httpd_req_t *req, httpd_err_code_t err);

//freeRTOS tasks
void lcd_server(void *args);
void log_memory(void *args);

//http handlers
httpd_handle_t server = NULL;
httpd_config_t server_config;

static const httpd_uri_t lcd_req_post = {
    .uri = "/lcd/*",
    .method = HTTP_POST,
    .handler = lcd_request_hander,
    .user_ctx = NULL,
};

static const httpd_uri_t lcd_req_get = {
    .uri = "/lcd/*",
    .method = HTTP_GET,
    .handler = lcd_request_hander,
    .user_ctx = NULL,
};

//wifi handlers
EventGroupHandle_t wifi_event_group_h;
esp_event_handler_instance_t wifi_id_event;
esp_event_handler_instance_t wifi_ip_event;
wifi_config_t wifi_sta_conf;
const char ip_placeholder[] = "xxx.xxx.xxx.xxx";
char sta_ip[16]{0};
char ssid[] = "SSID";
char password[] = "PASSWORD";


//lcd drive handler
TaskHandle_t lcd_server_task;
gptimer_handle_t lcd_driver_delay;
gptimer_config_t timer_config;
gptimer_alarm_config_t delay_config;
gptimer_event_callbacks_t delay_callback;
volatile bool timer_flag;



// network queues
QueueHandle_t request_msg;

extern "C" void app_main(void){
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_event_group_h = xEventGroupCreate();
    request_msg = xQueueCreate(10,sizeof(char*));
    wifi_init_sta();
    i2c_init();
    lcd_timer_init();
    xTaskCreatePinnedToCore(lcd_server,"lcd serve",2040,NULL,2,NULL,1);
    xTaskCreatePinnedToCore(log_memory,"log mem",2040,NULL,1,NULL,1);
    while(1)vTaskDelay(portMAX_DELAY);

}

static bool timer_delay_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx){
    timer_flag = true;
    return true;
}

void delay(uint32_t us){
    uint64_t count;
    gptimer_stop(lcd_driver_delay);
    gptimer_get_raw_count(lcd_driver_delay, &count);
    delay_config = {
        .alarm_count = (us+count),
        .reload_count = 0,
        .flags = 0,
    };
    timer_flag = false;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(lcd_driver_delay, &delay_config));
    gptimer_start(lcd_driver_delay);
    while(!timer_flag);
}

void i2c_init(){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = 100000,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER, conf.mode, 0, 0, 0));
}

void lcd_timer_init(void){
    timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_HZ,
        .intr_priority = 0,
        .flags = 0,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &lcd_driver_delay));
    delay_callback = {
        .on_alarm = timer_delay_callback,
    };
    delay_config = {
        .alarm_count = 0,
        .reload_count = 0,
        .flags = 0,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(lcd_driver_delay,&delay_callback,NULL));
    ESP_ERROR_CHECK(gptimer_enable(lcd_driver_delay));
    ESP_ERROR_CHECK(gptimer_start(lcd_driver_delay));
}

void wifi_init_sta(){
    ESP_LOGI("WIFI", "wifi start config");

    //start network stack config
    ESP_ERROR_CHECK(esp_netif_init());

    //start event loop for the WiFi task
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //create WiFi memory
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wconfig =  WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wconfig));
    ESP_LOGI("WIFI", "wifi memory completed");

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,&wifi_event_handler,NULL,&wifi_id_event));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,&wifi_event_handler,NULL,&wifi_id_event));
    ESP_LOGI("WIFI", "wifi events set");

    //SET sta config
    memcpy(wifi_sta_conf.sta.ssid,ssid,sizeof(ssid));
    memcpy(wifi_sta_conf.sta.password,password,sizeof(password));
    wifi_sta_conf.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_sta_conf.sta.sae_pwe_h2e = WPA3_SAE_PWE_UNSPECIFIED;

    //start Wifi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_conf));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("WIFI", "wifi start");
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data){

    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){

        esp_wifi_connect();
        xEventGroupSetBits(wifi_event_group_h,WiFi_CONNECTING_BIT);

    }else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){

        if(server != NULL){
            ESP_ERROR_CHECK(httpd_stop(server));
            server = NULL;
        }
        xEventGroupSetBits(wifi_event_group_h,WIFI_FAIL_BIT);
        vTaskDelay(1000);
        esp_wifi_connect();

    }else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){

        if(server == NULL)init_server();
        ip_event_got_ip_t *ip_result = (ip_event_got_ip_t*)event_data;
        snprintf(sta_ip, sizeof(sta_ip), "%03d.%03d.%03d.%03d", IP2STR(&ip_result->ip_info.ip));
        xEventGroupSetBits(wifi_event_group_h, WIFI_CONNECTED_BIT);
    }
}

void init_server(){
    server_config = HTTPD_DEFAULT_CONFIG();
    server_config.lru_purge_enable = true;
    server_config.uri_match_fn = httpd_uri_match_wildcard;
    ESP_ERROR_CHECK(httpd_start(&server, &server_config));
    httpd_register_uri_handler(server,&lcd_req_get);
    httpd_register_uri_handler(server,&lcd_req_post);
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND,not_found);
}

static esp_err_t lcd_request_hander(httpd_req_t *req){
    char json_buf[512] = {'\0'};
    size_t json_size = 0;
    char header_buf[128] = {'\0'};
    cJSON *req_json = NULL;
    cJSON *msg_item = NULL;
    char *uri_msg = NULL;

    if(httpd_req_get_hdr_value_str(req, "Content-Type", header_buf, sizeof(header_buf)-1) != ESP_OK){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid content-Type");
        return ESP_OK;
    }
    if(strcmp(header_buf, "application/json") != 0){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Type is not application/json");
        return ESP_OK;
    }
    json_size = httpd_req_recv(req, json_buf, sizeof(json_buf));
    if(json_size < 4){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty json");
        return ESP_OK;
    }
    req_json = cJSON_ParseWithLength(json_buf,json_size);
    if(!req_json){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "json parse error");
        return ESP_OK;
    }

    msg_item = cJSON_GetObjectItem(req_json, "message");
    if(!msg_item){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json item");
        return ESP_OK;
    }
    uri_msg = cJSON_Print(msg_item);
    if(xQueueSend(request_msg, &uri_msg, 0) == pdFAIL){
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "internal Queue fail");
    }else{
        httpd_resp_send(req, "OK", 3);
    }
    cJSON_Delete(req_json);
    return ESP_OK;
}

static esp_err_t not_found(httpd_req_t *req, httpd_err_code_t err){
    return ESP_OK;
}

void lcd_server(void *args){
    LcdDrive drive;
    universalLCD::UniversalLCD lcd(drive,universalLCD::Bus4Bits,delay);
    EventBits_t netwotk_flags;
    char status_msg[12];
    char ip_msg[16]{0}; //copy of ip string (sta_ip should to be read only to avoid using a lock)
    char *msg;

    lcd.begin()
        .selectLcd(0)
        .setCursor(0,0)
        .write("status: ")
        .setCursor(0, 20)
        .write("SSID: ")
        .setCursor(1,0)
        .write("IP: ")
        .selectLcd(1)
        .setCursor(0,0)
        .write("==========waiting for requests==========");
    while(1){
        netwotk_flags = xEventGroupWaitBits(wifi_event_group_h,
                                            WIFI_CONNECTED_BIT|WiFi_CONNECTING_BIT|WIFI_FAIL_BIT,
                                            pdTRUE,
                                            pdFALSE,
                                            0);

        if (netwotk_flags){
            lcd.selectLcd(0).setCursor(0,8);
            memset(status_msg, (int)' ', sizeof(status_msg));
            lcd.write_limit(status_msg, sizeof(status_msg));

            if (netwotk_flags & WiFi_CONNECTING_BIT){
                strcpy(status_msg, "connecting");
                strcpy(ip_msg, ip_placeholder);
            }
            else if (netwotk_flags & WIFI_CONNECTED_BIT){
                strcpy(status_msg, "connected");
                strcpy(ip_msg, sta_ip);
            }
            else if (netwotk_flags & WIFI_FAIL_BIT){
                strcpy(status_msg, "OFFLINE");
                strcpy(ip_msg, ip_placeholder);
            }
            lcd.setCursor(0, 8)
                .write(status_msg)
                .setCursor(0, 26)
                .write(ssid)
                .setCursor(1, 4)
                .write(ip_msg);
        }
        if(xQueueReceive(request_msg, &msg, 0) == pdPASS){
            lcd.selectLcd(1)
                .setCursor(0,0)
                .clear()
                .write(msg);
                vPortFree(msg);
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void log_memory(void *args){
    size_t free_mem = 0;
    while(1){
        free_mem  = xPortGetFreeHeapSize();
        ESP_LOGI("MEM LOG", "%d", free_mem);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
