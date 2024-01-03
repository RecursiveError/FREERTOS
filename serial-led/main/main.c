#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/queue.h>
#include <esp_log.h>


typedef struct {
    QueueHandle_t buffer1;
    QueueHandle_t buffer2;
}buf_t;

typedef struct{
    uint8_t *msg;
    size_t msg_size;
} msg_t;


const size_t buffer_max_size = 500;
const size_t uart_buff_size = 1024;
const msg_t response_ok = {
    .msg = (uint8_t*)"OK!\n",
    .msg_size = 5,
};
const msg_t response_err = {
    .msg = (uint8_t*)"ERROR!\n",
    .msg_size = 8,
};





int strint(const uint8_t *str, uint32_t *target){
    uint32_t c = 0;
    for(size_t i = 0; str[i] != '\n'; i++){
        if(str[i] < '0' || str[i] > '9'){
            return -1;
        }
        c = (c*10) + (str[i] - '0');
    }
    *target = c;
    return 0;
}

void blink_led(void *params){
    uint32_t blink_speed = 1000;
    buf_t *buff = (buf_t*)params;
    msg_t msg = {0};

    uint32_t led_std = 0;
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    while(1){
        if(xQueueReceive(buff->buffer1,&msg,0) == pdPASS){
            if(strint(msg.msg, &blink_speed) == 0){
                xQueueSend(buff->buffer2, &response_ok, 0);
            }else{
                xQueueSend(buff->buffer2, &response_err, 0);
            }
            vPortFree(msg.msg);
        }
        gpio_set_level(GPIO_NUM_2, led_std);
        led_std = !led_std;
        vTaskDelay(blink_speed/portTICK_PERIOD_MS);
    }
}

void read_uart(void *params){
    buf_t *buff = (buf_t*)params;
    size_t len = 0;
    size_t idx = 0;
    uint8_t *msg = NULL;
    msg_t msg_buf = {0};
    uint8_t data[100] = {0};
    uint8_t cinput = 0;
    msg_t rev = {0};

    while(1){
        if(xQueueReceive(buff->buffer2, &rev, 0) == pdTRUE){
            uart_write_bytes(UART_NUM_2, rev.msg, rev.msg_size);
        }

        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, &len));
        if(len > 0){
            uart_read_bytes(UART_NUM_2,&cinput, 1, 100);
            uart_write_bytes(UART_NUM_2, &cinput, 1);
            data[idx] = cinput;
            idx++;
            idx = idx % 100;
            if(cinput == '\n' && idx > 0){
                msg = pvPortMalloc(sizeof(uint8_t)*idx);
                if(msg == NULL){
                    ESP_LOGE("MALLOC", "msg alloc ERR");
                    while(1);
                }
                memcpy(msg, data, idx);
                msg_buf.msg = msg;
                msg_buf.msg_size = idx;
                if(xQueueSend(buff->buffer1,&msg_buf,0) == pdFAIL){
                    vPortFree(msg);
                    ESP_LOGI("QUEUE", "queue full");
                }
                idx = 0;
                cinput = 0;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void){
    QueueHandle_t uart_buf;
    buf_t *buffers = pvPortMalloc(sizeof(buf_t));
    uart_config_t uart = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buff_size, uart_buff_size, 10, &uart_buf, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPIO_NUM_17,GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    if(buffers == NULL){
        ESP_LOGE("MALLOC", "malloc err");
        while(1);
    }

    buffers->buffer1 = xQueueCreate(buffer_max_size, sizeof(msg_t));
    buffers->buffer2 = xQueueCreate(buffer_max_size, sizeof(msg_t));

    xTaskCreatePinnedToCore(blink_led, "led blink", 1024, buffers, 1, NULL, 1);
    xTaskCreatePinnedToCore(read_uart, "read uart", 2048, buffers, 3, NULL, 1);
}
