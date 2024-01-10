/*
------------------------------------------------
================ LCD SERIAL ===================
------------------------------------------------
Author - Guilherme Silva Schultz (RecursiveError)
data - 2024-01-10


freeRTOS + esp-idf learning project to control an alphanumeric lcd display using UART

the project consists of 3 tasks:

"read_uart" reads the information from the serial to \n or \r.
returns whether the operation was a success or not, sends the information through a queue to "terminal_lcd"

"terminal_lcd" receives a message from the read_serial queue, and checks if it is a valid command for the lcd
(a valid command consists of a value inside < > followed by an argument inside ( )
both are mandatory, sends the result to read_uart and sends the command to "lcd_server"

"lcd_server" uses the external library "UniversalLCD" to control the LCD using an I2C interface
UniversalLCD requires a delay of microseconds to work
which cannot be done with freeRTOS tiks, since the default is 1Tick/1Khz
the delay It is done with a gptimer from esp-idf to generate a delay (which does not block the task)

"lcd_server" receives a command and executes the task (this example only has 7 commands:
<write>(args), <clear>(), <setline>(line col), <cpp>(), <curso>(state), <display>(state) and <moved>(dir times)
*<cpp> show cpp logo
*/

//std includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

//freeRTOS includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

//esp-idf includes
#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/i2c.h>
#include <driver/gptimer.h>
#include <esp_log.h>
#include "esp_task_wdt.h"

//extra includes
#include <UniversalLCD/universallcd.hpp>
#include <cmdutil/cmduitl.hpp>

#define LCD_ADDR 0x27
#define I2C_MASTER I2C_NUM_0

#define TIMER_HZ (1000*1000)


struct LcdDrive : public universalLCD::BusInterface{
    void send(uint8_t config, uint8_t data){
        uint8_t package = (config & 0b00000111) | (data & 0xF0) | 0x08;
        i2c_master_write_to_device(I2C_MASTER, LCD_ADDR,&package,1,pdMS_TO_TICKS(100));
    }
};

//lcd driver delay (This code runs at a frequency of 1tick/1Khz, so it is not necessary to block the task for a microsecond delay)
static bool timer_delay_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
void delay(uint16_t us);



//init functions
void uart_init(void);
void i2c_init(void);
void lcd_timer_init(void);

//tasks
void terminal_lcd(void *params);
void read_uart(void *params);
void lcd_server(void *params);

//cpp logo

uint8_t cpp_logo[6][8] = {
    { 0b00000, 0b00000, 0b00000, 0b00011, 0b00111, 0b01100, 0b11000, 0b11000 },
    { 0b00000, 0b00000, 0b00000, 0b11100, 0b00111, 0b00011, 0b00001, 0b00000 },
    { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 },
    { 0b11000, 0b11000, 0b11000, 0b01100, 0b00111, 0b00011, 0b00000, 0b00000 },
    { 0b01000, 0b11100, 0b01001, 0b00011, 0b00111, 0b11100, 0b00000, 0b00000 },
    { 0b01000, 0b11100, 0b01000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
};

//buffer handlers
QueueHandle_t uart_msg;
QueueHandle_t lcd_server_buf;
QueueHandle_t uart_buf;
QueueHandle_t response_buf;
const size_t buffer_max_size = 500;
const size_t uart_buff_size = 1024;

//lcd drive handler
TaskHandle_t lcd_server_task;
gptimer_handle_t lcd_driver_delay;
gptimer_config_t timer_config;
gptimer_alarm_config_t delay_config;
gptimer_event_callbacks_t delay_callback;
volatile bool timer_flag;


extern "C" void app_main(void){

    uart_msg = xQueueCreate(buffer_max_size, sizeof(msg_t));
    if(!uart_msg){
        ESP_LOGE("QUEUE", "uart queue alloc error");
        while(1);
    }

    lcd_server_buf = xQueueCreate(buffer_max_size, sizeof(TokenMsg_t));
    if(!lcd_server_buf){
        ESP_LOGE("QUEUE", "lcd queue alloc error");
        while(1);
    }

    response_buf = xQueueCreate(buffer_max_size, sizeof(msg_t));
    if(!response_buf){
        ESP_LOGE("QUEUE", "response queue alloc error");
        while(1);
    }
    uart_init();
    i2c_init();
    lcd_timer_init();

    xTaskCreatePinnedToCore(terminal_lcd, "terminal", 1024, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(read_uart, "read uart", 4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(lcd_server, "lcd server", 4096,nullptr, 1, &lcd_server_task,1);

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


void uart_init(void){
    uart_config_t uart = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buff_size, uart_buff_size, 10, &uart_buf, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPIO_NUM_17,GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

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

void terminal_lcd(void *params){
    CmdToken_t token;
    TokenResult result;
    msg_t msg;
    TokenMsg_t cmd;
    msg_t args;
    size_t msg_size = 0;

    while(1){
        if(xQueueReceive(uart_msg,&msg,0) == pdPASS){
            result = token_raw(msg, &token);
            if(result == TokenResult::Pass){
                msg_size = (token.arg_end - token.arg_begin)+1;
                args.msg = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*msg_size);
                args.msg_size = msg_size;
                if(args.msg == NULL){
                    ESP_LOGE("MALLOC", "MALLOC ERROR");
                    while(1);
                }

                cmd.hash = create_cmd(token, msg);
                strncpy((char*)args.msg, (char*)msg.msg + token.arg_begin, msg_size);
                cmd.args = args;

                if(xQueueSend(lcd_server_buf, &cmd, 0) != pdPASS) vPortFree(args.msg);
                xQueueSend(response_buf,&response_ok,0);

            }else{
                xQueueSend(response_buf,&response_err,0);
            }
            vPortFree(msg.msg);
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void read_uart(void *params){
    size_t len = 0;
    size_t idx = 0;
    uint8_t *msg = nullptr;
    msg_t msg_buf;
    msg_t response;
    uint8_t data[100] = {0};
    uint8_t cinput = 0;

    while(1){
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, &len));
        if(xQueueReceive(response_buf,&response, 0) == pdPASS){
            uart_write_bytes(UART_NUM_2, response.msg, response.msg_size);
        }
        if(len > 0){
            uart_read_bytes(UART_NUM_2,&cinput, 1, 100);
            uart_write_bytes(UART_NUM_2, &cinput, 1);
            data[idx] = cinput;
            idx++;
            idx = idx % 100;
            if((cinput == '\n' || cinput == '\r') && idx > 0){
                msg = (uint8_t*)pvPortMalloc(sizeof(uint8_t)*idx);
                if(msg == NULL){
                    ESP_LOGE("MALLOC", "msg alloc ERR");
                    while(1);
                }
                memcpy(msg, data, idx);
                msg_buf.msg = msg;
                msg_buf.msg_size = idx;
                if(xQueueSend(uart_msg,&msg_buf,0) == pdFAIL){
                    vPortFree(msg);
                    ESP_LOGI("QUEUE", "queue full");
                }
                idx = 0;
                cinput = 0;
            }else if(cinput == 127){
                data[--idx] = 0;
                data[--idx] = 0;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void lcd_server(void *params){
    LcdDrive drive;
    universalLCD::UniversalLCD lcd(drive,universalLCD::Bus4Bits,delay);
    TokenMsg_t cmd;
    uint8_t cursor_std = 1;
    uint8_t display_std = 1;
    uint64_t lcd_posi_yx[2] = {0};
    size_t lcd_posi_yx_size = sizeof(lcd_posi_yx);
    uint64_t lcd_shift_args[2] = {0};
    size_t lcd_shift_args_size = sizeof(lcd_shift_args);


    //commands for the lcd, you can use a hashmap for this, but it would be overkill for this example
    CmdHash_t clear_cmd = create_cmd_from_str("clear");
    CmdHash_t write_cmd = create_cmd_from_str("write");
    CmdHash_t set_line = create_cmd_from_str("setline");
    CmdHash_t curso_cmd = create_cmd_from_str("curso");
    CmdHash_t display_cmd = create_cmd_from_str("display");
    CmdHash_t moved_cmd = create_cmd_from_str("moved");
    CmdHash_t cpp_cmd = create_cmd_from_str("cpp");
    CmdHash_t reset_cmd = create_cmd_from_str("reset");
    lcd.begin();

    //load cpp logo
    for(size_t i = 0; i<6; i++){
        lcd.createChar(cpp_logo[i], i);
    }
    lcd.setCursor(0,0);
    while(1){
        if(xQueueReceive(lcd_server_buf, &cmd, 0) == pdPASS){

            //execute command (no error checking yet)
            if(clear_cmd.hash_cmd == cmd.hash.hash_cmd){

                lcd.clear();

            }else if(write_cmd.hash_cmd == cmd.hash.hash_cmd){

                lcd.write_limit((char*)cmd.args.msg + 1, cmd.args.msg_size - 2);
                lcd_posi_yx[1] += cmd.hash.hash_cmd - 2;

            }else if (set_line.hash_cmd == cmd.hash.hash_cmd){

                parse_num_args(cmd.args, ' ', lcd_posi_yx, lcd_posi_yx_size);
                lcd.setCursor(lcd_posi_yx[0], lcd_posi_yx[1]);

            }else if(curso_cmd.hash_cmd == cmd.hash.hash_cmd){

                cursor_std = cmd.args.msg[1] - '0';
                if(cursor_std){
                    lcd.cursorOn().cursorBlinkOn();
                }else{
                    lcd.cursorOff().cursorBlinkOff();
                }

            }else if(display_cmd.hash_cmd == cmd.hash.hash_cmd){

                display_std = cmd.args.msg[1] - '0';
                if(display_std){
                    lcd.displayOn();
                }else{
                    lcd.displayOff();
                }

            }else if(moved_cmd.hash_cmd == cmd.hash.hash_cmd){

                parse_num_args(cmd.args, ' ', lcd_shift_args, lcd_shift_args_size);
                for(uint8_t i = 0; i < lcd_shift_args[1]; i++){
                    if(lcd_shift_args[0]){
                        lcd.moveDisplayLeft();
                    }else{
                        lcd.moveDisplayRight();
                    }
                }

            }else if(cpp_cmd.hash_cmd == cmd.hash.hash_cmd){

                lcd.setCursor(0,lcd_posi_yx[1])
                    .send(0,1).send(1,1).send(2,1)
                    .setCursor(1,lcd_posi_yx[1])
                    .send(3,1).send(4,1).send(5,1);

            }else if(reset_cmd.hash_cmd == cmd.hash.hash_cmd) {

                lcd.clear().reset();
                lcd_posi_yx[1] = 0;
            }

            vPortFree(cmd.args.msg);
        }
       vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
