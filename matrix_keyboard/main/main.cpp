//MINI BF computer
//author: Guilherme Silva Schultz
//data: 2024-01-29


//std includes
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

//esp-idf includes
#include <driver/i2c.h>
#include <driver/gptimer.h>
#include <driver/gpio.h>
#include <esp_log.h>

//freeRTOS includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>

//extra includes
#include <libs/UniversalLCD/universallcd.hpp>
#include <libs/brainfvck/brainfck.hpp>



//i2c defines
#define LCD_ADDR 0x27
#define I2C_MASTER I2C_NUM_0
#define TIMER_HZ (1000*1000)

//console defines
#define LCD_MAX            160
#define LCD_MAX_DISPLAY     80
#define LCD_HALF_DISPLAY    40
#define BF_OUTPUT_SIZE     100
#define BF_STACK_SIZE       15
#define BF_INPUT_LINE        3
#define BF_INPUT_COL         0
#define BF_MAX_INPUT         3
#define BF_MAX_DATA_OUTPUT  20
#define BF_OUTPUT_LINE       3
#define BF_OUTPUT_COL       20
#define BF_MAX_PROG  LCD_MAX*3
#define MSG_QUEUE_TIMEOUT  500
#define BF_INTER_SPEED     100

//lcd types
struct LcdDrive : public universalLCD::BusInterface{
    void send(uint8_t config, uint8_t data){
        uint8_t package = (config & 0b00001111) | (data & 0xF0);
        i2c_master_write_to_device(I2C_MASTER, LCD_ADDR,&package,1,pdMS_TO_TICKS(100));
    }
};

//keyboard types
typedef struct {
    uint8_t *pins_out;
    size_t pins_out_qtd;
    uint8_t *pins_in;
    size_t pins_in_qtd;
    size_t btn_qtd;
} keyboard_t;

typedef struct {
 uint8_t *key_arr;
 size_t key_arr_size;
}keylayout_t;


//console types
typedef enum {
    CmdPass,
    CmdClear,
    CmdBackspace,
    CmdWriteChar,
    CmdWriteStr,
    CmdSelectCurso,
}consoletype_t;

typedef struct {
    consoletype_t type;
    union{
        uint8_t c;
        char *str;
        struct{
            uint8_t row;
            uint8_t col;
        };
    };
}consolemsg_t;

typedef enum{
    ProgInput,
    ProgRun,
    ProgEnd,
    ProgWaitOutput,
    ProgWaitInput,
    ProgError,
}consolestate_t;

//LCD functions
static IRAM_ATTR bool timer_delay_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
void delay(uint32_t us);
void i2c_init(void);
void lcd_task(void *args);

//keyboard functions
static void keyboard_event(void *args);
void keyboard_init(keyboard_t keyboard);
void debouce_event(TimerHandle_t xtimer);
void console_task(void *args);

//util functions
void sendChar(char c);
void setCurso(uint8_t row, uint8_t col);
void sendStr(const char *str);
void sendClear();
void sendBackspace();
void sendMsg(consolemsg_t msg); //send and check for errors


//lcd drive handler
TaskHandle_t lcd_server_task;
gptimer_handle_t lcd_driver_delay;
gptimer_config_t timer_config;
gptimer_alarm_config_t delay_config;
gptimer_event_callbacks_t delay_callback;
volatile bool timer_flag;


//keyboard hander
TimerHandle_t debouce_handler;
static uint8_t pinsout[] = {19,18,5,17};
static uint8_t pinsin[] = {16,4,2};
static keyboard_t keyboard = {
    .pins_out = pinsout,
    .pins_out_qtd = sizeof(pinsout),
    .pins_in = pinsin,
    .pins_in_qtd = sizeof(pinsin),
    .btn_qtd = sizeof(pinsin) * sizeof(pinsout) - 1
};


//console handlers
TaskHandle_t console_hander;
QueueHandle_t console_msg;

static uint8_t bf_keys[] = {'+', '-', '<', '>', '[', ']', '.', ','};
static keylayout_t bf_layout = {
    .key_arr = bf_keys,
    .key_arr_size = sizeof(bf_keys),
};
const char erro_msg[] = "Programa invalido! pressione qualquer botao para continuar";
const char clear_input[] = "    ";
char bfoutput[BF_OUTPUT_SIZE];

extern "C" void app_main(void){
    debouce_handler = xTimerCreate("keyvorad,", pdMS_TO_TICKS(50),pdFALSE,  NULL, debouce_event);
    console_msg = xQueueCreate(10, sizeof(consolemsg_t));
    xTaskCreatePinnedToCore(console_task,"console",4096,NULL, 10, &console_hander,0);
    xTaskCreatePinnedToCore(lcd_task, "lcd", 4096,NULL,1,NULL, 1);
    while(1) vTaskDelay(portMAX_DELAY);
}


// ======================= LCD FUNCTIONS ===========================================
static bool IRAM_ATTR timer_delay_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx){
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

void lcd_task(void *args){
    LcdDrive lcd_drive;
    universalLCD::UniversalLCD lcd_hw(lcd_drive,universalLCD::Bus4Bits,delay);
    consolemsg_t rev;
    size_t lcd_idx = 0;
    size_t lcd_idx_offset = 0;
    size_t msglen = 0;
    size_t msgoffset = 0;
    i2c_init();
    lcd_timer_init();
    lcd_hw.begin();
    while(1){
        if(xQueueReceive(console_msg, &rev, portMAX_DELAY) == pdPASS){
            if(rev.type == CmdWriteChar){
                lcd_hw.send(rev.c, 1);
                lcd_idx++;
                if(lcd_idx == LCD_MAX_DISPLAY)lcd_hw.selectLcd(1).setCursor(0,0);
            }else if(rev.type == CmdWriteStr){
                msglen = strlen(rev.str);
                lcd_idx_offset = (lcd_idx % LCD_MAX_DISPLAY);
                msgoffset = lcd_idx_offset + msglen;
                if(msgoffset > LCD_MAX_DISPLAY){
                    lcd_hw.write_limit(rev.str, LCD_MAX_DISPLAY-lcd_idx_offset);
                    if(lcd_idx < LCD_MAX_DISPLAY){
                        lcd_hw.selectLcd(1).setCursor(0,0).write((char*)rev.str + (LCD_MAX_DISPLAY-lcd_idx_offset));
                    }
                }else{
                    lcd_hw.write(rev.str);
                }
                lcd_idx = (lcd_idx + msglen) % LCD_MAX;
            }else if(rev.type == CmdBackspace){
                if(!lcd_idx)continue;
                lcd_idx--;
                if(lcd_idx == LCD_MAX_DISPLAY - 1)lcd_hw.selectLcd(0).setCursor(1,LCD_HALF_DISPLAY);
                lcd_hw.moveCursorLeft().send(' ', 1).moveCursorLeft();
            }else if(rev.type == CmdClear){
                lcd_hw.echo().clear().setCursor(0,0).selectLcd(0);
                lcd_idx = 0;
            }else if(rev.type == CmdSelectCurso){
                lcd_hw.selectLcd(0);
                if(rev.row > 1)lcd_hw.selectLcd(1);
                lcd_hw.setCursor(rev.row%2,rev.col);
                lcd_idx = (40*rev.row) + rev.col;
                lcd_idx = lcd_idx > LCD_MAX ? LCD_MAX : lcd_idx;
            }
        }
    }
}

//==============================keyboard functions====================================

static void IRAM_ATTR keyboard_event(void *args){
    //set isr off
    for(size_t i = 0; i < keyboard.pins_in_qtd; i++){
        gpio_isr_handler_remove((gpio_num_t)keyboard.pins_in[i]);
    }
    xTimerStartFromISR(debouce_handler,NULL);
}

void debouce_event(TimerHandle_t xtimer){
    uint8_t key_read = 0;
    uint8_t state = 0;
    //set output to low
    for(size_t i = 0; i < keyboard.pins_out_qtd; i++){
        gpio_set_level((gpio_num_t) keyboard.pins_out[i], 0);
    }

    for(size_t i = 0; i < keyboard.pins_out_qtd; i++){
        gpio_set_level((gpio_num_t) keyboard.pins_out[i], 1);
        for(size_t i = 0; i < keyboard.pins_in_qtd; i++){
            state = gpio_get_level((gpio_num_t)keyboard.pins_in[i]);
            if(state) break;
            key_read++;
        }
        gpio_set_level((gpio_num_t) keyboard.pins_out[i], 0);
        if(state) break;
    }

    //set output to high
    for(size_t i = 0; i < keyboard.pins_out_qtd; i++){
        gpio_set_level((gpio_num_t) keyboard.pins_out[i], 1);
    }

    //set isr up agin
    for(size_t i = 0; i < keyboard.pins_in_qtd; i++){
        gpio_isr_handler_add((gpio_num_t)keyboard.pins_in[i], keyboard_event, NULL);
    }

    if(key_read <= keyboard.btn_qtd) xTaskNotifyFromISR(console_hander,key_read,eSetValueWithOverwrite,NULL);
}

void keyboard_init(keyboard_t keyboard){
    gpio_config_t io_config = {};
    uint64_t mask_out = 0;
    uint64_t mask_in = 0;

    for(size_t i = 0; i < keyboard.pins_out_qtd; i++){
        mask_out |= (1<<keyboard.pins_out[i]);
    }
    for(size_t i = 0; i < keyboard.pins_in_qtd; i++){
        mask_in |= (1<<keyboard.pins_in[i]);
    }

    io_config.pin_bit_mask = mask_out;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.intr_type = GPIO_INTR_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_config);

    for(size_t i = 0; i < keyboard.pins_out_qtd; i++){
        gpio_set_level((gpio_num_t) keyboard.pins_out[i], 1);
    }

    io_config.pin_bit_mask = mask_in;
    io_config.mode = GPIO_MODE_INPUT;
    io_config.intr_type = GPIO_INTR_POSEDGE;
    io_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_config);
    gpio_install_isr_service(0);

    for(size_t i = 0; i < keyboard.pins_in_qtd; i++){
        gpio_isr_handler_add((gpio_num_t)keyboard.pins_in[i], keyboard_event, NULL);
    }
}

//============================================console functions ================================
void console_task(void *args){
    uint64_t ret = 0;
    bferror_t pgerro = BFPass;
    uint8_t output_index = 0;
    uint8_t input_index = 0;
    uint8_t input_buff[BF_MAX_INPUT+1]{'\0'};
    bfio_t iocrtl = IOPass;
    uint8_t prog_buf[BF_MAX_PROG]{0};
    uint8_t data_buf[BF_STACK_SIZE]{0};
    char input_c = 0;
    bfprogm_t prog = {
        .prog_buf = prog_buf,
        .data_buf = data_buf,
        .prog_buf_size = sizeof(prog_buf),
        .data_buf_size = sizeof(data_buf),
        .prog_idx = 0,
        .data_idx = 0,
    };
    consolestate_t state = ProgInput;
    keyboard_init(keyboard);
    while(1){
        if(state == ProgInput){
            ret = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if(ret < bf_layout.key_arr_size){
                if(prog.prog_idx > prog.prog_buf_size)continue;
                if(prog.prog_idx % (LCD_MAX+1) == LCD_MAX){
                    sendClear();
                }

                prog.prog_buf[prog.prog_idx++] = bf_layout.key_arr[ret];
                sendChar(bf_layout.key_arr[ret]);
            }else if(ret == 11){
                if(prog.prog_idx == 0)continue;
                sendBackspace();
                prog.prog_buf[--prog.prog_idx] = 0;
                if(prog.prog_idx % (LCD_MAX+1) == LCD_MAX){
                    sendStr((char*)prog.prog_buf + (prog.prog_idx - LCD_MAX));
                    setCurso(3,LCD_HALF_DISPLAY);
                }
            }else if(ret == 9){
                prog.prog_idx = 0;
                state = ProgRun;
                continue;
            }
        }else if(state == ProgRun){
            pgerro = bf_check(prog);
            if(pgerro != BFPass){
                state = ProgError;
                continue;
            }
            if(prog.prog_idx < prog.prog_buf_size){
                if(prog.prog_buf[prog.prog_idx] == '\0') state = ProgEnd;
                setCurso(0,0);
                sprintf(bfoutput, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x            atual:%c index:%03lu",
                prog.data_buf[0], prog.data_buf[1], prog.data_buf[2], prog.data_buf[3],
                prog.data_buf[4],prog.data_buf[5],prog.data_buf[6],prog.data_buf[7],
                prog.data_buf[8], prog.data_buf[9], prog.data_buf[10], prog.data_buf[11],
                prog.data_buf[12],prog.prog_buf[prog.prog_idx], prog.prog_idx);
                sendStr(bfoutput);
                iocrtl = bf_step(&prog);
                if(iocrtl == WaitOutput){
                    state = ProgWaitOutput;
                    continue;
                }else if(iocrtl == WaitInput){
                    state = ProgWaitInput;
                    continue;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(BF_INTER_SPEED));
        }else if(state == ProgEnd){
            memset(prog.prog_buf,0,prog.prog_buf_size);
            memset(prog.data_buf,0,prog.data_buf_size);
            output_index = 0;
            prog.data_idx = 0;
            prog.prog_idx = 0;
            state = ProgInput;
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            sendClear();
        }else if(state == ProgWaitInput){
            setCurso(BF_INPUT_LINE,BF_INPUT_COL+input_index);
            ret = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if(ret == 9){
                prog.data_buf[prog.data_idx] = atoi((char*)input_buff);
                memset(input_buff,'\0',sizeof(input_buff));
                setCurso(BF_INPUT_LINE, BF_INPUT_COL);
                input_index = 0;
                sendStr(clear_input);
                state = ProgRun;
            }else if(ret == 11){
                if(input_index == 0)continue;
                input_buff[--input_index] = '\0';
                sendBackspace();
            }else{
                if(input_index > BF_MAX_INPUT)continue;
                input_c = ret + '0';
                input_buff[input_index++] = input_c;
                sendChar(input_c);
            }
        }else if(state == ProgWaitOutput){
            setCurso(BF_OUTPUT_LINE, BF_OUTPUT_COL+output_index);
            sendChar(prog.data_buf[prog.data_idx]);
            output_index++;
            state = ProgRun;
        }
        else if(state == ProgError){
            sendStr(erro_msg);
            state = ProgEnd;
        }
    }
}


//util functions
void sendChar(char c) {
    consolemsg_t msg = {
        .type = CmdWriteChar,
        .c = c,
    };
    sendMsg(msg);
}


void setCurso(uint8_t row, uint8_t col){
    consolemsg_t msg = {
        .type = CmdSelectCurso,
        .row = row,
        .col = col,
    };
    sendMsg(msg);
}
void sendStr(const char *str){
    consolemsg_t msg = {
        .type = CmdWriteStr,
        .str = (char*)str,
    };
    sendMsg(msg);
}


void sendClear(){
    consolemsg_t msg = {
        .type = CmdClear,
        .c = 0,
    };
    sendMsg(msg);
}

void sendBackspace(){
    consolemsg_t msg = {
        .type = CmdBackspace,
        .c = 0,
    };
    sendMsg(msg);
}
void sendMsg(consolemsg_t msg){
    if(xQueueSend(console_msg,&msg,MSG_QUEUE_TIMEOUT) == pdFAIL){
        ESP_LOGE("console", "QUEUE timeout");
        while(1);
    }
}
