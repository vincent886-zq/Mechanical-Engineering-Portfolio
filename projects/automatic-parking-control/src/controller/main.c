#include "stm32f10x.h"
#include "OLED.h"
#include <string.h>
#include <ctype.h>
#include "stdlib.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "delay.h"
#include "uart.h"
#include "stdio.h"

/* SPI硬件相关宏定义 */
#define SPIx_RCC      RCC_APB1Periph_SPI2  // SPI2时钟
#define SPIx          SPI2                 // 使用SPI2外设
#define SPI_GPIO_RCC  RCC_APB2Periph_GPIOB // GPIOB时钟
#define SPI_GPIO      GPIOB                // SPI引脚所在GPIO端口
#define SPI_PIN_MOSI  GPIO_Pin_15          // MOSI引脚号
#define SPI_PIN_MISO  GPIO_Pin_14          // MISO引脚号
#define SPI_PIN_SCK   GPIO_Pin_13          // SCK时钟引脚号
#define SPI_PIN_SS    GPIO_Pin_12          // 片选引脚号

// 硬件定义
#define LED_PIN    GPIO_Pin_13
#define LED_PORT   GPIOC
#define ESP_USART  USART1

// 引脚定义（PA0~PA8, PB5~PB7、PB10~PB15）
// 电机1
#define ENA_PIN    GPIO_Pin_0 //pa0
#define ENA_PORT   GPIOA
#define IN1_PIN    GPIO_Pin_1 //pa1
#define IN1_PORT   GPIOA
#define IN2_PIN    GPIO_Pin_2 //pa2
#define IN2_PORT   GPIOA

// 电机2
#define ENB_PIN    GPIO_Pin_3 //pa3
#define ENB_PORT   GPIOA
#define IN3_PIN    GPIO_Pin_4 //pa4
#define IN3_PORT   GPIOA
#define IN4_PIN    GPIO_Pin_5 //pa5
#define IN4_PORT   GPIOA

// 电机3
#define ENA2_PIN    GPIO_Pin_6      // PA6
#define ENA2_PORT   GPIOA
#define IN1_2_PIN    GPIO_Pin_7      // PA7
#define IN1_2_PORT   GPIOA
#define IN2_2_PIN    GPIO_Pin_8      // PA8
#define IN2_2_PORT   GPIOA

// 电机4
#define ENB2_PIN    GPIO_Pin_11     // PA11
#define ENB2_PORT   GPIOA
#define IN3_2_PIN    GPIO_Pin_12     // PA12
#define IN3_2_PORT   GPIOA
#define IN4_2_PIN    GPIO_Pin_0     // PB0
#define IN4_2_PORT   GPIOB

// 电机5
#define ENA3_PIN    GPIO_Pin_3       // PB3
#define ENA3_PORT   GPIOB
#define IN1_3_PIN   GPIO_Pin_4       // PB4
#define IN1_3_PORT  GPIOB
#define IN2_3_PIN   GPIO_Pin_5       // PB5
#define IN2_3_PORT  GPIOB

// 电机6
#define ENB3_PIN    GPIO_Pin_1      // PB1
#define ENB3_PORT   GPIOB
#define IN3_3_PIN   GPIO_Pin_6      // PB6
#define IN3_3_PORT  GPIOB
#define IN4_3_PIN   GPIO_Pin_7      // PB7
#define IN4_3_PORT  GPIOB

// 全局变量
volatile uint8_t ok_received = 0;
volatile uint8_t cmd_index = 0;
volatile uint32_t timeout_counter = 0;
volatile uint8_t data_ready = 0;
char display_data[17] = {0};  // OLED每行最多显示16字符

// 电机位置状态
volatile uint8_t motor_positions[7] = {0}; // 存储各电机位置状态

// 新增解析相关变量
volatile uint8_t parsing_enable = 0;
volatile uint8_t parse_index = 0;
char parse_buffer[32] = {0};
volatile uint8_t Q = 0;

// 状态机定义
typedef enum {
    RX_STATE_IDLE,
    RX_STATE_IPD_HEADER,
    RX_STATE_SKIP_PARAMS,
    RX_STATE_DATA_LEN,
    RX_STATE_DATA_CONTENT
} RxState;

volatile RxState rx_state = RX_STATE_IDLE;
volatile uint16_t data_remaining = 0;
volatile uint8_t rx_index = 0;

// 动作状态机
typedef enum {
    ACTION_IDLE,
    ACTION_MOVING_X,    // 移动横轴
    ACTION_MOVING_Y,    // 移动竖轴
    ACTION_MOVING_Z,    // 移动存取电机
    ACTION_WAITING,     // 等待时间
    ACTION_RETURNING_X, // 返回横轴
    ACTION_RETURNING_Y, // 返回竖轴
    ACTION_COMPLETED    // 动作完成
} ActionState;

volatile ActionState current_action = ACTION_IDLE;
volatile uint8_t action_step = 0;
volatile uint32_t action_timer = 0;
volatile uint8_t current_row = 0;
volatile uint8_t current_col = 0;

// 坐标定义
typedef struct {
    uint16_t x;  // 横轴位置 (电机5)
    uint16_t y;  // 竖轴位置 (电机6)
    uint16_t z;  // 存取位置 (电机1-4)
    uint8_t motor_id; // 使用的电机ID (1-4)
} ParkingPosition;

// 位置表
const ParkingPosition positions[3][3] = {
    // 第0行（全部不使用，初始化为0）
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
    
    // 第1行
    {{0, 0, 0, 0},           // 第0列不使用
     {110, 80, 180, 1},     // (1,1)
     {350, 80, 180, 1}},    // (1,2)
    
    // 第2行
    {{0, 0, 0, 0},           // 第0列不使用
     {110, 270, 180, 2},    // (2,1)
     {350, 270, 180, 2}}    // (2,2)
};

// 命令表
typedef enum {
    CMD_NONE,
    CMD_STOP,
    CMD_ZERO,
    CMD_PARK_1_1,
    CMD_PARK_1_2,
    CMD_PARK_2_1,
    CMD_PARK_2_2,
    CMD_RETRIEVE_1_1,
    CMD_RETRIEVE_1_2,
    CMD_RETRIEVE_2_1,
    CMD_RETRIEVE_2_2
} Command;

volatile Command current_command = CMD_NONE;

const char* at_cmds[] = {
    "ATE0\r\n",               // 关闭回显
    "AT+CWMODE=2\r\n",        // 设置为AP模式
    "AT+CWSAP=\"PARKING_DEMO\",\"CHANGE_ME\",5,3\r\n",
    "AT+CIPMUX=1\r\n",        // 允许多连接
    "AT+CIPSERVER=1,8080\r\n" // 启动TCP服务器
};

/* 函数声明 */
void NVIC_Configuration(void);       // 中断控制器配置
void SPIx_Init(void);                // SPI初始化
uint8_t is_SPIx_EnableSlave(void);   // 检测从设备使能状态
void USART1_Init(void);              // USART初始化（未实现）

/* 全局变量定义 */
volatile uint8_t receivedStr = 0;    // 接收完成标志位
uint8_t buffRx[100]= "";             // 接收数据缓冲区
volatile uint8_t RxIdx = 0;          // 接收数据索引
volatile uint8_t TxCnt = 0;          // 发送计数器

// 系统时钟配置（72MHz）
void SystemClock_Config(void) {
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while(RCC_GetSYSCLKSource() != 0x08);
}

// SysTick延时初始化
void SysTick_Init(void) {
    SysTick->CTRL = 0;          
    SysTick->LOAD = 72000 - 1;  // 1ms @72MHz
    SysTick->VAL = 0;          
    SysTick->CTRL = 0x05;       
}

void SysTick_Delay(uint32_t ms) {
    SysTick->LOAD = 72000 * ms;         // 72MHz时钟下，1ms的计数值
    SysTick->VAL = 0;                   // 清空计数器
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk; // 启动SysTick
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // 等待计时完成
    SysTick->CTRL = 0;                  // 关闭SysTick
}

void Delay_ms(uint32_t ms) {
    SysTick->VAL = 0;          
    for(uint32_t i=0; i<ms; i++) {
        while(!(SysTick->CTRL & 0x10000));
    }
}

// LED控制
void LED_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
}

void LED_Blink(uint8_t times) {
    for(uint8_t i=0; i<times; i++) {
        GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
        Delay_ms(200);
        GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);
        Delay_ms(200);
    }
}

// USART1初始化
void USART1_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    // TXD (PA9)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // RXD (PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(ESP_USART, &USART_InitStructure);
    USART_ITConfig(ESP_USART, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
    USART_Cmd(ESP_USART, ENABLE);
}

// USART1发送字符串
void ESP_SendCmd(const char* cmd) {
    while(*cmd) {
        USART_SendData(ESP_USART, *cmd++);
        while(USART_GetFlagStatus(ESP_USART, USART_FLAG_TXE) == RESET);
    }
}

// USART1中断服务程序（修正版）
void USART1_IRQHandler(void) {
    if(USART_GetITStatus(ESP_USART, USART_IT_RXNE) != RESET) {
        static char ok_buf[4] = {0};
        static uint8_t ipd_state = 0;
        static uint8_t data_flag = 0; // 0:空闲 1:找冒号 2:接收数据
        static uint8_t idx = 0;
        
        char c = USART_ReceiveData(ESP_USART);
        USART_ClearITPendingBit(ESP_USART, USART_IT_RXNE);

        // 更新OK检测滑动窗口
        memmove(ok_buf, ok_buf+1, 3);
        ok_buf[3] = c;
        if(strstr(ok_buf, "OK")) ok_received = 1;

        // IPD检测状态机
        switch(ipd_state) {
            case 0: if(c == '+') ipd_state++; break;
            case 1: if(c == 'I') ipd_state++; else ipd_state=0; break;
            case 2: if(c == 'P') ipd_state++; else ipd_state=0; break;
            case 3: 
                if(c == 'D') data_flag = 1;
                ipd_state = 0;
                break;
        }

        // 数据接收处理
        switch(data_flag) {
            case 1: // 寻找冒号
                if(c == ':') {
                    data_flag = 2;
                    idx = 0;
                    memset(display_data, 0, sizeof(display_data));
                }
                break;
            case 2: // 接收数据
                if(c == '\r' || c == '\n' || idx >= 16) {
                    display_data[idx] = '\0';
                    data_ready = 1;
                    data_flag = 0;
                } else {
                    display_data[idx++] = c;
                }
                break;
        }
    }
}

// OLED显示TCP就绪状态
void Show_TCP_Ready(void) {
    OLED_Clear();
    OLED_ShowString(1, 1, "IP: 192.168.4.1");
    OLED_ShowString(2, 1, "RCV:");
}

// 电机控制函数
static void motor1_cw(void) {
    // 电机1顺时针转动-前进
    GPIO_SetBits(IN1_PORT, IN1_PIN);
    GPIO_ResetBits(IN2_PORT, IN2_PIN);
    GPIO_SetBits(ENA_PORT, ENA_PIN);
}

static void motor1_ccw(void) {
    // 电机1逆时针转动—后退
    GPIO_ResetBits(IN1_PORT, IN1_PIN);
    GPIO_SetBits(IN2_PORT, IN2_PIN);
    GPIO_SetBits(ENA_PORT, ENA_PIN);
} 

// 电机1停止
static void motor1_stop(void) {
    GPIO_ResetBits(ENA_PORT, ENA_PIN);    
}

static void motor2_cw(void) {
    // 电机2顺时针转动-前进
    GPIO_SetBits(IN3_PORT, IN3_PIN);
    GPIO_ResetBits(IN4_PORT, IN4_PIN);
    GPIO_SetBits(ENB_PORT, ENB_PIN);
}

static void motor2_ccw(void) {
    // 电机2逆时针转动—后退
    GPIO_ResetBits(IN3_PORT, IN3_PIN);
    GPIO_SetBits(IN4_PORT, IN4_PIN);
    GPIO_SetBits(ENB_PORT, ENB_PIN);
} 

// 电机2停止
static void motor2_stop(void) {
    GPIO_ResetBits(ENB_PORT, ENB_PIN);    
}

static void motor3_cw(void) {
    // 电机3顺时针转动-前进
    GPIO_SetBits(IN1_2_PORT, IN1_2_PIN);
    GPIO_ResetBits(IN2_2_PORT, IN2_2_PIN);
    GPIO_SetBits(ENA2_PORT, ENA2_PIN);
}

static void motor3_ccw(void) {
    // 电机3逆时针转动—后退
    GPIO_ResetBits(IN1_2_PORT, IN1_2_PIN);
    GPIO_SetBits(IN2_2_PORT, IN2_2_PIN);
    GPIO_SetBits(ENA2_PORT, ENA2_PIN);
} 

// 电机3停止
static void motor3_stop(void) {
    GPIO_ResetBits(ENA2_PORT, ENA2_PIN);    
}

static void motor4_cw(void) {
    // 电机4顺时针转动-前进
    GPIO_SetBits(IN3_2_PORT, IN3_2_PIN);
    GPIO_ResetBits(IN4_2_PORT, IN4_2_PIN);
    GPIO_SetBits(ENB2_PORT, ENB2_PIN);
}

static void motor4_ccw(void) {
    // 电机4逆时针转动—后退
    GPIO_ResetBits(IN3_2_PORT, IN3_2_PIN);
    GPIO_SetBits(IN4_2_PORT, IN4_2_PIN);
    GPIO_SetBits(ENB2_PORT, ENB2_PIN);
} 

// 电机4停止
static void motor4_stop(void) {
    GPIO_ResetBits(ENB2_PORT, ENB2_PIN);     
}

static void motor5_cw(void) {
    // 电机5顺时针转动-前进
    GPIO_SetBits(IN1_3_PORT, IN1_3_PIN);
    GPIO_ResetBits(IN2_3_PORT, IN2_3_PIN);
    GPIO_SetBits(ENA3_PORT, ENA3_PIN);
}

static void motor5_ccw(void) {
    // 电机5逆时针转动—后退
    GPIO_ResetBits(IN1_3_PORT, IN1_3_PIN);
    GPIO_SetBits(IN2_3_PORT, IN2_3_PIN);
    GPIO_SetBits(ENA3_PORT, ENA3_PIN);
} 

// 电机5停止
static void motor5_stop(void) {
    GPIO_ResetBits(ENA3_PORT, ENA3_PIN);
}

static void motor6_cw(void) {
    // 电机6顺时针转动-前进
    GPIO_SetBits(IN3_3_PORT, IN3_3_PIN);
    GPIO_ResetBits(IN4_3_PORT, IN4_3_PIN);
    GPIO_SetBits(ENB3_PORT, ENB3_PIN);
}

static void motor6_ccw(void) {
    // 电机6逆时针转动—后退
    GPIO_ResetBits(IN3_3_PORT, IN3_3_PIN);
    GPIO_SetBits(IN4_3_PORT, IN4_3_PIN);
    GPIO_SetBits(ENB3_PORT, ENB3_PIN);
} 

// 电机6停止
static void motor6_stop(void) {
    GPIO_ResetBits(ENB3_PORT, ENB3_PIN);
}

// 所有电机停止
static void Motors_stop(void) {
    GPIO_ResetBits(ENA_PORT, ENA_PIN);       // 电机1
    GPIO_ResetBits(ENB_PORT, ENB_PIN);       // 电机2
    GPIO_ResetBits(ENA2_PORT, ENA2_PIN);     // 电机3
    GPIO_ResetBits(ENB2_PORT, ENB2_PIN);     // 电机4
    GPIO_ResetBits(ENA3_PORT, ENA3_PIN);     // 电机5
    GPIO_ResetBits(ENB3_PORT, ENB3_PIN);     // 电机6
}

// 回零动作库
static void Motors_zero(void) {
    // 电机1逆时针回初始点
    GPIO_SetBits(IN1_PORT, IN1_PIN);
    GPIO_ResetBits(IN2_PORT, IN2_PIN);
    GPIO_SetBits(ENA_PORT, ENA_PIN);
    
    // 电机2逆时针回初始点    
    GPIO_SetBits(IN3_PORT, IN3_PIN);
    GPIO_ResetBits(IN4_PORT, IN4_PIN);
    GPIO_SetBits(ENB_PORT, ENB_PIN);    
    
    // 电机3逆时针回初始点    
    GPIO_SetBits(IN1_2_PORT, IN1_2_PIN);
    GPIO_ResetBits(IN2_2_PORT, IN2_2_PIN);
    GPIO_SetBits(ENA2_PORT, ENA2_PIN);    
    
    // 电机4逆时针回初始点    
    GPIO_SetBits(IN3_2_PORT, IN3_2_PIN);
    GPIO_ResetBits(IN4_2_PORT, IN4_2_PIN);
    GPIO_SetBits(ENB2_PORT, ENB2_PIN);
    
    // 电机5逆时针回初始点    
    GPIO_SetBits(IN1_3_PORT, IN1_3_PIN);
    GPIO_ResetBits(IN2_3_PORT, IN2_3_PIN);
    GPIO_SetBits(ENA3_PORT, ENA3_PIN);    
    
    // 电机6逆时针回初始点    
    GPIO_SetBits(IN3_3_PORT, IN3_3_PIN);
    GPIO_ResetBits(IN4_3_PORT, IN4_3_PIN);
    GPIO_SetBits(ENB3_PORT, ENB3_PIN);
}

void Motor_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // 禁用JTAG，保留SWD
    
    // 初始化L298N1的电机引脚
    GPIO_InitStruct.GPIO_Pin = ENA_PIN | IN1_PIN | IN2_PIN | ENB_PIN | IN3_PIN | IN4_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 初始化L298N2的电机引脚
    GPIO_InitStruct.GPIO_Pin = ENA2_PIN | IN1_2_PIN | IN2_2_PIN | ENB2_PIN | IN3_2_PIN ;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // IN4_2_PIN单独初始化到GPIOB
    GPIO_InitStruct.GPIO_Pin = IN4_2_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 初始化L298N3的电机引脚
    GPIO_InitStruct.GPIO_Pin = ENA3_PIN | IN1_3_PIN | IN2_3_PIN | ENB3_PIN | IN3_3_PIN | IN4_3_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// SPI中断处理修改
void SPI2_IRQHandler(void) {
    if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_RXNE) != RESET) {
        uint8_t data = SPI_I2S_ReceiveData(SPIx);
        
        char buf[5];
        sprintf(buf, "0x%02X", data);  // 将数据格式化为十六进制字符串
        OLED_ShowString(4, 1, buf);    
        SysTick_Delay(1000);
        OLED_ShowString(4, 1, "                "); 
        
        // 更新电机位置状态
        switch(data) {
            case 0x01: motor_positions[1] = 0; break;    // 电机1左限位
            case 0x02: motor_positions[1] = 1; break;    // 电机1右限位
            case 0x03: motor_positions[2] = 0; break;    // 电机2左限位
            case 0x04: motor_positions[2] = 1; break;    // 电机2右限位
            case 0x05: motor_positions[3] = 0; break;    // 电机3左限位
            case 0x06: motor_positions[3] = 1; break;    // 电机3右限位
            case 0x07: motor_positions[4] = 0; break;    // 电机4左限位
            case 0x08: motor_positions[4] = 1; break;    // 电机4右限位
            case 0x09: motor_positions[5] = 0; break;    // 电机5左限位
            case 0x10: motor_positions[5] = 1; break;    // 电机5右限位
            case 0x11: motor_positions[6] = 0; break;    // 电机6左限位
            case 0x12: motor_positions[6] = 1; break;    // 电机6右限位
            case 0x13: motor_positions[5] = 2; break;    // 电机5中点
            case 0x14: motor_positions[6] = 2; break;    // 电机6中点
        }
        
        SPI_I2S_ClearITPendingBit(SPIx, SPI_I2S_IT_RXNE);
    }
}

// 处理当前动作
void process_action(void) {
    static uint8_t z_motor = 0;
    
    if(current_action == ACTION_IDLE) return;
    
    const ParkingPosition* pos = &positions[current_row][current_col];
    
    switch(current_action) {
        case ACTION_MOVING_X:
            // 等待横轴到达目标位置
            if(motor_positions[5] == 2 || (motor_positions[5] == 1 && pos->x > 0)) {
                motor_positions[5] = 0; // 重置状态
                motor5_stop();
                current_action = ACTION_MOVING_Y;
                
                // 开始移动竖轴
                if(pos->y > 0) {
                    motor6_cw();  // 向上移动
                } else {
                    motor6_ccw(); // 向下移动
                }
            }
            break;
            
        case ACTION_MOVING_Y:
            // 等待竖轴到达目标位置
            if(motor_positions[6] == 2 || (motor_positions[6] == 1 && pos->y > 0)) {
                motor_positions[6] = 0; // 重置状态
                motor6_stop();
                current_action = ACTION_MOVING_Z;
                
                // 开始移动存取电机
                z_motor = pos->motor_id;
                switch(z_motor) {
                    case 1: motor1_cw(); break;
                    case 2: motor2_cw(); break;
                    case 3: motor3_cw(); break;
                    case 4: motor4_cw(); break;
                }
            }
            break;
            
        case ACTION_MOVING_Z:
            // 等待存取电机到达目标位置
            if(motor_positions[z_motor] == 1) {
                switch(z_motor) {
                    case 1: motor1_stop(); break;
                    case 2: motor2_stop(); break;
                    case 3: motor3_stop(); break;
                    case 4: motor4_stop(); break;
                }
                
                // 根据命令执行下一步（存车或取车的不同动作）
                if(current_command >= CMD_PARK_1_1 && current_command <= CMD_PARK_2_2) {
                    // 存车：竖轴下降
                    motor6_ccw();
                    action_timer = 1000; // 1秒延时
                    current_action = ACTION_WAITING;
                } else if(current_command >= CMD_RETRIEVE_1_1 && current_command <= CMD_RETRIEVE_2_2) {
                    // 取车：竖轴上升
                    motor6_cw();
                    action_timer = 1000; // 1秒延时
                    current_action = ACTION_WAITING;
                }
            }
            break;
            
        case ACTION_WAITING:
            if(action_timer > 0) {
                action_timer--;
            } else {
                motor6_stop();
                
                // 存取电机返回
                switch(z_motor) {
                    case 1: motor1_ccw(); break;
                    case 2: motor2_ccw(); break;
                    case 3: motor3_ccw(); break;
                    case 4: motor4_ccw(); break;
                }
                
                current_action = ACTION_MOVING_Z;
                action_step = 1; // 返回步骤
            }
            break;
            
        case ACTION_RETURNING_X:
            // 等待横轴返回初始位置
            if(motor_positions[5] == 0) {
                motor5_stop();
                current_action = ACTION_COMPLETED;
            }
            break;
            
        case ACTION_RETURNING_Y:
            // 等待竖轴返回初始位置
            if(motor_positions[6] == 0) {
                motor6_stop();
                current_action = ACTION_RETURNING_X;
                
                // 开始返回横轴
                motor5_ccw(); // 向左移动（回初始位置）
            }
            break;
            
        case ACTION_COMPLETED:
            // 所有动作完成，回到空闲状态
            Motors_stop();
            
            // 重置所有电机状态
            for(int i = 1; i <= 6; i++) {
                motor_positions[i] = 0;
            }
            
            current_action = ACTION_IDLE;
            current_command = CMD_NONE;
            break;
    }
}

// 执行存车操作
void execute_parking(uint8_t row, uint8_t col) {
    if(row < 1 || row > 2 || col < 1 || col > 2) return;
    
    current_row = row;
    current_col = col;
    
    // 重置相关电机状态
    motor_positions[5] = 0;
    motor_positions[6] = 0;
    
    // 状态机初始化
    current_action = ACTION_MOVING_X;
    action_step = 0;
    
    // 开始移动横轴
    if(positions[row][col].x > 0) {
        motor5_cw();  // 向右移动
    } else {
        motor5_ccw(); // 向左移动
    }
}

// 执行取车操作
void execute_retrieving(uint8_t row, uint8_t col) {
    if(row < 1 || row > 2 || col < 1 || col > 2) return;
    
    current_row = row;
    current_col = col;
    
    // 重置相关电机状态
    motor_positions[5] = 0;
    motor_positions[6] = 0;
    
    // 状态机初始化
    current_action = ACTION_MOVING_X;
    action_step = 0;
    
    // 开始移动横轴
    if(positions[row][col].x > 0) {
        motor5_cw();  // 向右移动
    } else {
        motor5_ccw(); // 向左移动
    }
}

int main(void) {
    SystemClock_Config();
    SysTick_Init();
    LED_Init();
    USART1_Init();
    OLED_Init();
    NVIC_Configuration();
    SPIx_Init();
    Motor_GPIO_Init();
    GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET); // 初始熄灭

    // 发送AT指令配置ESP
    while(cmd_index < sizeof(at_cmds)/sizeof(at_cmds[0])) {
        ok_received = 0;
        timeout_counter = 0;
        ESP_SendCmd(at_cmds[cmd_index]);
        // 等待OK响应（带超时）
        while(!ok_received && timeout_counter++ < 5000) Delay_ms(1);
        if(ok_received) {
            LED_Blink(1);
            cmd_index++;
        } else break;
    }

    // 配置完成
    if(cmd_index == 5) {
        GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET); // LED常亮
        Show_TCP_Ready();
    }

    while(1) {
        // 处理当前动作
        process_action();
        
        // 处理接收到的数据
        if(data_ready) {
            OLED_ShowString(4, 1, "                "); // 清除旧数据
            
            if(strstr((char*)display_data, "stop")) {
                current_command = CMD_STOP;
                Motors_stop();
                current_action = ACTION_IDLE;
            }
            else if(strstr((char*)display_data, "zero")) {
                current_command = CMD_ZERO;
                Motors_zero();
            }
            else if(strstr((char*)display_data, "c11")) {
                current_command = CMD_PARK_1_1;
                execute_parking(1, 1);
            }
            else if(strstr((char*)display_data, "q11")) {
                current_command = CMD_RETRIEVE_1_1;
                execute_retrieving(1, 1);
            }
            else if(strstr((char*)display_data, "c12")) {
                current_command = CMD_PARK_1_2;
                execute_parking(1, 2);
            }
            else if(strstr((char*)display_data, "q12")) {
                current_command = CMD_RETRIEVE_1_2;
                execute_retrieving(1, 2);
            }
            else if(strstr((char*)display_data, "c21")) {
                current_command = CMD_PARK_2_1;
                execute_parking(2, 1);
            }
            else if(strstr((char*)display_data, "q21")) {
                current_command = CMD_RETRIEVE_2_1;
                execute_retrieving(2, 1);
            }
            else if(strstr((char*)display_data, "c22")) {
                current_command = CMD_PARK_2_2;
                execute_parking(2, 2);
            }
            else if(strstr((char*)display_data, "q22")) {
                current_command = CMD_RETRIEVE_2_2;
                execute_retrieving(2, 2);
            }
            
            OLED_ShowString(4, 1, display_data);
            SysTick_Delay(1000);
            OLED_ShowString(4, 1, "                "); 
            data_ready = 0;
        }
    }
}

/**
  * @brief 配置嵌套向量中断控制器
  */

void NVIC_Configuration() {
    NVIC_InitTypeDef NVIC_InitStructure;

    // USART1中断（更高优先级）
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    // SPI2中断（较低优先级）
    NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief SPI外设初始化（从模式）
  */
void SPIx_Init()
{
    SPI_InitTypeDef SPI_InitStruct;  // SPI配置结构体
    GPIO_InitTypeDef GPIO_InitStruct;// GPIO配置结构体

    /*-------- 第1步：配置SPI外设 --------*/
    RCC_APB1PeriphClockCmd(SPIx_RCC, ENABLE);  // 使能SPI时钟
    
    // 配置SPI工作参数（从模式）
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // 波特率预分频64
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;     // 第1个边沿数据采样
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;       // 时钟极性低电平
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;// 8位数据格式
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工模式
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// 高位先传
    SPI_InitStruct.SPI_Mode = SPI_Mode_Slave;     // 从模式
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;        // 软件控制NSS
    SPI_InitStruct.SPI_CRCPolynomial = 7;         // CRC多项式（未使用CRC时可忽略）
    SPI_Init(SPIx, &SPI_InitStruct);              // 应用SPI配置
    SPI_Cmd(SPIx, ENABLE);                        // 使能SPI外设
    
    SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE); // 使能接收中断
    //SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, ENABLE); // 发送中断（默认禁用）

    /*-------- 第2步：配置GPIO引脚 --------*/
    RCC_APB2PeriphClockCmd(SPI_GPIO_RCC, ENABLE); // 使能GPIO时钟
    
    // 配置MOSI和SCK为浮空输入（从模式输入引脚）
    GPIO_InitStruct.GPIO_Pin = SPI_PIN_MOSI | SPI_PIN_SCK;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO, &GPIO_InitStruct);
    
    // 配置MISO为复用推挽输出（从模式输出引脚）
    GPIO_InitStruct.GPIO_Pin = SPI_PIN_MISO;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO, &GPIO_InitStruct);

    // 配置SS引脚为下拉输入（检测主设备使能）
    GPIO_InitStruct.GPIO_Pin = SPI_PIN_SS;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;    // 下拉输入
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO, &GPIO_InitStruct);

    /*-------- 第3步：初始化LED指示灯 --------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // 使能GPIOC时钟
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;       // PC13（板载LED）
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief 检测从设备使能状态
  * @retval 1-从设备被选中（SS低电平），0-未被选中
  */
uint8_t is_SPIx_EnableSlave()
{
    // 通过IDR寄存器读取SS引脚状态
    if(SPI_GPIO->IDR & SPI_PIN_SS)
        return 0;  // SS高电平：未被选中
    else
        return 1;  // SS低电平：被选中
}
