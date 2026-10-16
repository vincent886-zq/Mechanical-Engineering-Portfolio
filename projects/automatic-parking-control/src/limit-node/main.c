#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "delay.h"
#include "uart.h"
#include "string.h"
#include "stdio.h"
#include "OLED.h"

/* SPI硬件相关宏定义 */
#define SPIx_RCC      RCC_APB1Periph_SPI2  // SPI2时钟
#define SPIx          SPI2                 // 使用SPI2外设
#define SPI_GPIO_RCC  RCC_APB2Periph_GPIOB // GPIOB时钟
#define SPI_GPIO      GPIOB                // SPI引脚所在GPIO端口
#define SPI_PIN_MOSI  GPIO_Pin_15          // MOSI引脚号
#define SPI_PIN_MISO  GPIO_Pin_14          // MISO引脚号
#define SPI_PIN_SCK   GPIO_Pin_13          // SCK时钟引脚号
#define SPI_PIN_SS    GPIO_Pin_12          // 片选引脚号

#define KEY_GPIO_RCC  RCC_APB2Periph_GPIOA
#define KEY_1_GPIO      GPIOA
#define KEY_1_PIN       GPIO_Pin_0
#define KEY_2_GPIO      GPIOA
#define KEY_2_PIN       GPIO_Pin_1
#define KEY_3_GPIO      GPIOA
#define KEY_3_PIN       GPIO_Pin_2
#define KEY_4_GPIO      GPIOA
#define KEY_4_PIN       GPIO_Pin_3
#define KEY_5_GPIO      GPIOA
#define KEY_5_PIN       GPIO_Pin_4
#define KEY_6_GPIO      GPIOA
#define KEY_6_PIN       GPIO_Pin_5
#define KEY_7_GPIO      GPIOA
#define KEY_7_PIN       GPIO_Pin_6
#define KEY_8_GPIO      GPIOA
#define KEY_8_PIN       GPIO_Pin_7
#define KEY_9_GPIO      GPIOA
#define KEY_9_PIN       GPIO_Pin_8
#define KEY_10_GPIO     GPIOA
#define KEY_10_PIN      GPIO_Pin_9
#define KEY_11_GPIO     GPIOA
#define KEY_11_PIN      GPIO_Pin_10
#define KEY_12_GPIO     GPIOB
#define KEY_12_PIN      GPIO_Pin_0
#define KEY_13_GPIO     GPIOB
#define KEY_13_PIN      GPIO_Pin_10
#define KEY_14_GPIO     GPIOB
#define KEY_14_PIN      GPIO_Pin_11
#define KEY_NUM 14 // 总按键数


/* 函数声明 */
void NVIC_Configuration(void);       // 中断控制器配置
void SPIx_Init(void);                // SPI初始化
void SPIx_EnableSlave(void);         // 使能从设备（拉低SS）
void SPIx_DisableSlave(void);        // 禁用从设备（拉高SS）
void Key_Init(void);                 // 按键初始化
uint8_t GetKeyState(uint8_t key_idx); // 获取按键状态
void SendKeyCommand(uint8_t key_idx); // 发送按键对应指令

/* 全局变量定义 */
uint8_t receivedByte;                // SPI接收数据缓存
uint8_t buff[]= "hello world\r\n\0"; // 发送数据缓冲区
uint8_t TxIdx = 0;                   // 发送数据索引
uint8_t RxCnt = 0;                   // 接收数据计数器

/**
  * @brief SPI2中断服务函数
  * @note 处理发送和接收中断事件
  */

// 主函数前添加
void Key_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	//初始化按键1-14
    GPIO_InitStruct.GPIO_Pin = 
	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
    GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 |
    GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10  ; // 
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
	
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_10|GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* 全局变量定义 */
uint8_t receivedByte;                // SPI接收数据缓存

/**
  * @brief 获取按键状态
  * @param key_idx 按键索引(0-13)
  * @return 0-按下，1-释放
  */
uint8_t GetKeyState(uint8_t key_idx)
{
    uint16_t pin_mask;
    GPIO_TypeDef* port = GPIOA;

    // 根据按键索引设置引脚和端口
    switch (key_idx) {
        case 0:  pin_mask = GPIO_Pin_0; break;
        case 1:  pin_mask = GPIO_Pin_1; break;
        case 2:  pin_mask = GPIO_Pin_2; break;
        case 3:  pin_mask = GPIO_Pin_3; break;
        case 4:  pin_mask = GPIO_Pin_4; break;
        case 5:  pin_mask = GPIO_Pin_5; break;
        case 6:  pin_mask = GPIO_Pin_6; break;
        case 7:  pin_mask = GPIO_Pin_7; break;
        case 8:  pin_mask = GPIO_Pin_8; break;
        case 9:  pin_mask = GPIO_Pin_9; break;
        case 10: pin_mask = GPIO_Pin_10; break;
        case 11: pin_mask = GPIO_Pin_0;  port = GPIOB; break;
        case 12: pin_mask = GPIO_Pin_10; port = GPIOB; break;
        case 13: pin_mask = GPIO_Pin_11; port = GPIOB; break;
        default: return 1; // 无效索引返回释放状态
    }

    return GPIO_ReadInputDataBit(port, pin_mask);
}

/**
  * @brief 发送按键对应的指令
  * @param key_idx 按键索引(0-13)
  */
void SendKeyCommand(uint8_t key_idx)
{
    uint8_t cmd = 0xFF; // 默认无效指令
    
    // 为每个按键单独分配指令，避免使用映射表
    switch (key_idx) {
        case 0:  cmd = 0x01; break; // 按键1对应指令0x01
        case 1:  cmd = 0x02; break; // 按键2对应指令0x02
        case 2:  cmd = 0x03; break; // 按键3对应指令0x03
        case 3:  cmd = 0x04; break; // 按键4对应指令0x04
        case 4:  cmd = 0x05; break; // 按键5对应指令0x05
        case 5:  cmd = 0x06; break; // 按键6对应指令0x06
        case 6:  cmd = 0x07; break; // 按键7对应指令0x07
        case 7:  cmd = 0x08; break; // 按键8对应指令0x08
        case 8:  cmd = 0x09; break; // 按键9对应指令0x09
        case 9:  cmd = 0x10; break; // 按键10对应指令0x10
        case 10: cmd = 0x11; break; // 按键11对应指令0x11
        case 11: cmd = 0x12; break; // 按键12对应指令0x12
        case 12: cmd = 0x13; break; // 按键13对应指令0x13
        case 13: cmd = 0x14; break; // 按键14对应指令0x14
        default: return; // 无效索引不发送任何指令
    }
    
    // 发送指令
    SPIx_EnableSlave();
    SPI_I2S_SendData(SPIx, cmd);
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
    // 确保接收缓冲区为空
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == SET) {
        receivedByte = SPI_I2S_ReceiveData(SPIx);
    }
    SPIx_DisableSlave();

    // 在OLED显示发送的指令
    char buf[16];
    sprintf(buf, "%2d: 0x%02X", key_idx+1, cmd);
    OLED_ShowString(3, 0, buf);
}

// 主函数
int main(void) {
    // 初始化部分保持不变
    DelayInit();
    OLED_Init();
    NVIC_Configuration();
    SPIx_Init();
    Key_Init();

    uint8_t last_state[KEY_NUM] = {1}; // 初始化为高电平（未按下）
    uint8_t debounce_count[KEY_NUM] = {0}; // 消抖计数器
    const uint8_t debounce_threshold = 5; // 消抖阈值

    OLED_Clear();
    OLED_ShowString(1, 0, " SPI KeyMonitor");

    while (1) {
        for (uint8_t i = 0; i < KEY_NUM; i++) {
            uint8_t current_state = GetKeyState(i);

            if (current_state != last_state[i]) {
                // 状态改变，增加消抖计数
                debounce_count[i]++;
                
                if (debounce_count[i] >= debounce_threshold) {
                    // 确认状态改变
                    last_state[i] = current_state;
                    debounce_count[i] = 0;
                    
                    // 检测下降沿（按键按下）
                    if (!current_state) {
                        SendKeyCommand(i); // 发送对应按键指令
                    }
                }
            } else {
                // 状态未改变，重置消抖计数
                debounce_count[i] = 0;
            }
        }
        
        // 短暂延时，降低CPU使用率
        DelayMs(2);
    }
}

void NVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 设置优先级分组：1位抢占优先级，3位子优先级 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    
    /* 配置SPI2中断通道 */
    NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;             // SPI2中断源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   // 抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          // 子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // 使能中断
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief SPI外设初始化
  */
void SPIx_Init()
{
    SPI_InitTypeDef SPI_InitStruct;  // SPI配置结构体
    GPIO_InitTypeDef GPIO_InitStruct;// GPIO配置结构体

    /*-------- 第1步：配置SPI外设 --------*/
    RCC_APB1PeriphClockCmd(SPIx_RCC, ENABLE);  // 使能SPI时钟
    
    // 配置SPI工作参数
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // 波特率预分频64
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;     // 第1个边沿数据采样
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;       // 时钟极性低电平
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;// 8位数据格式
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工模式
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// 高位先传
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;    // 主模式
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;        // 软件控制NSS
    SPI_InitStruct.SPI_CRCPolynomial = 7;         // CRC多项式（未使用CRC时可忽略）
    SPI_Init(SPIx, &SPI_InitStruct);              // 应用SPI配置
    SPI_Cmd(SPIx, ENABLE);                        // 使能SPI外设
    
    // 如需使用中断功能可取消注释以下两行
    //SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE); // 使能接收中断
    //SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, ENABLE);  // 使能发送中断

    /*-------- 第2步：配置GPIO引脚 --------*/
    RCC_APB2PeriphClockCmd(SPI_GPIO_RCC, ENABLE); // 使能GPIO时钟
    
    // 配置MOSI、MISO、SCK引脚为复用推挽输出
    GPIO_InitStruct.GPIO_Pin = SPI_PIN_MOSI | SPI_PIN_MISO | SPI_PIN_SCK;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;// 50MHz速度
    GPIO_Init(SPI_GPIO, &GPIO_InitStruct);
    
    // 配置SS引脚为推挽输出模式
    GPIO_InitStruct.GPIO_Pin = SPI_PIN_SS;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO, &GPIO_InitStruct);

    // 初始状态禁用从设备
    SPIx_DisableSlave();
}

/**
  * @brief 使能从设备（拉低SS引脚）
  */
void SPIx_EnableSlave()
{
    // 通过BRR寄存器将SS引脚置低（0有效）
    SPI_GPIO->BRR = SPI_PIN_SS;
}

/**
  * @brief 禁用从设备（拉高SS引脚）
  */
void SPIx_DisableSlave()
{
    // 通过BSRR寄存器将SS引脚置高
    SPI_GPIO->BSRR = SPI_PIN_SS;
}
