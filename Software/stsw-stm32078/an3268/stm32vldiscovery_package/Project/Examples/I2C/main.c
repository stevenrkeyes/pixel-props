/**
  * @file    I2C.c 
  * @author  Steven Keyes
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "STM32vldiscovery.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define mpu6050address 0x68

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
I2C_InitTypeDef  I2C_InitStructure;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);

/* Private functions ---------------------------------------------------------*/
void I2C_Setup()
{
  // Enable the I2C peripheral
  I2C_Cmd(I2C1, ENABLE);
  
  // Enable the I2C clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  
  // Configure the SDA and SCL pins
  // SCL is pin06 and SDA is pin 07 for I2C1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  // Set the speed to 50MHz
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  // Set the pin mode to Open Drain because I2C uses a pullup,
  // and idk if there's an internal pullup
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // set all the I2C init values to appropriate values
  I2C_StructInit(&I2C_InitStructure);
  
  // but enable ack
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  
  // use these values to initialize the I2C bus
  I2C_Init(I2C1, &I2C_InitStructure);

}

void init_sensor(void)
{
  /* initiate start sequence */
  I2C_GenerateSTART(I2C1, ENABLE);
  /* check start bit flag */
  //while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));
  
  /*send write command to chip*/
  I2C_Send7bitAddress(I2C1, mpu6050address<<1, I2C_Direction_Transmitter);
  /*check master is now in Tx mode*/
  //while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /*mode register address*/
  I2C_SendData(I2C1, 0x6B);
  /*wait for byte send to complete*/
  //while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /*clear bits*/
  I2C_SendData(I2C1, 0x00);
  /*wait for byte send to complete*/
  //while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /*generate stop*/
  I2C_GenerateSTOP(I2C1, ENABLE);
  /*stop bit flag*/
  //while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}

u8 Receive(u8 Address, u8 Register)
{
  u8 register_value;
  /* variables to store temporary values in */
  /*left align address*/
  Address = Address<<1;
  /*re-enable ACK bit incase it was disabled last call*/
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  /* Test on BUSY Flag */
  while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
  
  /* Enable the I2C peripheral */
  /*======================================================*/
  I2C_GenerateSTART(I2C1, ENABLE);
  /* Test on start flag */
  //while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_SB));
  
  /* Send device address for write */
  I2C_Send7bitAddress(I2C1, Address,I2C_Direction_Transmitter);
  /* Test on master Flag */
  //while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Send the device's internal address to write to */
  I2C_SendData(I2C1,Register);
  /* Test on TXE FLag (data sent) */
  //while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE));
  
  /*=====================================================*/
  /* Send START condition a second time (Re-Start) */
  I2C_GenerateSTART(I2C1, ENABLE);
  /* Test start flag */
  //while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_SB));
  
  /* Send address for read */
  I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Receiver);
  /* Test Receive mode Flag */
  //while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
  /* load in all 6 registers */
  //while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  register_value = I2C_ReceiveData(I2C1);
  //while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  
  /*enable NACK bit */
  //I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
  I2C_AcknowledgeConfig(I2C1, DISABLE);
  
  /* Send STOP Condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF)); // stop bit flag
  
  return register_value;
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
       
  /* Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the device
     immunity against EMI/EMC *************************************************/
  /*RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, DISABLE);*/
  
  /* Initialize Leds LD3 and LD4 mounted on STM32VLDISCOVERY board */
  STM32vldiscovery_LEDInit(LED3);
  STM32vldiscovery_LEDInit(LED4);
  
  I2C_Setup();
    
  uint8_t zacceleration = 0;
  
  STM32vldiscovery_LEDOn(LED3);
  
  init_sensor();
  
  while (1)
  {
    /*
    // send start sequence
    I2C_GenerateSTART(I2C1, ENABLE);
    // send device address and LSB low (write mode)
    I2C_Send7bitAddress(I2C1, mpu6050address, I2C_Direction_Transmitter);
    // send address of internal register of the slave device
    I2C_SendData(I2C1, 0x6B);
    // resend start sequence
    I2C_GenerateSTART(I2C1, ENABLE);
    // resend device address and LSB high (read mode)
    I2C_Send7bitAddress(I2C1, mpu6050address, I2C_Direction_Receiver);
    // read the data from the device
    zacceleration = I2C_ReceiveData(I2C1);
    // send stop sequence
    I2C_GenerateSTOP(I2C1, ENABLE);
    */
    zacceleration = Receive(mpu6050address, 0x6B);

    Delay(0x6000);
    
    if(zacceleration > 200){
      STM32vldiscovery_LEDOn(LED4); 
      STM32vldiscovery_LEDOff(LED3); 
    }
    
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
