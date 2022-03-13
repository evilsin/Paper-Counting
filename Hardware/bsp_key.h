#ifndef __KEY_H
#define	__KEY_H


#include "stm32f10x.h"

//  ���Ŷ���
#define    KEY1_GPIO_CLK     RCC_APB2Periph_GPIOE
#define    KEY1_GPIO_PORT    GPIOC//	E   
#define    KEY1_GPIO_PIN		 GPIO_Pin_8//4

#define    KEY2_GPIO_CLK     RCC_APB2Periph_GPIOE
#define    KEY2_GPIO_PORT    GPIOC	   //E 
#define    KEY2_GPIO_PIN		  GPIO_Pin_9//3


 /** �������±��ú�
	*  ��������Ϊ�ߵ�ƽ������ KEY_ON=1�� KEY_OFF=0
	*  ����������Ϊ�͵�ƽ���Ѻ����ó�KEY_ON=0 ��KEY_OFF=1 ����
	*/
#define KEY_ON	0
#define KEY_OFF	1

void Key_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);


#endif /* __KEY_H */
