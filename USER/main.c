#include "stm32f10x.h"
#include "USART_1.h"
#include "Delay.h"
#include "IIC.h"
#include "FDC2214.h"
#include "OLED_I2C.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_beep.h"
#include <stdio.h>
#define LEN 6
#define SHIFT 2
static double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
static double szlb(double _data);
static double nihe_result(double data);
static double calc(double data);
double lb_Value;
double min_value=39004;//39004
double max_value=71723;//71723
uint32_t date0, date1, date2, date3;
uint32_t fre0, fre1, fre2, fre3;
uint8_t pdata;
uint8_t pmax,pmin;
unsigned int datas[LEN];
extern const unsigned char BMP1[];
static double x_last;
static double p_last;

int main(void)
{
	  unsigned char i;
	  uint8_t paper_Num;
	  uint8_t cishu=2;
    Delay_Init();
    USART1_Init(115200);
    IIC_Init();
    FDC2214_Init();
	  Key_GPIO_Config();
    I2C_Configuration();//Ӳ��IIC
	  OLED_Init();
	  LED_GPIO_Config();
	  BEEP_GPIO_Config();	
//	  OLED_Fill(0xFF);//ȫ������
//		Delay_ms(1000);
		OLED_Fill(0x00);//ȫ����
	  Delay_ms(1000);
//		  for(i=0;i<4;i++)
//		 {
//			OLED_ShowCN(22+i*16,0,i);									//������ʾ����
//		 }
//		 OLED_ShowStr(0,3,(unsigned char*)"State:",2);				//����6*8�ַ�
//		 OLED_ShowStr(0,5,(unsigned char*)"Num:",2);	
//     OLED_Show_Num(30,5,paper_Num);		 
//		 OLED_ShowStr(30,5,(unsigned char*)"15",2);	
		 OLED_ShowStr(30,3,(unsigned char*)"Welcome!",2);	
		
		 while(cishu>0)
		 {
			if( Key_Scan(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON  )
			{
        LED1_ON;
				for(i=0;i<200;i++)
				{
					FDC2214_GetChannelData(FDC2214_Channel_0, &date0);   //	
				//fre0 = FDC2214_CalculateFrequency(FDC2214_Channel_0, date0);
					lb_Value = KalmanFilter((double)date0,0.05,5);
					//printf("%8d,%8d\n",date0,(uint32_t)lb_Value);
					 Delay_ms(5);
				}
				x_last = 0;
				p_last = 0;
				
			if(cishu==2)
			{
			   min_value = lb_Value/100;
				printf("min_value1:%f\n",min_value);
       OLED_CLS();
	      Delay_ms(100);
			  OLED_ShowStr(10,2,(unsigned char*)"Correction:1",2);
				OLED_ShowStr(0,5,(unsigned char*)"Value1:",2);	
			 OLED_Show_Num(60,5,(unsigned int)(min_value/100));
				LED1_OFF;
			}
			if(cishu==1)
			{
			   max_value = lb_Value/100;	
				printf("max_value2:%f\n",max_value);
    	 OLED_CLS();
				Delay_ms(100);
			OLED_ShowStr(10,2,(unsigned char*)"Correction:2",2);
					OLED_ShowStr(0,5,(unsigned char*)"Value2:",2);	
			 OLED_Show_Num(60,5,(unsigned int)(max_value/100));
				LED1_OFF;
			}
				cishu = cishu-1;
			}
     }			
			Delay_ms(5000);
		 OLED_CLS();
		  for(i=0;i<4;i++)
		 {
			OLED_ShowCN(30+i*16,0,i);									//������ʾ����
		 }
		 OLED_ShowStr(0,3,(unsigned char*)"State:",2);				//����6*8�ַ�
		 OLED_ShowStr(0,5,(unsigned char*)"Num:",2);	
		 
		 
    while(1)
    {
		if( Key_Scan(KEY2_GPIO_PORT,KEY2_GPIO_PIN) == KEY_ON  )
		{ 
			LED1_ON;
			for(i=0;i<200;i++)
			{
				FDC2214_GetChannelData(FDC2214_Channel_0, &date0);   //	
		  //fre0 = FDC2214_CalculateFrequency(FDC2214_Channel_0, date0);
		    lb_Value = KalmanFilter((double)date0,0.05,5);
				//printf("%8d,%8d\n",date0,(uint32_t)lb_Value);
				 Delay_ms(5);
			}
				x_last = 0;
				p_last = 0;
			if(date0<10000000&&date0>1000000)
			{
				paper_Num = (int)(calc(lb_Value)+0.5);
				printf("num:%d",paper_Num);
				//printf("\n%8d,%8d\n",date0,(uint32_t)lb_Value);
//      FDC2214_GetChannelData(FDC2214_Channel_0, &date0);   //	
//		  //fre0 = FDC2214_CalculateFrequency(FDC2214_Channel_0, date0);
//		  lb_Value = KalmanFilter((double)date0,0.05,5);
//			paper_Num =(int)nihe_result(lb_Value);
      
		  	OLED_CLS();
				for(i=0;i<4;i++)
			 {
				OLED_ShowCN(30+i*16,0,i);									//������ʾ����
			 }
//		 Delay_ms(1000);
			 OLED_ShowStr(0,3,(unsigned char*)"State:OK",2);				//����6*8�ַ�
			 OLED_ShowStr(0,5,(unsigned char*)"Num:",2);	
			 OLED_Show_Num(30,5,(unsigned int)paper_Num);	
			 BEEP(ON);
			 Delay_ms(5000);
			 BEEP(OFF);
			 LED1_OFF;
			 //printf("%d\n",(unsigned int)lb_Value);		
			}
			else
			{
			  OLED_CLS();
				for(i=0;i<4;i++)
			 {
				OLED_ShowCN(30+i*16,0,i);									//������ʾ����
			 }
//		 Delay_ms(1000);
			 OLED_ShowStr(0,3,(unsigned char*)"State:A-B ERROR!",2);				//����6*8�ַ�
			 OLED_ShowStr(0,5,(unsigned char*)"Num:",2);	
			 //OLED_Show_Num(30,5,(unsigned int)paper_Num);
			 BEEP(ON);
			 Delay_ms(5000);
			 BEEP(OFF);
			 LED1_OFF;
			}
		 }	
		 
			//key
//		if( Key_Scan(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON  )
//		{
//			printf("1");
//			Delay_ms(500);
//		} 

//		if( Key_Scan(KEY2_GPIO_PORT,KEY2_GPIO_PIN) == KEY_ON  )
//		{
//			printf("2");
//			Delay_ms(500);
//		}	


     //led
//		 LED1_ON;			  // ��PB5
//	 	 Delay_ms(20000);
//		 LED1_OFF;
//		 Delay_ms(20000);

     //beep
//		 BEEP(ON);
//     Delay_ms(20000);
//     BEEP(OFF);		
//		 Delay_ms(20000);
     
    }
}
static double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{

    double R = MeasureNoise_R;
    double Q = ProcessNiose_Q;

    extern double x_last;
    double x_mid = x_last;
    double x_now;

    extern double p_last;
    double p_mid ;
    double p_now;

    double kg;

    x_mid=x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����

    /*
     *  �������˲��������Ҫ��ʽ
     */
    kg=p_mid/(p_mid+R);                 //kgΪkalman filter��R Ϊ����
    x_now=x_mid+kg*(ResrcData-x_mid);   //���Ƴ�������ֵ
    p_now=(1-kg)*p_mid;                 //����ֵ��Ӧ��covariance
    p_last = p_now;                     //����covariance ֵ
    x_last = x_now;                     //����ϵͳ״ֵ̬

    return x_now;

}

static double szlb(double _data)
{ 
/****************************/
/* �ڵ��ô��ӳ���ǰ����� */
/* pdata,datas[]����,  */
/* pmax,pmin���г�ʼ��  */
/****************************/
uint8_t i;
unsigned int average=0;  //���㣬��������ƽ��ֵ
pdata=(pdata+1)%LEN; //ָ���±���0��LEN-1�ϻ���
datas[pdata]=_data;  //�����������ݴ������ݱ���
for(i=0;i<LEN;i++)
   average=average+datas[i]; //�����������ܺ�
/*******ȥ������Ϊ�����������******/
if(_data>datas[pmax])
      pmax=pdata;   //�õ����ֵ��ָ��
else if(_data<datas[pmin])
      pmin=pdata;   //�õ���Сֵ��ָ��
if(pdata==pmax)   //�����ǰ����ֵ�����뵱ǰ���ֵ��λ��ʱ
{      //�����Ϸ����������У����������λ���в��Ҽ�ֵ
      for(i=0;i<LEN;i++)
          if(datas[i]>datas[pmax])
          pmax=i;
}
else if(pdata==pmin)//�����ǰ����ֵ�����뵱ǰ���ֵ��λ��ʱ
{      //�����Ϸ����������У����������λ���в��Ҽ�ֵ
     for(i=0;i<LEN;i++)
          if(datas[i]<datas[pmin])
           pmin=i;
}
average=average-datas[pmax]-datas[pmin];//��ȥ����
return (average>>SHIFT);    //������ƽ��ֵ
}


static double nihe_result(double data)
{
      double result;
	    data = (unsigned int)data/100;
			data = (double)(data - 54828)/(60711-54828);
			result = 7.004*data*data+6.675*data+1.2;
	    return result;
}

static double calc(double data)
{
		  extern double min_value;//39004
	    extern double max_value;//71723
      double min_value31 = max_value;
	    double max_value31 = 73497;
      double result;
	    double temp1;
	    double temp2;
			printf("damin:%f\n",min_value);
	    data = data/100;
	    printf("data:%f\n",data);
			printf("damax:%f\n",max_value);
	    if(data-min_value31>200) 
			{ 
				data = (data - min_value31)/(max_value31-min_value31);
				 result = 22.71*data*data+0.5014*data+33.27;//31.27
			}
			else
			{
					if((data-min_value)<0)
					{
						data = min_value;
						printf("error:%f",data);
					}
					data = (data - min_value)/(max_value-min_value);
					printf("datagy:%f\n",data);
					temp1 = 8.172*data*data+0.8456*data+1.022;
					temp2 = 1450*data*data*data-3389*data*data+2664*data-695.7;
					if(temp1>temp2)
					{
						 result = temp1;
					}
					else
					{
						 result = temp2;
					}
					printf("result:%f\n",result);
			}
	     
	    return result;
}
