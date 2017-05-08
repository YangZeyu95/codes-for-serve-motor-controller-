/****************************************************************************************************
��ƽ    ̨������K60DNG2���İ�
����    д��CHIUSIR
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2016��09��01��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR7.3
��Target  ��K60DN512ZVLQ10
��busclock��100.000MHz
��pllclock��200.000MHz
=============================================================
�ӿڶ��壺
LED����GPIO
LED0--PTD15
LED1-PTA17
LED2--PTC0 
LED3--PTE26

OLED�ڶ��壺
LCD_DC --PTC19
LCD_RST--PTC18
LCD_SDA--PTC17
LCD_SCL--PTC16
=============================================================
ʵ������
LED0��˸
LED1��˸
LED2��˸
LED3��˸
OLED����ʾ����LOGO��Ȼ����ʾ�����������ܿƼ���beijing��LONG QIU I.S.T.
=============================================================
******************************************************************************************************/
#include "include.h"
#include "math.h"
//int Gray[26];
/*int Kp = 500, Ki = 10, Kd = 50;   //PIDϵ��
int Data = 0, b = 0, c = 0;
signed int Error = 0, LastError = 0, OutPut = 0, Integration = 0, Differential = 0;
uint16 Result = 0,Voltage = 0;*/

/*static unsigned int DecimaltoGray(unsigned int x)       
{          
  return x^(x>>1);       
} 
//������ת������
static unsigned int GraytoDecimal(unsigned int x)
{           
  unsigned int y = x;
  while(x>>=1)
    y ^= x;
  return y;
}
*/

void time_delay_ms(u32 ms)
{
  LPTMR_delay_ms(ms);
}
int Contral[8];
char a[8], State;//б���źŷ�����״̬ 0�ر� 1����
extern int InPut, Kp, Ki1, Kd, Threshold;

//������
void main(void)
{
    u16 duty;
     
   DisableInterrupts;        //�ر����ж�
   PLL_Init(PLL200);         //��ʼ��PLLΪ200M������Ϊ100MHZ  
   //PIT_Init(PIT0,1); 
   GPIO_Init (PORTA, 19, GPO, 0);
   GPIO_Init (PORTA, 17, GPI, 0);
   GPIO_Init (PORTD, 7, GPO, 0);
   


   LCD_Init();               //OLEDģ���ʼ����Ĭ��ʹ��PTB16-19
   LCD_CLS();                //OLED����
   LCD_P6x8Str(0,0,"ERROR:");
   LCD_P6x8Str(0,5,"PosT:");
   LCD_P6x8Str(0,3,"DIR:");
   LCD_P6x8Str(0,4,"Input:");
   LCD_P6x8Str(0,2,"IntT:");
   



   DAC_Init(DAC_0);
   //DAC_Init(DAC_1);
  // Result = (uint16)(1.5 * ((1 << 12) - 1));//���������ѹ    
   //DAC_Out(DAC_0, Result);
   PIT_Init(PIT0,1);
   UART_Init(UART1,115200);


   PIT_Init(PIT1,5);   


   EnableInterrupts;  //�������ж�
   
  while(1)
  {
    UART_Query_Str (UART1, a, 6);
    if(a[0] != 0x00)
    {
    for (int i = 0;i < 8; i++)
      Contral[i] = a[i];
    switch(Contral[0])
    {
    case 0x01: 
      State = 0;
      //DisableInterrupts;    //�ر����ж�
      //DAC_Out(DAC_0, 0x0800);
      break;
    case 0x02:
     // EnableInterrupts;  //�������ж�
      State = 1;
      break;
    case 0x03:
         //PIT_TCTRL(PIT1) |= ( PIT_TCTRL_TEN_SHIFT | PIT_TCTRL_TIE_SHIFT );   //ʹ�� PITn��ʱ��,����PITn�ж�
      if(Contral[1] >= 25)
      {
      State = 0;    
      InPut = Contral[1] * 100 + Contral[2];
      }
      //UART_Put_Char (UART1, InPut);
      break;
     case 0x04:
       Kp = Contral[1];
       //UART_Put_Char (UART1, Kp);
       break;
     case 0x05:
       Ki1 = Contral[1];
       //UART_Put_Char (UART1, Ki1);
       break;
     case 0x06:
       Kd = Contral[1];
       //UART_Put_Char (UART1, Kd);
       break;
     case 0x07:
       Threshold = Contral[1];
       //UART_Put_Char (UART1, Threshold);

       break;  
       case 0x0d:
       //if(Contral[1] == 0x0a)
       //{
         InPut = Contral[2] + Contral[3] + Contral[4]  + Contral[5];
       //}

       break; 
     default:
       break;

    }
    }
    
    for(int i = 0; i < 8; i++)
    {
      Contral[i] = 0;
    }

      
   /* for(int i = 0; i < 26; i++)
    {
      GPIO_Ctrl(PORTA, 19, 0);
      LPTMR_delay_us(8);
      GPIO_Ctrl (PORTA, 19, 1);
      LPTMR_delay_us(7);
      Gray[i] = GPIO_Get(PTA17);
    }*/
    /*for(int i = 0; i < 14; i++) 
      Data = Data + Gray[25-i] * pow(2, i);*/
    
    /*LCD_PrintU16(6,5,GraytoDecimal(Data));
    Error = 5000 - GraytoDecimal(Data); //�������
    LCD_PrintU16(0,0,Error);
    Integration = Integration +Error;
    Differential = (Error - LastError) / 0.005;
    LastError = Error;
    if(Error < 10) Differential = 0;
    
    OutPut = Error * Kp + Integration * Ki + Differential * Kd;
    
    if(OutPut > 2047) OutPut = 2047;
    if(OutPut < -2047) OutPut = -2047;

    Voltage = 0x0800 + OutPut;
    //Result = (uint16)(1.5 * ((1 << 12) - 1));//���������ѹ
    DAC_Out(DAC_0, Voltage);
    Data = 0;
    time_delay_ms(5); 
    */
  }
 
}