/******************** LQ_K60_������ v1.0 ********************
 * �ļ���           ��PIT.C
 * ����             ������PIT����ģʽ
 * ��ע             ���ٷ��������޸�
 * ����             ��2016-09-01
 * ʵ��ƽ̨         ������ k60DN512Z���İ� 
 * ��������         ��IAR 7.3
 * ����             �������� 006
 * �Ա���           ��https://longqiu.taobao.com
 * �������ܳ�����Ⱥ ��202949437
*************************************************************/

#include "include.h"
#include "PIT.h"
#include "math.h"
extern char State;
int Gray[26];
int Kp = 60, Ki1 = 100, Kd = 40,Ki = 0, Threshold = 20;   //PIDϵ��
int Data = 0, b = 0, c = 0, CountOfState = 0, InPut = 5000, Position = 0, IntT = 0;
signed int Error = 0, LastError = 0, OutPut = 0, Differential = 0, Dir = 1;
uint16 Result = 0,Voltage = 0;
double  Integration = 0;
unsigned char DataToSend;
//������ת������
static unsigned int GraytoDecimal(unsigned int x)
{           
  unsigned int y = x;
  while(x>>=1)
    y ^= x;
  return y;
}

//-------------------------------------------------------------------------*
//������: pit_init                                                        
//��  ��: ��ʼ��PIT                                                       
//��  ��: pitn:ģ����PIT0��PIT1��PIT2��PIT3
//        cnt �ж�ʱ�䣬��λ1ms
//��  ��: ��                                                              
//��  ��: pit_init(PIT0,1000); PIT0�жϣ�1000ms����1s����PIT0_interrupt()һ��                                  
//-------------------------------------------------------------------------*
void PIT_Init(PITn pitn, u32 cnt)
{
    //PIT �õ��� Bus Clock ����Ƶ��

    /* ����ʱ��*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                            //ʹ��PITʱ��

    /* PITģ����� PIT Module Control Register (PIT_MCR) */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );      //ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼�������

    /* ��ʱ������ֵ���� Timer Load Value Register (PIT_LDVALn) */
    PIT_LDVAL(pitn)  = cnt*bus_clk*1000;                                          //��������ж�ʱ��

    //��ʱʱ�䵽�˺�TIF �� 1 ��д1��ʱ��ͻ���0
    PIT_Flag_Clear(pitn);                                             //���жϱ�־λ

    /* ��ʱ�����ƼĴ��� Timer Control Register (PIT_TCTRL0) */
    PIT_TCTRL(pitn) |= ( PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK );   //ʹ�� PITn��ʱ��,����PITn�ж�

    enable_irq(pitn + 68);			                                //���������ŵ�IRQ�ж�
}

//-------------------------------------------------------------------------*
//������: PIT0_interrupt                                                        
//��  ��: PIT�жϺ���                                                       
//��  ��: ��
//��  ��: ��                                                              
//��  ��: �ɳ�ʼ���������೤ʱ�����һ��                                  
//-------------------------------------------------------------------------*

void PIT0_Interrupt()
{
  PIT_Flag_Clear(PIT0);       //���жϱ�־λ
  CountOfState++;
  

  if(CountOfState > 1) 
  {
    CountOfState = 0;
  }
  switch (CountOfState){
  case 0:
  for(int i = 0; i < 26; i++)
    {
      GPIO_Ctrl(PORTA, 19, 0);
      LPTMR_delay_us(8);
      GPIO_Ctrl (PORTA, 19, 1);
      LPTMR_delay_us(7);
      Gray[i] = GPIO_Get(PTA17);
    }
   for(int i = 0; i < 14; i++) 
      Data = Data + Gray[25-i] * pow(2, i);
   
   Position = GraytoDecimal(Data);
   
   //Position = InPut;
   DataToSend = Position % 100 + 128;
   UART_Put_Char (UART1, DataToSend);
   DataToSend = Position / 100;
   UART_Put_Char (UART1, DataToSend);
   DataToSend = InPut % 100 + 128;
   UART_Put_Char (UART1, DataToSend);
   DataToSend = InPut / 100;
   UART_Put_Char (UART1, DataToSend);
   
   
   

   //UART_Put_Char (UART1, '\n');
   //UART_Put_Str (UART1, "6");
   LCD_PrintU16(50, 5, Position);
   
    Error = InPut - Position; //�������
    LCD_PrintU16(50,0,Error);
    if(Error <= Threshold && Error >= -Threshold)
    {
      if(Integration <= 100 && Integration >= -100)
      {
        Integration = Integration + Error * 0.005;  
      }
      else if(Integration > 100) Integration = 100;
      else if(Integration < -100) Integration =-100;
      Ki = Ki1;
    }
    else Ki = 0;
    IntT = Integration/1;
    
    LCD_PrintU16(50, 2, IntT);

    break;
  case 1:
    Differential = (Error - LastError) / 0.005;
    LastError = Error;
    //if(Error < 10) Differential = 0;
    
    OutPut = Error * Kp + Integration * Ki + Differential * Kd;
    
    if(OutPut > 2047) OutPut = 2047;
    if(OutPut < -2047) OutPut = -2047;

    Voltage = 0x0800 + OutPut;
    DAC_Out(DAC_0, Voltage);
    Data = 0;
    GPIO_Reverse(PORTD, 7);

    break;
  }
}

void PIT1_Interrupt()
{
  PIT_Flag_Clear(PIT1);       //���жϱ�־λ
  if(State == 1)
   {
   switch(Dir)
   {
     case 1: InPut++;
    break;
    case 0: InPut--;
    break;
   }
   }
  LCD_PrintU16(50,4,InPut);
  //LCD_PrintU16(0,4,a[0]);
  //LCD_PrintU16(0,6,Contral[1]);


  
  if(InPut >= 6000) Dir = 0;
  if(InPut <= 4000) Dir = 1;
  LCD_PrintU16(50,3,Dir);
    LCD_PrintU16(15,6,Kp);
      LCD_PrintU16(50,6,Ki1);
        LCD_PrintU16(80,6,Kd);
        LCD_PrintU16(80,7,Threshold);



  //LCD_PrintU16(0, 1, a[0]);
   
 /*�û�����������*/
}

void PIT2_Interrupt()
{
  PIT_Flag_Clear(PIT2);       //���жϱ�־λ
  /*�û�����������*/
}

void PIT3_Interrupt()
{
  PIT_Flag_Clear(PIT3);       //���жϱ�־λ
  /*�û�����������*/
}