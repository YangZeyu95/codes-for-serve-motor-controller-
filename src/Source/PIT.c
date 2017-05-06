/******************** LQ_K60_函数库 v1.0 ********************
 * 文件名           ：PIT.C
 * 功能             ：设置PIT工作模式
 * 备注             ：官方例程上修改
 * 日期             ：2016-09-01
 * 实验平台         ：龙丘 k60DN512Z核心板 
 * 开发环境         ：IAR 7.3
 * 作者             ：龙丘技术 006
 * 淘宝店           ：https://longqiu.taobao.com
 * 龙丘智能车讨论群 ：202949437
*************************************************************/

#include "include.h"
#include "PIT.h"
#include "math.h"
extern char State;
int Gray[26];
int Kp = 60, Ki1 = 100, Kd = 40,Ki = 0, Threshold = 20;   //PID系数
int Data = 0, b = 0, c = 0, CountOfState = 0, InPut = 5000, Position = 0, IntT = 0;
signed int Error = 0, LastError = 0, OutPut = 0, Differential = 0, Dir = 1;
uint16 Result = 0,Voltage = 0;
double  Integration = 0;
unsigned char DataToSend;
//格雷码转二进制
static unsigned int GraytoDecimal(unsigned int x)
{           
  unsigned int y = x;
  while(x>>=1)
    y ^= x;
  return y;
}

//-------------------------------------------------------------------------*
//函数名: pit_init                                                        
//功  能: 初始化PIT                                                       
//参  数: pitn:模块名PIT0或PIT1或PIT2或PIT3
//        cnt 中断时间，单位1ms
//返  回: 无                                                              
//简  例: pit_init(PIT0,1000); PIT0中断，1000ms，即1s进入PIT0_interrupt()一次                                  
//-------------------------------------------------------------------------*
void PIT_Init(PITn pitn, u32 cnt)
{
    //PIT 用的是 Bus Clock 总线频率

    /* 开启时钟*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                            //使能PIT时钟

    /* PIT模块控制 PIT Module Control Register (PIT_MCR) */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );      //使能PIT定时器时钟 ，调试模式下继续运行

    /* 定时器加载值设置 Timer Load Value Register (PIT_LDVALn) */
    PIT_LDVAL(pitn)  = cnt*bus_clk*1000;                                          //设置溢出中断时间

    //定时时间到了后，TIF 置 1 。写1的时候就会清0
    PIT_Flag_Clear(pitn);                                             //清中断标志位

    /* 定时器控制寄存器 Timer Control Register (PIT_TCTRL0) */
    PIT_TCTRL(pitn) |= ( PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK );   //使能 PITn定时器,并开PITn中断

    enable_irq(pitn + 68);			                                //开接收引脚的IRQ中断
}

//-------------------------------------------------------------------------*
//函数名: PIT0_interrupt                                                        
//功  能: PIT中断函数                                                       
//参  数: 无
//返  回: 无                                                              
//简  例: 由初始化决定，多长时间进入一次                                  
//-------------------------------------------------------------------------*

void PIT0_Interrupt()
{
  PIT_Flag_Clear(PIT0);       //清中断标志位
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
   
    Error = InPut - Position; //计算误差
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
  PIT_Flag_Clear(PIT1);       //清中断标志位
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
   
 /*用户添加所需代码*/
}

void PIT2_Interrupt()
{
  PIT_Flag_Clear(PIT2);       //清中断标志位
  /*用户添加所需代码*/
}

void PIT3_Interrupt()
{
  PIT_Flag_Clear(PIT3);       //清中断标志位
  /*用户添加所需代码*/
}