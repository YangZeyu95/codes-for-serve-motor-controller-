/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�MK66FX�����˿���ƽ̨���ذ�
����    д��CHIUSIR
����    ע��
������汾��V1.0
�������¡�2016��08��20��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
���������䡿chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#ifndef __LED_H__
#define __LED_H__

//����ģ���
typedef enum
{
    LED0=0,
    LED1=1,
    LED2=2,
    LED3=3,
    LEDALL=4,//ȫ���ĸ�   
} LEDn_e;

typedef enum
{
    ON=0,  //��
    OFF=1, //��
    RVS=2, //��ת  
}LEDs_e;


/*********************** UART���ܺ��� **************************/
//��ʼ��
extern void LED_Init(void);
extern void LED_Ctrl(LEDn_e ledno, LEDs_e sta);
/********************************************************************/

#endif 
