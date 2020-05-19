#include "BSP.h"

BitAction Add_Sub_Flag = Bit_RESET;
BitAction Calibrate_Flag = Bit_RESET;
extern BitAction Read_Add_Sub_FLag;
u16 Add_Sub_Cnt = 0;
u16 Pvd_Cnt = 0;  //PVD状态上电计时
u8 pvd_temp[4];

void TIM2_IRQHandler(void)
{
    OS_ERR Err;
    OSIntEnter();   

    
    if (RESET != TIM_GetITStatus(TIM2, TIM_IT_Update))
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        Pvd_Cnt++;        
            
        if( (Pvd_Cnt > 50) && (RunVar.Pvd_Flag) )
        { 
          pvd_temp[0] = 0;
          RunVar.Pvd_Flag = 0;
          ProductPara.bFlashWEn = FLASH_WR_ENABLE;//Flash写使能
          I2C1_WriteNBytes(EEPROM_ADDRESS, PVD_FLAG, 1, pvd_temp );
          ProductPara.bFlashWEn = FLASH_WR_DISABLE;//Flash写禁止
        }  
        
        
        if(PCap_Res_Stau() & 0x100000)                                          //检查PCap的采集状态
        {
            RunVar.RawCap = PCap_Res_Value();                                   //读取采集PCap值
            PCap_MEASURE();                                                     //设置PCap模式        
            
            OSTaskSemPost(&AppFilterTCB, OS_OPT_POST_NONE, &Err);               //发送消息到任务
        }
        if(Read_Add_Sub_FLag)                                                   //数据保持1.6秒(终端会在读取一次数据不成功时1秒内再次读取，1.6s保证能够读取2次)
        {
            if(++Add_Sub_Cnt >= 16)                                     
            {
                Read_Add_Sub_FLag = Bit_RESET;                                  
                Add_Sub_Flag = Bit_RESET;
                ProductPara.AddOil = 0;
                ProductPara.SubOil = 0;
            }
        }	
    }
    OSIntExit();   
}

void Timer2_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = 999;                                    //TIM2定时100ms采集PCap值
    TIM_TimeBaseInitStruct.TIM_Prescaler = 4799;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
    NVIC_Init(&NVIC_InitStruct);

    BSP_IntVectSet(BSP_INT_ID_TIM2, TIM2_IRQHandler);

    TIM_SetCounter(TIM2, 0);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}


void TIM3_PWM_INIT(void)  
{
  
        GPIO_InitTypeDef GPIO_InitStructure;
  
                 /* 使能AHB时钟 */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);

        /* 使能APB2时钟 */
//        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

        /* 使能APB1时钟 */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
        
        
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;                          //LED引脚
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                       //复用模式
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //高速输出
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                     //推完输出
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                       //上拉
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);               //复用配置
}

void TIM3_CH2_PWM(uint32_t arr) //uint16_t Dutycycle)  //移植于：F:\我的资料\F0第一阶段软件工程\STM32F0xx_TIM输出PWM配置详细过程
{
        uint16_t tim3_period;
        uint16_t tim3_pulse; 
      
      
//        GPIO_InitTypeDef GPIO_InitStructure;
        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        TIM_OCInitTypeDef  TIM_OCInitStructure;
        
//                /* 使能AHB时钟 */
//        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);
//
//        /* 使能APB2时钟 */
////        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//
//        /* 使能APB1时钟 */
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//        
//        
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;                          //LED引脚
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                       //复用模式
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //高速输出
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                     //推完输出
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                       //上拉
//        GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);               //复用配置
        



       // tim3_period = (uint16_t)(TIM3_COUNTER_CLOCK/Freq - 1);             //计算出计数周期(决定输出的频率)  4799
        //tim3_pulse  = (tim3_period + 1)*Dutycycle / 100;                   //计算出脉宽值(决定PWM占空比)

        //tim3_pulse  = (tim3_period + 1)* 50 / 100;                   //计算出脉宽值(决定PWM占空比)
          
        /* TIM3时基单元配置 */
        TIM_TimeBaseStructure.TIM_Prescaler = 47;        //预分频值  100K
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //向上计数模式
        TIM_TimeBaseStructure.TIM_Period = arr;                    //定时周期(自动从装载寄存器ARR的值)
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //时钟分频因子
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

        /* TIM3通道2:PWM1模式配置 */
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;//TIM_OCMode_PWM1;                   //输出PWM1模式
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;       //使能输出
        TIM_OCInitStructure.TIM_Pulse = arr/2;                         //脉宽值
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCMode_PWM2;//TIM_OCPolarity_High;           //输出极性
        TIM_OC2Init(TIM3, &TIM_OCInitStructure);

        TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
        TIM_ARRPreloadConfig(TIM3, ENABLE);
        TIM_Cmd(TIM3, ENABLE);  
          
}


void Pcap_INTN(void)
{
    EXTI_InitTypeDef   EXTI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    
    BSP_IntVectSet(BSP_INT_ID_EXTI4_15, EXTI4_15_IRQHandler);
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource10);

    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Capture_TimerInit(void)
{
    Timer2_Init();
}



u16 CalcuFloaterDACode(u16 usRate, FloaterCalibTypeDef *pFcal)
{
    u32 i = 0;
    u32 head, tail;
    u32 u32Temp;

    for(i = 0; i < 8; i++)
    {
        if(usRate <= pFcal->Scale[i])
            break;
    }

    if(i == 0)  
    {
        return pFcal->DACode[0];
    }
    else if(i == 8) 
    {
        head = 6;
        tail = 7;
    }
    else
    {
        head = i - 1;
        tail = i;
    }

    /*
       usRate - pFcal->Scale[head]      pFcal->Scale[tail]  - pFcal->Scale[head]
      ---------------------------- = -------------------------------------------
        x - pFcal->DACode[head]         pFcal->DACode[tail] - pFcal->DACode[head]
       */

    u32Temp = (usRate - pFcal->Scale[head]);
    u32Temp *= (pFcal->DACode[tail] - pFcal->DACode[head]);
    u32Temp /= (pFcal->Scale[tail]  - pFcal->Scale[head]);
    u32Temp += pFcal->DACode[head];

    return((u16)u32Temp);
}

BitAction IsNeetResetPcap(void)
{
    static unsigned char ucRawCapEqualCnt = 0;
    BitAction bErrHappen = Bit_RESET;

    if(RunVar.RawCapBak == 0xffffffff)
    {
        RunVar.RawCapBak = RunVar.RawCap + 1;
    }

    if(RunVar.RawCapBak != RunVar.RawCap)
    {
        ucRawCapEqualCnt = 0;
    }
    else
    {
        ucRawCapEqualCnt += 1;
    }

    RunVar.RawCapBak = RunVar.RawCap;

    if(ucRawCapEqualCnt >= 10)
    {
        ucRawCapEqualCnt = 0;
        bErrHappen = Bit_SET;
    }
    return bErrHappen;
}


#define ADTIMEINTERVAL 100

u16 DAOutPutStabilised(u16 usRate, u16 usDACOutput)
{
    static u16 usBaseRate = 0xffff;
    static u16 usDABak;
    static u8 ucDirBak = 0;     //0表示相等，1表示变小，2表示增大
    static u16 usLargeThan5PercCnt = 0;
    static u16 usLargeThan1PercCnt = 0;

    u16 usRateDelta;
    u8 ucDir;

    //第一次运行的时候获取一个油量比率
    if (usBaseRate > 101)    //由于usRate是浮点转换过来，有误差，所以增加1
    {
        usBaseRate = usRate;
        usDABak = usDACOutput;
        ucDirBak = 0;
        usLargeThan5PercCnt = 0;
        usLargeThan1PercCnt = 0;
        
        return usDABak;
    }

    if(usBaseRate == usRate)
    {
        ucDirBak = 0;
        usLargeThan5PercCnt = 0;
        usLargeThan1PercCnt = 0;
        
        return usDABak;
    }
    else if(usBaseRate > usRate)
    {
        ucDir = 1;
        usRateDelta = usBaseRate - usRate;
    }
    else
    {
        ucDir = 2;
        usRateDelta = usRate - usBaseRate;
    }

    //当方向改变，不会改变输出，也不会改变判断的基准
    if(ucDir != ucDirBak)
    {
        ucDirBak = ucDir;
        usLargeThan5PercCnt = 0;
        usLargeThan1PercCnt = 0;
        
        return usDABak;
    }
    else
    {
        usLargeThan5PercCnt = (usRateDelta > 5) ? (usLargeThan5PercCnt + 1) : 0;

        usLargeThan1PercCnt = (usRateDelta > 1) ? (usLargeThan1PercCnt + 1) : 0;

        if((usLargeThan1PercCnt > (5000/ADTIMEINTERVAL)) || (usLargeThan5PercCnt > (6000/ADTIMEINTERVAL)))
        {
            usBaseRate = usRate;
            usDABak = usDACOutput;
            //ucDirBak = 0;
            usLargeThan5PercCnt = 0;
            usLargeThan1PercCnt = 0;
            
            return usDACOutput;
        }
        else
        {
            return usDABak;
        }
    }
}




/*
u8 ALGO_BubbleOrderFilter(u32 *pWDataBuf, u8 wLen, u8 wHeadLen, u8 wTailLen, u32 *pAverage)
{
  u8 i;
  u8 j;
  u32 wTemp;
  u32 u32FilterBuf[100];
    
  if (wLen < (wHeadLen + wTailLen))
  {
    return (0);
  }

  for(i = 0; i < wLen; i++)
  {
    u32FilterBuf[i] = *(pWDataBuf + i);
  }
  
  for (i = 0; i < wTailLen; i++)
  {
    for (j = 0; j < wLen - i - 1; j++)
    {
      if(u32FilterBuf[j] > u32FilterBuf[j + 1])
      {
        wTemp = u32FilterBuf[j];
        u32FilterBuf[j] = u32FilterBuf[j + 1];
        u32FilterBuf[j + 1] = wTemp;
      }
    }
  }

  for (i = 0; i < wHeadLen; i++)
  {
    for (j = wLen - wTailLen - 1; j > i; j--)
    {
      if (u32FilterBuf[j - 1] > u32FilterBuf[j])
      {
        wTemp = u32FilterBuf[j - 1];
        u32FilterBuf[j - 1] = u32FilterBuf[j];
        u32FilterBuf[j] = wTemp;
      }
    }
  }
    
  wTemp = 0;
  for(i = wHeadLen; i < wLen - wTailLen; i++)//dsdfsdf
  {
    wTemp += u32FilterBuf[i];
  }
    
  *pAverage = wTemp/(wLen - wHeadLen - wTailLen);
    
  return (1);
}
*/


u32 u60sFilter[60];                                                             //60秒滤波专用                
u32 u60sFilterBak[60];                                                          //60秒滤波复制数组                
EverySecFilTypeDef SecFilStr;

FlagStatus Get_EverySecPcap(void)                                               //获得每秒的电容值
{
    u8 i;
    
    RunVar.CapFromPCap = RunVar.RawCap;                                         //获得读取的电容值
  
    if(RunVar.CapFromPCap > ProductPara.CapMax)                                 //读取的电容值在标定的量程内
    {
        RunVar.CapFromPCap = ProductPara.CapMax;
    }
    else if(RunVar.CapFromPCap < ProductPara.CapMin)
    {
        RunVar.CapFromPCap = ProductPara.CapMin;  
    }
    
    if(Calibrate_Flag == Bit_SET)                                               //下发了标定邮箱油量数组    油箱标定数据已写入                     
    {
        SecFilStr.FilterStart = Bit_RESET;                                      //发了标定数组要重新初始化数组 
        Calibrate_Flag = Bit_RESET;
    }
    
    if(SecFilStr.FilterStart == Bit_RESET)                                      //初始状态填满数组                                   
    {
        SecFilStr.Ms100_Cycle = 0;
        SecFilStr.EverySecCap = 0;
        for(i = 0; i < sizeof(SecFilStr.FilArray)/sizeof(SecFilStr.FilArray[0]); i++)
        {
            SecFilStr.FilArray[i] = RunVar.CapFromPCap;
        }
        for(i = 0; i < sizeof(UserParam.HFil)/sizeof(UserParam.HFil[0]); i++)
        {
            UserParam.HFil[i] = RunVar.CapFromPCap;
        }
        for(i = 0; i < sizeof(UserParam.LFil)/sizeof(UserParam.LFil[0]); i++)
        {
            UserParam.LFil[i] = RunVar.CapFromPCap;
        }
        for(i = 0; i < 60; i++)
        {
            u60sFilter[i] = RunVar.CapFromPCap;
        }
        UserParam.PCap_Filter = RunVar.CapFromPCap;
        SecFilStr.FilterStart = Bit_SET;
        return RESET;
    }
    SecFilStr.FilArray[SecFilStr.Ms100_Cycle++] = RunVar.CapFromPCap;           //填数组
    if(SecFilStr.Ms100_Cycle >= 10)                                             //填满了，去头尾求均值
    {
        SecFilStr.EverySecCap = GetDelExtremeAndAverage(SecFilStr.FilArray,SecFilStr.Ms100_Cycle,SecFilStr.Ms100_Cycle/3,SecFilStr.Ms100_Cycle/3);
        SecFilStr.Ms100_Cycle = 0;
        return SET;                                                            //获得每秒的电容值
    }
    return RESET;
}



#include "string.h"
#define FILTER_LEN      3                                                       //计算最小二乘法数据个数                                                   
//#define MULTIPLE        20                                                      //计算加漏油数组系数倍数
//#define CAL_LEN         FILTER_LEN * MULTIPLE                                   //1分钟滤波数组，分为10组
//#define CAL_GRO         CAL_LEN / 6                                             //1分钟分为10组，去最大或最小作为起始值
//#define CAL_CHN         CAL_LEN / CAL_GRO

int xAxis[FILTER_LEN]={0,1,2};                                                  //每秒X轴的值，间隔相同，所以取间隔相同的数据作为X轴值就可以了                    
int yAxis[FILTER_LEN];                                                          //AD值Y轴数据

//u32 STA_END[CAL_GRO];                                                           //把加漏油之前/后的数据采集1分钟，分10段，得出开始和结束值
//u32 CalSted[CAL_LEN];                                                           //计算加漏油起始和结束值数组
//u32 CalStedBak[CAL_LEN];                                                      //计算加漏油起始和结束值数组备份
//u32 CalStedBakChn[CAL_CHN];
BitAction InitArrayFlag = Bit_RESET;                                            //初始化数组标志
BitAction Add_Finish_Flag = Bit_RESET;                                          //加油完成标志        
BitAction Sub_Finish_Flag = Bit_RESET;                                          //漏油完成标志
BitAction Add_Sub_Start_Flag = Bit_RESET;                                       //加漏油起始标志                                     
u8 UpCnt,DoCnt,WaCnt,TimeCnt,FinishTimeCnt;                                     //液位上升下降时间计数
u32 MinAd,MaxAd,AddAd;                                                          
int OilAddValue,OilSubValue;


//加漏油行进状态
typedef enum
{
    IDLE = 0,
    ADD,
    SUB
}OilTypedef;


//加漏油起始状态
typedef enum
{
    STOP = 0,
    STRT,
}Add_SubTypedef;


OilTypedef OilState;
Add_SubTypedef AddState,SubState;
u8 LFilCnt;

//取三个值计算最小二乘法，得到油位的变化率，由此判断加漏油情况
void Judge_Add_Sub_Oil(u32 SecPCap)
{
    u8 i;
    u16 ADValue, AddLiqThr, SubLiqThr;
    u32 RealSecPCap;
    float Rate;
    int yK;
    //int BlindAreaLen;                                                         //盲区长度

    if(ProductPara.CompenEn == 1)
    {                                                                           //获得每秒补偿的电容值
        RealSecPCap = (u32)(SecPCap * UserParam.UserFlowK + UserParam.UserFlowB * 100 - 100);
    }
    else
    {
        RealSecPCap = SecPCap;                                                  //不补偿的电容值
    }
                                                                                //计算液位占满量程的百分比
    Rate = (RealSecPCap - ProductPara.CapMin) * 1.0f / ProductPara.CapRange;
    if(Rate > 1.0f)
    {
        Rate = 1.0f;
    }
    ADValue = (u32)(Rate * ProductPara.Range);                                  //油杆量程作为AD的基准值(量程单位0.1mm)

    if(InitArrayFlag == Bit_RESET)
    {
        InitArrayFlag = Bit_SET;
        for(i = 0; i < FILTER_LEN; i++)
        {
            yAxis[i] = ADValue;
        }
        /*for(i = 0; i < CAL_LEN; i++)
        {
            CalSted[i] = ADValue;
        }*/
    }
 
    //memcpy((u8*)CalSted, (u8*)(CalSted + 1), (CAL_LEN - 1) * 4);
    //*(CalSted + CAL_LEN - 1) = ADValue;      

    memcpy((u8*)yAxis, (u8*)(yAxis+1), (FILTER_LEN-1) * 4);
    *(yAxis+FILTER_LEN-1) = ADValue;                                            //FIFO更新Y轴AD数据   
    yK = get_slop(xAxis, yAxis, FILTER_LEN);                                    //最小二乘法计算三个值的斜率（即每秒液位升高的高度*1000mm） 
   
    ProductPara.BottomArea = (ProductPara.BoxPara[0] - 2 * ProductPara.BoxPara[3])\
                                * (ProductPara.BoxPara[1] - 2*ProductPara.BoxPara[3]) / 1000000.0f;//平方米
    ProductPara.AddMapHeight = ProductPara.AddLiqCnt * 100.0f / ProductPara.AddLiqTime / ProductPara.BottomArea;//设置的阈值反映到的高度*1000mm
    ProductPara.SubMapHeight = ProductPara.SubLiqCnt * 100.0f / ProductPara.SubLiqTime / ProductPara.BottomArea;
    
    AddLiqThr = ProductPara.AddLiqCnt;            
    SubLiqThr = ProductPara.SubLiqCnt; 
    
    if(RunVar.AccStatus == 0x02)                                                //ACC开启
    {
        if(RunVar.CarSpeed <= 5)                                                //判断速度，小于等于5KM/H  
        {
            AddLiqThr = (u16)(1.5f * ProductPara.AddLiqCnt);                    //油量上升阈值为设定加油阈值的1.5倍
            SubLiqThr = (u16)(1.5f * ProductPara.SubLiqCnt);                    //油量下降阈值为设定加油阈值的1.5倍
        }
        else                                                                    //大于5KM/H  22倍
        {
            AddLiqThr = (u16)(2.2f * ProductPara.AddLiqCnt);                    //大于5km/h，说明车在运动，提高阈值
            SubLiqThr = (u16)(2.2f * ProductPara.SubLiqCnt);
        } 
    }
    if(RunVar.CarSpeed > 8)
    {
        AddLiqThr = (u16)(2.2f * ProductPara.AddLiqCnt);                        //大于5km/h，说明车在运动，提高阈值
        SubLiqThr = (u16)(2.2f * ProductPara.SubLiqCnt);        
    }
         
    if(++TimeCnt <= 20)                                                         //20秒内
    {
        if((yK > 0) && (yK > (u32)ProductPara.AddMapHeight))                    //斜率大于0且大于阈值斜率
        {
            UpCnt++;                                                            //液位上升次数+1
        }
        else if((yK < 0) && (abs(yK) > (u32)ProductPara.SubMapHeight))
        {
            DoCnt++;                                                            //液位下降次数+1
        }
        else                                                                    
        {
            WaCnt++;                                                            //波动次数                
        }
    }
    if(TimeCnt == 20)                                                           //20秒一个周期
    {
        TimeCnt = 0;
        if(UpCnt >= 8 * (DoCnt + WaCnt))                                        
        {
            if(OilState != ADD)
            {
                OilState = ADD;                                                 //正在加油
                if(AddState == STOP)
                {
                    SubState = STOP;
                    AddState = STRT;
                    if(Add_Sub_Start_Flag == Bit_RESET)                 
                    {
                        /*for(i = 0; i < CAL_GRO; i++)
                        {                                                       //对数据进行滤波                
                            memcpy((u8*)CalStedBakChn, (u8*)(CalSted + CAL_CHN * i), CAL_CHN * 4);
                            STA_END[i] = GetDelExtremeAndAverage(CalStedBakChn, CAL_CHN, CAL_CHN / 3, CAL_CHN / 3);
                        }
                        MinAd = Get_Min_Max(STA_END, CAL_GRO, 0);               //获得最小值作为加油起点  */
                        MinAd = RunVar.LiquidHeight;
                        MaxAd = 0;
                        Add_Sub_Start_Flag = Bit_SET;                           //开始加漏油标志置位
                    }
                }
            }
        }
        else if(DoCnt >= 8 * (UpCnt + WaCnt))        
        {
            if(OilState != SUB)
            {
                OilState = SUB;                                                 //正在漏油
                if(SubState == STOP)
                {
                    AddState = STOP;
                    SubState = STRT;
                    if(Add_Sub_Start_Flag == Bit_RESET)
                    {
                        /*for(i = 0; i < CAL_GRO; i++)
                        {
                            memcpy((u8*)CalStedBakChn, (u8*)(CalSted + CAL_CHN * i), CAL_CHN * 4);
                            STA_END[i] = GetDelExtremeAndAverage(CalStedBakChn, CAL_CHN, CAL_CHN / 3, CAL_CHN / 3);
                        }
                        MaxAd = Get_Min_Max(STA_END, CAL_GRO, 1);*/
                        MaxAd = RunVar.LiquidHeight;
                        MinAd = 0;
                        Add_Sub_Start_Flag = Bit_SET;
                    }
                }       
            }
        }
        else
        {           
            OilState = IDLE;                                                    //加漏油结束或是只是在震荡
        }
        UpCnt = 0;
        DoCnt = 0;
        WaCnt = 0;                                                              //清0
        if(OilState == IDLE)                                                    //闲置模式下
        {
            FinishTimeCnt++;
            if((AddState == STRT) && (Add_Finish_Flag == Bit_RESET))            //检测到加油
            {            
                /*emcpy((u8*)CalStedBak, (u8*)CalSted, FILTER_LEN * 10 * 4);
                MaxAd = GetDelExtremeAndAverage(CalStedBak, FILTER_LEN * 10,FILTER_LEN * 8, FILTER_LEN);
                BlindAreaLen = ProductPara.BoxPara[2] * 10 + 200 - ProductPara.Range;
                if(BlindAreaLen <= 0)                                           //盲区小于0，则平台输入参数错误
                {
                    BlindAreaLen = 330;                         
                    //盲区固定为33mm
                }       
                OilAddValue = Get_TankOil(MaxAd + BlindAreaLen) - Get_TankOil(MinAd + BlindAreaLen);
                if(OilAddValue > AddLiqThr)
                {
                    ProductPara.AddOil = OilAddValue;                           //加油值
                    Add_Sub_Flag = Bit_SET;                                     //加漏油标志
                }*/
                Add_Finish_Flag = Bit_SET;
                AddState = STOP;
            }
            if((SubState == STRT) && (Sub_Finish_Flag == Bit_RESET))            //检测到漏油
            {
                /*memcpy((u8*)CalStedBak, (u8*)CalSted, FILTER_LEN * 10 * 4);
                MinAd = GetDelExtremeAndAverage(CalStedBak, FILTER_LEN * 10, FILTER_LEN, FILTER_LEN * 8);
                BlindAreaLen = ProductPara.BoxPara[2] * 10 + 200 - ProductPara.Range;
                if(BlindAreaLen <= 0)                                           //盲区小于0，则平台输入参数错误
                {
                    BlindAreaLen = 330;                                         //盲区固定为33mm
                }   
                OilSubValue = Get_TankOil(MaxAd + BlindAreaLen) - Get_TankOil(MinAd + BlindAreaLen);
                if(OilSubValue > SubLiqThr)
                {
                    ProductPara.SubOil = OilSubValue;                           //加油值
                    Add_Sub_Flag = Bit_SET;                                     //加漏油标志
                }*/
                Sub_Finish_Flag = Bit_SET;
                SubState = STOP;              
            }
            //MaxAd = 0;
            //MinAd = 0;
        }
        else
        {
            FinishTimeCnt = 0;
            Add_Finish_Flag = Bit_RESET;
            Sub_Finish_Flag = Bit_RESET;
        }
    } 
    if(FinishTimeCnt >= 6)                                                      //有加漏油120s后
    {
        FinishTimeCnt = 0;
        if((Add_Finish_Flag == Bit_SET) || (Sub_Finish_Flag == Bit_SET))
        {
            /*for(i = 0; i < CAL_GRO; i++)
            {
                memcpy((u8*)CalStedBakChn, (u8*)(CalSted + CAL_CHN * i), CAL_CHN * 4);
                STA_END[i] = GetDelExtremeAndAverage(CalStedBakChn, CAL_CHN, CAL_CHN / 3, CAL_CHN / 3);
            }*/
            //BlindAreaLen = ProductPara.BoxPara[2] * 10 + 200 - ProductPara.SensorLen;
            /*if(BlindAreaLen <= 0)                                               //盲区小于0，则平台输入参数错误
            {
                BlindAreaLen = 330;                                             //盲区固定为33mm
            }*/
            if(Add_Finish_Flag == Bit_SET)
            {
                Add_Finish_Flag = Bit_RESET;
                MaxAd = RunVar.LiquidHeight;                                          //有加油，最高液位设置为滤波值
                //MaxAd = Get_Min_Max(STA_END, CAL_GRO, 1); 
       
                
               OilAddValue = Get_TankOil(MaxAd) - Get_TankOil(MinAd);
                   
                
                
                
                if(OilAddValue > AddLiqThr)
                {
                    ProductPara.AddOil = OilAddValue;                           //加油值
                    Add_Sub_Flag = Bit_SET;                                     //加漏油标志
                }
            }
            else if(Sub_Finish_Flag == Bit_SET)
            {
                Sub_Finish_Flag = Bit_RESET;
                //MinAd = Get_Min_Max(STA_END, CAL_GRO, 0);
                MinAd = RunVar.LiquidHeight;                                    
                OilSubValue = Get_TankOil(MaxAd) - Get_TankOil(MinAd);
                if(OilSubValue > SubLiqThr)
                {
                    ProductPara.SubOil = OilSubValue;                           //加油值
                    Add_Sub_Flag = Bit_SET;                                     //加漏油标志
                }                    
            }
            MaxAd = 0;
            MinAd = 0;  
            Add_Sub_Start_Flag = Bit_RESET;                                     //加漏油开始标志清零
        }
    }
}





void DA_Handle(u16 LiquidAD, float rate)
{
    float smfTemp;
    
    if(ProductPara.LiquitHeightforDAMax > 0.1)
    {
        if(LiquidAD >= ProductPara.LiquitHeightforDAMax)
        {
            smfTemp = 1.0;
        }
        else
        {
            smfTemp = LiquidAD / ProductPara.LiquitHeightforDAMax;
        }
    }
    else
    {
        smfTemp = rate;
    }

    RunVar.DAForFloater = CalcuFloaterDACode((u16)(smfTemp * 100.0), &ProductPara.Floater);
    
    RunVar.DAForFloater = DAOutPutStabilised((u16)(smfTemp * 100.0), RunVar.DAForFloater);
    
    if(Bit_RESET == DAOilDebug.bDADebugEn)
    {
        DA_Write(RunVar.DAForFloater);
    }
    if(Bit_RESET == DAOutDebug.bDADebugEn)
    {
        DA2_Write((u16)(rate * (ProductPara.DAMax - ProductPara.DAMin) + ProductPara.DAMin));
    }    
}


u8 timecnt = 0;
u8 l_flag = 0;
u32 Curr_LiquidHeight = 0;
u32 Last_LiquidHeight = 0;

void Get_Filter_PCap(void)
{
    float rate;
    int difference;
    
    if(Get_EverySecPcap())                                                      //得到每秒数据
    {    
        if(UserParam.FilterLevel == 1)                                          //实时滤波
        {
            UserParam.PCap_Filter = SecFilStr.EverySecCap;                      //滤波值即为每秒的值
        }
        else if(UserParam.FilterLevel == 2)                                     //平滑滤波60s，单独处理
        {
            memcpy((u8*)u60sFilter, (u8*)(u60sFilter + 1), 236);
            *(u60sFilter+59) = SecFilStr.EverySecCap;
            //memset((u8*)u60sFilterBak, 0x00, 720);
            memcpy((u8*)u60sFilterBak, (u8*)u60sFilter, 240);
            UserParam.PCap_Filter = GetDelExtremeAndAverage(u60sFilterBak, 60, 20, 20);          
        }
        else                                                                    //平滑和平稳滤波
        {                                                                       //低滤波值向左FIFO
            memcpy((u8*)UserParam.LFil, (u8*)(UserParam.LFil+1), (UserParam.FilterBufMax-1)*4);
            *(UserParam.LFil+UserParam.FilterBufMax-1) = SecFilStr.EverySecCap; //当前值放入队尾 
            if(++LFilCnt >= UserParam.FilterBufMax)                             //低滤波数组填满
            {
                LFilCnt = 0;
                memcpy((u8*)UserParam.LFilBak, (u8*)UserParam.LFil, UserParam.FilterBufMax*4);//低滤波值放入备份数组
                memcpy((u8*)UserParam.HFil, (u8*)(UserParam.HFil+1), 36);       //高滤波值向左FIFO    新值放入队尾
                *(UserParam.HFil+9) = GetDelExtremeAndAverage(UserParam.LFilBak,UserParam.FilterBufMax,UserParam.FilterBufMax/3,UserParam.FilterBufMax/3);
                memcpy((u8*)UserParam.HFilBak, (u8*)UserParam.HFil, 40);        //放入备份数组
                UserParam.PCap_Filter = GetDelExtremeAndAverage(UserParam.HFilBak,10,3,3);
            }
        }
        rate = (UserParam.PCap_Filter - ProductPara.CapMin) * 1.0f / ProductPara.CapRange;
        if(rate > 1.0f) rate = 1.0f;
        else if(rate < 0.00001) rate = 0;
        RunVar.LiquidAD = (u16)(rate * 65535);                                  //液位高度AD     
        RunVar.LiquidPercent = (u16)(rate * 1000.0f);                           //对量程的百分比
                  
        //Curr_LiquidHeight = (u32)(rate * ProductPara.Range + ProductPara.BoxPara[2] * 10 - ProductPara.SensorLen + 200);//对邮箱底部的液位高度    
        Curr_LiquidHeight = (u32)(rate * ProductPara.Range  );//- ProductPara.SensorLen + 200);//对邮箱底部的液位高度    
        
        if(!l_flag)                                                             //上电执行一次，获得液位初始值
        {
            Last_LiquidHeight = Curr_LiquidHeight;                              //初始化液位高度
            RunVar.LiquidHeight = Curr_LiquidHeight;
            l_flag = 1;
        }
        
        difference = Curr_LiquidHeight - Last_LiquidHeight;                     //当前和上次的差值        
        if(Add_Sub_Start_Flag)                                                  //加漏油状态时实更新数据
        {                     
            RunVar.LiquidHeight = Curr_LiquidHeight;                            //开始加漏油液位高度要设置为实时高度        
            Last_LiquidHeight = Curr_LiquidHeight;
            timecnt = 0;
        }
        else                                                                    //非加漏油状态
        {
            if(timecnt++ >= 5)                                                 //15s液位变化一次
            {
                timecnt = 0;
                if(abs(difference) > 1)                                         //液位变化超过0.1mm
                {
                    if(difference > 0)                                          //只变化0.1mm
                    {
                        RunVar.LiquidHeight += 1;
                    }
                    if(difference < 0)                                          //下降0.4mm（原则是缓慢上升快速下降）
                    {
                        if(difference < -4)
                        {
                            RunVar.LiquidHeight -= 4;
                        }
                        else
                        {
                            RunVar.LiquidHeight += difference;
                        }
                    }
                }        
                Last_LiquidHeight = RunVar.LiquidHeight;
            }
        }       
        RunVar.OilQuantity = Get_TankOil(RunVar.LiquidHeight);                  //查找对应表，计算油箱的油量，要放在加漏油判断前面，因为加漏油判断需要用到这个值
        Judge_Add_Sub_Oil(SecFilStr.EverySecCap);                               //参数为每秒Pcap值、当前液位高度值
        DA_Handle(RunVar.LiquidAD, rate);                                       //DA输出
    }
}

void App_Filter_Task (void *p_arg)
{
    OS_ERR Err; 
    
    uint32_t psc,fre;
      
    BitAction bNeetResetPcap = Bit_RESET;
    Capture_TimerInit();
    DA_Init();
    Last_LiquidHeight = Curr_LiquidHeight;
    
    TIM3_PWM_INIT();
    
    TIM3_CH2_PWM(99);
    
    while(1)
    {
#if IWDG_EN > 0
        IWDG_ReloadCounter();
#endif
        OSTaskSemPend(0, OS_OPT_PEND_BLOCKING, NULL, &Err);
        if (Err == OS_ERR_NONE)
        {
            bNeetResetPcap = IsNeetResetPcap();                                 //是否需要重启PCap
            
            if(Bit_SET == bNeetResetPcap)
            {
                bNeetResetPcap = Bit_RESET;
                resetPcap();                                                    //重启
                RunVar_Init();
                Capture_TimerInit();
                continue;
            }
            Get_Filter_PCap(); 
            
            
             if(Pvd_Cnt % 10 == 0)  //1秒更新一次
        {       
            
          if(RunVar.LiquidHeight <= 3350)      // y = 5910 -2.7164*x 
          {
               fre = (uint32_t )(5910 - 0.2716417910448 * RunVar.LiquidHeight);  //需要的频率  
          }
          else if(( RunVar.LiquidHeight > 3350 )&& (RunVar.LiquidHeight <= 5350) )
          {
               fre = (uint32_t )(6038.5 - 0.31 * RunVar.LiquidHeight);  //需要的频率  
          }
          else if(( RunVar.LiquidHeight > 5350 )&& (RunVar.LiquidHeight <= 7350) )
          {
               fre = (uint32_t)(5824.5 - 0.27 * RunVar.LiquidHeight);  //需要的频率  
          }
          else if(( RunVar.LiquidHeight > 7350 )&& (RunVar.LiquidHeight <= 9350) )
          {
               fre = (uint32_t )(5383.5 - 0.21 * RunVar.LiquidHeight);  //需要的频率  
          }
           else if(( RunVar.LiquidHeight > 9350 )&& (RunVar.LiquidHeight <= 11350) )
          {
              fre = (uint32_t )(6271.75 - 0.305 * RunVar.LiquidHeight);  //需要的频率  
          }
          else
          {     
               fre = 2810;  //需要的频率
          }
          
          
            psc=480000/(fre +  0);         //更改后的分频系数
        
            //TIM_Cmd(TIM3,DISABLE);            
           // TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
            TIM_PrescalerConfig(TIM3,psc-1,TIM_PSCReloadMode_Immediate);
            TIM_Cmd(TIM3,ENABLE);

        }
        }
        
    }
}

