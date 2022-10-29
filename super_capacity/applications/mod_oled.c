#include "mod_oled.h"

static int Show_Buff_PixRem, Show_Buff_SqareType_Rem;
static int Show_Energy_PixRem, Show_Energy_SqareType_Rem;

//刷新缓冲能量进度条图形
void OLED_FreshBuff(float BuffPercentage, char Sqare_Type)
{
    int Show_Buff_PixNow, Pix_Change_Count;
    //检查输入限幅
    if (BuffPercentage > 0.999f)
    {
        BuffPercentage = 1;
    }
    else if (BuffPercentage < 0.001f)
    {
        BuffPercentage = 0;
    }

    //计算需要显示的进度条长度
    Show_Buff_PixNow = (int)(BuffPercentage * 14);
    Pix_Change_Count = Show_Buff_PixNow - Show_Buff_PixRem; //计算需要修改的像素数

    if (Sqare_Type != Show_Buff_SqareType_Rem)
    {
        //需要转换显示样式
        if (Pix_Change_Count < 0)
        {
            //需要先缩短进度条长度
            OLED_Draw_sqar(110 + Show_Buff_PixNow, SHOW_BLOCK_POSY, SHOW_SQARETYPE_CLEAR, -Pix_Change_Count);
        }
        Pix_Change_Count = Show_Buff_PixNow;
        Show_Buff_PixRem = 0;
        Show_Buff_SqareType_Rem = Sqare_Type;
    }

    if (Pix_Change_Count > 0)
    {
        //需要增加进度条长度
        OLED_Draw_sqar(110 + Show_Buff_PixRem, SHOW_BLOCK_POSY, Sqare_Type, Pix_Change_Count);
    }
    else if (Pix_Change_Count < 0)
    {
        //需要缩短进度条长度
        OLED_Draw_sqar(110 + Show_Buff_PixNow, SHOW_BLOCK_POSY, SHOW_SQARETYPE_CLEAR, -Pix_Change_Count);
    }
    Show_Buff_PixRem = Show_Buff_PixNow; //刷新像素显示位置记录
}

//刷新电容能量进度条图形
void OLED_FreshEnergy(float EnergyPercentage, char Sqare_Type)
{
    int Show_Energy_PixNow,Pix_Change_Count;
    //检查输入限幅
    if (EnergyPercentage > 0.999f)
    {
        EnergyPercentage = 1;
    }
    else if (EnergyPercentage < 0.001f)
    {
        EnergyPercentage = 0;
    }

    //计算需要显示的进度条长度
    Show_Energy_PixNow = (int)(EnergyPercentage * 100);
    Pix_Change_Count = Show_Energy_PixNow - Show_Energy_PixRem;//计算需要修改的像素数

    if (Sqare_Type != Show_Energy_SqareType_Rem)
    {
        //需要转换显示样式
        if (Pix_Change_Count < 0)
        {
            //需要先缩短进度条长度
            OLED_Draw_sqar(4 + Show_Energy_PixNow, SHOW_BLOCK_POSY, SHOW_SQARETYPE_CLEAR, -Pix_Change_Count);
        }
        Pix_Change_Count = Show_Energy_PixNow;
        Show_Energy_PixRem = 0;
        Show_Energy_SqareType_Rem = Sqare_Type;
    }

    if (Pix_Change_Count>0)
    {
        //需要增加进度条长度
        OLED_Draw_sqar(4 + Show_Energy_PixRem, SHOW_BLOCK_POSY, Sqare_Type, Pix_Change_Count);
    }
    else if (Pix_Change_Count <0)
    {
        //需要缩短进度条长度
        OLED_Draw_sqar(4 + Show_Energy_PixNow, SHOW_BLOCK_POSY, SHOW_SQARETYPE_CLEAR, -Pix_Change_Count);
    }
    Show_Energy_PixRem = Show_Energy_PixNow;//刷新像素显示位置记录
}

//显示HERO_RM
void OLED_Show_Team()
{
    //显示字符
    OLED_show6x8string(SHOW_HERO_RM_POSX, SHOW_HERO_RM_POSY, "HERO_RM POWER");
}

//用于显示三位数功率设定值
void OLED_Show_Power_Num(rt_uint16_t PowerSet)
{
    char ShowCharBuff[5];
    ShowCharBuff[3] = 'W';
    ShowCharBuff[4] = '\0';
    if(PowerSet>999)
    {
        PowerSet = 999;
    }
    if(PowerSet>=100)
    {
        ShowCharBuff[0] = '0' + PowerSet / 100;
        PowerSet %= 100;
    }
    else
    {
        ShowCharBuff[0] = ' ';
    }
    if (PowerSet >= 10)
    {
        ShowCharBuff[1] = '0' + PowerSet / 10;
        PowerSet %= 10;
    }
    else
    {
        ShowCharBuff[1] = ' ';
    }
    ShowCharBuff[2] = '0' + PowerSet;
    while(ShowCharBuff[0]==' ')
    {
        ShowCharBuff[0] = ShowCharBuff[1];
        ShowCharBuff[1] = ShowCharBuff[2];
        ShowCharBuff[2] = ShowCharBuff[3];
        ShowCharBuff[3] = ShowCharBuff[4];
        ShowCharBuff[4] = ' ';
    }
    OLED_show6x8string(SHOW_POWERSET_POSX, SHOW_POWERSET_POSY, ShowCharBuff);
}

//刷新OLED功率设定值示数
void OLED_Fresh_PowerSet(rt_uint16_t PowerSet)
{
    static rt_uint16_t PowerSet_REM = 999;
    if (PowerSet != PowerSet_REM)
    {//需要刷新屏幕功率示数
        PowerSet_REM = PowerSet;
        OLED_Show_Power_Num(PowerSet);
    }
}

//初始化进度条显示
void OLED_Show_Energy_Init(float EnergyPercentage)
{
    //绘制能量框
    OLED_Draw_sqar(0, SHOW_BLOCK_POSY-1, 0x0F, 2);
    OLED_Draw_sqar(0, SHOW_BLOCK_POSY, 0x0F, 2);
    OLED_Draw_sqar(0, SHOW_BLOCK_POSY+1, 0x0F, 2);

    OLED_Draw_sqar(2, SHOW_BLOCK_POSY - 1, 0x03, 124);
    OLED_Draw_sqar(2, SHOW_BLOCK_POSY + 1, 0x0C, 124);

    OLED_Draw_sqar(126, SHOW_BLOCK_POSY - 1, 0x0F, 2);
    OLED_Draw_sqar(126, SHOW_BLOCK_POSY, 0x0F, 2);
    OLED_Draw_sqar(126, SHOW_BLOCK_POSY + 1, 0x0F, 2);

    OLED_Draw_sqar(106, SHOW_BLOCK_POSY - 1, 0x0F, 2);
    OLED_Draw_sqar(106, SHOW_BLOCK_POSY , 0x0F, 2);
    OLED_Draw_sqar(106, SHOW_BLOCK_POSY + 1, 0x0F, 2);

    Show_Energy_PixRem = 0;                            //刚开始显示时屏幕上没有显示能量条，对应当前显示像素数为0
    Show_Energy_SqareType_Rem = SHOW_SQARETYPE_HOLLOW; //默认显示空心能量条
    Show_Buff_PixRem = 0;                            //刚开始显示时屏幕上没有显示能量条，对应当前显示像素数为0
    Show_Buff_SqareType_Rem = SHOW_SQARETYPE_HOLLOW; //默认显示空心能量条

    //绘制能量条
    OLED_FreshEnergy(EnergyPercentage, SHOW_SQARETYPE_HOLLOW);
}


//OLED显示模块初始化
void OLED_Mod_Init()
{
    rt_thread_delay(100);       //上电等待一段时间，保证OLED上电
    OLED_init();                //完成OLED初始化
    OLED_Show_Team();
    OLED_Fresh_PowerSet(0);
    OLED_Show_Energy_Init(0);   //完成进度条显示初始化
}
