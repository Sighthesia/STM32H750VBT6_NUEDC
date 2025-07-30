#include"stepmot_pzt.h"

//云台数据
typedef struct stepmot_pzt_t
{
    i32 mot1_mov;
    i32 mot2_mov;
}stepmot_pzt_t;
static stepmot_pzt_t stepmot_pzt_data={0};

//云台初始化
void stepmot_pzt_init();
//云台设置
void stepmot_pzt_set(u32 stp1,float spd1,u32 stp2,float spd2);
//云台设置2
void stepmot_pzt_set_2(i32 stp1,i32 stp2);
//云台检查
u8 stepmot_pzt_check();
//云台复位
void stepmot_pzt_reset();
//云台绘制
void stepmot_pzt_draw(i32 x,i32 y);

//云台初始化
void stepmot_pzt_init()
{
    //电机初始化
    zdt_stepmot_init();
    zdt_stepmot_init_ad1();
}

//云台检查
u8 stepmot_pzt_check()
{
    //电机状态检查（仅在电机空闲时可操作）
    if(zdt_stepmot_check())
    {
        return 1;
    }
    if(zdt_stepmot_check_ad1())
    {
        return 1;
    }
    return 0;
}

//云台设置
void stepmot_pzt_set(u32 stp1,float spd1,u32 stp2,float spd2)
{
    //等待云台空闲
    while(stepmot_pzt_check())
    {
        osDelay(1);
    }
    if(spd1!=0)
    {
        if(spd1>0)
        {
            //电机设置
            zdt_stepmot_set(1,stp1,spd1);
            //记录数据
            stepmot_pzt_data.mot1_mov+=stp1;
        }else
        {
            //电机设置
            zdt_stepmot_set(0,stp1,-spd1);
            //记录数据
            stepmot_pzt_data.mot1_mov-=stp1;
        }
    }
    if(spd2!=0)
    {
        if(spd2>0)
        {
            //电机设置
            zdt_stepmot_set_ad1(1,stp2,spd2);
            //记录数据
            stepmot_pzt_data.mot2_mov+=stp2;
        }else
        {
            //电机设置
            zdt_stepmot_set_ad1(0,stp2,-spd2);
            //记录数据
            stepmot_pzt_data.mot2_mov-=stp2;
        }
    }
}

//云台设置2
void stepmot_pzt_set_2(i32 stp1,i32 stp2)
{
    if(stp1!=0)
    {
        if(stp1>0)
        {
            stepmot_pzt_set(stp1,800,0,0);
        }else
        {
            stepmot_pzt_set(-stp1,-800,0,0);
        }
    }
    if(stp2!=0)
    {
        if(stp2>0)
        {
            stepmot_pzt_set(0,0,stp2,800);
        }else
        {
            stepmot_pzt_set(0,0,-stp2,-800);
        }
    }
}

//云台复位
void stepmot_pzt_reset()
{
    //电机1
    if(stepmot_pzt_data.mot1_mov>0)
    {        
        //电机设置
        zdt_stepmot_set(0,stepmot_pzt_data.mot1_mov,1000);
    }
    if(stepmot_pzt_data.mot1_mov<0)
    {
        //电机设置
        zdt_stepmot_set(1,-stepmot_pzt_data.mot1_mov,1000);
    }
    //清零
    stepmot_pzt_data.mot1_mov=0;
    //电机2
    if(stepmot_pzt_data.mot2_mov>0)
    {        
        //电机设置
        zdt_stepmot_set_ad1(0,stepmot_pzt_data.mot2_mov,1000);
    }
    if(stepmot_pzt_data.mot2_mov<0)
    {
        //电机设置
        zdt_stepmot_set_ad1(1,-stepmot_pzt_data.mot2_mov,1000);
    }
    //清零
    stepmot_pzt_data.mot2_mov=0;
}

//云台绘制
void stepmot_pzt_draw(i32 x,i32 y)
{
    i32 x_e=x-stepmot_pzt_data.mot1_mov;
    i32 y_e=y-stepmot_pzt_data.mot2_mov;
    stepmot_pzt_set_2(x_e,y_e);
}
