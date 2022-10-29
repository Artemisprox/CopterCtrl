/****************
 * drv_CustomUI_func.c/h
 * 各种图形的具体绘制操作都写在里面
 *
//TODO 可通过ID号查找函数,并对图形进行相应操作
 * ***************/

#include "func_CustomUI.h"
#include "CustomUI.h"
#include "CustomUI_GraphParameters.h"
#include "mod_refsystem.h"
#include "drv_CustomUI_AuxAiming.h"
#include "SuperCap_Com.h"
#include "app_GetGim.h"
#include "drv_GimMotor.h"
#include "app_GetRef.h"
#include "mod_Monitor.h"
#include "drv_IMU.h"
#include "string.h"
#include "drv_ui_list.h"
#include "HLxfunc.h"
#include "drv_utils.h"

extern struct rt_semaphore custom_ui_sem;
extern struct rt_mutex custom_ui_mutex;

graph_operation_t operation;
rt_uint8_t ui_erase_flg = 0;

ui_basic_e Draw_test(void *param)
{
    rt_uint8_t text[2] = {"ab"};
    // custom_ui_append_int_operate(BACKGROUND_LAYER, 999, GRAPH_OPERATION_ADD, 5, 5, GRAPH_COLOR_CYAN, 900, 540, 999);
    // custom_ui_append_float_operate(BACKGROUND_LAYER, 999, GRAPH_OPERATION_ADD, 5, 5,5, GRAPH_COLOR_CYAN, 900, 540, 999.9);
    UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 999, GRAPH_OPERATION_ADD, (char *)text, UI_OPTION_SIZE, sizeof(text), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
    return UI_OK;
}

/***
 * @name
 * @brief  填充整个屏幕
 * @param	none
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author dxy
 ***/
ui_basic_e Fill_Full_Screen(void *param)
{
    UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 1, GRAPH_OPERATION_ADD, 1080, GRAPH_COLOR_GREEN, 0, 540, 1920, 540));
    return UI_OK;
}

/***
 * @name Clear_Screen(void *param)
 * @brief  清屏
 * @param	none
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author dxy
 ***/
ui_basic_e Clear_Screen(void *param)
{

    //清除三次,防止有遗漏
    UI_ASSERT(custom_ui_erase_graph(CLEAR_GRAPH_TYPE_CLEAR_ALL, 1));
    UI_ASSERT(custom_ui_erase_graph(CLEAR_GRAPH_TYPE_CLEAR_ALL, 1));
    UI_ASSERT(custom_ui_erase_graph(CLEAR_GRAPH_TYPE_CLEAR_ALL, 1));
    UI_ASSERT(custom_ui_erase_graph(CLEAR_GRAPH_TYPE_CLEAR_ALL, 1));

    return UI_OK;
}

/***
 * @name Draw_SC_Bar(void *param)
 * @brief  画出超级电容剩余电量
 * @param	none
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author dxy
 ***/
ui_basic_e Draw_SC_Bar_dyn(void *param)
{

    uint16_t remain_cap = Get_RemainCapcity(); //获取超级电容剩余值
    graph_color_t color;
    if (remain_cap > 40)
        color = GRAPH_COLOR_GREEN; //值大于40时显示绿色
    else if (remain_cap > 20)
        color = GRAPH_COLOR_YELLOW; //值大于20时显示黄色
    else
        color = GRAPH_COLOR_REDBLUE; //值小于20时显示红色

    //画出剩余电量
    UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 1, operation, UI_SC_BAR_SIZE, color, UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 5.5, UI_SC_BAR_CENTER_Y, UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 5.5 + remain_cap / 9.1 * UI_SC_BAR_SIZE, UI_SC_BAR_CENTER_Y));
    return UI_OK;
}

ui_basic_e Draw_SC_Bar_bkgd(void *param)
{

    char txt[10] = "SC";
    UI_ASSERT(custom_ui_append_rect_operate(BACKGROUND_LAYER, 3, GRAPH_OPERATION_ADD, 2, GRAPH_COLOR_GREEN, UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 6, UI_SC_BAR_CENTER_Y - 0.7 * UI_SC_BAR_SIZE, UI_SC_BAR_CENTER_X + UI_SC_BAR_SIZE * 6, UI_SC_BAR_CENTER_Y + 0.7 * UI_SC_BAR_SIZE));
    UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 2, GRAPH_OPERATION_ADD, txt, UI_SC_BAR_SIZE, sizeof(txt), 2, GRAPH_COLOR_GREEN, UI_SC_BAR_CENTER_X, UI_SC_BAR_CENTER_Y + UI_SC_BAR_SIZE * 2));

    return UI_OK;
}
ui_basic_e Draw_HERO_Cream_Help_bkgd(void *param)
{
#ifdef CORE_USING_HERO
    UI_ASSERT(custom_ui_append_rect_operate(BACKGROUND_LAYER, 4, GRAPH_OPERATION_ADD, 2, GRAPH_COLOR_WHITE, UI_HERO_CREAM_HELP_CENTER_X_1 * 2, UI_HERO_CREAM_HELP_CENTER_Y_1 * 2, UI_HERO_CREAM_HELP_CENTER_X_2 * 2, UI_HERO_CREAM_HELP_CENTER_Y_2 * 2));
#endif
    return UI_OK;
}
/***
 * @name  Draw_Bullet
 * @brief	画子弹图形
 * @param	x_c:x中心,
 * @param	y_c:y中心,
 * @param	size:图标尺寸
 * @param	color:图标颜色
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author dxy
 ***/
static ui_basic_e Draw_Bullet(uint16_t id, graph_operation_t operation, rt_uint16_t x_c, rt_uint16_t y_c, uint16_t size, graph_color_t color)
{

    //一个矩形加半个椭圆,图形类似一个子弹
    UI_ASSERT(custom_ui_append_rect_operate(DYNAMIC_LAYER, id++, operation, 2, color, x_c - size, y_c - 3 * size, x_c + size, y_c + size));
    UI_ASSERT(custom_ui_append_arc_operate(DYNAMIC_LAYER, id++, operation, 270, 90, 2, color, x_c, y_c + size, size, 2 * size));

    return UI_OK;
}

/***
 * @name  Draw_Strike_Mode(void *param)
 * @brief	显示子弹模式,有单发与三连发模式,单发画一个子弹,三连发画三个子弹
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author dxy
 ***/

ui_basic_e Draw_Strike_Mode(void *param)
{   
    static graph_color_t setcolor_last;
    static graph_color_t setcolor;
	static rt_int8_t bullet_mode_last = -1;
    strike_mode_e strike_mode = Get_StrikeMode(); //获取发射模式
    // 通过读取当前自方颜色来设置子弹颜色
    setcolor = ((Get_Color_Myself() == My_Color_Red) ? GRAPH_COLOR_ORANGE : GRAPH_COLOR_CYAN);
	if (strike_mode != bullet_mode_last || ui_erase_flg || 	setcolor_last != setcolor)
    {
        switch (strike_mode)
        {
            case SINGLE_STRIKE: //单发画一个子弹
				UI_ASSERT(Draw_Bullet(10, GRAPH_OPERATION_ERASE, UI_STRIKE_MODE_CENTER_X, UI_STRIKE_MODE_CENTER_Y, UI_STRIKE_MODE_SIZE, setcolor));
                UI_ASSERT(Draw_Bullet(10, GRAPH_OPERATION_ADD, UI_STRIKE_MODE_CENTER_X, UI_STRIKE_MODE_CENTER_Y, UI_STRIKE_MODE_SIZE, setcolor));
                UI_ASSERT(Draw_Bullet(13, GRAPH_OPERATION_ERASE, UI_STRIKE_MODE_CENTER_X - UI_STRIKE_MODE_SIZE * 3, UI_STRIKE_MODE_CENTER_Y, UI_STRIKE_MODE_SIZE, setcolor));
                UI_ASSERT(Draw_Bullet(16, GRAPH_OPERATION_ERASE, UI_STRIKE_MODE_CENTER_X + UI_STRIKE_MODE_SIZE * 3, UI_STRIKE_MODE_CENTER_Y, UI_STRIKE_MODE_SIZE, setcolor));
			    setcolor_last = setcolor;
			    bullet_mode_last = strike_mode; //所有图形发送完后再设标志位
                break;
            case TRIPLE_STRIKE: //三连发画三个子弹
			    UI_ASSERT(Draw_Bullet(10, GRAPH_OPERATION_ERASE, UI_STRIKE_MODE_CENTER_X, UI_STRIKE_MODE_CENTER_Y, UI_STRIKE_MODE_SIZE, setcolor));
                UI_ASSERT(Draw_Bullet(10, GRAPH_OPERATION_ADD, UI_STRIKE_MODE_CENTER_X, UI_STRIKE_MODE_CENTER_Y, UI_STRIKE_MODE_SIZE, setcolor));
                UI_ASSERT(Draw_Bullet(13, GRAPH_OPERATION_ADD, UI_STRIKE_MODE_CENTER_X - UI_STRIKE_MODE_SIZE * 3, UI_STRIKE_MODE_CENTER_Y, UI_STRIKE_MODE_SIZE, setcolor));
                UI_ASSERT(Draw_Bullet(16, GRAPH_OPERATION_ADD, UI_STRIKE_MODE_CENTER_X + UI_STRIKE_MODE_SIZE * 3, UI_STRIKE_MODE_CENTER_Y, UI_STRIKE_MODE_SIZE, setcolor));
			    setcolor_last = setcolor;
			    bullet_mode_last = strike_mode; //所有图形发送完后再设标志位
                break;
            default:
                break;
        }
    }

    return UI_OK;
}

ui_basic_e Draw_AuxAim_Ballistic_Line(void *param)
{
    float x_offset = 9.0f;
    float y_offset = 50.0f;
    float x_len = 25.0f;
    float y_len = 30.0f;
    float alpha = 0;
    static float alpha_last = -1;
#ifdef CORE_USING_HERO
    float roll_c, pitch_c = 0;
    float yaw_c, yaw_g;
    float phi;
    float delta;
    roll_c = Get_Chassis_IMU_Data(IMU_ROLL);
    pitch_c = Get_Chassis_IMU_Data(IMU_PITCH);
    phi = acosf(cosf(roll_c * PI / 180) * cosf(pitch_c * PI / 180)) * 180 / PI;
    yaw_g = Get_GM_360f(GM_YAW);
    yaw_c = Get_Chassis_IMU_Data(IMU_YAW);
    delta = CIRCLE_SHORTEST_DIS(yaw_g, yaw_c, -180.0f, 180.0f);
    alpha = acosf(cosf(delta * PI / 180) * sinf(phi * PI / 180)) * 180.0f / PI;
    alpha = 90.f - alpha;
#endif
    //最小分辨率为0.2°
    if (__fabsf(alpha - alpha_last) > 0.2f || ui_erase_flg)
    {
        UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 300, operation, 2, UI_AUXAIM_BKGD_COLOR,
                                                X_OFFSET - x_offset * cosf(alpha * PI / 180.0f) - y_offset * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + x_offset * sinf(alpha * PI / 180.0f) - y_offset * cosf(alpha * PI / 180.0f),
                                                X_OFFSET - x_offset * cosf(alpha * PI / 180.0f) - (y_offset + 9 * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + x_offset * sinf(alpha * PI / 180.0f) - (y_offset + 9 * y_len) * cosf(alpha * PI / 180.0f)));

        UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 301, operation, 2, UI_AUXAIM_BKGD_COLOR,
                                                X_OFFSET - (x_offset + 3.0f * x_len) * cosf(alpha * PI / 180.0f) - (y_offset + 0.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset + 3.0f * x_len) * sinf(alpha * PI / 180.0f) - (y_offset + 0.0f * y_len) * cosf(alpha * PI / 180.0f),
                                                X_OFFSET - (x_offset)*cosf(alpha * PI / 180.0f) - (y_offset + 0.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset)*sinf(alpha * PI / 180.0f) - (y_offset + 0.0f * y_len) * cosf(alpha * PI / 180.0f)));

        UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 302, operation, 2, UI_AUXAIM_BKGD_COLOR,
                                                X_OFFSET - (x_offset + 1.0f * x_len) * cosf(alpha * PI / 180.0f) - (y_offset + 1.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset + 1.0f * x_len) * sinf(alpha * PI / 180.0f) - (y_offset + 1.0f * y_len) * cosf(alpha * PI / 180.0f),
                                                X_OFFSET - (x_offset)*cosf(alpha * PI / 180.0f) - (y_offset + 1.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset)*sinf(alpha * PI / 180.0f) - (y_offset + 1.0f * y_len) * cosf(alpha * PI / 180.0f)));

        UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 303, operation, 2, UI_AUXAIM_BKGD_COLOR,
                                                X_OFFSET - (x_offset + 2.0f * x_len) * cosf(alpha * PI / 180.0f) - (y_offset + 2.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset + 2.0f * x_len) * sinf(alpha * PI / 180.0f) - (y_offset + 2.0f * y_len) * cosf(alpha * PI / 180.0f),
                                                X_OFFSET - (x_offset)*cosf(alpha * PI / 180.0f) - (y_offset + 2.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset)*sinf(alpha * PI / 180.0f) - (y_offset + 2.0f * y_len) * cosf(alpha * PI / 180.0f)));

        UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 304, operation, 2, UI_AUXAIM_BKGD_COLOR,
                                                X_OFFSET - (x_offset + 1.0f * x_len) * cosf(alpha * PI / 180.0f) - (y_offset + 3.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset + 1.0f * x_len) * sinf(alpha * PI / 180.0f) - (y_offset + 3.0f * y_len) * cosf(alpha * PI / 180.0f),
                                                X_OFFSET - (x_offset)*cosf(alpha * PI / 180.0f) - (y_offset + 3.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset)*sinf(alpha * PI / 180.0f) - (y_offset + 3.0f * y_len) * cosf(alpha * PI / 180.0f)));

        UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 305, operation, 2, UI_AUXAIM_BKGD_COLOR,
                                                X_OFFSET - (x_offset + 2.0f * x_len) * cosf(alpha * PI / 180.0f) - (y_offset + 4.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset + 2.0f * x_len) * sinf(alpha * PI / 180.0f) - (y_offset + 4.0f * y_len) * cosf(alpha * PI / 180.0f),
                                                X_OFFSET - (x_offset)*cosf(alpha * PI / 180.0f) - (y_offset + 4.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset)*sinf(alpha * PI / 180.0f) - (y_offset + 4.0f * y_len) * cosf(alpha * PI / 180.0f)));

        UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 306, operation, 2, UI_AUXAIM_BKGD_COLOR,
                                                X_OFFSET - (x_offset + 1.0f * x_len) * cosf(alpha * PI / 180.0f) - (y_offset + 5.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset + 1.0f * x_len) * sinf(alpha * PI / 180.0f) - (y_offset + 5.0f * y_len) * cosf(alpha * PI / 180.0f),
                                                X_OFFSET - (x_offset)*cosf(alpha * PI / 180.0f) - (y_offset + 5.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset)*sinf(alpha * PI / 180.0f) - (y_offset + 5.0f * y_len) * cosf(alpha * PI / 180.0f)));

        UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 307, operation, 2, UI_AUXAIM_BKGD_COLOR,
                                                X_OFFSET - (x_offset + 2.0f * x_len) * cosf(alpha * PI / 180.0f) - (y_offset + 6.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset + 2.0f * x_len) * sinf(alpha * PI / 180.0f) - (y_offset + 6.0f * y_len) * cosf(alpha * PI / 180.0f),
                                                X_OFFSET - (x_offset)*cosf(alpha * PI / 180.0f) - (y_offset + 6.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset)*sinf(alpha * PI / 180.0f) - (y_offset + 6.0f * y_len) * cosf(alpha * PI / 180.0f)));

        UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 308, operation, 2, UI_AUXAIM_BKGD_COLOR,
                                                X_OFFSET - (x_offset + 1.0f * x_len) * cosf(alpha * PI / 180.0f) - (y_offset + 7.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset + 1.0f * x_len) * sinf(alpha * PI / 180.0f) - (y_offset + 7.0f * y_len) * cosf(alpha * PI / 180.0f),
                                                X_OFFSET - (x_offset)*cosf(alpha * PI / 180.0f) - (y_offset + 7.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset)*sinf(alpha * PI / 180.0f) - (y_offset + 7.0f * y_len) * cosf(alpha * PI / 180.0f)));

        UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 309, operation, 2, UI_AUXAIM_BKGD_COLOR,
                                                X_OFFSET - (x_offset + 2.0f * x_len) * cosf(alpha * PI / 180.0f) - (y_offset + 8.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset + 2.0f * x_len) * sinf(alpha * PI / 180.0f) - (y_offset + 8.0f * y_len) * cosf(alpha * PI / 180.0f),
                                                X_OFFSET - (x_offset)*cosf(alpha * PI / 180.0f) - (y_offset + 8.0f * y_len) * sinf(alpha * PI / 180.0f),
                                                Y_OFFSET + (x_offset)*sinf(alpha * PI / 180.0f) - (y_offset + 8.0f * y_len) * cosf(alpha * PI / 180.0f)));
        alpha_last = alpha;
    }
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 301, operation, 2, UI_AUXAIM_BKGD_COLOR, X_OFFSET - x_offset - 3.0f * x_len, Y_OFFSET - y_offset - 0.0f * y_len, X_OFFSET - x_offset, Y_OFFSET - y_offset - 0.0f * y_len));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 302, operation, 2, UI_AUXAIM_BKGD_COLOR, X_OFFSET - x_offset - 1.0f * x_len, Y_OFFSET - y_offset - 1.0f * y_len, X_OFFSET - x_offset, Y_OFFSET - y_offset - 1.0f * y_len));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 303, operation, 2, UI_AUXAIM_BKGD_COLOR, X_OFFSET - x_offset - 2.0f * x_len, Y_OFFSET - y_offset - 2.0f * y_len, X_OFFSET - x_offset, Y_OFFSET - y_offset - 2.0f * y_len));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 304, operation, 2, UI_AUXAIM_BKGD_COLOR, X_OFFSET - x_offset - 1.0f * x_len, Y_OFFSET - y_offset - 3.0f * y_len, X_OFFSET - x_offset, Y_OFFSET - y_offset - 3.0f * y_len));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 305, operation, 2, UI_AUXAIM_BKGD_COLOR, X_OFFSET - x_offset - 2.0f * x_len, Y_OFFSET - y_offset - 4.0f * y_len, X_OFFSET - x_offset, Y_OFFSET - y_offset - 4.0f * y_len));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 306, operation, 2, UI_AUXAIM_BKGD_COLOR, X_OFFSET - x_offset - 1.0f * x_len, Y_OFFSET - y_offset - 5.0f * y_len, X_OFFSET - x_offset, Y_OFFSET - y_offset - 5.0f * y_len));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 307, operation, 2, UI_AUXAIM_BKGD_COLOR, X_OFFSET - x_offset - 2.0f * x_len, Y_OFFSET - y_offset - 6.0f * y_len, X_OFFSET - x_offset, Y_OFFSET - y_offset - 6.0f * y_len));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 308, operation, 2, UI_AUXAIM_BKGD_COLOR, X_OFFSET - x_offset - 1.0f * x_len, Y_OFFSET - y_offset - 7.0f * y_len, X_OFFSET - x_offset, Y_OFFSET - y_offset - 7.0f * y_len));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 309, operation, 2, UI_AUXAIM_BKGD_COLOR, X_OFFSET - x_offset - 2.0f * x_len, Y_OFFSET - y_offset - 8.0f * y_len, X_OFFSET - x_offset, Y_OFFSET - y_offset - 8.0f * y_len));
    return UI_OK;
}

/***
 * @name Draw_Attitude_Angle(void *param)
 * @brief    画出此时车身的姿态(云台与底盘的夹角)
 * @param none
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author   dxy
 ***/
ui_basic_e Draw_Attitude_Angle_dyn(void *param)
{
    float yaw = 0;
    yaw = Get_GM_360f(GM_YAW) * PI / 180.0f; //得到电机yaw轴数据
    // 如果当前处于回头模式就需要旋转一次
    if (Get_Motion_Mode() == UI_FOLLOWBACK_GIMBAL)
    {
        yaw += 180.f;
        utils_norm_circle_number(&yaw, -180.f, 360.f);
    }
    //以底盘为基准画出云台当前角度
    UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 31, operation, 2, UI_ATTITUDE_ANGLE_COLOR, X_OFFSET - SCOPE_CIRCLE_RADIUS * 0.6 * sinf(yaw), Y_OFFSET + SCOPE_CIRCLE_RADIUS * 0.6 * cosf(yaw), X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(yaw), Y_OFFSET + SCOPE_CIRCLE_RADIUS * cosf(yaw)));
    UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 32, operation, 2, UI_ATTITUDE_ANGLE_COLOR, X_OFFSET - SCOPE_CIRCLE_RADIUS * 0.8 * sinf(yaw + PI / 2), Y_OFFSET + SCOPE_CIRCLE_RADIUS * 0.8 * cosf(yaw + PI / 2), X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(yaw + PI / 2), Y_OFFSET + SCOPE_CIRCLE_RADIUS * cosf(yaw + PI / 2)));
    UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, 33, operation, 2, UI_ATTITUDE_ANGLE_COLOR, X_OFFSET - SCOPE_CIRCLE_RADIUS * 0.8 * sinf(yaw - PI / 2), Y_OFFSET + SCOPE_CIRCLE_RADIUS * 0.8 * cosf(yaw - PI / 2), X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(yaw - PI / 2), Y_OFFSET + SCOPE_CIRCLE_RADIUS * cosf(yaw - PI / 2)));

    return UI_OK;
}
/***
 * @brief    背景绘制
 ***/
ui_basic_e Draw_Attitude_Angle_bkgd(void *param)
{

    UI_ASSERT(custom_ui_append_cirle_operate(DYNAMIC_LAYER, 34, GRAPH_OPERATION_ADD, 2, UI_ATTITUDE_ANGLE_COLOR, X_OFFSET, Y_OFFSET, SCOPE_CIRCLE_RADIUS));

    return UI_OK;
}

/***
 * @name
 * @brief        显示弹速
 * @param
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author
 ***/
ui_basic_e Draw_Bullet_Speed_dyn(void *param)
{

    int bullet_speed = Get_Bullet_Speed(); //获取弹速
    static int bullet_speed_last = 0;
    char txt[10] = {0};
    if (bullet_speed != bullet_speed_last || ui_erase_flg)
    {
        rt_sprintf(txt, "%d", bullet_speed);
        //写出弹速数值
        // UI_ASSERT(custom_ui_append_int_operate(DYNAMIC_LAYER, 46, operation, UI_BULLET_SPEED_SIZE, 2, UI_BULLET_SPEED_COLOR, UI_BULLET_SPEED_CENTER_X + UI_BULLET_SPEED_SIZE * 8, UI_BULLET_SPEED_CENTER_Y, bullet_speed));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 46, operation, txt, UI_BULLET_SPEED_SIZE, sizeof(txt), 2, UI_BULLET_SPEED_COLOR, UI_BULLET_SPEED_CENTER_X + UI_BULLET_SPEED_SIZE * 8, UI_BULLET_SPEED_CENTER_Y));
        bullet_speed_last = bullet_speed; //注意绘制完毕后再更新数据
    }

    return UI_OK;
}
/***
 * @brief    背景绘制
 ***/
ui_basic_e Draw_Bullet_Speed_bkgd(void *param)
{

    static char speed_string[5] = "speed";
    UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 47, GRAPH_OPERATION_ADD, speed_string, UI_BULLET_SPEED_SIZE, sizeof(speed_string), 2, UI_BULLET_SPEED_COLOR, UI_BULLET_SPEED_CENTER_X - UI_BULLET_SPEED_SIZE * 4, UI_BULLET_SPEED_CENTER_Y));

    return UI_OK;
}

/***
 * @name
 * @brief    显示剩余弹量
 * @param
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author
 ***/
ui_basic_e Draw_Remain_Ammo_dyn(void *param)
{

    int ammo_remain = Ref_AmmoRemain();
    static int ammo_remain_last = 0;
    char txt[10] = {0};
    if (ammo_remain != ammo_remain_last || ui_erase_flg)
    {
        rt_sprintf(txt, "%d", ammo_remain);
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 51, operation, txt, UI_REMAIN_AMMO_SIZE, sizeof(txt), 2, UI_REMAIN_AMMO_COLOR, UI_REMAIN_AMMO_CENTER_X + UI_REMAIN_AMMO_SIZE * 8, UI_REMAIN_AMMO_CENTER_Y));
        ammo_remain_last = ammo_remain; ////注意绘制完毕后再更新数据
    }

    return UI_OK;
}
/***
 * @brief    背景绘制
 ***/
ui_basic_e Draw_Remain_Ammo_bkgd(void *param)
{

    static char ammo[4] = "ammo";
    UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 52, GRAPH_OPERATION_ADD, ammo, UI_REMAIN_AMMO_SIZE, sizeof(ammo), 2, UI_REMAIN_AMMO_COLOR, UI_REMAIN_AMMO_CENTER_X - UI_REMAIN_AMMO_SIZE * 4, UI_REMAIN_AMMO_CENTER_Y));

    return UI_OK;
}

/***
 * @name
 * @brief    显示当前运动模式(跟随/不跟随/陀螺)
 * @param
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author
 ***/
ui_basic_e Draw_Motion_Mode_dyn(void *param)
{

    static char motionmode_string[7][12] = {"No Follow", "Follow", "Follow Back", "Slow Gyro", "Fast Gyro", "Move Back", "Auto"};
    int motionmode = 0;
    static int motionmode_last = -1;
    motionmode = Get_Motion_Mode();
    if (motionmode != motionmode_last || ui_erase_flg)
    {
        //写出当前运动模式
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 60, operation, motionmode_string[motionmode], UI_MOTION_MODE_SIZE, sizeof(motionmode_string[motionmode]), 2, UI_MOTION_MODE_DYN_COLOR, UI_MOTION_MODE_CENTER_X + UI_MOTION_MODE_SIZE * 8, UI_MOTION_MODE_CENTER_Y));
        motionmode_last = motionmode;
    }

    return UI_OK;
}
/***
 * @brief    背景绘制
 ***/
ui_basic_e Draw_Motion_Mode_bkgd(void *param)
{

    static char mode[10] = "motion";
    //写"motionmode"字样
    UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 61, GRAPH_OPERATION_ADD, mode, UI_MOTION_MODE_SIZE, sizeof(mode), 2, UI_MOTION_MODE_BKGD_COLOR, UI_MOTION_MODE_CENTER_X - UI_MOTION_MODE_SIZE * 4, UI_MOTION_MODE_CENTER_Y));

    return UI_OK;
}
/***
 * @name
 * @brief    显示弹仓开关
 * @param
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author
 ***/
ui_basic_e Draw_Magazine_dyn(void *param)
{

    static rt_int8_t status_last = -1;
    static rt_int8_t status_char[2][5] = {"CLOSE", "OPEN"};
    rt_int8_t status = Get_Magazine_Status();
    if (status_last != status || ui_erase_flg)
    {
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 71, operation, (char *)status_char[status], UI_MAGAZINE_STATUS_SIZE, sizeof(status_char[status]), 2, UI_MAGAZINE_STATUS_DYN_COLOR, UI_MAGAZINE_STATUS_CENTER_X + UI_MAGAZINE_STATUS_SIZE * 8, UI_MAGAZINE_STATUS_CENTER_Y));
        status_last = status;
    }
    return UI_OK;
}

/***
 * @brief    背景绘制
 ***/
ui_basic_e Draw_Magazine_bkgd(void *param)
{

    static char magazine[8] = "magazine";
    UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 72, GRAPH_OPERATION_ADD, magazine, UI_MAGAZINE_STATUS_SIZE, sizeof(magazine), 2, UI_MAGAZINE_STATUS_BKGD_COLOR, UI_MAGAZINE_STATUS_CENTER_X - UI_MAGAZINE_STATUS_SIZE * 4, UI_MAGAZINE_STATUS_CENTER_Y));

    return UI_OK;
}

/***
 * @brief    显示底盘各模块状态
 * @param    none
 * @author dxy
 ***/
ui_basic_e Draw_Monitor_dyn(void *param)
{
    uint16_t monitor_flg = 0;
    static uint16_t monitor_flg_last = 0xFFFF;
    static uint8_t cnt = 0;
    uint8_t label = 0;
    static char txt[15][20] = {"NONE", "GIMYAW OFFLINE", "GIMPITCH OFFLINE", "RUBR OFFLINE", "RUBL OFFLINE", "LAUNCH OFFLINE",
                               "RFWHEEL OFFLINE", "LFWHEEL OFFLINE", "LBWHEEL OFFLINE", "RBWHEEL OFFLINE", "CHASYAW OFFLINE",
                               "REF OFFLINE", "SC OFFLINE", "RUB STOPPED"};

    //    for (swdg_deviceID i = F_Left_ID; i <= SC_ID; i++)
    //    {
    //        if (Swdg_If_Error(i) == RT_TRUE)
    //        {
    //            monitor_flg |= (1 << (i - F_Left_ID));
    //        }
    //    }
    if (monitor_flg ^ monitor_flg_last || ui_erase_flg) // when status changes
    {
        if (ui_erase_flg)
            cnt = 0;
        while (cnt)
        {
            cnt--;
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 121 + cnt, GRAPH_OPERATION_ERASE, txt[cnt], UI_MONITOR_SIZE, sizeof(txt[cnt]), 2, UI_MONITOR_COLOR, UI_MONITOR_CENTER_X - UI_MONITOR_SIZE * 4, UI_MONITOR_CENTER_Y - 1.5f * UI_MONITOR_SIZE * cnt));
        }
        if (monitor_flg == 0) // normal condition
        {
            cnt++;
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 120 + cnt, GRAPH_OPERATION_ADD, txt[0], UI_MONITOR_SIZE, sizeof(txt[0]), 2, UI_MONITOR_COLOR, UI_MONITOR_CENTER_X - UI_MONITOR_SIZE * 4, UI_MONITOR_CENTER_Y - 1.5f * UI_MONITOR_SIZE * cnt));
        }
        else
        {
            while (monitor_flg)
            {
                if (monitor_flg & 0x01)
                {
                    cnt++;
                    UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 120 + cnt, GRAPH_OPERATION_ADD, txt[label], UI_MONITOR_SIZE, sizeof(txt[label]), 2, UI_MONITOR_COLOR, UI_MONITOR_CENTER_X - UI_MONITOR_SIZE * 4, UI_MONITOR_CENTER_Y - 1.5f * UI_MONITOR_SIZE * cnt));
                }
                monitor_flg = monitor_flg >> 1;
                label++;
            }
        }
    }
    monitor_flg_last = monitor_flg;
    return UI_OK;
}

//
ui_basic_e Draw_Monitor_bkgd(void *param)
{

    char txt[20] = "error module : ";
    UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 120, GRAPH_OPERATION_ADD, txt, UI_MONITOR_SIZE, sizeof(txt), 2, UI_MONITOR_COLOR, UI_MONITOR_CENTER_X - UI_MONITOR_SIZE * 4, UI_MONITOR_CENTER_Y));

    return UI_OK;
}

/***
 * @name
 * @brief
 * @param    angle : rad
 ***/
static ui_basic_e Draw_Triangle(uint16_t id, graph_operation_t operation, rt_uint16_t x_c, rt_uint16_t y_c, uint16_t size, float angle)
{

    //红色三角形,表示敌方单位
    UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, id++, operation, 2, UI_ENEMY_COLOR, x_c + size * sinf(angle), y_c + size * cosf(angle), x_c + size * sinf(angle + PI * 2.0f / 3.0f), y_c + size * cosf(angle + PI * 2.0f / 3.0f)));
    UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, id++, operation, 2, UI_ENEMY_COLOR, x_c + size * sinf(angle + PI * 2.0f / 3.0f), y_c + size * cosf(angle + PI * 2.0f / 3.0f), x_c + size * sinf(angle + PI * 4.0f / 3.0f), y_c + size * cosf(angle + PI * 4.0f / 3.0f)));
    UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, id++, operation, 2, UI_ENEMY_COLOR, x_c + size * sinf(angle + PI * 4.0f / 3.0f), y_c + size * cosf(angle + PI * 4.0f / 3.0f), x_c + size * sinf(angle), y_c + size * cosf(angle)));

    return UI_OK;
}

static ui_basic_e Draw_Wall(uint16_t id, graph_operation_t operation, rt_uint16_t x_c, rt_uint16_t y_c, uint16_t size, float angle)
{

    //红色三角形,表示敌方单位
    UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, id++, operation, 10, UI_WALL_COLOR, x_c - size * cosf(angle), y_c + size * sinf(angle), x_c + size * cosf(angle), y_c - size * sinf(angle)));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, id++, operation, 2, UI_WALL_COLOR, x_c - 0.6f * size * cosf(angle), y_c + 0.6f * size * sinf(angle), x_c - 0.6f * size * cosf(angle) + 0.4f * size * cosf(PI / 4 - angle), y_c + 0.6f * size * sinf(angle) + 0.4f * size *sinf(PI / 4 - angle)));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, id++, operation, 2, UI_WALL_COLOR, x_c - 0.2f * size * cosf(angle), y_c + 0.2f * size * sinf(angle), x_c - 0.2f * size * cosf(angle) + 0.4f * size * cosf(PI / 4 - angle), y_c + 0.2f * size * sinf(angle) + 0.4f * size * sinf(PI / 4 - angle)));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, id++, operation, 2, UI_WALL_COLOR, x_c + 0.2f * size * cosf(angle), y_c - 0.2f * size * sinf(angle), x_c + 0.2f * size * cosf(angle) + 0.4f * size * cosf(PI / 4 - angle), y_c - 0.2f * size * sinf(angle) + 0.4f * size *sinf(PI / 4 - angle)));
    // UI_ASSERT(custom_ui_append_line_operate(DYNAMIC_LAYER, id++, operation, 2, UI_WALL_COLOR, x_c + 0.6f * size * cosf(angle), y_c - 0.6f * size * sinf(angle), x_c + 0.6f * size * cosf(angle) + 0.4f * size * cosf(PI / 4 - angle), y_c - 0.6f * size * sinf(angle) + 0.4f * size * sinf(PI / 4 - angle)));

    return UI_OK;
}
/***
 * @brief   受到攻击时显示出敌方位置,同时提醒开启陀螺
 * @author dxy
 ***/
ui_basic_e Draw_Enemy_Orientation_dyn(void *param)
{
    // define variables
    static ui_list_t enemy_list;     //敌方信息链表
    ui_list_t *list_pos, *q;         //中间变量
    uint16_t id_offset = 140;        //图形编号
    static uint8_t id_remain = 0xFF; //剩余可用图像ID,这里设置为最大8个
    uint8_t id = 0;
    static uint8_t is_init = 0; //初始化标志位
    uint8_t new_enemy = 0;      //有新敌人
    uint8_t robot_if_hurt = Ref_Get_Robot_If_Hurt();
    ext_robot_hurt_t robot_hurt_data = Ref_Robot_Hurt_Data();
    float encoder_yaw = -Get_GM_360f(GM_YAW) * PI / 180.0f;     //得到电机yaw轴数据
    float imu_yaw = Get_Gimbal_IMU_Data(IMU_YAW) * PI / 180.0f; //得到 IMU yaw轴数据
    static float imu_yaw_last;                                  //上一次 IMU yaw轴数据
    float enemy_position = 0;                                   //敌方位置与云台正前的夹角
    int motionmode = Get_Motion_Mode();
    static int motionmode_last = -1;
    static char string[15] = "ROTATE PLEASE";
    static int rotate_tips_time;
    static rt_uint8_t rotate_tips_on = 0;
    /////////////////////
    if (!is_init)
    {
        UI_List_Init(&enemy_list);
        is_init = 1;
    }
    //当伤害类型为子弹(0x00)或者撞击(0x05)时显示
    if ((robot_if_hurt == 1 && (robot_hurt_data.hurt_type == 0x00 || robot_hurt_data.hurt_type == 0x05)))
    {
        enemy_position = encoder_yaw - (robot_hurt_data.armor_id) * PI / 2.0f; //算出对方位置
        new_enemy = 1;                                                         //可能有新敌人
        //受到攻击时,提醒开启陀螺
        if (robot_hurt_data.hurt_type == 0x00 && (motionmode != UI_SLOW_GYRO && motionmode != UI_FAST_GYRO))
        {
            rotate_tips_time = rt_tick_get();
            rotate_tips_on = 1;
            UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 290, GRAPH_OPERATION_ADD, string, UI_ROTATE_TIPS_SIZE, sizeof(string), 2, UI_ROTATE_TIPS_COLOR, UI_ROTATE_TIPS_CENTER_X, UI_ROTATE_TIPS_CENTER_Y));
        }
    }
    //已经开启陀螺则关闭提示
    if (rotate_tips_on &&
        ((motionmode != motionmode_last && (motionmode == UI_SLOW_GYRO || motionmode == UI_FAST_GYRO)) ||
         rt_tick_get() - rotate_tips_time > UI_ROTATE_TIPS_RESERVING_TIME))
    {
        UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 290, GRAPH_OPERATION_ERASE, string, UI_ROTATE_TIPS_SIZE, sizeof(string), 2, UI_ROTATE_TIPS_COLOR, UI_ROTATE_TIPS_CENTER_X, UI_ROTATE_TIPS_CENTER_Y));
        rotate_tips_on = 0;
    }

    motionmode_last = motionmode;
    Ref_Robot_Reset_Hurt();

    list_for_each_entry_safe(list_pos, q, &enemy_list.list, list)
    {
        list_pos->member.enemy.position += imu_yaw - imu_yaw_last; //敌方位置随云台转动而变化
        if (enemy_position != 0)
        {
            //若本次算出的位置与之前记录的位置相差不大则认为是同一敌人,并刷新记录时间
            if (fabsf(list_pos->member.enemy.position - enemy_position) < 10.0f * PI / 180.0f)
            {
                list_pos->member.enemy.hurt_time = rt_tick_get();
                new_enemy = 0; //不是新敌人
            }
        }
        //更新敌方位置
        if (list_pos->member.enemy.hurt_type == 0x00)
            UI_ASSERT(Draw_Triangle(list_pos->member.enemy.id, GRAPH_OPERATION_MODIFY, X_OFFSET + (OUTER_SCOPE_CIRCLE_RADIUS - UI_ENEMY_SIZE) * sinf(list_pos->member.enemy.position), Y_OFFSET + (OUTER_SCOPE_CIRCLE_RADIUS - UI_ENEMY_SIZE) * cosf(list_pos->member.enemy.position), UI_ENEMY_SIZE, list_pos->member.enemy.position));

        if (rt_tick_get() - list_pos->member.enemy.hurt_time > UI_ENEMY_RESERVING_TIME) //标识显示3秒后清除
        {
            if (list_pos->member.enemy.hurt_type == 0x00)
                UI_ASSERT(Draw_Triangle(list_pos->member.enemy.id, GRAPH_OPERATION_ERASE, X_OFFSET + (OUTER_SCOPE_CIRCLE_RADIUS - UI_ENEMY_SIZE) * sinf(list_pos->member.enemy.position), Y_OFFSET + (OUTER_SCOPE_CIRCLE_RADIUS - UI_ENEMY_SIZE) * cosf(list_pos->member.enemy.position), UI_ENEMY_SIZE, list_pos->member.enemy.position));
            else if (list_pos->member.enemy.hurt_type == 0x05)
                UI_ASSERT(Draw_Wall(list_pos->member.enemy.id, GRAPH_OPERATION_ERASE, X_OFFSET + (OUTER_SCOPE_CIRCLE_RADIUS - UI_WALL_SIZE) * sinf(list_pos->member.enemy.position), Y_OFFSET + (OUTER_SCOPE_CIRCLE_RADIUS - UI_WALL_SIZE) * cosf(list_pos->member.enemy.position), UI_WALL_SIZE, list_pos->member.enemy.position));
            id_remain |= 1 << list_pos->member.enemy.id; // id空闲出来
            UI_List_Delete(list_pos);
        }
    }
    //若有新敌人则添加到链表末
    if (new_enemy)
    {
        //查询有无可用ID
        for (int i = 0; i < 8; i++)
        {
            if (id_remain & (1 << i))
            {
                id = i;
                break;
            }
        }
        if (id < 8)
        {
            UI_Enemy_List_Add(&enemy_list, id_offset + id * 5, enemy_position, rt_tick_get(), robot_hurt_data.hurt_type);
            id_remain &= ~(1 << id);
            if (robot_hurt_data.hurt_type == 0x00)
                UI_ASSERT(Draw_Triangle(id_offset + id * 5, GRAPH_OPERATION_ADD, X_OFFSET + (OUTER_SCOPE_CIRCLE_RADIUS - UI_ENEMY_SIZE) * sinf(enemy_position), Y_OFFSET + (OUTER_SCOPE_CIRCLE_RADIUS - UI_ENEMY_SIZE) * cosf(enemy_position), UI_ENEMY_SIZE, enemy_position));
            else if (robot_hurt_data.hurt_type == 0x05)
                UI_ASSERT(Draw_Wall(id_offset + id * 5, GRAPH_OPERATION_ADD, X_OFFSET + (OUTER_SCOPE_CIRCLE_RADIUS - UI_WALL_SIZE) * sinf(enemy_position), Y_OFFSET + (OUTER_SCOPE_CIRCLE_RADIUS - UI_WALL_SIZE) * cosf(enemy_position), UI_WALL_SIZE, enemy_position));
        }
    }
    imu_yaw_last = imu_yaw;
    return UI_OK;
}

/***
 * @brief    当建筑受到攻击时警告
 * @author   dxy
 ***/
ui_basic_e Draw_Building_Attacked_dyn(void *param)
{
    char txt[3][25] = {"SENTRY BEING ATTACKED !!", "OUTPOST BEING ATTACKED !!", "BASE BEING ATTACKED !!"};
    rt_uint8_t _building_attacked_flag = Ref_Get_Bulid_If_Hurt();
    static rt_uint32_t reserving_time[3];
    static rt_uint8_t drawn_flag;
    for (Ref_Build_e i = REF_OUTPOST; i <= REF_BASEMENT; i++)
    {
        //标志位置位,建筑血量变化
        if (_building_attacked_flag & (1 << i))
        {
            if (!(drawn_flag & (1 << i))) //未绘制
            {
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 200 + i, GRAPH_OPERATION_ADD, txt[i], UI_WARNING_SIZE, sizeof(txt[i]), 2, UI_WARNING_COLOR, UI_WARNING_CENTER_X - UI_MONITOR_SIZE * 4, UI_WARNING_CENTER_Y - 1.5f * UI_WARNING_SIZE * i));
                drawn_flag |= 1 << i;
                Ref_Reset_Bulid_If_Hurt(i);
                reserving_time[i] = rt_tick_get();
            }
            else //已绘制
            {
                reserving_time[i] = rt_tick_get(); //    说明建筑再次受到攻击,故刷新时间
            }
        }
        //建筑血量未变化
        else
        {
            if (drawn_flag & (1 << i))
            {
                //保留一段时间后清除图像
                if (rt_tick_get() - reserving_time[i] > UI_WARNING_RESERVING_TIME)
                {
                    UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 200 + i, GRAPH_OPERATION_ERASE, txt[i], UI_WARNING_SIZE, sizeof(txt[i]), 2, UI_WARNING_COLOR, UI_WARNING_CENTER_X - UI_MONITOR_SIZE * 4, UI_WARNING_CENTER_Y - 1.5f * UI_WARNING_SIZE * i));
                    drawn_flag &= ~(1 << i);
                }
            }
        }
    }
    return UI_OK;
}

/***
 * @brief    绘制车道线
 ***/
ui_basic_e Draw_LaneLine_bkgd(void *param)
{
#ifdef CORE_USING_BLACKHEAD_HERO
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 220, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET - 64 - OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.57029f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.57029f), 823, 1080 - 654));
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 221, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET + 64 + OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.57029f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.57029f), 1098, 1080 - 654));
#endif
#ifdef CORE_USING_C_BOARD_HERO
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 220, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET - 64 - OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.57029f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.57029f), 823, 1080 - 654));
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 221, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET + 64 + OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.57029f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.57029f), 1098, 1080 - 654));
#endif
#ifdef CORE_USING_ENGINEER
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 220, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 221, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET + OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET + SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
#endif
#ifdef CORE_USING_BLACKHEAD_INFANTRY
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 220, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 221, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET + OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET + SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
#endif
#ifdef CORE_USING_CLOUD_INFANTRY1
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 220, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 221, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET + OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET + SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
#endif
#ifdef CORE_USING_CLOUD_INFANTRY2
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 220, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 221, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET + OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET + SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
#endif
#ifdef CORE_USING_GREENHEAD_INFANTRY
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 220, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 221, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET + OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET + SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
#endif
#ifdef CORE_USING_SMALL_INFANTRY
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 220, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
    UI_ASSERT(custom_ui_append_line_operate(BACKGROUND_LAYER, 221, GRAPH_OPERATION_ADD, 2, UI_LANELINE_COLOR, X_OFFSET + OUTER_SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(0.5586f), X_OFFSET + SCOPE_CIRCLE_RADIUS * sinf(0.5586f), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(0.5586f)));
#endif

    return UI_OK;
}

/***
 * @name Draw_HeatLimit_dyn(void *param)
 * @brief  画出热量限制
 * @param	none
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author dxy
 ***/
ui_basic_e Draw_HeatLimit_dyn(void *param)
{

    static char txt[2][10] = {"ON ", "Off "};
    int status = 0;
    static int status_last = -1;
    status = Get_HeatLimit_Status();
    if (status != status_last || ui_erase_flg)
    {
        //写出当前运动模式
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 230, operation, txt[status], UI_HEATLIMIT_SIZE, sizeof(txt[status]), 2, UI_HEATLIMIT_DYN_COLOR, UI_HEATLIMIT_CENTER_X + UI_HEATLIMIT_SIZE * 8, UI_HEATLIMIT_CENTER_Y));
        status_last = status;
    }
    return UI_OK;
}

ui_basic_e Draw_HeatLimit_bkgd(void *param)
{
    static char mode[15] = "Heatlimit ";
    //写"motionmode"字样
    UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 231, GRAPH_OPERATION_ADD, mode, UI_HEATLIMIT_SIZE, sizeof(mode), 2, UI_HEATLIMIT_BKGD_COLOR, UI_HEATLIMIT_CENTER_X - UI_HEATLIMIT_SIZE * 4, UI_HEATLIMIT_CENTER_Y));
    return UI_OK;
}

/***
 * @name
 * @brief    显示当前运动模式(跟随/不跟随/陀螺)
 * @param
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author
 ***/
ui_basic_e Draw_Aimbot_Mode_dyn(void *param)
{
#ifdef CORE_USING_HERO
    static char string[5][15] = {"AIMBOT", "SNIPE", "ROTATE OUTPOST", "STATIC OUTPOST", "OUTPOST 3"};
#elif defined CORE_USING_INFANTRY
    static char string[3][15] = {"AIMBOT", "AIMBUFF SMALL", "AIMBUFF LARGE"};
#endif
    int mode = 0;
    static int mode_last = -1;
    mode = Get_Aimbot_Mode();
    if (mode != mode_last || ui_erase_flg)
    {
        //写出当前运动模式
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 240, operation, string[mode], UI_AIMBOT_MODE_SIZE, sizeof(string[mode]), 2, UI_AIMBOT_MODE_DYN_COLOR, UI_AIMBOT_MODE_CENTER_X + UI_AIMBOT_MODE_SIZE * 8, UI_AIMBOT_MODE_CENTER_Y));
        mode_last = mode;
    }

    return UI_OK;
}
/***
 * @brief    背景绘制
 ***/
ui_basic_e Draw_Aimbot_Mode_bkgd(void *param)
{

    static char mode[10] = "aim mode";
    //写"motionmode"字样
    UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 241, GRAPH_OPERATION_ADD, mode, UI_AIMBOT_MODE_SIZE, sizeof(mode), 2, UI_AIMBOT_MODE_BKGD_COLOR, UI_AIMBOT_MODE_CENTER_X - UI_AIMBOT_MODE_SIZE * 4, UI_AIMBOT_MODE_CENTER_Y));

    return UI_OK;
}

/***
 * @name
 * @brief    显示Pitch轴角度
 * @param
 * @retval   UI_OK       操作成功
 * @retval   UI_FULL     发送队列已满
 * @retval   UI_CHAR     发送字节成功
 * @author   dxy
 ***/
ui_basic_e Draw_PITCH_dyn(void *param)
{
    static float pitch_last = 0;
    float delta_pitch;
    float pitch;
    pitch = Get_Gimbal_IMU_Data(IMU_PITCH);
    pitch_last = pitch;
    delta_pitch = pitch - pitch_last;
    // pitch值
    UI_ASSERT(custom_ui_append_float_operate(DYNAMIC_LAYER, 280, operation, UI_PITCH_SIZE, 2, 2, UI_PITCH_DYN_COLOR, UI_ABS_PITCH_CENTER_X, UI_ABS_PITCH_CENTER_Y, pitch));

    // pitch相对值
    UI_ASSERT(custom_ui_append_float_operate(DYNAMIC_LAYER, 281, operation, UI_PITCH_SIZE, 2, 2, UI_PITCH_DYN_COLOR, UI_RELAT_PITCH_CENTER_X, UI_RELAT_PITCH_CENTER_Y, delta_pitch));
    return UI_OK;
}
/***
 * @brief    背景绘制
 ***/
ui_basic_e Draw_PITCH_bkgd(void *param)
{

    static char string[10] = "pitch";
    //写"motionmode"字样
    UI_ASSERT(custom_ui_draw_text(BACKGROUND_LAYER, 288, GRAPH_OPERATION_ADD, string, UI_PITCH_SIZE, sizeof(string), 2, UI_PITCH_BKGD_COLOR, UI_ABS_PITCH_CENTER_X, UI_ABS_PITCH_CENTER_Y + UI_PITCH_SIZE * 1.5f));
    return UI_OK;
}
/***
 * @brief  选项卡菜单清除
 * @author dzq
 * **/
void Option_Clear(rt_int8_t Option_Flag_Mode, rt_int8_t Option_Flag_Last)
{
    /*底盘模式*/
    static char motion_option_string[7][18] = {"MOTIONMODE", "G. NOFOLLOW", "E. FOLLOW", "R. SLOWGYRO", "W. FASTGYRO", "C. MOVEBACK"};
    /*自瞄模式*/
#if defined CORE_USING_HERO
    static char aimbot_option_string[6][18] = {"AIMBOTMODE", "G. AIMBOT", "Z. OUTPOST 1", "X. OUTPOST 2", "C. OUTPOST 3", "V. SNIPE"};
#elif defined CORE_USING_INFANTRY
    static char aimbot_option_string[4][18] = {"AIMBOTMODE", "G. AIMBOT", "V. SMALL AIMBUFF", "B. LARGE AIMBUFF"};
#endif
    /*杂项选项卡*/
#if defined CORE_USING_INFANTRY
    static char misc_option_string[7][18] = {"MISC", "Z. UI_RESET", "G. SC_CTL", "R. ROBOT_RESET", "C. OPEN MAGAZINE", "V. CLOSE MAGAZINE"};
#elif defined CORE_USING_HERO
    static char misc_option_string[5][18] = {"MISC", "Z. UI_RESET", "G. SC_CTL", "R. ROBOT_RESET"};
#endif
    /*弹速设置选项卡*/
    static char GunSpeed_option_string[4][18] = {"GUN_SPEED", "Z. 10 M", "X. 16 M"};
    /*机器人离线性能设置*/
    static char Performance_option_string[6][18] = {"Performance", "Z. Level 1", "X. Level 2", "C. Level 3", "B. Other_SET"};
    static char Performance_Choose_option_string[8][18] = {"Performance_Choose", "Z. Burst", "C. Speed", "V. Power", "B. HP", "G. REF_YES", "R. REF_NO"};
		/*杂项选项卡——单片机复位菜单*/
    static char reset_option_string[5][18] = {"MISC——RESET", "G. GIMBAL", "C. CHASSIS", "B. ALL_RESET"};
    /*杂项选项卡——超级电容控制菜单*/
    static char SC_Ctl_option_string[4][18] = {"MISC——SC_CTL", "C. OPEN_SC", "V. ClOSE_SC"};
		switch (Option_Flag_Last)
    {
    case MainMenu:

        break;
    case ChassisModeMenu:
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, motion_option_string[0], UI_OPTION_SIZE, sizeof(motion_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, motion_option_string[1], UI_OPTION_SIZE, sizeof(motion_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, motion_option_string[2], UI_OPTION_SIZE, sizeof(motion_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ERASE, motion_option_string[3], UI_OPTION_SIZE, sizeof(motion_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ERASE, motion_option_string[4], UI_OPTION_SIZE, sizeof(motion_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 256, GRAPH_OPERATION_ERASE, motion_option_string[5], UI_OPTION_SIZE, sizeof(motion_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 8 * UI_OPTION_SIZE));
        break;
    case AimModeMenu:
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, aimbot_option_string[0], UI_OPTION_SIZE, sizeof(aimbot_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, aimbot_option_string[1], UI_OPTION_SIZE, sizeof(aimbot_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, aimbot_option_string[2], UI_OPTION_SIZE, sizeof(aimbot_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ERASE, aimbot_option_string[3], UI_OPTION_SIZE, sizeof(aimbot_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
#if defined CORE_USING_HERO
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ERASE, aimbot_option_string[4], UI_OPTION_SIZE, sizeof(aimbot_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 256, GRAPH_OPERATION_ERASE, aimbot_option_string[5], UI_OPTION_SIZE, sizeof(aimbot_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 8 * UI_OPTION_SIZE));
#endif
        break;
    case MiscMenu:
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, misc_option_string[0], UI_OPTION_SIZE, sizeof(misc_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, misc_option_string[1], UI_OPTION_SIZE, sizeof(misc_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, misc_option_string[2], UI_OPTION_SIZE, sizeof(misc_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ERASE, misc_option_string[3], UI_OPTION_SIZE, sizeof(misc_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
#if defined CORE_USING_INFANTRY
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ERASE, misc_option_string[4], UI_OPTION_SIZE, sizeof(misc_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 256, GRAPH_OPERATION_ERASE, misc_option_string[5], UI_OPTION_SIZE, sizeof(misc_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 8 * UI_OPTION_SIZE));
#endif
        break;
    case RobotResetMenu:
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, reset_option_string[0], UI_OPTION_SIZE, sizeof(reset_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, reset_option_string[1], UI_OPTION_SIZE, sizeof(reset_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, reset_option_string[2], UI_OPTION_SIZE, sizeof(reset_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ERASE, reset_option_string[3], UI_OPTION_SIZE, sizeof(reset_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
        break;
    case SCAPCtrlMenu:
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, SC_Ctl_option_string[0], UI_OPTION_SIZE, sizeof(SC_Ctl_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, SC_Ctl_option_string[1], UI_OPTION_SIZE, sizeof(SC_Ctl_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, SC_Ctl_option_string[2], UI_OPTION_SIZE, sizeof(SC_Ctl_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
        break;
    case GunSpeedMenu:
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, GunSpeed_option_string[0], UI_OPTION_SIZE, sizeof(GunSpeed_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, GunSpeed_option_string[1], UI_OPTION_SIZE, sizeof(GunSpeed_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, GunSpeed_option_string[2], UI_OPTION_SIZE, sizeof(GunSpeed_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
        break;
    case PerformanceMenu:
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, Performance_option_string[0], UI_OPTION_SIZE, sizeof(Performance_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, Performance_option_string[1], UI_OPTION_SIZE, sizeof(Performance_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, Performance_option_string[2], UI_OPTION_SIZE, sizeof(Performance_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ERASE, Performance_option_string[3], UI_OPTION_SIZE, sizeof(Performance_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ERASE, Performance_option_string[4], UI_OPTION_SIZE, sizeof(Performance_option_string[4]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
        break;
    case PerformanceChooseMenu:
		UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, Performance_Choose_option_string[0], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, Performance_Choose_option_string[1], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, Performance_Choose_option_string[2], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ERASE, Performance_Choose_option_string[3], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ERASE, Performance_Choose_option_string[4], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[4]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
        UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 256, GRAPH_OPERATION_ERASE, Performance_Choose_option_string[5], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[5]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 8 * UI_OPTION_SIZE));
								
        break;
    default:

        break;
    }
}
/***
 * @brief    各类选项卡绘制
 ***/
ui_basic_e Draw_option(void *param)
{   
    static char Temp_CNT = 0;
    rt_int8_t Option_Flag_Mode = Get_UI_Option_Flag();
    static rt_int8_t Option_Flag_Last = 0;
    /*底盘模式*/
    static char motion_option_string[7][18] = {"MOTIONMODE", "G. NOFOLLOW", "E. FOLLOW", "R. SLOWGYRO", "W. FASTGYRO", "C. MOVEBACK"};
    /*自瞄模式*/
#if defined CORE_USING_HERO
    static char aimbot_option_string[6][18] = {"AIMBOTMODE", "G. AIMBOT", "Z. OUTPOST 1", "X. OUTPOST 2", "C. OUTPOST 3", "V. SNIPE"};
#elif defined CORE_USING_INFANTRY
    static char aimbot_option_string[4][18] = {"AIMBOTMODE", "G. AIMBOT", "V. SMALL AIMBUFF", "B. LARGE AIMBUFF"};
#endif
    /*杂项选项卡*/
#if defined CORE_USING_INFANTRY
    static char misc_option_string[7][18] = {"MISC", "Z. UI_RESET", "G. SC_CTL", "R. ROBOT_RESET", "C. OPEN MAGAZINE", "V. CLOSE MAGAZINE"};
#elif defined CORE_USING_HERO
    static char misc_option_string[5][18] = {"MISC", "Z. UI_RESET", "G. SC_CTL", "R. ROBOT_RESET"};
#endif
    /*杂项选项卡——单片机复位菜单*/
    static char reset_option_string[5][18] = {"MISC——RESET", "G. GIMBAL", "C. CHASSIS", "B. ALL_RESET"};
    /*杂项选项卡——超级电容控制菜单*/
    static char SC_Ctl_option_string[4][18] = {"MISC——SC_CTL", "C. OPEN_SC", "V. ClOSE_SC"};
    /*弹速设置选项卡*/
    static char GunSpeed_option_string[4][18] = {"GUN_SPEED", "Z. 10 M", "X. 16 M"};
    /*机器人离线性能设置*/
    static char Performance_option_string[6][18] = {"Performance", "Z. Level 1", "X. Level 2", "C. Level 3", "B. Other_SET"};
    /*机器人离线性能选择设置*/
    static char Performance_Choose_option_string[8][18] = {"Performance_Choose", "Z. Burst", "C. Speed", "V. Power", "B. HP", "G. REF_YES", "R. REF_NO"};
    if (Option_Flag_Mode != Option_Flag_Last)
    {
        switch (Option_Flag_Mode)
        {
        case MainMenu:
            Temp_CNT = 0;
            Option_Clear(Option_Flag_Mode, Option_Flag_Last);
            Option_Flag_Last = Option_Flag_Mode; //所有图形发送完后再设标志位
            break;
        case ChassisModeMenu:
            //画出选择界面
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ADD, motion_option_string[0], UI_OPTION_SIZE, sizeof(motion_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ADD, motion_option_string[1], UI_OPTION_SIZE, sizeof(motion_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ADD, motion_option_string[2], UI_OPTION_SIZE, sizeof(motion_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ADD, motion_option_string[3], UI_OPTION_SIZE, sizeof(motion_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ADD, motion_option_string[4], UI_OPTION_SIZE, sizeof(motion_option_string[4]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 256, GRAPH_OPERATION_ADD, motion_option_string[5], UI_OPTION_SIZE, sizeof(motion_option_string[5]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 8 * UI_OPTION_SIZE));
            Option_Flag_Last = Option_Flag_Mode; //所有图形发送完后再设标志位
            break;
        case AimModeMenu:
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ADD, aimbot_option_string[0], UI_OPTION_SIZE, sizeof(aimbot_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ADD, aimbot_option_string[1], UI_OPTION_SIZE, sizeof(aimbot_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ADD, aimbot_option_string[2], UI_OPTION_SIZE, sizeof(aimbot_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ADD, aimbot_option_string[3], UI_OPTION_SIZE, sizeof(aimbot_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
#if defined CORE_USING_HERO
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ADD, aimbot_option_string[4], UI_OPTION_SIZE, sizeof(aimbot_option_string[4]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 256, GRAPH_OPERATION_ADD, aimbot_option_string[5], UI_OPTION_SIZE, sizeof(aimbot_option_string[5]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 8 * UI_OPTION_SIZE));
#endif
            Option_Flag_Last = Option_Flag_Mode; //所有图形发送完后再设标志位
            break;
        case MiscMenu:
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ADD, misc_option_string[0], UI_OPTION_SIZE, sizeof(misc_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ADD, misc_option_string[1], UI_OPTION_SIZE, sizeof(misc_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ADD, misc_option_string[2], UI_OPTION_SIZE, sizeof(misc_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ADD, misc_option_string[3], UI_OPTION_SIZE, sizeof(misc_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
#if defined CORE_USING_INFANTRY
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ADD, misc_option_string[4], UI_OPTION_SIZE, sizeof(misc_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 256, GRAPH_OPERATION_ADD, misc_option_string[5], UI_OPTION_SIZE, sizeof(misc_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 8 * UI_OPTION_SIZE));
#endif
            Option_Flag_Last = Option_Flag_Mode; //所有图形发送完后再设标志位
            break;
        case RobotResetMenu:
            if (Temp_CNT == 0)
            {
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, misc_option_string[0], UI_OPTION_SIZE, sizeof(misc_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, misc_option_string[1], UI_OPTION_SIZE, sizeof(misc_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, misc_option_string[2], UI_OPTION_SIZE, sizeof(misc_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ERASE, misc_option_string[3], UI_OPTION_SIZE, sizeof(misc_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
                Temp_CNT++;
            }
            else if (Temp_CNT == 1)
            {
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ADD, reset_option_string[0], UI_OPTION_SIZE, sizeof(reset_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ADD, reset_option_string[1], UI_OPTION_SIZE, sizeof(reset_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ADD, reset_option_string[2], UI_OPTION_SIZE, sizeof(reset_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ADD, reset_option_string[3], UI_OPTION_SIZE, sizeof(reset_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
                Option_Flag_Last = Option_Flag_Mode; //所有图形发送完后再设标志位
            }
            break;
        case SCAPCtrlMenu:
            if (Temp_CNT == 0)
            {
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, misc_option_string[0], UI_OPTION_SIZE, sizeof(misc_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, misc_option_string[1], UI_OPTION_SIZE, sizeof(misc_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, misc_option_string[2], UI_OPTION_SIZE, sizeof(misc_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ERASE, misc_option_string[3], UI_OPTION_SIZE, sizeof(misc_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
                Temp_CNT++;
            }
            else if (Temp_CNT == 1)
            {
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ADD, SC_Ctl_option_string[0], UI_OPTION_SIZE, sizeof(SC_Ctl_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ADD, SC_Ctl_option_string[1], UI_OPTION_SIZE, sizeof(SC_Ctl_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ADD, SC_Ctl_option_string[2], UI_OPTION_SIZE, sizeof(SC_Ctl_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
                Option_Flag_Last = Option_Flag_Mode; //所有图形发送完后再设标志位
            }

            break;
        case GunSpeedMenu:
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ADD, GunSpeed_option_string[0], UI_OPTION_SIZE, sizeof(GunSpeed_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ADD, GunSpeed_option_string[1], UI_OPTION_SIZE, sizeof(GunSpeed_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ADD, GunSpeed_option_string[2], UI_OPTION_SIZE, sizeof(GunSpeed_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
            Option_Flag_Last = Option_Flag_Mode; //所有图形发送完后再设标志位
            break;
        case PerformanceMenu:
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ADD, Performance_option_string[0], UI_OPTION_SIZE, sizeof(Performance_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ADD, Performance_option_string[1], UI_OPTION_SIZE, sizeof(Performance_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ADD, Performance_option_string[2], UI_OPTION_SIZE, sizeof(Performance_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ADD, Performance_option_string[3], UI_OPTION_SIZE, sizeof(Performance_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
            UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ADD, Performance_option_string[4], UI_OPTION_SIZE, sizeof(Performance_option_string[4]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
            Option_Flag_Last = Option_Flag_Mode; //所有图形发送完后再设标志位
            break;
        case PerformanceChooseMenu:
            if (Temp_CNT == 0)
            {
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ERASE, Performance_option_string[0], UI_OPTION_SIZE, sizeof(Performance_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ERASE, Performance_option_string[1], UI_OPTION_SIZE, sizeof(Performance_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ERASE, Performance_option_string[2], UI_OPTION_SIZE, sizeof(Performance_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ERASE, Performance_option_string[3], UI_OPTION_SIZE, sizeof(Performance_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ERASE, Performance_option_string[4], UI_OPTION_SIZE, sizeof(Performance_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
				Temp_CNT++;
            }
            else if (Temp_CNT == 1)
            {
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 251, GRAPH_OPERATION_ADD, Performance_Choose_option_string[0], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[0]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X, UI_OPTION_CENTER_Y));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 252, GRAPH_OPERATION_ADD, Performance_Choose_option_string[1], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[1]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 2 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 253, GRAPH_OPERATION_ADD, Performance_Choose_option_string[2], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[2]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 3.5 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 254, GRAPH_OPERATION_ADD, Performance_Choose_option_string[3], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[3]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 5 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 255, GRAPH_OPERATION_ADD, Performance_Choose_option_string[4], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[4]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 6.5 * UI_OPTION_SIZE));
                UI_ASSERT(custom_ui_draw_text(DYNAMIC_LAYER, 256, GRAPH_OPERATION_ADD, Performance_Choose_option_string[5], UI_OPTION_SIZE, sizeof(Performance_Choose_option_string[5]), 2, UI_OPTION_COLOR, UI_OPTION_CENTER_X - UI_OPTION_SIZE, UI_OPTION_CENTER_Y - 8 * UI_OPTION_SIZE));
				Option_Flag_Last = Option_Flag_Mode; //所有图形发送完后再设标志位
			}
            
            break;
        default:

            break;
        }
    }
    return UI_OK;
}
/***
 * @brief    set the flag when UI finished drawing.
 ***/
ui_basic_e UI_Draw_END(void *param)
{
    UI_Set_Init_Flag();
    return UI_OK;
}

void UI_Clear_Init_Flag(void)
{
    ui_erase_flg = 1;
    operation = GRAPH_OPERATION_ADD;
}

void UI_Set_Init_Flag(void)
{
    ui_erase_flg = 0;
    operation = GRAPH_OPERATION_MODIFY;
}

////////////////////////////////////////////////////////END////////////////////////////////////////////////////////
