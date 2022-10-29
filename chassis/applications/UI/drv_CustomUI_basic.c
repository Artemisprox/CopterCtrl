/****
 * drv_Custom_UI_basic.c/h
 * 裁判系统绘制自定义uI的图形操作库，定义了通用的自定义ui绘制包结构体，
 * 将线段、矩形、圆、椭圆、圆弧、整形、浮点数和字符串绘制的操作封装成函数，
 * 便于自定义UI绘制的快速开发。
 *
 * 使用说明:
 * 声明一个 custom_ui_buff_t变量,并使用 custom_ui_init函数对其进行初始化.
 * 若要进行线段、矩形、圆、椭圆、圆弧、整形、浮点数的操作,则调用相应的图形操作新增函数对custom_ui_buff_t变量进行处理即可
 * 注意图形操作的数量不可大干7
 * 若要进行图像删除,或者字符串绘制的操作,则无法与别的图形操作叠加，须单次调用custom_ui_erase_graph或者 ui_draw_text函数处理
 * 对 custom_ui_buff变量的图像操作完成后,即可通过裁判系统交互发送函数将其直接发送给对应客户端即可,
 * 其中,内容ID,为 custom_ui_buff_t变量内的cmd成员:图像数据长度(不包括3个ID的6个字节),为其中的length成员,全部为自动生成。
***/

#include "mod_RefSystem.h"
#include "drv_CRC.h"
#include "drv_CustomUI_basic.h"
#include "string.h"

const uint16_t LENGTH_DRAW_NUM[7] = {LENGTH_DRAW_1_GRAPH,LENGTH_DRAW_2_GRAPH,LENGTH_DRAW_5_GRAPH,LENGTH_DRAW_5_GRAPH,LENGTH_DRAW_5_GRAPH,LENGTH_DRAW_7_GRAPH,LENGTH_DRAW_7_GRAPH};
//1-7的图像数量对应的内容数据段长度
const uint16_t CMD_DRAW_NUM[7] = {CMD_DRAW_1_GRAPH, CMD_DRAW_2_GRAPH,CMD_DRAW_5_GRAPH,CMD_DRAW_5_GRAPH,CMD_DRAW_5_GRAPH,CMD_DRAW_7_GRAPH,CMD_DRAW_7_GRAPH};
//1-7的图像搡作数量对应的内容ID

static graphic_data_struct_t graph_data_struct_init(void);
static ext_client_custom_character_t char_data_struct_init(void);
static custom_ui_buff_t ui;
static rt_device_t serial;
static uint8_t referee_intercom_buff[128] = {0};

/**
*@brief 将自定义ui绘制包填充为清除图层
*@param ui:需要修改的自定义ui绘制包结构体,参数是输入
*@param clear_type:清除类型枚举clear_graph_type_t
*@param layer：需要清除的图层号，仅在clear_type为CLEAR_GRAPH_TYPE_CLEAR_LAYER时有效
*@retval UI_FULL:发送队列已满(擦除函数只能单独调用)
*@retval UI_OK: 操作成功
*/
ui_basic_e custom_ui_erase_graph(clear_graph_type_t clear_type, uint8_t layer)
{
    if (ui.graph_num > 0)
        return UI_FULL;
    ui.length =2;
    ui.cmd = CMD_ERASE_GRAPH;
    ui.graph_num = 1;
    ui.data_buff[0] = clear_type;
    ui.data_buff[1] = layer;
    return UI_OK;
}

/**
*@brief 将自定义ui绘制包填充为图像绘制
*@param ui:需要修改的自定义ui绘制包结构体,参数是输入;
*@param txt:绘制的字符串地址,参数是输入
*@param layer:图层编号
*@param name:图像名称,最大长度3字节
*@param operation:图形操作类型枚举graph_operation_t
*@param text:绘制的字符串地址
*@param font size字体大小
*@param length:绘制的字符串长度
*@param width:绘制线宽
*@param color:颜色
*@param start:字符串绘制坐标x
*@param starty:字符串绘制坐标y
*@retval UI_FULL:发送队列已满(发送字节函数只能单独调用)
*@retval UI_CHAR: 发送字节成功
*/
ui_basic_e custom_ui_draw_text(uint8_t layer, uint32_t name, graph_operation_t operation,
                             char *text, uint16_t font_size,
                             uint16_t length, uint16_t width,
                             graph_color_t color, uint16_t start_x, uint16_t start_y)
{
    ext_client_custom_character_t custom_char = char_data_struct_init(); //返回初始化完毕的字符ui绘制结构体
    if(ui.graph_num>0)
        return UI_FULL;
    memcpy(custom_char.grapic_data_struct.graphic_name, &name, 3);
    custom_char.grapic_data_struct.layer = layer;
    custom_char.grapic_data_struct.graphic_tpye = GRAPH_TYPE_CHAR;
    custom_char.grapic_data_struct.operate_tpye = operation;
    custom_char.grapic_data_struct.start_angle = font_size;
    custom_char.grapic_data_struct.end_angle = length;
    custom_char.grapic_data_struct.width = width;
    custom_char.grapic_data_struct.color = color;
    custom_char.grapic_data_struct.start_x =start_x;
    custom_char.grapic_data_struct.start_y =start_y;

    memcpy(custom_char.data,text,length);
    memcpy(ui.data_buff, &custom_char, LENGTH_DRAW_TEXT);
    ui.graph_num = 1 ;
    ui.length = LENGTH_DRAW_TEXT;
    ui.cmd = CMD_DRAW_TEXT;
    return UI_CHAR;
}

/**
*@brief 向自定义ui绘制包内新增-线条绘制操作
*@param ui: 需要修改的自定义ui绘制包结构体
*@param	layer:图层编号
*@param	name:图像名称，最大长度3字节
*@param	operation:图形操作类型枚举graph_operation_t
*@param	width:绘制线宽
*@param	color:颜色
*@param	start_ x:起点坐标x
*@param	start_ y:起点坐标y
*@param	end_х : 终点坐标x
*@param	end_y : 终点坐标y
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
ui_basic_e custom_ui_append_line_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                         uint16_t width, graph_color_t color,
                                         uint16_t start_x, uint16_t start_y,
                                         uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t graph_data = graph_data_struct_init();
    if(ui.graph_num>=7)
        return UI_FULL;
    memcpy(graph_data.graphic_name,&name,3);
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_LINE;
    graph_data.operate_tpye = operation;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = start_x;
    graph_data.start_y = start_y;
    graph_data. property.graph_property.end_x = end_x;
    graph_data. property.graph_property.end_y = end_y;
    memcpy(ui.data_buff + ui.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH) ;
    ui.graph_num ++;
    ui.length = LENGTH_DRAW_NUM[ui.graph_num - 1] ;
    ui.cmd = CMD_DRAW_NUM[ui.graph_num - 1];
    return UI_OK;
}

/**
*@brief	向自定义ui绘制包内新增-矩形绘制操作
*@param ui:需要修改的自定义ui绘制包结构体，参数是输入
*@param	layer:图层编号
*@param	name:图像名称，最大长度3字节
*@param	operat 1on:图形操作类型枚举graph_operation_t
*@param	width:绘制线宽
*@param	color:颜色
*@param	start_ x:起点坐标x
*@param	start_ y:起点坐标y
*@param	end_x:对角顶点x坐标
*@param	end_y:对角顶点y坐标
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
ui_basic_e custom_ui_append_rect_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                         uint16_t width, graph_color_t color,
                                         uint16_t start_x, uint16_t start_y,
                                         uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t graph_data = graph_data_struct_init();
    if (ui.graph_num>=7)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name,3) ;
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_RECT;
    graph_data.operate_tpye = operation;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = start_x;
    graph_data.start_y = start_y;
    graph_data. property.graph_property.end_x = end_x;
    graph_data. property.graph_property.end_y = end_y;
    memcpy(ui.data_buff + ui.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH) ;
    ui.graph_num ++;
    ui.length = LENGTH_DRAW_NUM[ui.graph_num - 1] ;
    ui.cmd = CMD_DRAW_NUM[ui.graph_num - 1];
    return UI_OK;
}

/*0brief	向自定义ui绘制包内新增-radious : 半径
@parem[: in] [out] ui:需要修改的自定义ui绘制包结构体，参数是输入
@par am[ in]	layer:图层编号
				name:图像名称，最大长度3字节
				operat 1on:图形操作类型枚举graph_operation_t
				width:绘制线宽
				color:颜色
				centre_x:圆心坐标x
				centre_y:圆心坐标y
				radious : 半径
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
ui_basic_e custom_ui_append_cirle_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                          uint16_t width, graph_color_t color,
                                          uint16_t centre_x, uint16_t centre_y,
                                          uint16_t radious)
{
    graphic_data_struct_t graph_data = graph_data_struct_init();
    if (ui.graph_num>=7)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name,3) ;
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_CIRCLE;
    graph_data.operate_tpye = operation;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = centre_x;
    graph_data.start_y = centre_y;
    graph_data. property.graph_property.radious=radious;
    memcpy(ui.data_buff + ui.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH) ;
    ui.graph_num ++;
    ui.length = LENGTH_DRAW_NUM[ui.graph_num - 1];
    ui.cmd = CMD_DRAW_NUM[ui.graph_num - 1];
    return UI_OK;
}

/*0brief	向自定义ui绘制包内新增-椭圆绘制操作
@parem[: in] [out] ui:需要修改的自定义ui绘制包结构体，参数是输入
@par am[ in]	layer:图层编号
				name:图像名称，最大长度3字节
				operat 1on:图形操作类型枚举graph_operation_t
				width:绘制线宽
				color:颜色
				centre_x:圆心坐标x
				centre_y:圆心坐标y
				axis_x : x半轴长度
				axis_y : y半轴长度
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
ui_basic_e custom_ui_append_ellipse_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                            uint16_t width, graph_color_t color,
                                            uint16_t centre_x, uint16_t centre_y,
                                            uint16_t axis_x, uint16_t axis_y)
{
    graphic_data_struct_t graph_data = graph_data_struct_init();
    if (ui.graph_num>=7)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name,3) ;
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_ELLIPSE;
    graph_data.operate_tpye = operation;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = centre_x;
    graph_data.start_y = centre_y;
    graph_data. property.graph_property.end_x = axis_x;
    graph_data. property.graph_property.end_y = axis_y;
    memcpy(ui.data_buff + ui.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH) ;
    ui.graph_num ++;
    ui.length = LENGTH_DRAW_NUM[ui.graph_num - 1] ;
    ui.cmd = CMD_DRAW_NUM[ui.graph_num - 1];
    return UI_OK;
}

/*0brief	向自定义ui绘制包内新增-（椭）圆弧绘制操作
@parem[: in] [out] ui:需要修改的自定义ui绘制包结构体，参数是输入
@par am[ in]	layer:图层编号
				name:图像名称，最大长度3字节
				start_angle
				end_angle
				operation:图形操作类型枚举graph_operation_t
				width:绘制线宽
				color:颜色
				centre_x:圆心坐标x
				centre_y:圆心坐标y
				radious : 半径
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
ui_basic_e custom_ui_append_arc_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                        uint16_t start_angle, uint16_t end_angle,
                                        uint16_t width, graph_color_t color,
                                        uint16_t centre_x, uint16_t centre_y,
                                        uint16_t axis_x, uint16_t axis_y)
{
    graphic_data_struct_t graph_data = graph_data_struct_init();
    if (ui.graph_num>=7)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name,3) ;
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_ARC;
    graph_data.operate_tpye = operation;
    graph_data.start_angle = start_angle;
    graph_data.end_angle = end_angle;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = centre_x;
    graph_data.start_y = centre_y;
    graph_data. property.graph_property.end_x = axis_x;
    graph_data. property.graph_property.end_y = axis_y;
    memcpy(ui.data_buff + ui.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH) ;
    ui.graph_num ++;
    ui.length = LENGTH_DRAW_NUM[ui.graph_num - 1] ;
    ui.cmd = CMD_DRAW_NUM[ui.graph_num - 1];
    return UI_OK;
}

/*0brief	向自定义ui绘制包内新增-浮点数绘制操作
@parem[: in] [out] ui:需要修改的自定义ui绘制包结构体，参数是输入
@par am[ in]	layer:图层编号
				name:图像名称，最大长度3字节
				operation:图形操作类型枚举graph_operation_t
				font_size:字体大小
				digit:小数点位数
				width:绘制线宽
				color:颜色
				start_x:起点坐标x
				start_y:起点坐标y
				value : 要绘制的浮点数
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
ui_basic_e custom_ui_append_float_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                          uint16_t font_size, uint16_t digit,
                                          uint16_t width, graph_color_t color,
                                          uint16_t start_x, uint16_t start_y, float value)
{
    graphic_data_struct_t graph_data = graph_data_struct_init();
    if (ui.graph_num>=7)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name,3) ;
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_FLOAT;
    graph_data.operate_tpye = operation;
    graph_data.start_angle = font_size;
    graph_data.end_angle = digit;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = start_x;
    graph_data.start_y = start_y;
    graph_data.property.float_val = (int32_t)(value * 1000);

    memcpy(ui.data_buff + ui.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH) ;
    ui.graph_num ++;
    ui.length = LENGTH_DRAW_NUM[ui.graph_num - 1] ;
    ui.cmd = CMD_DRAW_NUM[ui.graph_num - 1];
    return UI_OK;
}

/*0brief	向自定义ui绘制包内新增-整形绘制操作
@parem[: in] [out] ui:需要修改的自定义ui绘制包结构体，参数是输入
@par am[ in]	layer:图层编号
				name:图像名称，最大长度3字节
				operation:图形操作类型枚举graph_operation_t
				font_size:字体大小
				width:绘制线宽
				color:颜色
				start_x:起点坐标x
				start_y:起点坐标y
				value : 要绘制的整形
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
ui_basic_e custom_ui_append_int_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                        uint16_t font_size, uint16_t width, graph_color_t color,
                                        uint16_t start_x, uint16_t start_y, int32_t value)
{
    graphic_data_struct_t graph_data = graph_data_struct_init();
    if (ui.graph_num>=7)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name,3) ;
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_INT;
    graph_data.operate_tpye = operation;
    graph_data.start_angle = font_size;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = start_x;
    graph_data.start_y = start_y;
    graph_data. property.int_val = value;

    memcpy(ui.data_buff + ui.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH) ;
    ui.graph_num ++;
    ui.length = LENGTH_DRAW_NUM[ui.graph_num - 1] ;
    ui.cmd = CMD_DRAW_NUM[ui.graph_num - 1];
    return UI_OK;
}


/*
@brier				初始化自定义ui绘制包结构体
@param[in][out]		ui:需要初始化的自定义u1绘制包结构体指针
@retval				none
*/
void custom_ui_init(void)
{
    ui.length = 0;
    ui.length = 0;
    ui.graph_num = 0;//相关参数清零
}


/*
@brief		返回初始化完毕的图形绘制原始数据结构体
@Bparem[in] [out] none
@param[in]	none
@retval		graphic_data_struct_t
*/
static graphic_data_struct_t graph_data_struct_init(void)
{
    graphic_data_struct_t graph_data;
    memset(&graph_data,0x00, sizeof (graphic_data_struct_t)) ;
    return graph_data;
}


/*
@brief			返回初始化完毕的字符ui绘制结构体
@param[in][out] none
@parem[ in]		none
@retval			初始化完毕的字符ui绘制结构体ext_client_custom_character_t
*/
static ext_client_custom_character_t char_data_struct_init(void)
{
    ext_client_custom_character_t char_data;
    memset(&char_data, 0x00, sizeof(ext_client_custom_character_t));
    return char_data;
}

/***
* @name
* @brief	向裁判系统传输UI数据
* @param	senderID: 发送者ID:	红方机器人	英雄 1; 工程 2; 步兵 3/4/5; 空中 6;
													 蓝方机器人    英雄 11; 工程 12; 步兵 13/14/15; 空中 16;
* @param	receiverID: 接受者ID 红方操作手客户端	英雄 0x101; 工程 0x102; 步兵 0x103/104/105; 空中 0x106;
													 蓝方操作手客户端    英雄 0x111; 工程 0x112; 步兵 0x113/114/115; 空中 0x116;
* @retval
***/
rt_err_t ui_referee_intercom_tranamit(uint16_t senderID, uint16_t receiverID)
{
    uint16_t index = 0;
    uint16_t header = ID_student_interactive_data;
    uint16_t datalength = ui.length + 6;
    uint16_t tempcrc16 = 0;
    uint16_t sender_Id = (uint16_t)senderID;
    uint16_t receiver_id = (uint16_t)receiverID;
    static uint8_t seq = 0x01; //sequence number of frame
    if(ui.graph_num == 0)   //若发送包中无数据,则添加一个空操作,否则会花屏
    {
        custom_ui_append_line_operate(0, 0xff0, GRAPH_OPERATION_NOOP,1, GRAPH_COLOR_BLACK, 1, 1, 1, 1);
    }
    memset(referee_intercom_buff, 0x00, sizeof(referee_intercom_buff));
    referee_intercom_buff[index] = HEADER_SOF;
    index += 1; //index =1
    memcpy(referee_intercom_buff + index, &datalength, 2);
    index += 2; //index = 3
    memcpy(referee_intercom_buff + index, &seq, 1);
    //seq++;
    index += 1; //index = 4
    referee_intercom_buff[index] = Get_CRC8_Check_Sum(referee_intercom_buff, index, CRC8_INIT);
    index += 1; //index = 5
    memcpy(referee_intercom_buff + index, &header, 2);
    index += 2; //index - 7

    memcpy(referee_intercom_buff + index, &ui.cmd, 2);
    index += 2; //index ■9
    memcpy(referee_intercom_buff + index, &sender_Id, 2);
    index += 2; //index■11
    memcpy(referee_intercom_buff + index, &receiver_id, 2);
    index += 2; //index = 13
    memcpy(referee_intercom_buff + index, ui.data_buff, ui.length);
    index += ui.length; //index = 13 + lenth
    tempcrc16 = Get_CRC16_Check_Sum(referee_intercom_buff, index, CRC_INIT);
    memcpy(referee_intercom_buff + index, &tempcrc16, 2);
    index += 2; //1ndex = 15 + 1enth
    /**南**以下用自定义的串口发送函数，发送rereree_ intercom burr内的长度为index的数据*****/
    //usart6_tx_dna_enab1e (rereree_ 1ntercom_ butr，index) ;
    ui.graph_num = 0;
    serial = rt_device_find(DJI_UART);
    if(!rt_device_write(serial, 0, referee_intercom_buff, index))
        return RT_ERROR; //rt_kprintf("fail");//如果发送数据为0计数一次发送失败，失败次数过多发出警告
    else
        return RT_EOK;
}

