#ifndef __DRV_CUSTOM_UI_BASIC_H
#define __DRV_CUSTOM_UI_BASIC_H

//#include "struct_typedef.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "stdbool.h"
#include "stdint.h"


#define CMD_ERASE_GRAPH     0x0100
#define CMD_DRAW_1_GRAPH    0x0101
#define CMD_DRAW_2_GRAPH    0x0102
#define CMD_DRAW_5_GRAPH    0x0103
#define CMD_DRAW_7_GRAPH    0x0104
#define CMD_DRAW_TEXT       0x0110

#define LENGTH_DRAW_1_GRAPH 15
#define LENGTH_DRAW_2_GRAPH 30
#define LENGTH_DRAW_5_GRAPH 75
#define LENGTH_DRAW_7_GRAPH 105
#define LENGTH_DRAW_TEXT    45

#define MAX_CUSTOM_UI_BUFF_LENGTH   113
#define ONE_GRAPH_LENGTH            15

typedef enum
{
    UI_OK = 0,
    UI_FULL,
    UI_ERR,
    UI_CHAR //字符只能单独发送,要特殊处理
} ui_basic_e;

typedef struct
{
    uint16_t cmd;
    uint8_t data_buff[MAX_CUSTOM_UI_BUFF_LENGTH];
    uint8_t graph_num;
    uint16_t length;
} custom_ui_buff_t;


typedef enum
{
    CLEAR_GRAPH_TYPE_CLEAR_LAYER = 0x01,
    CLEAR_GRAPH_TYPE_CLEAR_ALL = 0x02,
} clear_graph_type_t;

typedef enum
{
    GRAPH_OPERATION_NOOP= 0x00,
    GRAPH_OPERATION_ADD = 0x01,
    GRAPH_OPERATION_MODIFY = 0x02,
    GRAPH_OPERATION_ERASE = 0x03,
} graph_operation_t;

typedef enum
{
    GRAPH_TYPE_LINE = 0x00,
    GRAPH_TYPE_RECT = 0x01,
    GRAPH_TYPE_CIRCLE = 0x02,
    GRAPH_TYPE_ELLIPSE = 0x03,
    GRAPH_TYPE_ARC = 0x04,
    GRAPH_TYPE_FLOAT = 0x05,
    GRAPH_TYPE_INT = 0x06,
    GRAPH_TYPE_CHAR = 0x07,
} graph_type_t;

typedef enum
{
    GRAPH_COLOR_REDBLUE = 0x00,
    GRAPH_COLOR_YELLOW = 0x01,
    GRAPH_COLOR_GREEN = 0x02,
    GRAPH_COLOR_ORANGE = 0x03,
    GRAPH_COLOR_VIOLET = 0x04,
    GRAPH_COLOR_PINK = 0x05,
    GRAPH_COLOR_CYAN = 0x06,
    GRAPH_COLOR_BLACK = 0x07,
    GRAPH_COLOR_WHITE = 0x08,
} graph_color_t;

typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye: 3;
    uint32_t graphic_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t start_angle: 9;
    uint32_t end_angle: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    __packed union
    {
        __packed struct
        {
            uint32_t radious: 10;
            uint32_t end_x: 11;
            uint32_t end_y: 11;

        } graph_property;
        int32_t int_val;
        int32_t float_val;
    } property;
} graphic_data_struct_t;

typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

ui_basic_e custom_ui_erase_graph(clear_graph_type_t clear_type, uint8_t layer);
void custom_ui_init(void);

ui_basic_e custom_ui_append_line_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                       uint16_t width, graph_color_t color,
                                       uint16_t start_x, uint16_t start_y,
                                       uint16_t end_x, uint16_t end_y);

ui_basic_e custom_ui_append_rect_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                         uint16_t width, graph_color_t color,
                                         uint16_t start_x, uint16_t start_y,
                                         uint16_t end_x, uint16_t end_y);

ui_basic_e custom_ui_append_cirle_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                        uint16_t width, graph_color_t color,
                                        uint16_t centre_x, uint16_t centre_y,
                                        uint16_t radious);

ui_basic_e custom_ui_append_ellipse_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                          uint16_t width, graph_color_t color,
                                          uint16_t centre_x, uint16_t centre_y,
                                          uint16_t avis_x, uint16_t avis_y);

ui_basic_e custom_ui_append_arc_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                     uint16_t start_angle, uint16_t end_angle,
                                     uint16_t width, graph_color_t color,
                                     uint16_t centre_x, uint16_t centre_y,
                                     uint16_t axis_x, uint16_t axis_y);

ui_basic_e custom_ui_append_float_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                        uint16_t font_size, uint16_t digit,
                                        uint16_t width, graph_color_t color,
                                        uint16_t start_x, uint16_t start_y, float value);

ui_basic_e custom_ui_append_int_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                      uint16_t font_size, uint16_t width, graph_color_t color,
                                      uint16_t start_x, uint16_t start_y, int32_t value);

ui_basic_e custom_ui_draw_text(uint8_t layer, uint32_t name, graph_operation_t operation,
                             char *text, uint16_t font_size,
                             uint16_t length, uint16_t width,
                             graph_color_t color, uint16_t start_x, uint16_t start_y);
rt_err_t ui_referee_intercom_tranamit(uint16_t senderID, uint16_t receiverID);
#endif
