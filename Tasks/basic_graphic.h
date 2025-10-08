#ifndef _BASIC_GRAPHIC_H
#define _BASIC_GRAPHIC_H
#include "comm.h"
#define GRAPHIC_CMD_BUFFER_SIZE 1024
#define SELF_ID (reference_robot_state.robot_id)

// extern uint8_t gbuf[2][GRAPHIC_CMD_BUFFER_SIZE];


/* This file defines the following things:
 * 
 * enum UI_CLEAR_OP_TYPE
 * enum UI_OP_TYPE
 * enum UI_GRAPH_TYPE
 * enum UI_COLOR
 * graphic_clear_op_t
 * graphic_name_t
 * grpahic_base_t
 * graphic_line_t
 * graphic_rect_t
 * graphic_circle_t
 * graphic_ellipse_t
 * graphic_arc_t
 * graphic_float_t
 * graphic_int_t
 * graphic_text_t
 *
 */

#define GRAPHIC_SOF 0xa5

#define DCMD_UI_CLEAR 0x0100

enum UI_CLEAR_OP_TYPE {
	UI_CLEAR_NONE = 0,
	UI_CLEAR_LAYER,
	UI_CLEAR_ALL
};
typedef __packed struct {
	uint8_t operate_type;
	uint8_t layer;
} graphic_clear_op_t;

#define DCMD_UI_OP_1 0x0101
#define DCMD_UI_OP_2 0x0102
#define DCMD_UI_OP_5 0x0103
#define DCMD_UI_OP_7 0x0104
#define DCMD_UI_TEXT 0x0110
#define DCMD_UI_INVALID 0x01ff

enum UI_OP_TYPE {
	UI_OP_NONE = 0,
	UI_OP_ADD,
	UI_OP_MOD,
	UI_OP_DEL
};

enum UI_GRAPH_TYPE {
	UI_GRAPH_LINE = 0,
	UI_GRAPH_RECT,
	UI_GRAPH_CIRCLE,
	UI_GRAPH_ELLIPSE,
	UI_GRAPH_ARC,
	UI_GRAPH_FLOAT,
	UI_GRAPH_INT,
	UI_GRAPH_TEXT
};

enum UI_COLOR {
	UI_COLOR_MAIN = 0,
	UI_COLOR_YELLOW,
	UI_COLOR_GREEN,
	UI_COLOR_ORANGE,
	UI_COLOR_MAGENTA,
	UI_COLOR_PINK,
	UI_COLOR_CYAN,
	UI_COLOR_BLACK,
	UI_COLOR_WHITE
};

typedef __packed struct {
  uint8_t v[3];
} graphic_name_t;

typedef __packed struct {
  graphic_name_t graphic_name;
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
  uint32_t _1:18;
  uint32_t _2[2];
} graphic_base_t;

typedef __packed struct {
	graphic_name_t graphic_name;
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t _1:18;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t _2:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_line_t;

typedef __packed struct {
	graphic_name_t graphic_name;
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t _1:18;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t _2:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_rect_t;

typedef __packed struct {
	graphic_name_t graphic_name;
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t _1:18;
	uint32_t width:10;
	uint32_t center_x:11;
	uint32_t center_y:11;
	uint32_t radius:10;
	uint32_t _2:22;
} graphic_circle_t;

typedef __packed struct {
	graphic_name_t graphic_name;
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t _1:18;
	uint32_t width:10;
	uint32_t center_x:11;
	uint32_t center_y:11;
	uint32_t _2:10;
	uint32_t radius_x:11;
	uint32_t radius_y:11;
} graphic_ellipse_t;

typedef __packed struct {
	graphic_name_t name;
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t center_x:11;
	uint32_t center_y:11;
	uint32_t _2:10;
	uint32_t radius_x:11;
	uint32_t radius_y:11;
} graphic_arc_t;

typedef __packed struct {
	graphic_name_t name;
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t font_size:9;
	uint32_t precision:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	int32_t value;
} graphic_float_t;

typedef __packed struct {
	graphic_name_t name;
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t font_size:9;
	uint32_t _:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	int32_t value;
} graphic_int_t;

typedef __packed struct {
	graphic_name_t name;
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t font_size:9;
	uint32_t length:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t _;
} graphic_text_t;

int sendGraphic(graphic_base_t const * packet, void const * buf);
int clearGraphic(enum UI_CLEAR_OP_TYPE op, uint8_t layer);
void graphSenderInit(void);

static inline void genGraphName(graphic_name_t* name) {
  uint32_t t = time();
  name->v[0] = t & 0xff;
  name->v[1] = (t >> 8) & 0xff;
  name->v[2] = (t >> 16) & 0xff;
}

#endif
