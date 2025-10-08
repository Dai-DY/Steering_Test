#include "graphic.h"
#include "basic_graphic.h"
#include "controller.h"
#include "comm.h"
#include "math.h"
#include "board_transmit.h"

extern thread_status_t thread_status;

#define MAX_ELEM_NUM 20
//dynamic line used to show trafficability 
bool dynalign_calc(uint16_t* screenPos){
	if(__fabs(attitude.roll) < 10) return false;
	else{// In x_start, x_end, y_start, y_end
		screenPos[0] = 955;
		screenPos[1] = 955 + (540 - 200) * tanf(attitude.roll * 3.1416f / 180);
		screenPos[2] = 540;
		screenPos[3] = 200;
	}
	return true;
}
//debug UI
#ifdef DEBUG_UI
#define GRAPH_CNT 4
//add a changing graph:
//2.CHANGE ModifyBag->content[]
//3.CHANGE Updating Part
typedef __packed struct {
	uint16_t content_ID;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_int_t content[GRAPH_CNT];
}ModifyBag_t;
//add a label:
//1.LABEL_CNT
//2.CHANGE labels[]
//3.CHANGE wordLen & words
#define LABEL_CNT 4
typedef __packed struct {
	uint16_t content_ID;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_text_t settings;
	uint8_t content[30];
}TextBag_t;

typedef __packed struct {
	uint16_t content_ID;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_line_t content[7];
}LineBag_t;

typedef __packed struct {
	uint16_t content_ID;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_circle_t content;
}CircleBag_t;

const uint8_t words[LABEL_CNT][30] = {\
	{'s', 'i', 'n', 'g', 'l', 'e'},
	{'o' ,'r', 'g', 'a', 'n'},
	{'v', 'c', 'a', 'p'},
	{'t', 'o', 'p'}
	};
#endif

void display_updater(void* args){
    
    thread_status.ui = THREAD_OK;
    while(!(thread_status.uartcom  && //waiting till all task threads are initialized
            thread_status.cancom   &&    
            thread_status.ui       &&
            thread_status.chassis  &&
            //thread_status.gimbal   &&
            //thread_status.shoot    &&
            thread_status.mainctrl )){
        delay(50);
    }
    
	while(SELF_ID == 0) {
		delay(5);
	}
	#ifdef DEBUG_UI
	TextBag_t labels[LABEL_CNT];
	for(int label_idx = 0; label_idx < LABEL_CNT; label_idx ++) {
		//default setting: text, can be modified
		labels[label_idx].content_ID = 0x0110;
		labels[label_idx].sender_ID = SELF_ID;
		labels[label_idx].receiver_ID = 0x0100 + SELF_ID;
		labels[label_idx].settings.operate_type = UI_OP_ADD;
		labels[label_idx].settings.graphic_type = UI_GRAPH_TEXT;
		labels[label_idx].settings.name.v[0] = 'L';
		labels[label_idx].settings.name.v[1] = 'B';
		labels[label_idx].settings.name.v[2] = '0' + label_idx;
		labels[label_idx].settings.layer = 5;
		labels[label_idx].settings.color = UI_COLOR_ORANGE;
		labels[label_idx].settings.font_size = 20;
		labels[label_idx].settings.width = 3;
		labels[label_idx].settings.start_x = 10;
		labels[label_idx].settings.start_y = 800 - label_idx * 40;
		labels[label_idx].settings.length = 3;
		for(int charIdx = 0; charIdx < 30; charIdx ++)
			if(labels[label_idx].content[charIdx] == 0)
				labels[label_idx].content[charIdx] = ' ';
			else 
				labels[label_idx].content[charIdx] = words[label_idx][charIdx];
	}
	
	//below is data
	ModifyBag_t *modifyBag;
	modifyBag = (ModifyBag_t *)malloc(sizeof(ModifyBag_t));
	if(GRAPH_CNT == 1)
		modifyBag->content_ID = 0x0101;
	if(GRAPH_CNT == 2)
		modifyBag->content_ID = 0x0102;
	if(GRAPH_CNT >= 3 && GRAPH_CNT <= 5)
		modifyBag->content_ID = 0x0103;
	if(GRAPH_CNT >= 6 && GRAPH_CNT <= 7)
		modifyBag->content_ID = 0x0104;
	for(int graph_idx = 0; graph_idx < GRAPH_CNT; graph_idx ++) {
		//default setting: int, float can use the same struct
		modifyBag->content[graph_idx].operate_type = UI_OP_NONE;
		modifyBag->content[graph_idx].graphic_type = UI_GRAPH_INT;
		modifyBag->content[graph_idx].operate_type = UI_OP_ADD;
		modifyBag->content[graph_idx].name.v[0] = 'D';
		modifyBag->content[graph_idx].name.v[1] = 'T';
		modifyBag->content[graph_idx].name.v[2] = '0' + graph_idx;
		modifyBag->content[graph_idx].layer = 5;
		modifyBag->content[graph_idx].color = UI_COLOR_PINK;
		modifyBag->content[graph_idx].font_size = 20;
		modifyBag->content[graph_idx].width = 3;
		modifyBag->content[graph_idx].start_x = 250;
		modifyBag->content[graph_idx].start_y = 800 - 40 * graph_idx;
		//modifyBag->content[graph_idx].value = (int)yaw.spd_get;
	}
	modifyBag->sender_ID = SELF_ID;
	modifyBag->receiver_ID = 0x0100 + SELF_ID;
	
	//single
	//organ
	//vcap
	modifyBag->content[2].graphic_type = UI_GRAPH_FLOAT;
	//top
//-----------------------------------------Aiming Marks---------------------------------------------------------------
	#define LINESETCNT 1
	LineBag_t lines[LINESETCNT];
	for(int line_idx = 0; line_idx < LINESETCNT; line_idx ++) {
		lines[line_idx].sender_ID = SELF_ID;
		lines[line_idx].receiver_ID = 0x0100 + SELF_ID;
		lines[line_idx].content_ID = 0x0104;
		for(int i = 0; i < 7; i++) {
		lines[line_idx].content[i].operate_type = UI_OP_ADD;
		lines[line_idx].content[i].graphic_type = UI_GRAPH_LINE;
		lines[line_idx].content[i].graphic_name.v[0] = 'L';
		lines[line_idx].content[i].graphic_name.v[1] = 'I';
		lines[line_idx].content[i].graphic_name.v[2] = '0' + line_idx * 7 + i;
		lines[line_idx].content[i].layer = 5;
		lines[line_idx].content[i].color = UI_COLOR_WHITE;
		lines[line_idx].content[i].width = 3;
		lines[line_idx].content[i].start_x = 0;
		lines[line_idx].content[i].end_x = 0;
		lines[line_idx].content[i].start_y = 0;
		lines[line_idx].content[i].end_y = 0;		
		}
	}
	lines[0].content[0].start_x = 880;
	lines[0].content[0].start_y = 440; //x=160
	lines[0].content[0].end_x = 1040;
	lines[0].content[0].end_y = 440;
	
	lines[0].content[1].start_x = 870;
	lines[0].content[1].start_y = 520; //y=60
	lines[0].content[1].end_x = 870;
	lines[0].content[1].end_y = 460;
	
	lines[0].content[2].start_x = 1050;
	lines[0].content[2].start_y = 520;  //y=60
	lines[0].content[2].end_x = 1050;
	lines[0].content[2].end_y = 460;
	
	lines[0].content[3].start_y = 515;
	lines[0].content[3].end_y = 515;  //x=44
	lines[0].content[3].start_x = 945;
	lines[0].content[3].end_x = 985;
	
	lines[0].content[4].start_x = 965;
	lines[0].content[4].end_x = 965;   //y=30
	lines[0].content[4].start_y = 500;
	lines[0].content[4].end_y = 530;
	
	lines[0].content[5].start_x = 960;
	lines[0].content[5].end_x = 960;
	lines[0].content[5].start_y = 434; //y=14
	lines[0].content[5].end_y = 420;
	
	lines[0].content[6].start_y = 402;
	lines[0].content[6].end_y = 402;
	lines[0].content[6].start_x = 950; //x=20
	lines[0].content[6].end_x = 970;
	
	CircleBag_t organCircle;
	organCircle.sender_ID = SELF_ID;
	organCircle.receiver_ID = 0x0100 + SELF_ID;
	organCircle.content_ID = 0x0101;
	organCircle.content.center_x = 960;
	organCircle.content.center_y = 377;
	organCircle.content.color = UI_COLOR_WHITE;
	organCircle.content.graphic_name.v[0] = 'C';
	organCircle.content.graphic_name.v[1] = 'C';
	organCircle.content.graphic_name.v[2] = 'O';
	organCircle.content.graphic_type = UI_GRAPH_CIRCLE;
	organCircle.content.layer = 1;
	organCircle.content.operate_type = UI_OP_ADD;
	organCircle.content.radius = 1;
	organCircle.content.width = 4;
	while(1){
		//update part
		//modifyBag->content[0].value = control_mode.singleShoot || !control_mode.chassis_following;
		//modifyBag->content[1].value = !control_mode.chassis_following;
			modifyBag->content[2].value = V_Cap *1000;
		//modifyBag->content[3].value = chassis.top.enable * chassis.top.direction;
		//sending part
		if(KEY_V) {
			//sending an initialize bag for changing graphs
			for(int graph_idx = 0; graph_idx < GRAPH_CNT; graph_idx ++) {
				modifyBag->content[graph_idx].operate_type = UI_OP_ADD;
			}
			comm_transmit_data(uart_reference.huart, 0xa5, 0x0301, (uint8_t *) modifyBag, sizeof(ModifyBag_t));
			//initialize the labels
			for(int label_idx = 0; label_idx < LABEL_CNT; label_idx ++) {
				delay(300);
				comm_transmit_data(uart_reference.huart, 0xa5, 0x0301, (uint8_t *) &labels[label_idx], sizeof(TextBag_t));
			}
			for(int line_idx = 0; line_idx < LINESETCNT; line_idx++) {
				delay(300);
				comm_transmit_data(uart_reference.huart, 0xa5, 0x0301, (uint8_t *) &lines[line_idx], sizeof(LineBag_t));
			}
			delay(300);
			comm_transmit_data(uart_reference.huart, 0xa5, 0x0301, (uint8_t *) &organCircle, sizeof(CircleBag_t));
		}
		else if(KEY_G) {
			
		}
		else {
			for(int graph_idx = 0; graph_idx < GRAPH_CNT; graph_idx ++) {
				modifyBag->content[graph_idx].operate_type = UI_OP_MOD;
			}
			comm_transmit_data(uart_reference.huart, 0xa5, 0x0301, (uint8_t *) modifyBag, sizeof(ModifyBag_t));
		}
		delay(100);
	}
	#else
	
	#endif
}

void AHUD_init(void){
	graphSenderInit();
	osThreadCreate("huds", display_updater, NULL,osPriorityNormal, 256);
}
