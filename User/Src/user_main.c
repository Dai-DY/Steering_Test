#include "user_main.h"
#include "os.h"
#include "robot_conf.h"
#include "main_control.h"
#include "graphic.h"
#include "performance_test.h"
#include "board_transmit.h"
/*******************************************************************
 * >>>>>>>>>>>>>>>>>>>>>>>>>>> NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<<<< *
 *******************************************************************
 * You are allowed to do whatever you like to this file as long as *
 * the User_Init() function is kept here, and the header file      *
 * user_main.h are included.                                       *
 *                                                                 *
 * You are supposed to wrap your main codes in functions, not      *
 * placing them into User_Init(). This function is only meant to   *
 * be the place where you init your global variables, create and   *
 * start your own threads. Make sure you exit it as soon as possi- *
 * ble. NEVER should there be any infinite loops.                  *
 *                                                                 *
 * If you need to define any macros or global variables, please    *
 * create your own header file under the group Application/main.   *
 * So should your own source file (*.c) be.                        *
 *                                                                 *
 * If you have any problem, feel free to open an issue on GitLab.  *
 *                                                                 *
 * Happy coding, RoboMasters :)                                    *
 *                                                                 *
 *******************************************************************/
 
extern thread_status_t thread_status;
 
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	huart->TxXferCount = 1;
}
 
void User_Init(void){
    //clear thread status to init threads 
    thread_status.chassis = THREAD_INIT;    
    thread_status.mainctrl = THREAD_INIT;
    thread_status.uartcom = THREAD_INIT;
    thread_status.cancom = THREAD_INIT;
    //Tasks
	osThreadCreate("chassis_task",chassis_task,NULL,osPriorityNormal,128);
    //User application
	osThreadCreate("main_control_task",main_control_task,NULL,osPriorityNormal,128);
    //DeBUG
	#if CV_PID_ADJUST
	osThreadCreate("performance_test", Test_task, NULL, osPriorityNormal,128);
	#endif
	//Communication start
 	//AHUD_init();
	Comm_init();
    board_transmit_init();
}
