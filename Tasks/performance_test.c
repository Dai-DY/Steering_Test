#include "performance_test.h"
int cnt;
gimbal_test_t gimbal_test;

void Test_task(void *args) {
	gimbal_test.pitch = 0;
	gimbal_test.pitch_speed = 0;
	gimbal_test.yaw = 0;
	gimbal_test.yaw_speed = 0;
	static int TEST_PERIOD = 1;
	static int TEST_PITCH_RANGE = 10;
	static int TEST_YAW_RANGE = 30;
	uint32_t start_time = time();
	
	while(1) {
		float T = (time() - start_time) / 1000.0f;
		gimbal_test.pitch = TEST_PITCH_RANGE * sin(T * 3.14f / TEST_PERIOD);
		gimbal_test.pitch_speed = (3.1415926f / 180.0f) * TEST_PITCH_RANGE * (3.14f / TEST_PERIOD) * cos(T * 3.14f / TEST_PERIOD);
		gimbal_test.yaw = TEST_YAW_RANGE * sin(T * 6.283f / TEST_PERIOD);
		gimbal_test.yaw_speed = (3.1415926f / 180.0f) * TEST_YAW_RANGE * (6.283f / TEST_PERIOD) * cos(T * 6.283f / TEST_PERIOD);
		if(!(cnt % 10)) {
			organ_aimbot.pit = gimbal_test.pitch;
			organ_aimbot.pitch_spd = gimbal_test.pitch_speed;
			organ_aimbot.yaw = gimbal_test.yaw;
			organ_aimbot.yaw_spd = gimbal_test.yaw_speed;
		}
		NX_receive_time = time();
		++cnt;
		delay(5);
	}
}

