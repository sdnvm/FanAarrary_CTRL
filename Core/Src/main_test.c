//
// Created by 86158 on 2026/7/27.
//
int main_test(void)
{
	/*测试pwm输出与单通道极性翻转捕获 */
	if(capture_end_flag == 1)
	{
		extern int TIM3CH2_Duty;
		float Duty_want = 0;
		float err_fg = 0.0;
		Duty_want = 5000;
		Duty_want = 5000 / SPEED_MX;
		err_fg = TIM3CH2_Duty - Duty_want;
		if(err_fg < 1.0 || err_fg > 1.0)
		{
			OLED_ShowString(1, 1, "Err");
		}
		else{
			OLED_ShowNum(1, 1, TIM3CH2_Duty, 4);
		}
		capture_end_flag = 3;
	}
}