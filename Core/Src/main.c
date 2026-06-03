/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"
#include "string.h"
#include "stdio.h"
#include "valuepack.h"
#include "stdint.h"
#include "oled.h"
#include "stdlib.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LEN 4096
float SAMPLE_RATE = 2000000;
#define M_PI 3.14159265358979323846f

#define MAX_FFT_POINTS 4096
__attribute__((aligned(8))) static float32_t fft_input[2 * MAX_FFT_POINTS];
__attribute__((aligned(8))) static float32_t power_spectrum[MAX_FFT_POINTS];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t period = 25;
uint16_t adc_buf[LEN];
uint16_t adc_test1[LEN];
uint16_t adc_test2[LEN];
uint16_t adc_test3[LEN];
float windowed_buf[LEN];

//float fft_input[LEN * 2];
float fft_mag[LEN]; //????
arm_rfft_fast_instance_f32 fft_instance;

float THD = 0.0f;				//???
float adc_norm[LEN]; 			//?????
uint32_t fund_index;			//????
float fund_freq;
float32_t fund_value;			//????
float harm_power = 0;			//???

volatile uint8_t adc_ready = 0;	//???
float unify_mag[5];
uint8_t send_flag = 0;
uint16_t adc_sent[25];
uint8_t SampleMode = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//串口重定向
int fputc(int ch,FILE *f)
{
HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,HAL_MAX_DELAY);
return ch;
}
int fgetc(FILE *f)
{
uint8_t ch;
HAL_UART_Receive( &huart1,(uint8_t*)&ch,1, HAL_MAX_DELAY );
return ch;
}
//----------平顶窗------------
float a0 = 1;
float a1 = 1.93;
float a2 = 1.29;
float a3 = 0.388;
float a4 = 0.028;
float windowing[LEN];
void genarate_windowing(){
	for(int i = 0; i < LEN; i++) {
    windowing[i] = a0 
                 - a1 * cosf(2 * PI * i / (LEN - 1)) 
                 + a2 * cosf(4 * PI * i / (LEN - 1)) 
                 - a3 * cosf(6 * PI * i / (LEN - 1))
				 + a4 * cosf(8 * PI * i / (LEN - 1));
	}
}
//捕捉波形函数
// 找第一个峰值索引
uint16_t start;
int find_first_peak(float *buf, int len) {
    for(int i=1;i<len-1;i++){
        if(buf[i] > buf[i-1] && buf[i] > buf[i+1]){
            return i;
        }
    }
    return 0; // 没找到峰值就从0开始
}

//能量重心法
float32_t freq_est;
float prev_freq_est;
float32_t amp_est;
void ImprovedFFT_EnergyCentroid(uint16_t *adc_data, uint32_t fs, uint16_t N, 
                               float32_t *freq_est, float32_t *amp_est) 
{
//    float32_t sum = 0;
//    for (uint16_t i = 0; i < N; i++) {
//        // ADC转换 (12-bit ADC, 3.3V参考电压)
//        float32_t sample = adc_data[i] * 3.3f / 4096.0f;
//        sum += sample;
//        fft_input[2*i] = sample;  // 实部
//        fft_input[2*i+1] = 0;     // 虚部
//    }
//    // 去除直流偏置
//    float32_t mean = sum / N;
//    for (uint16_t i = 0; i < N; i++) {
//        fft_input[2*i] -= mean;
//    }
    // 1. 时域加汉宁窗
    for (uint16_t i = 0; i < N; i++) {
        // 汉宁窗函数: w(n) = 0.5 - 0.5*cos(2πn/N)
        float32_t window = 0.5f - 0.5f * arm_cos_f32(2 * PI * i / (N-1));
        fft_input[2*i] = fft_input[2*i] * window;  // 实部
        fft_input[2*i+1] = 0;              // 虚部
    }
    // 2. 执行FFT
    arm_cfft_instance_f32 cfft_inst;
    arm_cfft_init_f32(&cfft_inst, N);
    arm_cfft_f32(&cfft_inst, fft_input, 0, 1);
    // 3. 计算功率谱 (|X(k)|²)
    for (uint16_t k = 0; k < N; k++) {
        float32_t real = fft_input[2*k];
        float32_t imag = fft_input[2*k+1];
        power_spectrum[k] = real*real + imag*imag;
    }
    // 4. 寻找主瓣峰值位置 (只搜索正频率)
    uint32_t l0 = 1;
    float32_t max_power = power_spectrum[1];
    for (uint16_t k = 2; k < N/2; k++) {
        if (power_spectrum[k] > max_power) {
            max_power = power_spectrum[k];
            l0 = k;
        }
    }
    // 5. 计算矩形窗FFT（用于相位差计算）
    __attribute__((aligned(8))) static float32_t rect_fft[2 * MAX_FFT_POINTS];
    for (uint16_t i = 0; i < N; i++) {
        float32_t sample = adc_data[i] * 3.3f / 4096.0f;
        rect_fft[2*i] = sample;  // 已去直流
        rect_fft[2*i+1] = 0;
    }
    arm_cfft_f32(&cfft_inst, rect_fft, 0, 1);
//——————————————————————————————————————————————————————————————————//
	
    // 6. 计算相位差指标 (式16)
    float32_t X_l0_real = rect_fft[2*l0];
    float32_t X_l0_imag = rect_fft[2*l0+1];
    
    // 安全边界处理
    uint16_t idx_m1 = (l0 == 0) ? N-1 : l0-1;
    uint16_t idx_p1 = (l0 == N-1) ? 0 : l0+1;
    
    // R(-1) = Re[X_rect(l0-1) * conj(X_rect(l0))]
    float32_t X_m1_real = rect_fft[2*idx_m1];
    float32_t X_m1_imag = rect_fft[2*idx_m1+1];
    float32_t R_m1 = X_m1_real * X_l0_real + X_m1_imag * X_l0_imag;  // Re[AB*]
    
    // R(+1) = Re[X_rect(l0+1) * conj(X_rect(l0))]
    float32_t X_p1_real = rect_fft[2*idx_p1];
    float32_t X_p1_imag = rect_fft[2*idx_p1+1];
    float32_t R_p1 = X_p1_real * X_l0_real + X_p1_imag * X_l0_imag;

    // 7. 计算频偏δ (式18)
    float32_t delta;
    uint16_t idx_m2 = (l0 < 2) ? (N + l0 - 2) % N : l0 - 2;
    uint16_t idx_p2 = (l0 > N-3) ? (l0 + 2) % N : l0 + 2;
    
    if (R_p1 < 0 && R_m1 > 0) {
        // 次大谱线在右侧 (式4)
        float32_t Y0 = power_spectrum[l0];
        float32_t Ym1 = power_spectrum[idx_m1];
        float32_t Yp1 = power_spectrum[idx_p1];
        float32_t Yp2 = power_spectrum[idx_p2];
        
        delta = (Yp1 + 2*Yp2 - Ym1) / (Ym1 + Y0 + Yp1 + Yp2);
    } 
    else if (R_p1 > 0 && R_m1 < 0) {
        // 次大谱线在左侧 (式5)
        float32_t Y0 = power_spectrum[l0];
        float32_t Ym1 = power_spectrum[idx_m1];
        float32_t Ym2 = power_spectrum[idx_m2];
        float32_t Yp1 = power_spectrum[idx_p1];
        
        delta = (Yp1 - Ym1 - 2*Ym2) / (Ym2 + Ym1 + Y0 + Yp1);
    } 
    else {
        // 取两种估计的平均值 (论文式18)
        float32_t Y0 = power_spectrum[l0];
        float32_t Ym1 = power_spectrum[idx_m1];
        float32_t Ym2 = power_spectrum[idx_m2];
        float32_t Yp1 = power_spectrum[idx_p1];
        float32_t Yp2 = power_spectrum[idx_p2];
        
        float32_t delta1 = (Yp1 + 2*Yp2 - Ym1) / (Ym1 + Y0 + Yp1 + Yp2);
        float32_t delta2 = (Yp1 - Ym1 - 2*Ym2) / (Ym2 + Ym1 + Y0 + Yp1);
        delta = 0.5f * (delta1 + delta2);
    }

    // 8. 频率估计 f = (l0 + δ) * (fs / N)
    *freq_est = (l0 + delta) * (fs / (float32_t)N);
    
    // 9. 精确幅值校准 (汉宁窗恢复系数)
    // |A| = 2 * sqrt(|X(k)|²) / (0.5 * N * S_w)
    // 汉宁窗相干增益 S_w = 0.5
    *amp_est = 2.0f * sqrtf(max_power) / (0.5f * N * 0.5f);
}

//ADC回调
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	 if(hadc->Instance == ADC1 )
		{
			HAL_ADC_Stop_DMA(&hadc1);
			adc_ready = 1;
		}
}
//按键回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_8){
		HAL_TIM_Base_Start_IT(&htim7);
	}
}

//定时器蓝牙显示
unsigned char tx_buffer[25];
int f =0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) // 判断是哪个定时器
    {
		if(send_flag == 1){
			startValuePack(tx_buffer);
			putShort(adc_sent[f]);
			putFloat(unify_mag[1]);
			putFloat(unify_mag[2]);
			putFloat(unify_mag[3]);
			putFloat(unify_mag[4]);
			putFloat(THD);
			putFloat(freq_est);
			f++;
			if(f >= period-1) {
				f = 0;
				send_flag = 0;
			}
			HAL_UART_Transmit(&huart3, tx_buffer, endValuePack(), HAL_MAX_DELAY);
		}
    }
	if (htim->Instance == TIM7) // 按键消抖20Ms
    {
		HAL_TIM_Base_Stop_IT(&htim7);
		start = find_first_peak((float*)adc_buf, LEN);
		for(int i = 0; i<period; i++){
			adc_sent[i] = adc_buf[start + i];
		}
		for(int i = 0; i<period; i++){
			printf("%d\r\n", adc_sent[i]);
		}
		send_flag = 1;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, LEN);
	HAL_TIM_Base_Start(&htim2);

	unify_mag[0] = 1.0f;
	genarate_windowing();
	HAL_Delay(20);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(adc_ready){
		adc_ready = 0;
		
		//一个周期采样period个点
		start = find_first_peak((float*)adc_buf, LEN);
		for(int i = 0; i<period; i++){
			adc_sent[i] = adc_buf[start + i];
		}
		
//归一化，去直流对于这个项目不适用， 更加不准
		//增幅
		for(int i = 0; i<LEN; i++){
			adc_norm[i] = adc_buf[i]*15;
		}

		//加窗
		for(int i = 0; i < LEN; i++) {
			float w = 0.5f * (1.0f - cosf(2.0f * PI * i / (LEN - 1)));//汉宁窗
			windowed_buf[i] = adc_norm[i] * w; 
		}
		
		// fft填数
		for (int i = 0; i < LEN; i++) {
			fft_input[2*i]   = windowed_buf[i];  //
			fft_input[2*i+1] = 0.0f;         //
		}
		//汉宁窗fft计算
		const arm_cfft_instance_f32 *cfft_struct = &arm_cfft_sR_f32_len4096;
		arm_cfft_f32(cfft_struct, fft_input, 0, 1);
		arm_cmplx_mag_f32(fft_input, fft_mag, LEN);
		
		for(int i =0; i<LEN; i++){
			adc_test3[i] = fft_mag[i];
		}
		//寻找主瓣峰值位置
//		int start_bin = (int)((1000.0f * LEN) / SAMPLE_RATE);
//		if (start_bin < 1) start_bin = 1;
//		int end_bin   = LEN/2;
		fund_value = 0;
		fund_index = 1;
		for (int k = 2; k < LEN/2; k++) {
				if (fft_mag[k] > fund_value){
					fund_value = fft_mag[k];
					fund_index = k;
				}
		}
		
		// 5. 计算矩形窗FFT（用于相位差计算）
		__attribute__((aligned(8))) static float32_t rect_fft[2 * MAX_FFT_POINTS];
		for (uint16_t i = 0; i < LEN; i++) {
			rect_fft[2*i] = adc_buf[i];
			rect_fft[2*i+1] = 0;
		}
		arm_cfft_f32(cfft_struct, rect_fft, 0, 1);
		
//		// 抛物线插值
//		float alpha = fft_mag[fund_index - 1];
//		float beta  = fft_mag[fund_index];
//		float gamma = fft_mag[fund_index + 1];
//		float delta = 0.5f * (alpha - gamma) / (alpha - 2*beta + gamma);
//		float fund_bin = fund_index + delta;
//		fund_freq = fund_bin * SAMPLE_RATE / LEN;
		
		// 6. 计算相位差指标 (式16)
		float32_t X_l0_real = rect_fft[2*fund_index];
		float32_t X_l0_imag = rect_fft[2*fund_index+1];
		
		// 安全边界处理
		uint16_t idx_m1 = (fund_index == 0) ? LEN-1 : fund_index-1;
		uint16_t idx_p1 = (fund_index == LEN-1) ? 0 : fund_index+1;
		
		// R(-1) = Re[X_rect(l0-1) * conj(X_rect(l0))]
		float32_t X_m1_real = rect_fft[2*idx_m1];
		float32_t X_m1_imag = rect_fft[2*idx_m1+1];
		float32_t R_m1 = X_m1_real * X_l0_real + X_m1_imag * X_l0_imag;  // Re[AB*]
		
		// R(+1) = Re[X_rect(fund_index+1) * conj(X_rect(fund_index))]
		float32_t X_p1_real = rect_fft[2*idx_p1];
		float32_t X_p1_imag = rect_fft[2*idx_p1+1];
		float32_t R_p1 = X_p1_real * X_l0_real + X_p1_imag * X_l0_imag;

		// 7. 计算频偏δ (式18)
		float32_t delta;
		uint16_t idx_m2 = (fund_index < 2) ? (LEN + fund_index - 2) % LEN : fund_index - 2;
		uint16_t idx_p2 = (fund_index > LEN-3) ? (fund_index + 2) % LEN : fund_index + 2;
		
		if (R_p1 < 0 && R_m1 > 0) {
			// 次大谱线在右侧 (式4)
			float32_t Y0 = fft_mag[fund_index];
			float32_t Ym1 = fft_mag[idx_m1];
			float32_t Yp1 = fft_mag[idx_p1];
			float32_t Yp2 = fft_mag[idx_p2];
			
			delta = (Yp1 + 2*Yp2 - Ym1) / (Ym1 + Y0 + Yp1 + Yp2);
		} 
		else if (R_p1 > 0 && R_m1 < 0) {
			// 次大谱线在左侧 (式5)
			float32_t Y0 = fft_mag[fund_index];
			float32_t Ym1 = fft_mag[idx_m1];
			float32_t Ym2 = fft_mag[idx_m2];
			float32_t Yp1 = fft_mag[idx_p1];
			
			delta = (Yp1 - Ym1 - 2*Ym2) / (Ym2 + Ym1 + Y0 + Yp1);
		} 
		else {
			// 取两种估计的平均值 (论文式18)
			float32_t Y0 = fft_mag[fund_index];
			float32_t Ym1 = fft_mag[idx_m1];
			float32_t Ym2 = fft_mag[idx_m2];
			float32_t Yp1 = fft_mag[idx_p1];
			float32_t Yp2 = fft_mag[idx_p2];
			
			float32_t delta1 = (Yp1 + 2*Yp2 - Ym1) / (Ym1 + Y0 + Yp1 + Yp2);
			float32_t delta2 = (Yp1 - Ym1 - 2*Ym2) / (Ym2 + Ym1 + Y0 + Yp1);
			delta = 0.5f * (delta1 + delta2);
		}

		// 8. 频率估计 f = (fund_index + δ) * (fs / N)
		freq_est = (fund_index + delta) * (SAMPLE_RATE / (float32_t)LEN);
		
		// 9. 精确幅值校准 (汉宁窗恢复系数)
		// |A| = 2 * sqrt(|X(k)|²) / (0.5 * N * S_w)
		// 汉宁窗相干增益 S_w = 0.5
		amp_est = 2.0f * sqrtf(fund_value) / (0.5f * LEN * 0.5f);
	
		if(SampleMode == 1){
			freq_est *=26;
		}
		
		//动态改变采样率
		if(freq_est > (1000-100)){
			__HAL_TIM_DISABLE(&htim2); 
			if(freq_est <=100000){
				period = 25;
				SAMPLE_RATE = (uint32_t)(freq_est * 25);//较低频率下，过采样一个周期采样25个点
				__HAL_TIM_SET_AUTORELOAD(&htim2, 240000000/SAMPLE_RATE -1); 
				SampleMode = 0;
			}

			else{
				period = 25;
				if(fabs(prev_freq_est - freq_est)/prev_freq_est > 0.05f){
					SAMPLE_RATE = freq_est*25/26;	
				}
				prev_freq_est = freq_est;
				
				__HAL_TIM_SET_AUTORELOAD(&htim2, 240000000/SAMPLE_RATE -1); 
				SampleMode = 1;
			}
			__HAL_TIM_ENABLE(&htim2); 
		}
		// 能量积分（±2 点）
		float energy = 0;
		for(int k = fund_index-2; k <= fund_index+2; k++) {
			if(k > 0 && k < LEN/2) {
				energy += fft_mag[k] * fft_mag[k];
			}
		}
		float fund_energy = sqrtf(energy);
		
		//找谐波,抛物线插值修正
		harm_power = 0;
		for(int h = 2; h <= 5; h++){
			float harm_bin_est = h * fund_index;
			float search_min = harm_bin_est -5;
			float search_max = harm_bin_est +5;
			
			float local_max = 0;
			int idx_max = harm_bin_est;
			
			for(int k = search_min; k <= search_max; k++){
				if(k > 0 && k < LEN/2 && fft_mag[k] > local_max){
					local_max = fft_mag[k];
					idx_max = k;
				}
			}
			
			float alpha = fft_mag[idx_max-1];
			float beta  = fft_mag[idx_max];
			float gamma = fft_mag[idx_max+1];
			float delta = 0.5f * (alpha - gamma) / (alpha - 2*beta + gamma);
			float harm_bin = idx_max + delta;
			float freq_est = harm_bin * SAMPLE_RATE / LEN;
			float mag_est  = beta - 0.25f * (alpha - gamma) * delta;
			
			//能量积分
			 energy = 0;
            for(int k = idx_max-2; k <= idx_max+2; k++) {
                if(k > 0 && k < LEN / 2) {
                    energy += fft_mag[k] * fft_mag[k];
                }
            }
            float harm_amp = sqrtf(energy);

			//归一化
			unify_mag[h-1] = harm_amp / fund_energy;
			//功率累计
			harm_power += harm_amp * harm_amp;
		}
		THD = sqrtf(harm_power) / fund_energy *100.0f;
		
//		for(int i = 0; i<LEN; i++){
//			printf("%d\r\n", adc_test3[i]);
//		}
//		for(int i = 0; i<LEN/2; i++){
//			printf("%d\r\n", adc_test1[i]);
//		}
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, LEN);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
