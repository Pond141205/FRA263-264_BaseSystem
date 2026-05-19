/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "base_system.h"
#include "cascade_control.h"
#include "trajectory.h"
#include "encoder.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Encoder_t henc2;
//Trapz_Profile_t my_profile;
SCurve_Profile_t my_profile;

typedef enum {
    MODE_HOMING,
    MODE_AUTO,
    MODE_MANUAL
} SystemMode_t;
volatile SystemMode_t current_system_mode = MODE_HOMING;
SystemMode_t new_mode ;

typedef enum {
    STATE_IDLE,    // สถานะว่าง (รอคนกดปุ่ม)
    STATE_MOVING,  // กำลังหมุนมอเตอร์ไปเป้าหมาย
    STATE_WAITING  // ถึงเป้าหมายแล้ว (กำลังนับเวลาหน่วง 2 วินาที)
} SequenceState_t;
SequenceState_t seq_state = STATE_IDLE; // เริ่มต้นที่สถานะว่าง

const float WAIT_TIME = 2;           // กำหนดเวลาหน่วง (2 วินาที)
float waypoints_deg[] = {90.0f, 0.0f, 180.0f, 0.0f, 360.0f, 0.0f};
float current_degree_out;
float wait_timer = 0.0f;                // ตัวจับเวลารอ
float ref_q = 0.0f;
float ref_qd = 0.0f;
float ref_qdd = 0.0f;
float ref_j = 0.0f;


int signal= 0;
int current_wp_index = 0;               // จุดที่กำลังจะไป


const uint8_t total_waypoints = 6;
static uint8_t btn_prev_state = 0;

volatile uint8_t debug_pb0_state;
volatile uint8_t debug_pb1_state;

uint8_t rx_buffer[128];


float tune_target_qd = 0.0f;       // ความเร็วเป้าหมายที่จะส่งเข้า Cascade
float tune_timer = 0.0f;           // ตัวจับเวลาสำหรับสลับเฟส
const float TUNE_STEP_TIME = 3.0f; // สลับทิศทางทุกๆ 1.5 วินาที
const float TUNE_SPEED_RADS = 3.0f;

/* [TUNE] ตัวแปร Velocity Tune - ประกาศไว้นี่เพื่อให้ main() reset ได้ */
volatile float tune_ref_qd = 0.0f;
         float tune_t      = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//Update_Telemetry_To_PC(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        /* ตรวจว่าเป็น Modbus request จริงๆ (byte แรก = SLAVE_ID)
         * ถ้าไม่ใช่ = loopback ของ response เอง → ทิ้งไป */
        if (Size >= 8 && rx_buffer[0] == SLAVE_ID) {
            Modbus_Parse_Frame(rx_buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  Heartbeat_Init();
  __HAL_UART_CLEAR_OREFLAG(&huart2);   /* เคลียร์ error flags ก่อน */
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));
  henc2.htim = &htim2;

  Encoder_Init(&henc2, &htim2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  SCurve_Init(&my_profile);
  Cascade_Control_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_MOE_ENABLE(&htim1);

  //Homing Testing Code
//  while(!(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))){
//	  current_counter = __HAL_TIM_GET_COUNTER(&htim2);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	  signal =HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
//  }
//  while(!(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4))){
//	  current_counter = __HAL_TIM_GET_COUNTER(&htim2);
//	  HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin, GPIO_PIN_RESET);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
//  }
//  while(!(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))){
//	  current_counter = __HAL_TIM_GET_COUNTER(&htim2);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//  }
//  current_counter = __HAL_TIM_SET_COUNTER(&htim2,0);
//  while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)){
//	  current_counter = __HAL_TIM_GET_COUNTER(&htim2);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
//  }
//  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  //Homing Testing Code

  current_system_mode = MODE_MANUAL;
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* [ใช้ interrupt mode แทน polling] */

	  Heartbeat_Update();

	  //current_counter = __HAL_TIM_GET_COUNTER(&htim2);
	  debug_pb0_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	  debug_pb1_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

      SystemMode_t detected_mode = current_system_mode;
      if      (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET &&
               HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET)  detected_mode = MODE_AUTO;
      else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET &&
               HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) detected_mode = MODE_MANUAL;

      /* [TUNE] Reset integral + KF เมื่อสลับเข้า MODE_MANUAL ครั้งแรก */
      if (detected_mode != current_system_mode) {
          if (detected_mode == MODE_MANUAL) {
              Cascade_Control_Init();   /* reset PID integral + KF */
              tune_t = 0.0f;           /* reset square wave timer */
          }
          current_system_mode = detected_mode;
      }


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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ════════════════════════════════════════════════════════════════════════
 *  [TUNE - TEMPORARY] Velocity PID Tuning via Modbus
 *  ────────────────────────────────────────────────────────────────────
 *  Register Map (PC เขียนผ่าน FC06 / อ่านผ่าน FC03):
 *
 *  WRITE (PC → STM):
 *    0x10  Kp  * 100   (int16)  เช่น Kp=7.0  → เขียน 700
 *    0x11  Ki  * 100   (int16)  เช่น Ki=1.0  → เขียน 100
 *    0x12  Kd  * 100   (int16)  เช่น Kd=0.0  → เขียน 0 (ใช้ค่าเดิม)
 *    0x13  ความเร็วทดสอบ * 10 (int16)  เช่น 3.0 rad/s → เขียน 30
 *    0x14  ระยะเวลาแต่ละเฟส (ms) uint16  เช่น 3000 ms
 *
 *  READ (STM → PC):
 *    0x20  ref_qd  * 100  (int16)  ความเร็วอ้างอิง
 *    0x21  qd_out  * 100  (int16)  ความเร็วจริง (จาก KF)
 *    0x22  V_out   * 100  (int16)  แรงดันที่สั่ง motor
 *
 *  วิธีลบทิ้ง: ลบฟังก์ชันนี้ + บรรทัดเรียกใน MODE_MANUAL ISR
 * ════════════════════════════════════════════════════════════════════════ */
/* monitor_V_in, qd_out  → cascade_control.h
   tune_ref_qd, tune_t   → USER CODE BEGIN PV  (ประกาศไว้แล้ว) */

void Velocity_Tune_Update(void)
{
    /* --- อ่านพารามิเตอร์จาก Modbus --- */
    float tune_speed = (modbus_registers[0x13] != 0)
                       ? (float)(int16_t)modbus_registers[0x13] / 10.0f
                       : TUNE_SPEED_RADS;   /* fallback ค่า default */

    float tune_half  = (modbus_registers[0x14] != 0)
                       ? (float)modbus_registers[0x14] / 1000.0f
                       : TUNE_STEP_TIME;    /* fallback ค่า default */

    /* --- สร้าง Square Wave Reference --- */
    tune_t += 0.001f;
    if      (tune_t < tune_half)              tune_ref_qd =  tune_speed;
    else if (tune_t < tune_half * 2.0f)       tune_ref_qd = -tune_speed;
    else                                      tune_t = 0.0f;

    /* --- ส่งเข้า Cascade (bypass position loop) --- */
    Cascade_Control_Update(0.0f, tune_ref_qd);

    /* --- เขียน Telemetry กลับ Modbus --- */
    modbus_registers[0x20] = (uint16_t)(int16_t)(tune_ref_qd * 100.0f);
    modbus_registers[0x21] = (uint16_t)(int16_t)(qd_out      * 100.0f);
    modbus_registers[0x22] = (uint16_t)(int16_t)(monitor_V_in * 100.0f);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) // ตรวจสอบว่าใช่ขา PC13 หรือไม่
    {
    	// อ่านค่าสถานะปัจจุบันของปุ่ม (PC13 Pull-up: กด=0, ปล่อย=1)
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
		{
			// --- [กรณีที่ 1: กดปุ่ม E-Stop] ---
			// 1. หยุด Hardware ทันที
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_MOE_DISABLE(&htim1);

			// 2. ล็อกสถานะ Software
			seq_state = STATE_IDLE;
			my_profile.is_running = 0;
			modbus_registers[0x31] = 1; // บอก PC ว่า Error
		}
		else
		{
			// --- [กรณีที่ 2: ปล่อยปุ่ม E-Stop] ---
			// 1. เคลียร์สถานะ Error ใน Modbus เพื่อให้หน้าจอ UI กลับมาปกติ
			modbus_registers[0x31] = 0;

			// 2. คืนค่า Main Output Enable (MOE) เพื่อเตรียมพร้อมรับคำสั่งใหม่
			// แต่ยังไม่ต้องสั่ง PWM ให้หมุน จนกว่า UI จะสั่งมา
			__HAL_TIM_MOE_ENABLE(&htim1);

			// *หมายเหตุ: ห้ามเปลี่ยน seq_state เป็น STATE_RUNNING ที่นี่
			// ให้มันอยู่ที่ STATE_IDLE เพื่อความปลอดภัย*
		}
    }
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == TIM6) { // ตรวจสอบว่าเป็น Timer ที่คุณใช้ทำ Control Loop
//
//
//    	if (current_system_mode == MODE_AUTO) {
//
//			if (seq_state == STATE_IDLE) {
//				seq_state = STATE_MOVING;
//				current_wp_index = 0; // รีเซ็ตให้ไปจุดเริ่มต้น (Index 0)
//
//				// ดึงตำแหน่งปัจจุบัน และคำนวณตำแหน่งเป้าหมาย
//				float start_q = Encoder_GetPositionRad(&henc2);
//				float target_q = waypoints_deg[current_wp_index] * (PI / 180.0f);
//
//				// สั่งเดินเครื่อง
//				//Trapz_MoveTo(&my_profile, start_q, target_q);
//				SCurve_MoveTo(&my_profile, start_q, target_q);
//			}
//
//			// --- ส่วนที่ 2: ระบบ State Machine (จัดการลำดับอัตโนมัติ) ---
//			switch (seq_state) {
//				case STATE_IDLE:
//					// ไม่ต้องทำอะไร รอจนกว่าจะมีการกดปุ่ม
//					break;
//
//				case STATE_MOVING:
//					// ถ้ามอเตอร์วิ่งไปถึงเป้าหมายแล้ว (is_running เปลี่ยนเป็น 0)
//					if (my_profile.is_running == 0) {
//						seq_state = STATE_WAITING; // สลับไปสถานะรอ 2 วิ
//						wait_timer = 0.0f;         // รีเซ็ตตัวนับเวลารอ
//					}
//					break;
//
//				case STATE_WAITING:
//					wait_timer += 0.001f; // นับเวลาไปเรื่อยๆ (DT คือเวลาต่อลูปจาก cascade_control.h)
//
//					// ถ้านับเวลาครบ 2 วินาทีแล้ว
//					if (wait_timer >= WAIT_TIME) {
//						current_wp_index++; // เลื่อน Index ไปจุดถัดไป
//
//						// เช็คว่ายังมีจุดเหลือใน Array ให้วิ่งต่อไหม?
//						if (current_wp_index < total_waypoints) {
//							//float start_q = Encoder_GetPositionRad(&henc2);
//							float start_q = ref_q;
//							float target_q = waypoints_deg[current_wp_index] * (PI / 180.0f);
//
//							//Trapz_MoveTo(&my_profile, start_q, target_q); // สั่งวิ่งต่อ
//							SCurve_MoveTo(&my_profile, start_q, target_q);
//							seq_state = STATE_MOVING; // เปลี่ยนสถานะกลับไปเป็น MOVING
//						} else {
//							// ถ้าวิ่งครบทุกจุดใน Array แล้ว
//							seq_state = STATE_IDLE; // หยุดและกลับไปรอคนกดปุ่มเริ่มรอบใหม่
//						}
//					}
//					break;
//			}
//
//			// --- ส่วนที่ 3: อัปเดตการควบคุมมอเตอร์ ---
//			// (ฟังก์ชันนี้ไม่ต้องส่ง DT ไปแล้ว เพราะโค้ดในไลบรารีไปดึงมาจาก cascade_control.h ให้เอง)
//			//Trapz_Update(&my_profile, &ref_q, &ref_qd, &ref_qdd,&ref_j);
//			SCurve_Update(&my_profile, &ref_q, &ref_qd, &ref_qdd, &ref_j);
//
//			// ส่งต่อเป้าหมายเข้า Cascade Controller
//			Cascade_Control_Update(ref_q, ref_qd);
//
//			// ดึงองศาปัจจุบันมาไว้ดูผลลัพธ์ผ่าน Live Watch
//			//Encoder_Update(&henc2);
//			current_degree_out = Encoder_GetPositionDeg(&henc2);
//		}else if (current_system_mode == MODE_MANUAL) {
//
//
//
//
//
//		}
//
//    }
//}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) { // รันทุกๆ 1 ms (DT_VEL)

        /* [DEBUG] นับจำนวนครั้งที่ ISR ถูกเรียก (อ่านผ่าน Modbus 0x30) */
        modbus_registers[0x30]++;

        if (current_system_mode == MODE_AUTO) {

            /* ════════════════════════════════════════════════════════════
             *  TRAJECTORY ชั่วคราวสำหรับจูน VELOCITY LOOP (SQUARE WAVE)
             * ════════════════════════════════════════════════════════════ */
            tune_timer += 0.001f; // เพิ่มเวลาทีละ 1ms ตามคาบของ TIM6

            if (tune_timer < TUNE_STEP_TIME) {
                tune_target_qd = TUNE_SPEED_RADS;  // เฟสหมุนตามเข็ม
            }
            else if (tune_timer < (TUNE_STEP_TIME * 2.0f)) {
                tune_target_qd = -TUNE_SPEED_RADS; // เฟสหมุนทวนเข็ม
            }
            else {
                tune_timer = 0.0f; // รีเซ็ตเวลากลับไปเริ่มรอบใหม่
            }

            /* ════════════════════════════════════════════════════════════
             *  BYPASS POSITION LOOP -> ส่งเข้า VELOCITY CONTROL โดยตรง
             * ════════════════════════════════════════════════════════════ */
            Cascade_Control_Update(0.0f, tune_target_qd);

            current_degree_out = Encoder_GetPositionDeg(&henc2);

            /* [TELEMETRY] เขียนค่ากลับ Modbus สำหรับ MODE_AUTO */
            modbus_registers[0x20] = (uint16_t)(int16_t)(tune_target_qd * 100.0f);
            modbus_registers[0x21] = (uint16_t)(int16_t)(qd_out         * 100.0f);
            modbus_registers[0x22] = (uint16_t)(int16_t)(monitor_V_in   * 100.0f);
        }
        else if (current_system_mode == MODE_MANUAL) {
            /* [TUNE - TEMPORARY] เรียกฟังก์ชันจูน Velocity PID */
            Velocity_Tune_Update();
        }
    }
}



//void Update_Telemetry_To_PC(void)
//{
//    // --- 1. กลุ่มข้อมูลการเคลื่อนที่ (ต้องคูณ 10 เสมอ) ---
//    // สมมติว่าคุณมีตัวแปร float ที่เก็บค่าจริงๆ อยู่
//    float current_position_deg = 45.5; // ตำแหน่งปัจจุบัน
//    float current_speed_rads = 1.2;    // ความเร็วปัจจุบัน
//    float current_accel_rads2 = 0.5;   // ความเร่งปัจจุบัน
//
//    // แปลงเป็น int16_t แล้วคูณ 10 ก่อนยัดลง Array (เช่น 45.5 * 10 = 455)
//    // C จะจัดการเรื่อง Two's complement (ค่าติดลบ) ให้เองเมื่อเรายัดลง uint16_t
//    modbus_registers[0x28] = (uint16_t)(int16_t)(current_position_deg * 10.0f);
//    modbus_registers[0x29] = (uint16_t)(int16_t)(current_speed_rads * 10.0f);
//    modbus_registers[0x30] = (uint16_t)(int16_t)(current_accel_rads2 * 10.0f);
//
//    // --- 2. กลุ่มสถานะ Gripper (Address 0x26) ---
//    // เอกสารบอกว่า: Bit 0 = บน(Up), Bit 1 = ล่าง(Down), Bit 2 = หนีบ(Closed)
//    uint16_t gripper_status = 0;
//
//    // สมมติฐานการเช็ค Limit Switch หรือสถานะคำสั่ง (แก้ตามพินที่คุณต่อจริง)
//    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
//        gripper_status |= 0x01; // เซ็ตบิต 0 -> แปลว่าอยู่ข้างบน (Up)
//    }
//    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
//        gripper_status |= 0x04; // เซ็ตบิต 2 -> แปลว่าหนีบอยู่ (Closed)
//    }
//    // ยัดค่าลง Array
//    modbus_registers[0x26] = gripper_status;
//
//    // --- 3. กลุ่ม Task/Mode ปัจจุบัน (Address 0x27) ---
//    // ต้องดูว่าตอนนี้ State Machine ของคุณทำงานอะไรอยู่
//    // 0 = Idle, 1 = Homing, 2 = Go Pick, 4 = Go Place
//    if (seq_state == STATE_HOMING) {
//        modbus_registers[0x27] = 1;
//    } else if (seq_state == STATE_IDLE) {
//        modbus_registers[0x27] = 0;
//    }
//    // (ใส่ให้ครบตาม State ของคุณ)
//
//    // --- 4. สถานะ Emergency (Address 0x31) ---
//    // (ถ้าคุณเขียนไว้ใน EXTI Callback แบบที่คุยกันรอบก่อนแล้ว ก็ไม่ต้องใส่ตรงนี้ซ้ำครับ)
//}

/* USER CODE END 4 */

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
