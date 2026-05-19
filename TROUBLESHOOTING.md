# OneDofRobot — Troubleshooting & Knowledge Base

> **เป้าหมาย**: ไฟล์นี้รวบรวมปัญหาและวิธีแก้ทั้งหมดที่เจอระหว่าง development  
> อ่านก่อนเริ่ม debug ทุกครั้ง — ประหยัดเวลาได้มาก

> **สถานะปัจจุบัน (2026-05-19)**: กำลัง tune Kalman Filter + Velocity PID ผ่าน dashboard ของเราเอง  
> Base System (production) ยังไม่ได้ใช้ — อ่านส่วนที่ 13 ก่อน migrate

---

## สารบัญ
1. [Hardware & Project Overview](#1-hardware--project-overview)
2. [Modbus RTU via UART (USART2)](#2-modbus-rtu-via-uart-usart2)
3. [VCP Loopback Problem](#3-vcp-loopback-problem)
4. [CubeIDE/GDB ล็อค COM Port](#4-cubeidegdb-ล็อค-com-port)
5. [UART Interrupt ไม่ Trigger](#5-uart-interrupt-ไม่-trigger)
6. [GPIO ไม่ Configure → LED/Output ไม่ทำงาน](#6-gpio-ไม่-configure--ledoutput-ไม่ทำงาน)
7. [TIM6 ISR / Control Loop ไม่รัน](#7-tim6-isr--control-loop-ไม่รัน)
8. [GPIO Pull-up กับ Mode Detection ผิดพลาด](#8-gpio-pull-up-กับ-mode-detection-ผิดพลาด)
9. [Python Dashboard — minimalmodbus](#9-python-dashboard--minimalmodbus)
10. [pyocd ใช้ไม่ได้ (Debug Probe)](#10-pyocd-ใช้ไม่ได้-debug-probe)
11. [Register Map (Modbus)](#11-register-map-modbus)
12. [Checklist ก่อนทดสอบทุกครั้ง](#12-checklist-ก่อนทดสอบทุกครั้ง)

---

## 1. Hardware & Project Overview

| รายการ | รายละเอียด |
|---|---|
| MCU | STM32G474RE (Nucleo-G474RE) |
| Clock | 170 MHz |
| UART | USART2 — PA2(TX), PA3(RX) |
| Baud / Format | **19200, 8E1** (Even parity, 9-bit word length) |
| Modbus Slave ID | **21** (0x15) |
| Control Loop | TIM6 interrupt @ 1 ms (Velocity PID inner loop) |
| Position Loop | ทุก 10 ms ภายใน Cascade_Control_Update() |
| Encoder | TIM2 (quadrature encoder mode) |
| PWM | TIM1 CH1 — Motor drive |
| Motor Dir | GPIO Output (Motor_Dir_Pin) |
| LED Debug | PA5 (LD2 บน Nucleo) — GPIO Output |

---

## 2. Modbus RTU via UART (USART2)

### การตั้งค่า usart.c
```c
huart2.Init.BaudRate     = 19200;
huart2.Init.WordLength   = UART_WORDLENGTH_9B;   // ← 9-bit เพราะใช้ Even Parity
huart2.Init.StopBits     = UART_STOPBITS_1;
huart2.Init.Parity       = UART_PARITY_EVEN;
huart2.Init.Mode         = UART_MODE_TX_RX;
huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
huart2.Init.OverSampling = UART_OVERSAMPLING_16;
```

> ⚠️ **สำคัญ**: ถ้าใช้ Even/Odd Parity ต้องตั้ง `UART_WORDLENGTH_9B` (ไม่ใช่ 8B)  
> 8B + Even Parity = frame ผิด → Python อ่านไม่ได้

### รับข้อมูลด้วย Interrupt (ที่ใช้จริง)
```c
// USER CODE BEGIN 2 — init
__HAL_UART_CLEAR_OREFLAG(&huart2);   // ← ต้องทำก่อนเสมอ
HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));

// USER CODE BEGIN 0 — callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        if (Size >= 8 && rx_buffer[0] == SLAVE_ID) {  // ← กรอง loopback
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
```

---

## 3. VCP Loopback Problem

### อาการ
- Modbus success rate ต่ำ ~20–30%
- Callback ถูกเรียกบ่อยกว่าที่ควร
- บางทีตอบ response เอง แล้ว receive callback รับ response ของตัวเองกลับมา

### สาเหตุ
ST-Link Virtual COM Port (VCP) มีฟีเจอร์ echo — TX ของ STM32 ถูก echo กลับมาที่ RX  
ทำให้ `HAL_UARTEx_ReceiveToIdle_IT` trigger ด้วย **response ที่ STM32 เพิ่ง TX ออกไป**  
แทนที่จะ trigger เฉพาะ **request จาก PC**

### แก้
กรอง frame ด้วยการเช็ค `rx_buffer[0] == SLAVE_ID` และ `Size >= 8` ก่อน parse:
```c
if (Size >= 8 && rx_buffer[0] == SLAVE_ID) {
    Modbus_Parse_Frame(rx_buffer, Size);
}
```
Response ของ STM32 เอง (byte แรกก็คือ SLAVE_ID เหมือนกัน แต่ SIZE ต่างกัน หรือ content ต่างกัน) — **ในทางปฏิบัติเงื่อนไข Size >= 8 กับ byte[0] == SLAVE_ID เพียงพอ**

### ผล
Success rate จาก ~20–30% → **100%**

---

## 4. CubeIDE/GDB ล็อค COM Port

### อาการ
```
serial.serialutil.SerialException: [Errno 13] could not open port COM9: [WinError 13] Access is denied
```
หรือ `PermissionError: [Errno 13]`

### สาเหตุ
Process เหล่านี้ยึด COM port ไว้:
- `arm-none-eabi-gdb.exe` (CubeIDE GDB)
- `ST-LINK_gdbserver.exe`  
- `putty.exe` (ถ้าเปิดค้างไว้)
- CubeIDE เอง (บางครั้ง)

### แก้
```bat
taskkill /IM arm-none-eabi-gdb.exe /F
taskkill /IM ST-LINK_gdbserver.exe /F
taskkill /IM putty.exe /F
```
หรือปิด CubeIDE ทั้งหมดก่อนรัน Python

> ⚠️ **หมายเหตุ**: COM port อาจเปลี่ยนหมายเลขหลัง ST-Link reset (เช่น COM3 → COM9)  
> ตรวจสอบใน Device Manager ก่อนเสมอ

### วิธี Flash โดยไม่ต้องปิด CubeIDE
ใช้ **Run** (Ctrl+F11) แทน **Debug** — Run mode ไม่ lock GDB session  
หลัง flash แล้วกด Stop / ปิด debug session ก่อนรัน Python

---

## 5. UART Interrupt ไม่ Trigger

### อาการ
- `HAL_UARTEx_ReceiveToIdle_IT` return `HAL_OK` แต่ callback ไม่ถูกเรียก
- Polling mode (`HAL_UARTEx_ReceiveToIdle`) ทำงานได้ปกติ

### สาเหตุ
มี **UART error flag ค้างอยู่** (โดยเฉพาะ Overrun Error — ORE)  
เกิดจาก UART receive data มาในขณะที่ยังไม่ได้เปิด IT หรือมี noise บน line

### แก้
ล้าง error flag ก่อนเรียก `ReceiveToIdle_IT` ทุกครั้ง:
```c
__HAL_UART_CLEAR_OREFLAG(&huart2);
HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));
```
และทำใน `HAL_UART_ErrorCallback` ด้วย เพื่อ auto-recover:
```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));
    }
}
```

---

## 6. GPIO ไม่ Configure → LED/Output ไม่ทำงาน

### อาการ
- Flash firmware แล้ว LED (PA5/LD2) ไม่กะพริบ
- `HAL_GPIO_WritePin()` ไม่มีผล

### สาเหตุ
CubeMX generate `gpio.c` โดยไม่ configure PA5 เป็น Output  
เพราะ PA5 อาจถูก assign ให้ SPI1 หรือไม่ได้เพิ่มใน `.ioc` file

### แก้ (ใน gpio.c)
```c
void MX_GPIO_Init(void)
{
    // ...existing code...

    // เพิ่มส่วนนี้สำหรับ LD2 debug LED
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
```

> ⚠️ **ข้อควรระวัง**: เมื่อ re-generate code จาก CubeMX จะทับโค้ดนอก USER CODE block  
> ต้องเพิ่มโค้ดนี้ใหม่ทุกครั้งถ้าไม่ได้ configure ใน .ioc  
> **วิธีถาวร**: เพิ่ม PA5 เป็น GPIO_Output ใน `.ioc` แล้ว re-generate

---

## 7. TIM6 ISR / Control Loop ไม่รัน

### อาการ
- Modbus registers 0x20–0x22 (telemetry) = 0 ตลอด
- Motor ไม่หมุน แม้จะมี code ใน ISR

### Debug วิธีที่ 1 — ISR Counter
เพิ่ม counter ใน ISR แล้วอ่านผ่าน Modbus:
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        modbus_registers[0x30]++;  // ← debug: นับ ISR calls
        // ... rest of ISR
    }
}
```
อ่าน register `0x30` ผ่าน Python — ถ้าเพิ่มขึ้น = ISR ทำงาน ถ้า 0 ตลอด = ISR ไม่ถูกเรียก

### Debug วิธีที่ 2 — LED Toggle ใน ISR
```c
if (htim->Instance == TIM6) {
    static uint32_t cnt = 0;
    if (++cnt >= 500) { cnt = 0; HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); } // toggle ทุก 500ms
}
```

### สาเหตุที่พบบ่อย
| สาเหตุ | วิธีตรวจ | แก้ |
|---|---|---|
| ลืม `HAL_TIM_Base_Start_IT(&htim6)` | ค้นหาใน main.c | เพิ่มใน USER CODE BEGIN 2 |
| Firmware เก่าที่ flash ไม่มี TIM6 init | Build + Flash ใหม่ | Rebuild all, flash ใหม่ |
| Init code crash ก่อนถึง TIM6 start | LED ไม่กะพริบ/ค้าง | ทดสอบ LED toggle ในแต่ละขั้นตอน |
| Mode ผิด (ดู #8) | อ่าน mode register | ดูส่วนที่ 8 |

### ตรวจ TIM6 config ใน MX_TIM6_Init
TIM6 ต้องเปิด ARR register (auto-reload) และ set period ให้ถูก  
สำหรับ 1 ms @ 170 MHz: `Period = 170000 - 1` (ถ้า Prescaler = 1-1)  
หรือ `Period = 17000 - 1` (Prescaler = 10-1) — ขึ้นกับ CubeMX config

---

## 8. GPIO Pull-up กับ Mode Detection ผิดพลาด

### อาการ
- ตั้ง `current_system_mode = MODE_MANUAL` ใน init แต่ระบบ switch ไป `MODE_AUTO` ทันที
- `Velocity_Tune_Update()` ไม่ถูกเรียก
- Modbus registers telemetry = 0 ทั้งหมด

### สาเหตุ
Code ตรวจ mode จาก GPIO PB0/PB1 ที่ configure เป็น `GPIO_PULLUP`:
```c
// ถ้า PB1=HIGH, PB0=HIGH → MODE_AUTO
// ถ้า PB1=HIGH, PB0=LOW  → MODE_MANUAL
```
เมื่อไม่ต่อสาย PB0/PB1 กับอะไร → ทั้งคู่ถูก pull-up ขึ้น HIGH → ตรวจได้ `MODE_AUTO`

### แก้
**แนวทางที่ 1** (ระยะสั้น): เพิ่ม telemetry write ใน MODE_AUTO branch ด้วย  
```c
// ใน HAL_TIM_PeriodElapsedCallback, ส่วน MODE_AUTO:
modbus_registers[0x20] = (uint16_t)(int16_t)(tune_target_qd * 100.0f);
modbus_registers[0x21] = (uint16_t)(int16_t)(qd_out         * 100.0f);
modbus_registers[0x22] = (uint16_t)(int16_t)(monitor_V_in   * 100.0f);
```

**แนวทางที่ 2** (ถาวร): ต่อสายจริงๆ หรือเพิ่ม default mode ที่ทนต่อ floating input:
```c
// เปลี่ยน default: ถ้าไม่เข้าเงื่อนไขใดๆ = MODE_MANUAL
SystemMode_t detected_mode = MODE_MANUAL;  // ← default manual แทน current
if (PB1=SET && PB0=SET)   detected_mode = MODE_AUTO;
if (PB1=SET && PB0=RESET) detected_mode = MODE_MANUAL;
```

---

## 9. Python Dashboard — minimalmodbus

### Version ที่ใช้
```
minimalmodbus 2.1.1
```

### API ที่เปลี่ยนใน v2.x (Breaking Change)
```python
# ❌ v1.x (ใช้ไม่ได้แล้ว)
instrument.write_registers(addr, values, functioncode=16)

# ✅ v2.x (ถูกต้อง) — FC16 เป็น default แล้ว
instrument.write_registers(addr, values)

# read_registers ยังมี functioncode parameter
instrument.read_registers(addr, count, functioncode=3)
```

### ตั้งค่า minimalmodbus ให้ตรงกับ STM32
```python
import minimalmodbus

instrument = minimalmodbus.Instrument('COM9', 21)  # port, slave_id
instrument.serial.baudrate   = 19200
instrument.serial.bytesize   = 8
instrument.serial.parity     = 'E'   # Even
instrument.serial.stopbits   = 1
instrument.serial.timeout    = 0.3
instrument.mode              = minimalmodbus.MODE_RTU
instrument.debug             = False
```

> ⚠️ **bytesize = 8** ไม่ใช่ 9 — `bytesize` ใน pyserial หมายถึง data bits ไม่รวม parity  
> STM32 ตั้ง `WORDLENGTH_9B` เพราะรวม parity bit แต่ pyserial คิดแยก

### Register Scale (อ่าน/เขียนต้องหาร/คูณ)
| Register | ชื่อ | Scale | หน่วย |
|---|---|---|---|
| 0x10 | Kp | ×100 | — |
| 0x11 | Ki | ×100 | — |
| 0x12 | Kd | ×100 | — |
| 0x13 | tune_speed | ×10 | rad/s |
| 0x14 | tune_period_half | ×1 | ms |
| 0x20 | ref_qd | ×100 | rad/s |
| 0x21 | qd_out | ×100 | rad/s |
| 0x22 | Vin | ×100 | V |
| 0x30 | ISR counter | ×1 | counts |
| 0x31 | E-stop status | ×1 | 0=OK, 1=ERR |

---

## 10. pyocd ใช้ไม่ได้ (Debug Probe)

### อาการ
```
No available debug probes are connected
```

### สาเหตุที่พบ
1. CubeMonitor หรือ CubeIDE ใช้ ST-Link อยู่ (ล็อค probe)
2. Windows driver ผิด (ต้องเป็น WinUSB ไม่ใช่ libusb)
3. pyocd version ไม่รองรับ STM32G4 target

### สรุปการตัดสินใจ
**ละทิ้ง pyocd ทั้งหมด** — เปลี่ยนมาใช้ **Modbus RTU ผ่าน VCP** แทน  
เหตุผล: ไม่ต้องพึ่ง ST-Link, ทำงานได้ใน production (ไม่ต้องต่อ debugger), latency ต่ำกว่า

---

## 11. Register Map (Modbus)

### Write-only (PC → STM32) — FC06 หรือ FC16
| Address | ชื่อ | Scale | ตัวอย่าง |
|---|---|---|---|
| 0x10 | Kp | ×100 | Kp=1.5 → write 150 |
| 0x11 | Ki | ×100 | Ki=0.5 → write 50 |
| 0x12 | Kd | ×100 | Kd=0.0 → write 0 |
| 0x13 | tune_speed | ×10 | 3.0 rad/s → write 30 |
| 0x14 | tune_period_half | ms | 3000 ms → write 3000 |

### Read-only (STM32 → PC) — FC03
| Address | ชื่อ | Scale | หน่วย |
|---|---|---|---|
| 0x20 | ref_qd | ×100 | rad/s (int16, signed) |
| 0x21 | qd_out | ×100 | rad/s (int16, signed) |
| 0x22 | Vin | ×100 | V |
| 0x30 | ISR counter | — | debug เท่านั้น |
| 0x31 | E-stop | — | 0=OK, 1=ESTOP |

### อ่านค่า signed จาก uint16
```python
raw = instrument.read_register(0x20, functioncode=3)
value = raw if raw < 32768 else raw - 65536  # แปลง uint16 → int16
qd_ref = value / 100.0
```

---

## 12. Checklist ก่อนทดสอบทุกครั้ง

### ฝั่ง STM32
- [ ] Build ผ่านไม่มี error
- [ ] Flash ด้วย **Run** mode (ไม่ใช่ Debug)
- [ ] LED PA5 กะพริบ = firmware ทำงาน
- [ ] ปิด CubeIDE Debug session ก่อนเสมอ

### ฝั่ง PC
- [ ] ตรวจ COM port ใน Device Manager (อาจเปลี่ยนหมายเลข)
- [ ] Kill GDB processes ถ้า Python เข้า port ไม่ได้:
  ```bat
  taskkill /IM arm-none-eabi-gdb.exe /F
  taskkill /IM ST-LINK_gdbserver.exe /F
  ```
- [ ] ตรวจสอบ baud/parity ใน Python ตรงกับ `usart.c`

### ทดสอบ Modbus
```python
# ขั้นตอนที่ 1: อ่าน ISR counter — ถ้าเพิ่มขึ้น = TIM6 ทำงาน
isr_count_1 = instrument.read_register(0x30, functioncode=3)
time.sleep(1)
isr_count_2 = instrument.read_register(0x30, functioncode=3)
assert isr_count_2 > isr_count_1, "TIM6 ISR ไม่ทำงาน!"

# ขั้นตอนที่ 2: อ่าน telemetry
regs = instrument.read_registers(0x20, 3, functioncode=3)
qd_ref = (regs[0] if regs[0] < 32768 else regs[0]-65536) / 100.0
qd_act = (regs[1] if regs[1] < 32768 else regs[1]-65536) / 100.0
v_in   = regs[2] / 100.0
print(f"ref={qd_ref:.2f} act={qd_act:.2f} Vin={v_in:.2f}")
```

---

---

## 13. Base System (Production) — Register Map & Migration Notes

> ⚠️ **ยังไม่ได้ใช้ในขั้นตอน Tune PID**  
> Base System จะใช้ตอน deploy จริงในภารกิจ FRA263/264  
> อ่านส่วนนี้ก่อน migrate เพื่อไม่ให้ register ชนกัน

### 13.1 Overview

| รายการ | รายละเอียด |
|---|---|
| Protocol | Modbus RTU (เหมือนกัน ✓) |
| Baud / Format | **19200, 8E1** (เหมือนกัน ✓) |
| Slave ID | **21** (เหมือนกัน ✓) |
| Connection | PC → `main.exe` (Python) → USART2 |
| UI | Docker + Web browser (http://localhost:3000) |
| Backend | WebSocket ws://localhost:8765 |

### 13.2 Heartbeat Protocol (สำคัญมาก!)

Base System ใช้ **handshake แบบพิเศษ** บน register `0x00`:

```
Robot  → writes 22881 ("YA") to 0x00  (บอกว่าพร้อม)
PC     → reads 0x00, เห็น YA → writes 18537 ("HI") กลับ
```

> ❗ ถ้า firmware ไม่ implement heartbeat นี้ UI จะแสดง **"link not alive"**  
> ต้องเพิ่มใน firmware ก่อน connect กับ Base System

### 13.3 ⚠️ REGISTER CONFLICTS — ตารางชนกันระหว่าง Tune Dashboard กับ Base System

**นี่คือจุดอันตรายที่สุด** เมื่อ migrate ต้องเปลี่ยน register address ใน firmware

| Address | Tune Dashboard (ปัจจุบัน) | Base System (production) | สถานะ |
|:-------:|--------------------------|--------------------------|:------:|
| **0x00** | (ไม่ใช้) | Heartbeat YA/HI | ต้องเพิ่ม |
| **0x10** | **Kp × 100** | Precision test: Final position | ❌ ชนกัน |
| **0x11** | **Ki × 100** | Precision test: Repeat count | ❌ ชนกัน |
| **0x12** | **Kd × 100** | Pick & Place slot 0 | ❌ ชนกัน |
| **0x13** | tune_speed × 10 | Pick & Place slot 1 | ❌ ชนกัน |
| **0x14** | tune_period (ms) | Pick & Place slot 2 | ❌ ชนกัน |
| **0x20** | ref_qd × 100 (telemetry) | Pick & Place slot 14 | ❌ ชนกัน |
| **0x21** | qd_out × 100 (telemetry) | Pick & Place slot 15 | ❌ ชนกัน |
| **0x22** | Vin × 100 (telemetry) | N/A pairs count | ❌ ชนกัน |
| **0x25** | (ไม่ใช้) | Soft stop | ต้องเพิ่ม |
| **0x28** | (ไม่ใช้) | Position ÷ 10 | ต้องเพิ่ม |
| **0x29** | (ไม่ใช้) | Velocity ÷ 10 | ต้องเพิ่ม |
| **0x30** | ISR counter (debug) | **Acceleration ÷ 10** | ❌ ชนกัน |
| **0x31** | E-stop status | E-stop status | ✅ ตรงกัน |

### 13.4 แผน Migration (ทำทีหลัง ตอน Tune เสร็จแล้ว)

**ขั้นตอนที่ต้องทำก่อน connect Base System:**

1. **ย้าย Kp/Ki/Kd** ออกจาก 0x10–0x12 ไปที่ address ว่าง (เช่น 0x40–0x42)
2. **ย้าย tune registers** (0x13, 0x14) ออกไปเช่นกัน
3. **ย้าย telemetry** (0x20–0x22) ไปที่ format ที่ Base System ต้องการ:
   - 0x28 = Position × 10 (int16)
   - 0x29 = Velocity × 10 (int16) ← ปัจจุบันเราใช้ × 100 ต้องเปลี่ยน
   - 0x30 = Acceleration × 10 (int16)
4. **เพิ่ม Heartbeat** ที่ 0x00:
   ```c
   // ใน HAL_TIM_PeriodElapsedCallback หรือ while(1):
   if (modbus_registers[0x00] == 22881) {  // "YA"
       modbus_registers[0x00] = 18537;     // ตอบ "HI"
   }
   ```
5. **เพิ่ม Current Task** ที่ 0x27 (bits: Homing=1, GoPick=2, GoPlace=4, GoPoint=8)
6. **เพิ่ม Reed sensors** ที่ 0x26
7. **เพิ่ม Soft stop** ที่ 0x25

### 13.5 Base System WRITE Register Map (สรุป)

| Addr | ชื่อ | Type | ความหมาย |
|:----:|------|:----:|-----------|
| 0x00 | Heartbeat | uint16 | PC ตอบ 18537 ("HI") |
| 0x01 | Mode | bits | 1=Home, 2=Jog, 4=Auto, 8=SetHome, 16=Test |
| 0x02 | Gripper manual | bits | 0=Up, 1=Down, 2=Open, 4=Close |
| 0x03 | Gripper seq | bits | 1=Pick, 2=Place |
| 0x04 | Gripper enable | bit0 | 0=disable, 1=enable (ใน AUTO mode) |
| 0x05 | Jog | int16 | องศา (+CCW, −CW) |
| 0x06 | Test type | bit0 | 0=Precision, 1=Performance |
| 0x07 | Perf speed | int16 | ความเร็วที่ต้องการ |
| 0x08 | Perf accel | int16 | ความเร่งที่ต้องการ |
| 0x09 | Precision init | int16 | ตำแหน่งเริ่มต้น |
| 0x10 | Precision final | int16 | ตำแหน่งเป้าหมาย |
| 0x11 | Precision repeat | int16 | จำนวนรอบ (+degree, −index) |
| 0x12–0x21 | Pick&Place slots | int16 | index + direction ต่อ slot (mag=index, sign=direction) |
| 0x22 | Pair count | uint16 | จำนวนคู่ pick+place |
| 0x23 | P2P unit | bit0 | 0=degree, 1=index |
| 0x24 | P2P target | int16 | เป้าหมาย |
| 0x25 | Soft stop | bit0 | 0=run, 1=stop |

### 13.6 Base System READ Register Map (สรุป)

| Addr | ชื่อ | Scale | ความหมาย |
|:----:|------|:-----:|-----------|
| 0x00 | Heartbeat | — | Robot ส่ง 22881, PC ตอบ 18537 |
| 0x26 | Reed sensors | bits | bit0=Reed1, bit1=Reed2, bit2=Reed3(jaw) |
| 0x27 | Current task | bits | bit0=Homing, bit1=GoPick, bit2=GoPlace, bit3=GoPoint, 0=Idle |
| 0x28 | Position | ÷10 | องศา (int16) |
| 0x29 | Velocity | ÷10 | (int16) |
| 0x30 | Acceleration | ÷10 | (int16) |
| 0x31 | Emergency | bit0 | 0=OK, 1=E-Stop active |

**Reed sensor ที่ 0x26:**

| Reed1 | Reed2 | แสดงผล |
|:-----:|:-----:|--------|
| ON | OFF | Up |
| OFF | ON | Down |
| other | other | Idle |

Reed3 (bit2): ON = Closed, OFF = Open

### 13.7 Scale ต่างกัน! (ต้องระวัง)

| ข้อมูล | Tune Dashboard (ปัจจุบัน) | Base System |
|--------|:------------------------:|:-----------:|
| velocity ref/act | × **100** | × **10** |
| position | (ไม่มี) | × **10** |
| Kp/Ki/Kd | × **100** | (ไม่มี — internal) |

เมื่อ migrate ต้องเปลี่ยน firmware ให้ใช้ scale × 10 แทน × 100 ใน register 0x28–0x30

---

*อัปเดตล่าสุด: 2026-05-19*  
*โปรเจกต์: OneDofRobot / STM32G474RE / Nucleo-G474RE*
