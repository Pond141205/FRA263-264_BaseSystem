/*
 * base_system.c
 *
 *  Created on: 15 พ.ค. 2569
 *      Author: POND
 */


#include "base_system.h"
#include "main.h" // เพื่อให้ใช้ HAL_GetTick() ได้
#include "usart.h"

// ตัวแปรเก็บสถานะภายในไฟล์นี้เท่านั้น

static uint32_t last_heartbeat_time = 0;
static bool is_connected = false;
uint16_t modbus_registers[64] = {0};


// ฟังก์ชันเริ่มต้น (เรียกครั้งเดียวก่อนเข้า while loop)
void Heartbeat_Init(void) {
    // ใส่ค่า YA เริ่มต้นลงไปใน Register 0 รอไว้เลย
    modbus_registers[HEARTBEAT_REG_ADDR] = ROBOT_SAYS_YA;
    last_heartbeat_time = HAL_GetTick();
    is_connected = false;
}

// ฟังก์ชันอัปเดต (เรียกซ้ำๆ ใน while(1) loop)
void Heartbeat_Update(void) {
    uint32_t current_time = HAL_GetTick();

    // 1. ตรวจสอบว่า PC เขียน HI ลงมาทับใน Register 0 หรือไม่
    if (modbus_registers[HEARTBEAT_REG_ADDR] == PC_SAYS_HI) {
        // PC เชื่อมต่ออยู่!
        is_connected = true;

        // รีเซ็ตตัวจับเวลา
        last_heartbeat_time = current_time;

        // เขียน YA กลับไปใหม่ เพื่อให้ PC รู้ว่าบอร์ดยังมีชีวิตและพร้อมรับรอบต่อไป
        modbus_registers[HEARTBEAT_REG_ADDR] = ROBOT_SAYS_YA;
    }

    // 2. ตรวจสอบ Timeout (กรณี PC ค้าง หรือสายหลุด)
    if ((current_time - last_heartbeat_time) > HEARTBEAT_TIMEOUT) {
        is_connected = false;

        // (ตัวเลือก) หากสายหลุด อาจจะสั่งให้เอาค่า 0x00 ใส่แทน เพื่อรีเซ็ตสถานะ
        // modbus_registers[HEARTBEAT_REG_ADDR] = 0;
    }
}

// ฟังก์ชันสำหรับให้ระบบอื่น (เช่น ระบบขับมอเตอร์) เช็คว่าเน็ตหลุดไหม
bool Heartbeat_IsConnected(void) {
    return is_connected;
}


// ฟังก์ชันคำนวณ CRC-16 แบบมาตรฐาน Modbus
static uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ฟังก์ชันแกะแพ็กเกจและตอบกลับ
void Modbus_Parse_Frame(uint8_t *frame, uint16_t length) {
    // 1. เช็คความยาวขั้นต่ำ และ Slave ID ว่าตรงกับบอร์ดเราไหม
    if (length < 8) return;
    if (frame[0] != SLAVE_ID) return;

    // 2. ตรวจสอบความถูกต้องของข้อมูล (CRC)
    // Modbus ส่ง CRC สลับหน้าหลัง (Low Byte มาก่อน High Byte)
    uint16_t crc_received = (frame[length - 1] << 8) | frame[length - 2];
    uint16_t crc_calculated = Modbus_CRC16(frame, length - 2);
    if (crc_received != crc_calculated) return; // ข้อมูลพังระหว่างทาง ให้ทิ้งไป



    uint8_t function_code = frame[1];


    // ---------------------------------------------------------
    // กรณีที่ 1: PC ขออ่านข้อมูล (Function 0x03)
    // ---------------------------------------------------------
    if (function_code == 0x03) {
        uint16_t start_addr = (frame[2] << 8) | frame[3];
        uint16_t num_regs = (frame[4] << 8) | frame[5];



        // ป้องกันการอ่านเกิน Array ที่เรามี
        if (num_regs > 60) return;
        if (start_addr + num_regs > 64) return;

        uint8_t tx_buffer[135];
        tx_buffer[0] = SLAVE_ID;
        tx_buffer[1] = 0x03;
        tx_buffer[2] = num_regs * 2; // จำนวน Byte ของข้อมูลทั้งหมด (1 Register = 2 Bytes)

        uint8_t idx = 3;
        for(int i = 0; i < num_regs; i++) {
            uint16_t val = modbus_registers[start_addr + i];
            // Modbus ต้องการแบบ Big-Endian (High Byte ก่อน Low Byte)
            tx_buffer[idx++] = (val >> 8) & 0xFF;
            tx_buffer[idx++] = val & 0xFF;
        }

        // แนบ CRC ปิดท้ายก่อนส่ง
        uint16_t crc_tx = Modbus_CRC16(tx_buffer, idx);
        tx_buffer[idx++] = crc_tx & 0xFF;         // Low Byte
        tx_buffer[idx++] = (crc_tx >> 8) & 0xFF;  // High Byte

        // ส่งกลับไปยัง PC (เปลี่ยน huart2 เป็นตัวที่คุณต่อสายอยู่)
        HAL_UART_Transmit(&huart2, tx_buffer, idx, 100);
    }
    // ---------------------------------------------------------
    // ---------------------------------------------------------
    // กรณีที่ 2: PC สั่งเขียน 1 Register (Function 0x06)
    // ---------------------------------------------------------
    else if (function_code == 0x06) {
        uint16_t reg_addr = (frame[2] << 8) | frame[3];
        uint16_t reg_val  = (frame[4] << 8) | frame[5];

        if (reg_addr < 64) {
            modbus_registers[reg_addr] = reg_val;
        }
        // ตอบกลับด้วยข้อความเดิม (Echo)
        HAL_UART_Transmit(&huart2, frame, length, 100);
    }
    // ---------------------------------------------------------
    // กรณีที่ 3: PC สั่งเขียนหลาย Register (Function 0x10 = FC16)
    // ---------------------------------------------------------
    else if (function_code == 0x10) {
        uint16_t start_addr = (frame[2] << 8) | frame[3];
        uint16_t num_regs   = (frame[4] << 8) | frame[5];
        // frame[6] = byte count = num_regs * 2

        if (start_addr + num_regs <= 64) {
            for (uint16_t i = 0; i < num_regs; i++) {
                uint16_t val = (frame[7 + i * 2] << 8) | frame[8 + i * 2];
                modbus_registers[start_addr + i] = val;
            }
        }

        // ตอบกลับ: [ID][0x10][start_hi][start_lo][num_hi][num_lo][CRC_lo][CRC_hi]
        uint8_t resp[8];
        resp[0] = SLAVE_ID;
        resp[1] = 0x10;
        resp[2] = frame[2];
        resp[3] = frame[3];
        resp[4] = frame[4];
        resp[5] = frame[5];
        uint16_t crc = Modbus_CRC16(resp, 6);
        resp[6] = crc & 0xFF;
        resp[7] = (crc >> 8) & 0xFF;
        HAL_UART_Transmit(&huart2, resp, 8, 100);
    }
}
