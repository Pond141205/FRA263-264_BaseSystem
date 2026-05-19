/*
 * base_system.h
 *
 *  Created on: 15 พ.ค. 2569
 *      Author: POND
 */

#ifndef BASE_SYSTEM_H_
#define BASE_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>

// กำหนดค่าคงที่ตาม Document
#define SLAVE_ID 21
#define HEARTBEAT_REG_ADDR  0       // Address 0x00
#define ROBOT_SAYS_YA       22881   // บอร์ดส่งค่านี้
#define PC_SAYS_HI          18537   // PC ตอบกลับด้วยค่านี้
#define HEARTBEAT_TIMEOUT   2000    // Timeout 2 วินาที (2000 ms)

// ดึง Array ของ Modbus ที่น่าจะประกาศไว้ใน main.c หรือไฟล์ modbus หลักมาใช้
extern uint16_t modbus_registers[64];

// ประกาศฟังก์ชัน
void Modbus_Parse_Frame(uint8_t *frame, uint16_t length);
void Heartbeat_Init(void);
void Heartbeat_Update(void);
bool Heartbeat_IsConnected(void);

#endif /* BASE_SYSTEM_H_ */
