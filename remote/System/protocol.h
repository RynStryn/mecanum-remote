#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * protocol.h
 * Protocol definitions for Remote -> Gateway -> (Chassis / Arm)
 *
 * 统一 10 字节定长协议 (Fixed Length 10 Bytes)
 * Layout:
 * [0] HEAD (0xAA)
 * [1] TYPE (0x01 Chassis, 0x02 Arm)
 * [2..7] DATA[6] (底盘用data[0], 机械臂用data[0]..[5])
 * [8] CHECKSUM (XOR of TYPE and DATA[0..5])
 * [9] TAIL (0x55)
 */

#define MAX_PWM 7199

/* --- Packet Constants --- */
#define PKT_HEADER_BYTE   0xFF
#define PKT_TAIL_BYTE     0xFE
#define PKT_LENGTH        10U  // 统一长度

/* --- Packet Types --- */
#define PKT_TYPE_CHASSIS  0x01
#define PKT_TYPE_ARM      0x02

/* --- Button Bits (Remote Mask) --- */
#define BTN_BIT_W         (1U<<0)
#define BTN_BIT_A         (1U<<1)
#define BTN_BIT_S         (1U<<2)
#define BTN_BIT_D         (1U<<3)
#define BTN_BIT_SHIFT     (1U<<4)
#define BTN_BIT_Q         (1U<<5)  // 逆时针旋转 (CCW)
#define BTN_BIT_E         (1U<<6)  // 顺时针旋转 (CW)
#define BTN_BIT_STOP      (1U<<7)  // 急停 (Emergency Stop)
/* Remote state helper (Optional, for bit access) */
typedef union {
    uint8_t all;
    struct {
        uint8_t W     :1;
        uint8_t A     :1;
        uint8_t S     :1;
        uint8_t D     :1;
        uint8_t Shift :1;
        uint8_t Q:1;
        uint8_t E:1;
        uint8_t STOP:1;
    } bits;
} Remote_State_t;
typedef struct {
    uint8_t head;       // [0]
    uint8_t type;       // [1]
    uint8_t data[6];    // [2]..[7]
    uint8_t checksum;   // [8]
    uint8_t tail;       // [9]
} __attribute__((packed)) Packet_t;

static inline uint8_t calc_sum(Packet_t *pkt) {
    uint8_t sum = pkt->type;
    for(int i=0; i<6; i++) {
        sum ^= pkt->data[i];
    }
    return sum;
}
static inline void pack_chassis(Packet_t *pkt, uint8_t mask) {
    // 1. 清空数据区
    memset(pkt->data, 0, 6);

    // 2. 填充字段
    pkt->head = PKT_HEADER_BYTE;
    pkt->type = PKT_TYPE_CHASSIS;
    pkt->data[0] = mask; // 底盘数据只占第1个字节

    // 3. 计算校验和
    pkt->checksum = calc_sum(pkt);
    pkt->tail = PKT_TAIL_BYTE;
}
static inline void pack_arm(Packet_t *pkt, uint8_t *servos) {
    pkt->head = PKT_HEADER_BYTE;
    pkt->type = PKT_TYPE_ARM;

    // 复制 6 个舵机数据
    memcpy(pkt->data, servos, 6);

    pkt->checksum = calc_sum(pkt);
    pkt->tail = PKT_TAIL_BYTE;
}

/**
 * @brief 校验数据包 (必须传入 10 字节的 buffer)
 */
static inline bool protocol_validate_packet(const uint8_t *buf) {
    // 强转为结构体指针方便访问
    const Packet_t *pkt = (const Packet_t*)buf;

    // 1. 检查头尾
    if (pkt->head != PKT_HEADER_BYTE) return false;
    if (pkt->tail != PKT_TAIL_BYTE)   return false;

    // 2. 手动计算校验和 (不能直接调 calc_sum 因为入参是 const)
    uint8_t expected_sum = pkt->type;
    for(int i=0; i<6; i++) {
        expected_sum ^= pkt->data[i];
    }

    // 3. 比对
    if (expected_sum != pkt->checksum) return false;

    return true;
}

#endif /* PROTOCOL_H */