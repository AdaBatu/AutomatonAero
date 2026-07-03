#ifndef CONFIG_PROTOCOL_H
#define CONFIG_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/* Versioned request/response frame carried as one LoRa packet.
 * Multi-byte values are explicitly little-endian; C structs never go on air.
 * CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF. */
#define CONFIG_MAGIC_0          0xA5U
#define CONFIG_MAGIC_1          0x5AU
#define CONFIG_VERSION          1U
#define CONFIG_TYPE_SET         1U
#define CONFIG_TYPE_ACK         2U
#define CONFIG_WIRE_SIZE        16U

typedef enum {
    CONFIG_PARAM_ROLL_KP = 1,
    CONFIG_PARAM_ROLL_KI,
    CONFIG_PARAM_ROLL_KD,
    CONFIG_PARAM_PITCH_KP,
    CONFIG_PARAM_PITCH_KI,
    CONFIG_PARAM_PITCH_KD,
    CONFIG_PARAM_TELEMETRY_RATE,
    CONFIG_PARAM_ZERO_ATTITUDE
} config_param_t;

typedef enum {
    CONFIG_STATUS_OK = 0,
    CONFIG_STATUS_BAD_PACKET,
    CONFIG_STATUS_UNAUTHORIZED,
    CONFIG_STATUS_ARMED,
    CONFIG_STATUS_RANGE,
    CONFIG_STATUS_UNKNOWN_PARAM
} config_status_t;

typedef struct {
    uint8_t type;
    uint32_t sequence;
    uint8_t parameter;
    uint8_t status;
    int32_t value_milli;
} config_message_t;

static inline uint16_t config_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                    ? (uint16_t)((crc << 1) ^ 0x1021U)
                    : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static inline void config_put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
}

static inline uint32_t config_get_u32_le(const uint8_t *src)
{
    return (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

static inline void config_encode(uint8_t frame[CONFIG_WIRE_SIZE],
                                 const config_message_t *message)
{
    frame[0] = CONFIG_MAGIC_0;
    frame[1] = CONFIG_MAGIC_1;
    frame[2] = CONFIG_VERSION;
    frame[3] = message->type;
    config_put_u32_le(&frame[4], message->sequence);
    frame[8] = message->parameter;
    frame[9] = message->status;
    config_put_u32_le(&frame[10], (uint32_t)message->value_milli);
    uint16_t crc = config_crc16(frame, CONFIG_WIRE_SIZE - 2U);
    frame[14] = (uint8_t)crc;
    frame[15] = (uint8_t)(crc >> 8);
}

static inline int config_decode(config_message_t *message,
                                const uint8_t *frame, size_t length,
                                uint8_t expected_type)
{
    if (length != CONFIG_WIRE_SIZE || frame[0] != CONFIG_MAGIC_0 ||
        frame[1] != CONFIG_MAGIC_1 || frame[2] != CONFIG_VERSION ||
        frame[3] != expected_type) {
        return 0;
    }
    uint16_t received_crc = (uint16_t)frame[14] |
                            ((uint16_t)frame[15] << 8);
    if (config_crc16(frame, CONFIG_WIRE_SIZE - 2U) != received_crc) {
        return 0;
    }
    message->type = frame[3];
    message->sequence = config_get_u32_le(&frame[4]);
    message->parameter = frame[8];
    message->status = frame[9];
    message->value_milli = (int32_t)config_get_u32_le(&frame[10]);
    return 1;
}

#endif
