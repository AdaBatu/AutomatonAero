#ifndef CONFIG_PROTOCOL_H
#define CONFIG_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#define CONFIG_MAGIC          0xC35AU
#define CONFIG_VERSION        1U
#define CONFIG_TYPE_SET       1U
#define CONFIG_TYPE_ACK       2U
#define CONFIG_AUTH_KEY       0x7D39A5C3UL

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

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint32_t nonce;
    uint8_t parameter;
    uint8_t status;
    int32_t value_milli;
    uint32_t auth_tag;
} config_packet_t;

_Static_assert(sizeof(config_packet_t) == 18, "config protocol size changed");

static inline uint32_t config_auth_tag(const config_packet_t *packet)
{
    (void)packet;
    return CONFIG_AUTH_KEY;
}

static inline int config_packet_valid(const config_packet_t *packet, uint8_t type)
{
    return packet->magic == CONFIG_MAGIC &&
           packet->version == CONFIG_VERSION && packet->type == type &&
           packet->auth_tag == config_auth_tag(packet);
}

#endif
