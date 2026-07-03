#ifndef CONFIG_PROTOCOL_H
#define CONFIG_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#define CONFIG_MAGIC          0xC35AU
#define CONFIG_VERSION        1U
#define CONFIG_TYPE_SET       1U
#define CONFIG_TYPE_ACK       2U
#define CONFIG_AUTH_KEY_0     UINT64_C(0xA4D37C91E2568B0F)
#define CONFIG_AUTH_KEY_1     UINT64_C(0x19F06E2DB87345CA)

typedef enum {
    CONFIG_PARAM_ROLL_KP = 1,
    CONFIG_PARAM_ROLL_KI,
    CONFIG_PARAM_ROLL_KD,
    CONFIG_PARAM_PITCH_KP,
    CONFIG_PARAM_PITCH_KI,
    CONFIG_PARAM_PITCH_KD,
    CONFIG_PARAM_TELEMETRY_RATE
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
    uint64_t auth_tag;
} config_packet_t;

_Static_assert(sizeof(config_packet_t) == 22, "config protocol size changed");

static inline uint64_t config_rotl64(uint64_t value, uint8_t bits)
{
    return (value << bits) | (value >> (64U - bits));
}

static inline void config_sip_round(uint64_t *v0, uint64_t *v1,
                                    uint64_t *v2, uint64_t *v3)
{
    *v0 += *v1; *v1 = config_rotl64(*v1, 13); *v1 ^= *v0;
    *v0 = config_rotl64(*v0, 32);
    *v2 += *v3; *v3 = config_rotl64(*v3, 16); *v3 ^= *v2;
    *v0 += *v3; *v3 = config_rotl64(*v3, 21); *v3 ^= *v0;
    *v2 += *v1; *v1 = config_rotl64(*v1, 17); *v1 ^= *v2;
    *v2 = config_rotl64(*v2, 32);
}

static inline uint64_t config_auth_tag(const config_packet_t *packet)
{
    const uint8_t *data = (const uint8_t *)packet;
    const size_t length = offsetof(config_packet_t, auth_tag);
    uint64_t v0 = UINT64_C(0x736f6d6570736575) ^ CONFIG_AUTH_KEY_0;
    uint64_t v1 = UINT64_C(0x646f72616e646f6d) ^ CONFIG_AUTH_KEY_1;
    uint64_t v2 = UINT64_C(0x6c7967656e657261) ^ CONFIG_AUTH_KEY_0;
    uint64_t v3 = UINT64_C(0x7465646279746573) ^ CONFIG_AUTH_KEY_1;
    size_t offset = 0;
    while (offset + 8U <= length) {
        uint64_t word = 0;
        for (uint8_t i = 0; i < 8U; ++i) word |= (uint64_t)data[offset + i] << (8U * i);
        v3 ^= word;
        config_sip_round(&v0, &v1, &v2, &v3);
        config_sip_round(&v0, &v1, &v2, &v3);
        v0 ^= word;
        offset += 8U;
    }
    uint64_t tail = (uint64_t)length << 56;
    for (uint8_t i = 0; offset + i < length; ++i) tail |= (uint64_t)data[offset + i] << (8U * i);
    v3 ^= tail;
    config_sip_round(&v0, &v1, &v2, &v3);
    config_sip_round(&v0, &v1, &v2, &v3);
    v0 ^= tail;
    v2 ^= 0xffU;
    for (uint8_t i = 0; i < 4U; ++i) config_sip_round(&v0, &v1, &v2, &v3);
    return v0 ^ v1 ^ v2 ^ v3;
}

static inline int config_packet_valid(const config_packet_t *packet, uint8_t type)
{
    return packet->magic == CONFIG_MAGIC &&
           packet->version == CONFIG_VERSION && packet->type == type &&
           packet->auth_tag == config_auth_tag(packet);
}

#endif
