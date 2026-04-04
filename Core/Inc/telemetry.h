/**
 * @file telemetry.h
 * @brief Telemetry data packaging and transmission over LoRa
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "flight_types.h"
#include "sx1278.h"

/* Telemetry packet markers */
#define TELEMETRY_HEADER_1      0xAA
#define TELEMETRY_HEADER_2      0x55

/* Telemetry transmission rate */
#define TELEMETRY_RATE_HZ       10      // 10 packets per second

/* Telemetry manager */
typedef struct {
    SX1278_Handle_t *radio;
    
    Telemetry_Packet_t packet;
    uint32_t last_tx_time;
    uint32_t tx_interval_ms;
    uint32_t packet_count;
    bool tx_in_progress;
} Telemetry_Handle_t;

/* Function prototypes */
void Telemetry_Init(Telemetry_Handle_t *htelem, SX1278_Handle_t *radio);
void Telemetry_SetRate(Telemetry_Handle_t *htelem, uint8_t rate_hz);

/* Build packet from flight state */
void Telemetry_BuildPacket(Telemetry_Handle_t *htelem, const FlightState_t *state,
                           uint8_t servo_roll, uint8_t servo_pitch, 
                           uint8_t servo_yaw, uint8_t esc_throttle);

/* Send telemetry (non-blocking) */
HAL_StatusTypeDef Telemetry_Send(Telemetry_Handle_t *htelem);

/* Check if ready to send next packet */
bool Telemetry_ReadyToSend(Telemetry_Handle_t *htelem);

/* Update transmission state (call in main loop) */
void Telemetry_Update(Telemetry_Handle_t *htelem);

/* Calculate checksum */
uint8_t Telemetry_CalculateChecksum(const Telemetry_Packet_t *packet);

#endif /* TELEMETRY_H */
