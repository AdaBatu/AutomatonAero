/**
 * @file telemetry.c
 * @brief Telemetry data packaging and LoRa transmission
 */
#include "telemetry.h"
#include "config_protocol.h"
#include "pid.h"
#include "flight_config_generated.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Radians to degrees conversion */
#define RAD_TO_DEG(x) ((x) * 180.0f / M_PI)

extern FlightPID_t flight_pid;
extern void Flight_RequestAttitudeZero(void);

/* Initialize telemetry manager */
void Telemetry_Init(Telemetry_Handle_t *htelem, SX1278_Handle_t *radio)
{
    htelem->radio = radio;
    htelem->last_tx_time = 0;
    htelem->tx_start_time = 0;
    htelem->tx_interval_ms = 1000 / TELEMETRY_RATE_HZ;  // 100ms for 10Hz
    htelem->packet_count = 0;
    htelem->recovery_count = 0;
    htelem->last_config_window = HAL_GetTick();
    htelem->config_window_until = 0;
    htelem->last_config_nonce = 0;
    htelem->last_config_parameter = 0;
    htelem->last_config_value_milli = 0;
    htelem->last_config_status = 0xFFU;
    htelem->config_rx_count = 0U;
    htelem->config_invalid_count = 0U;
    htelem->config_debug_byte0 = 0U;
    htelem->config_debug_byte1 = 0U;
    htelem->tx_in_progress = false;
    htelem->rx_active = false;
    
    // Initialize packet header
    htelem->packet.header[0] = TELEMETRY_HEADER_1;
    htelem->packet.header[1] = TELEMETRY_HEADER_2;
}

/* Set telemetry transmission rate */
void Telemetry_SetRate(Telemetry_Handle_t *htelem, uint8_t rate_hz)
{
    if (rate_hz == 0) rate_hz = 1;
    if (rate_hz > 50) rate_hz = 50;  // Max 50Hz
    
    htelem->tx_interval_ms = 1000 / rate_hz;
}

/* Calculate packet checksum (simple XOR) */
uint8_t Telemetry_CalculateChecksum(const Telemetry_Packet_t *packet)
{
    const uint8_t *data = (const uint8_t *)packet;
    uint8_t checksum = 0;
    
    // XOR all bytes except the checksum field itself
    for (size_t i = 0; i < sizeof(Telemetry_Packet_t) - 1; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

/* Build telemetry packet from flight state */
void Telemetry_BuildPacket(Telemetry_Handle_t *htelem, const FlightState_t *state,
                           uint8_t servo_roll, uint8_t servo_pitch, 
                           uint8_t servo_yaw, uint8_t esc_throttle)
{
    Telemetry_Packet_t *pkt = &htelem->packet;
    
    // Header
    pkt->header[0] = TELEMETRY_HEADER_1;
    pkt->header[1] = TELEMETRY_HEADER_2;
    
    // Timestamp
    pkt->timestamp = HAL_GetTick();
    
    // Orientation (convert radians to degrees * 100)
    pkt->roll = (int16_t)(RAD_TO_DEG(state->orientation.roll) * 100.0f);
    pkt->pitch = (int16_t)(RAD_TO_DEG(state->orientation.pitch) * 100.0f);
    pkt->yaw = (int16_t)(RAD_TO_DEG(state->orientation.yaw) * 100.0f);
    
    // GPS data
    if (state->gps.fix_valid) {
        pkt->latitude = (int32_t)(state->gps.latitude * 1e7f);
        pkt->longitude = (int32_t)(state->gps.longitude * 1e7f);
        pkt->gps_altitude = (int16_t)(state->gps.altitude);
        pkt->satellites = state->gps.satellites;
        float speed_scaled = state->gps.speed * 100.0f;
        if (speed_scaled < 0.0f) speed_scaled = 0.0f;
        if (speed_scaled > 65535.0f) speed_scaled = 65535.0f;
        pkt->ground_speed = (uint16_t)speed_scaled;
    } else {
        pkt->latitude = 0;
        pkt->longitude = 0;
        pkt->gps_altitude = 0;
        pkt->satellites = 0;
        pkt->ground_speed = 0;
    }

    if (state->imu.valid) {
        float ax = state->imu.accel.x * 100.0f;
        float ay = state->imu.accel.y * 100.0f;
        float az = state->imu.accel.z * 100.0f;
        if (ax > 32767.0f) ax = 32767.0f; else if (ax < -32768.0f) ax = -32768.0f;
        if (ay > 32767.0f) ay = 32767.0f; else if (ay < -32768.0f) ay = -32768.0f;
        if (az > 32767.0f) az = 32767.0f; else if (az < -32768.0f) az = -32768.0f;
        pkt->accel_x = (int16_t)ax;
        pkt->accel_y = (int16_t)ay;
        pkt->accel_z = (int16_t)az;
    } else {
        pkt->accel_x = 0;
        pkt->accel_y = 0;
        pkt->accel_z = 0;
    }

    if (state->navigation.valid) {
        pkt->fused_latitude = (int32_t)(state->navigation.latitude * 1e7);
        pkt->fused_longitude = (int32_t)(state->navigation.longitude * 1e7);
        float vn = state->navigation.velocity_north * 100.0f;
        float ve = state->navigation.velocity_east * 100.0f;
        float fs = state->navigation.speed * 100.0f;
        if (vn > 32767.0f) vn = 32767.0f; else if (vn < -32768.0f) vn = -32768.0f;
        if (ve > 32767.0f) ve = 32767.0f; else if (ve < -32768.0f) ve = -32768.0f;
        if (fs > 65535.0f) fs = 65535.0f;
        pkt->velocity_north = (int16_t)vn;
        pkt->velocity_east = (int16_t)ve;
        pkt->fused_speed = (uint16_t)fs;
        pkt->navigation_valid = 1U;
    } else {
        pkt->fused_latitude = 0;
        pkt->fused_longitude = 0;
        pkt->velocity_north = 0;
        pkt->velocity_east = 0;
        pkt->fused_speed = 0;
        pkt->navigation_valid = 0U;
    }
    
    // Barometer altitude
    if (state->baro.valid) {
        pkt->baro_altitude = (int16_t)(state->baro.altitude);
    } else {
        pkt->baro_altitude = 0;
    }
    
    // Power
    if (state->power.valid) {
        pkt->voltage = (uint16_t)(state->power.voltage * 1000.0f);  // mV
        pkt->current = (uint16_t)(state->power.current * 1000.0f);  // mA
        pkt->power = (uint16_t)(state->power.power);
        pkt->consumed_mah = (uint32_t)(state->power.ampere_hours * 1000.0f);
    } else {
        pkt->voltage = 0;
        pkt->current = 0;
        pkt->power = 0;
        pkt->consumed_mah = 0;
    }
    
    // Control outputs
    pkt->servo_roll = servo_roll;
    pkt->servo_pitch = servo_pitch;
    /* This airframe has neither yaw servo nor throttle output. Reuse those
       bytes for uplink diagnostics without changing the 40-byte packet. */
    (void)servo_yaw;
    (void)esc_throttle;
    if (htelem->last_config_status == 0x40U) {
        /* On bad magic expose the actual first bytes for remote diagnosis. */
        pkt->servo_yaw = htelem->config_debug_byte0;
        pkt->esc_throttle = htelem->config_debug_byte1;
    } else {
        pkt->servo_yaw = htelem->config_rx_count;
        pkt->esc_throttle = htelem->last_config_status;
    }
    
    // Calculate checksum
    pkt->checksum = Telemetry_CalculateChecksum(pkt);
}

/* Check if ready to send next packet */
bool Telemetry_ReadyToSend(Telemetry_Handle_t *htelem)
{
    // Check if previous transmission is complete
    if (htelem->tx_in_progress) {
        if (SX1278_IsTxDone(htelem->radio)) {
            htelem->tx_in_progress = false;
        } else if ((HAL_GetTick() - htelem->tx_start_time) >= TELEMETRY_TX_TIMEOUT_MS) {
            /* A lost DIO0/TxDone must not permanently stop telemetry. */
            HAL_SPI_Abort(htelem->radio->hspi);
            SX1278_SetMode(htelem->radio, SX1278_MODE_STDBY);
            SX1278_WriteRegister(htelem->radio, SX1278_REG_IRQ_FLAGS, 0xFF);
            htelem->tx_in_progress = false;
            htelem->recovery_count++;
        } else {
            return false;
        }
    }

    /* LoRa is half-duplex. Reserve a real receive window; the short gap
       between 10 Hz telemetry packets is not long enough for a command and
       its acknowledgement. */
    uint32_t now = HAL_GetTick();
    if (htelem->config_window_until != 0U &&
        (int32_t)(now - htelem->config_window_until) < 0) {
        return false;
    }
    
    // Check if enough time has passed
    if (now - htelem->last_tx_time >= htelem->tx_interval_ms) {
        return true;
    }
    
    return false;
}

/* Send telemetry packet (non-blocking) */
HAL_StatusTypeDef Telemetry_Send(Telemetry_Handle_t *htelem)
{
    if (htelem->tx_in_progress) {
        return HAL_BUSY;
    }
    
    // Start async transmission
    HAL_StatusTypeDef status = SX1278_TransmitAsync(htelem->radio, 
                                                     (uint8_t *)&htelem->packet, 
                                                     sizeof(Telemetry_Packet_t));
    
    if (status == HAL_OK) {
        htelem->tx_in_progress = true;
        htelem->rx_active = false;
        htelem->last_tx_time = HAL_GetTick();
        htelem->tx_start_time = htelem->last_tx_time;
        htelem->packet_count++;
    }
    
    return status;
}

static uint8_t Telemetry_ApplyConfig(Telemetry_Handle_t *htelem,
                                     uint8_t parameter, int32_t value_milli)
{
    float value = (float)value_milli / 1000.0f;

    if (parameter == CONFIG_PARAM_TELEMETRY_RATE) {
        if (value_milli < 1000 || value_milli > 20000 ||
            (value_milli % 1000) != 0) {
            return CONFIG_STATUS_RANGE;
        }
        Telemetry_SetRate(htelem, (uint8_t)(value_milli / 1000));
        return CONFIG_STATUS_OK;
    }

    if (parameter == CONFIG_PARAM_ZERO_ATTITUDE) {
        Flight_RequestAttitudeZero();
        return CONFIG_STATUS_OK;
    }

    switch (parameter) {
    case CONFIG_PARAM_ROLL_KP:
    case CONFIG_PARAM_PITCH_KP:
        if (value < 0.0f || value > 10.0f) return CONFIG_STATUS_RANGE;
        break;
    case CONFIG_PARAM_ROLL_KI:
    case CONFIG_PARAM_PITCH_KI:
    case CONFIG_PARAM_ROLL_KD:
    case CONFIG_PARAM_PITCH_KD:
        if (value < 0.0f || value > 2.0f) return CONFIG_STATUS_RANGE;
        break;
    default:
        return CONFIG_STATUS_UNKNOWN_PARAM;
    }

    switch (parameter) {
    case CONFIG_PARAM_ROLL_KP:  flight_pid.roll.Kp = value; break;
    case CONFIG_PARAM_ROLL_KI:  flight_pid.roll.Ki = value; break;
    case CONFIG_PARAM_ROLL_KD:  flight_pid.roll.Kd = value; break;
    case CONFIG_PARAM_PITCH_KP: flight_pid.pitch.Kp = value; break;
    case CONFIG_PARAM_PITCH_KI: flight_pid.pitch.Ki = value; break;
    case CONFIG_PARAM_PITCH_KD: flight_pid.pitch.Kd = value; break;
    default: return CONFIG_STATUS_UNKNOWN_PARAM;
    }
    return CONFIG_STATUS_OK;
}

void Telemetry_PollConfig(Telemetry_Handle_t *htelem)
{
    if (htelem->tx_in_progress) return;

    uint32_t now = HAL_GetTick();
    if ((now - htelem->last_config_window) >= CONFIG_LORA_RX_WINDOW_INTERVAL_MS) {
        htelem->last_config_window = now;
        htelem->config_window_until = now + CONFIG_LORA_RX_WINDOW_DURATION_MS;
        /* RX_SINGLE automatically returns to standby after a packet or
           timeout. Re-arm it explicitly at the start of every command slot. */
        htelem->rx_active = false;
    }

    if (!htelem->rx_active) {
        if (SX1278_StartReceive(htelem->radio) != HAL_OK) return;
        htelem->rx_active = true;
    }

    uint8_t frame[64];
    config_message_t command;
    int16_t len = SX1278_ReadPacket(htelem->radio, frame, sizeof(frame));
    if (len <= 0) return;
    htelem->config_rx_count++;
    htelem->config_debug_byte0 = frame[0];
    htelem->config_debug_byte1 = len > 1 ? frame[1] : 0U;
    if (!config_decode(&command, frame, (size_t)len, CONFIG_TYPE_SET)) {
        htelem->config_invalid_count++;
        if (len != (int16_t)CONFIG_WIRE_SIZE) {
            htelem->last_config_status = (uint8_t)(0x80U | ((uint8_t)len & 0x3FU));
        } else if (frame[0] != CONFIG_MAGIC_0 || frame[1] != CONFIG_MAGIC_1) {
            htelem->last_config_status = 0x40U;
        } else if (frame[2] != CONFIG_VERSION) {
            htelem->last_config_status = 0x41U;
        } else if (frame[3] != CONFIG_TYPE_SET) {
            htelem->last_config_status = 0x42U;
        } else {
            htelem->last_config_status = 0x43U; /* application CRC */
        }
        printf("LoRa config rejected: len=%d reason=%u bytes=",
               (int)len, htelem->last_config_status);
        for (int16_t i = 0; i < len && i < (int16_t)CONFIG_WIRE_SIZE; ++i) {
            printf("%02X", frame[i]);
        }
        printf("\r\n");
        return;
    }

    printf("LoRa config received: parameter=%u sequence=%08lX\r\n",
           command.parameter, (unsigned long)command.sequence);

    uint8_t status;
    if (command.sequence == htelem->last_config_nonce &&
        command.parameter == htelem->last_config_parameter) {
        status = htelem->last_config_status;
    } else {
        status = Telemetry_ApplyConfig(htelem, command.parameter,
                                       command.value_milli);
        htelem->last_config_nonce = command.sequence;
        htelem->last_config_parameter = command.parameter;
        htelem->last_config_value_milli = command.value_milli;
        htelem->last_config_status = status;
    }

    config_message_t ack = {
        .type = CONFIG_TYPE_ACK,
        .sequence = command.sequence,
        .parameter = command.parameter,
        .status = status,
        .value_milli = command.value_milli
    };
    uint8_t ack_frame[CONFIG_WIRE_SIZE];
    config_encode(ack_frame, &ack);
    htelem->rx_active = false;
    /* Give the requester time to leave TX and enter RX, then repeat the same
       idempotent ACK. Repeated ACKs are safe because sequence identifies the
       transaction and greatly improve half-duplex LoRa turnaround reliability. */
    HAL_Delay(30U);
    for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
        (void)SX1278_Transmit(htelem->radio, ack_frame, sizeof(ack_frame));
        if (attempt < 2U) HAL_Delay(20U);
    }
    printf("LoRa config ACK burst sent: status=%u sequence=%08lX\r\n",
           status, (unsigned long)command.sequence);
    (void)SX1278_StartReceive(htelem->radio);
    htelem->rx_active = true;
}

/* Update transmission state (call in main loop) */
void Telemetry_Update(Telemetry_Handle_t *htelem)
{
    if (htelem->tx_in_progress) {
        if (SX1278_IsTxDone(htelem->radio)) {
            htelem->tx_in_progress = false;
        } else if ((HAL_GetTick() - htelem->tx_start_time) >= TELEMETRY_TX_TIMEOUT_MS) {
            HAL_SPI_Abort(htelem->radio->hspi);
            SX1278_SetMode(htelem->radio, SX1278_MODE_STDBY);
            SX1278_WriteRegister(htelem->radio, SX1278_REG_IRQ_FLAGS, 0xFF);
            htelem->tx_in_progress = false;
            htelem->recovery_count++;
        }
    }
}
