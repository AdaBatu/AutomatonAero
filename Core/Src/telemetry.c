/**
 * @file telemetry.c
 * @brief Telemetry data packaging and LoRa transmission
 */
#include "telemetry.h"
#include "config_protocol.h"
#include "pid.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Radians to degrees conversion */
#define RAD_TO_DEG(x) ((x) * 180.0f / M_PI)

extern FlightPID_t flight_pid;

/* Initialize telemetry manager */
void Telemetry_Init(Telemetry_Handle_t *htelem, SX1278_Handle_t *radio)
{
    htelem->radio = radio;
    htelem->last_tx_time = 0;
    htelem->tx_start_time = 0;
    htelem->tx_interval_ms = 1000 / TELEMETRY_RATE_HZ;  // 100ms for 10Hz
    htelem->packet_count = 0;
    htelem->recovery_count = 0;
    htelem->last_config_nonce = 0;
    htelem->last_config_parameter = 0;
    htelem->last_config_value_milli = 0;
    htelem->last_config_status = CONFIG_STATUS_BAD_PACKET;
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
    } else {
        pkt->latitude = 0;
        pkt->longitude = 0;
        pkt->gps_altitude = 0;
        pkt->satellites = 0;
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
    pkt->servo_yaw = servo_yaw;
    pkt->esc_throttle = esc_throttle;
    
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
    
    // Check if enough time has passed
    uint32_t now = HAL_GetTick();
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

    if (!htelem->rx_active) {
        if (SX1278_StartReceive(htelem->radio) != HAL_OK) return;
        htelem->rx_active = true;
    }

    config_packet_t command;
    int16_t len = SX1278_ReadPacket(htelem->radio, (uint8_t *)&command,
                                    sizeof(command));
    if (len <= 0) return;
    if (len != (int16_t)sizeof(command) ||
        !config_packet_valid(&command, CONFIG_TYPE_SET)) {
        return;
    }

    uint8_t status;
    if (command.nonce == htelem->last_config_nonce &&
        command.parameter == htelem->last_config_parameter) {
        status = htelem->last_config_status;
    } else {
        status = Telemetry_ApplyConfig(htelem, command.parameter,
                                       command.value_milli);
        htelem->last_config_nonce = command.nonce;
        htelem->last_config_parameter = command.parameter;
        htelem->last_config_value_milli = command.value_milli;
        htelem->last_config_status = status;
    }

    config_packet_t ack = {
        .magic = CONFIG_MAGIC,
        .version = CONFIG_VERSION,
        .type = CONFIG_TYPE_ACK,
        .nonce = command.nonce,
        .parameter = command.parameter,
        .status = status,
        .value_milli = command.value_milli,
        .auth_tag = 0
    };
    ack.auth_tag = config_auth_tag(&ack);
    htelem->rx_active = false;
    (void)SX1278_Transmit(htelem->radio, (uint8_t *)&ack, sizeof(ack));
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
