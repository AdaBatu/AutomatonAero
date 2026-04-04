/**
 * @file gps_nmea.c
 * @brief GPS NMEA parser implementation for NEO-6M
 */
#include "gps_nmea.h"
#include <string.h>
#include <stdlib.h>

/* Internal helper functions */
static float GPS_ParseCoordinate(const char *coord, const char *dir);
static float GPS_ParseFloat(const char *str);
static int GPS_ParseInt(const char *str);
static bool GPS_ValidateChecksum(const char *sentence);
static char *GPS_GetField(char *sentence, uint8_t field_num);

/* Initialize GPS module */
HAL_StatusTypeDef GPS_Init(GPS_Handle_t *hgps, UART_HandleTypeDef *huart)
{
    hgps->huart = huart;
    hgps->rx_head = 0;
    hgps->rx_tail = 0;
    hgps->sentence_idx = 0;
    hgps->sentence_started = false;
    hgps->last_valid_fix = 0;
    
    // Initialize GPS data
    memset(&hgps->data, 0, sizeof(GPS_Data_t));
    hgps->data.fix_valid = false;
    
    // PPS not configured
    hgps->pps_port = NULL;
    hgps->pps_pin = 0;
    hgps->pps_timestamp = 0;
    hgps->pps_count = 0;
    hgps->pps_valid = false;
    
    // Start receiving in interrupt mode
    return HAL_UART_Receive_IT(hgps->huart, &hgps->rx_byte, 1);
}

/* Initialize GPS module with PPS support */
HAL_StatusTypeDef GPS_InitWithPPS(GPS_Handle_t *hgps, UART_HandleTypeDef *huart,
                                   GPIO_TypeDef *pps_port, uint16_t pps_pin)
{
    HAL_StatusTypeDef status = GPS_Init(hgps, huart);
    
    hgps->pps_port = pps_port;
    hgps->pps_pin = pps_pin;
    
    return status;
}

/* Process a single received byte */
void GPS_ProcessByte(GPS_Handle_t *hgps, uint8_t byte)
{
    // Store in ring buffer
    uint16_t next_head = (hgps->rx_head + 1) % GPS_RX_BUFFER_SIZE;
    if (next_head != hgps->rx_tail) {
        hgps->rx_buffer[hgps->rx_head] = byte;
        hgps->rx_head = next_head;
    }
    
    // NMEA sentence parsing
    if (byte == '$') {
        hgps->sentence_started = true;
        hgps->sentence_idx = 0;
    }
    else if (hgps->sentence_started) {
        if (byte == '\r' || byte == '\n') {
            if (hgps->sentence_idx > 0) {
                hgps->sentence[hgps->sentence_idx] = '\0';
                GPS_ParseSentence(hgps, hgps->sentence);
            }
            hgps->sentence_started = false;
            hgps->sentence_idx = 0;
        }
        else if (hgps->sentence_idx < GPS_SENTENCE_MAX - 1) {
            hgps->sentence[hgps->sentence_idx++] = byte;
        }
    }
}

/* UART receive interrupt handler - call from HAL_UART_RxCpltCallback */
void GPS_IRQHandler(GPS_Handle_t *hgps)
{
    GPS_ProcessByte(hgps, hgps->rx_byte);
    HAL_UART_Receive_IT(hgps->huart, &hgps->rx_byte, 1);
}

/* Get latest GPS data */
bool GPS_GetData(GPS_Handle_t *hgps, GPS_Data_t *data)
{
    memcpy(data, &hgps->data, sizeof(GPS_Data_t));
    return hgps->data.fix_valid;
}

/* Check if GPS has valid fix */
bool GPS_HasFix(GPS_Handle_t *hgps)
{
    return hgps->data.fix_valid;
}

/* Check if PPS signal is valid (received within last 2 seconds) */
bool GPS_HasPPS(GPS_Handle_t *hgps)
{
    if (!hgps->pps_valid) return false;
    
    uint32_t now = HAL_GetTick();
    return (now - hgps->pps_timestamp) < 2000;
}

/* Get last PPS timestamp */
uint32_t GPS_GetPPSTimestamp(GPS_Handle_t *hgps)
{
    return hgps->pps_timestamp;
}

/* PPS interrupt handler - call from EXTI callback for PA4 */
void GPS_PPS_IRQHandler(GPS_Handle_t *hgps)
{
    hgps->pps_timestamp = HAL_GetTick();
    hgps->pps_count++;
    hgps->pps_valid = true;
}

/* Parse NMEA sentence */
NMEA_SentenceType_t GPS_ParseSentence(GPS_Handle_t *hgps, const char *sentence)
{
    // Validate checksum
    if (!GPS_ValidateChecksum(sentence)) {
        return NMEA_UNKNOWN;
    }
    
    // Make a mutable copy for parsing
    char buffer[GPS_SENTENCE_MAX];
    strncpy(buffer, sentence, GPS_SENTENCE_MAX - 1);
    buffer[GPS_SENTENCE_MAX - 1] = '\0';
    
    // Identify sentence type (skip $GP or $GN prefix)
    if (strncmp(buffer, "GPGGA", 5) == 0 || strncmp(buffer, "GNGGA", 5) == 0) {
        GPS_ParseGGA(hgps, buffer);
        return NMEA_GGA;
    }
    else if (strncmp(buffer, "GPRMC", 5) == 0 || strncmp(buffer, "GNRMC", 5) == 0) {
        GPS_ParseRMC(hgps, buffer);
        return NMEA_RMC;
    }
    
    return NMEA_UNKNOWN;
}

/* Parse GGA sentence (Global Positioning System Fix Data) */
bool GPS_ParseGGA(GPS_Handle_t *hgps, const char *sentence)
{
    char buffer[GPS_SENTENCE_MAX];
    strncpy(buffer, sentence, GPS_SENTENCE_MAX - 1);
    
    char *field;
    uint8_t field_num = 0;
    char *save_ptr;
    
    // Parse fields
    field = strtok_r(buffer, ",", &save_ptr);
    while (field != NULL && field_num < 15) {
        switch (field_num) {
            case 2: // Latitude
                {
                    char *dir = strtok_r(NULL, ",", &save_ptr);
                    field_num++;
                    if (dir && strlen(field) > 0) {
                        hgps->data.latitude = GPS_ParseCoordinate(field, dir);
                    }
                }
                break;
                
            case 4: // Longitude
                {
                    char *dir = strtok_r(NULL, ",", &save_ptr);
                    field_num++;
                    if (dir && strlen(field) > 0) {
                        hgps->data.longitude = GPS_ParseCoordinate(field, dir);
                    }
                }
                break;
                
            case 6: // Fix quality (0=invalid, 1=GPS fix, 2=DGPS fix)
                {
                    int fix = GPS_ParseInt(field);
                    hgps->data.fix_valid = (fix >= 1);
                    if (hgps->data.fix_valid) {
                        hgps->last_valid_fix = HAL_GetTick();
                    }
                }
                break;
                
            case 7: // Number of satellites
                hgps->data.satellites = GPS_ParseInt(field);
                break;
                
            case 9: // Altitude (meters above sea level)
                hgps->data.altitude = GPS_ParseFloat(field);
                break;
        }
        
        field = strtok_r(NULL, ",", &save_ptr);
        field_num++;
    }
    
    return hgps->data.fix_valid;
}

/* Parse RMC sentence (Recommended Minimum Navigation Information) */
bool GPS_ParseRMC(GPS_Handle_t *hgps, const char *sentence)
{
    char buffer[GPS_SENTENCE_MAX];
    strncpy(buffer, sentence, GPS_SENTENCE_MAX - 1);
    
    char *field;
    uint8_t field_num = 0;
    char *save_ptr;
    
    field = strtok_r(buffer, ",", &save_ptr);
    while (field != NULL && field_num < 13) {
        switch (field_num) {
            case 2: // Status (A=active/valid, V=void)
                hgps->data.fix_valid = (field[0] == 'A');
                if (hgps->data.fix_valid) {
                    hgps->last_valid_fix = HAL_GetTick();
                }
                break;
                
            case 3: // Latitude
                {
                    char *dir = strtok_r(NULL, ",", &save_ptr);
                    field_num++;
                    if (dir && strlen(field) > 0) {
                        hgps->data.latitude = GPS_ParseCoordinate(field, dir);
                    }
                }
                break;
                
            case 5: // Longitude
                {
                    char *dir = strtok_r(NULL, ",", &save_ptr);
                    field_num++;
                    if (dir && strlen(field) > 0) {
                        hgps->data.longitude = GPS_ParseCoordinate(field, dir);
                    }
                }
                break;
                
            case 7: // Speed (knots)
                hgps->data.speed = GPS_ParseFloat(field) * 0.514444f; // Convert to m/s
                break;
                
            case 8: // Course (degrees)
                hgps->data.course = GPS_ParseFloat(field);
                break;
        }
        
        field = strtok_r(NULL, ",", &save_ptr);
        field_num++;
    }
    
    return hgps->data.fix_valid;
}

/* Parse coordinate from NMEA format to decimal degrees */
static float GPS_ParseCoordinate(const char *coord, const char *dir)
{
    if (coord == NULL || strlen(coord) < 4) return 0.0f;
    
    // Format: DDDMM.MMMM or DDMM.MMMM
    float raw = atof(coord);
    int degrees = (int)(raw / 100);
    float minutes = raw - (degrees * 100);
    float decimal = degrees + (minutes / 60.0f);
    
    // Apply direction
    if (dir[0] == 'S' || dir[0] == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

/* Parse float from string */
static float GPS_ParseFloat(const char *str)
{
    if (str == NULL || strlen(str) == 0) return 0.0f;
    return atof(str);
}

/* Parse integer from string */
static int GPS_ParseInt(const char *str)
{
    if (str == NULL || strlen(str) == 0) return 0;
    return atoi(str);
}

/* Validate NMEA checksum */
static bool GPS_ValidateChecksum(const char *sentence)
{
    if (sentence == NULL) return false;
    
    // Find asterisk
    const char *asterisk = strchr(sentence, '*');
    if (asterisk == NULL) return false;
    
    // Calculate XOR checksum
    uint8_t checksum = 0;
    for (const char *p = sentence; p < asterisk; p++) {
        checksum ^= *p;
    }
    
    // Parse provided checksum
    uint8_t provided = (uint8_t)strtol(asterisk + 1, NULL, 16);
    
    return checksum == provided;
}
