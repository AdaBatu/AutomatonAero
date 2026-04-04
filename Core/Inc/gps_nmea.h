/**
 * @file gps_nmea.h
 * @brief GPS NMEA parser for NEO-6M
 */
#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include "stm32l4xx_hal.h"
#include "flight_types.h"

/* GPS UART Baud Rate */
#define GPS_BAUD_RATE           9600

/* Buffer sizes */
#define GPS_RX_BUFFER_SIZE      256
#define GPS_SENTENCE_MAX        128

/* NMEA sentence types */
typedef enum {
    NMEA_UNKNOWN = 0,
    NMEA_GGA,       // Fix data
    NMEA_RMC,       // Recommended minimum
    NMEA_GSA,       // DOP and active satellites
    NMEA_VTG        // Ground speed
} NMEA_SentenceType_t;

/* GPS Handle */
typedef struct {
    UART_HandleTypeDef *huart;
    
    // Ring buffer for incoming data
    uint8_t rx_buffer[GPS_RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    
    // Sentence parsing
    char sentence[GPS_SENTENCE_MAX];
    uint8_t sentence_idx;
    bool sentence_started;
    
    // Parsed data
    GPS_Data_t data;
    uint32_t last_valid_fix;
    
    // Single byte for interrupt reception
    uint8_t rx_byte;
    
    // PPS (Pulse Per Second) timing
    GPIO_TypeDef *pps_port;
    uint16_t pps_pin;
    volatile uint32_t pps_timestamp;     // Tick when PPS pulse received
    volatile uint32_t pps_count;         // Total PPS pulses received
    volatile bool pps_valid;             // PPS signal detected
} GPS_Handle_t;

/* Function prototypes */
HAL_StatusTypeDef GPS_Init(GPS_Handle_t *hgps, UART_HandleTypeDef *huart);
HAL_StatusTypeDef GPS_InitWithPPS(GPS_Handle_t *hgps, UART_HandleTypeDef *huart,
                                   GPIO_TypeDef *pps_port, uint16_t pps_pin);
void GPS_ProcessByte(GPS_Handle_t *hgps, uint8_t byte);
void GPS_IRQHandler(GPS_Handle_t *hgps);
void GPS_PPS_IRQHandler(GPS_Handle_t *hgps);  // Call from EXTI callback
bool GPS_GetData(GPS_Handle_t *hgps, GPS_Data_t *data);
bool GPS_HasFix(GPS_Handle_t *hgps);
bool GPS_HasPPS(GPS_Handle_t *hgps);
uint32_t GPS_GetPPSTimestamp(GPS_Handle_t *hgps);

/* Internal parsing functions */
NMEA_SentenceType_t GPS_ParseSentence(GPS_Handle_t *hgps, const char *sentence);
bool GPS_ParseGGA(GPS_Handle_t *hgps, const char *sentence);
bool GPS_ParseRMC(GPS_Handle_t *hgps, const char *sentence);

#endif /* GPS_NMEA_H */
