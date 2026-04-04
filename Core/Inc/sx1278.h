/**
 * @file sx1278.h
 * @brief SX1278 LoRa 433MHz Radio Driver
 */
#ifndef SX1278_H
#define SX1278_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>

/* SX1278 Register addresses */
#define SX1278_REG_FIFO                 0x00
#define SX1278_REG_OP_MODE              0x01
#define SX1278_REG_FRF_MSB              0x06
#define SX1278_REG_FRF_MID              0x07
#define SX1278_REG_FRF_LSB              0x08
#define SX1278_REG_PA_CONFIG            0x09
#define SX1278_REG_PA_RAMP              0x0A
#define SX1278_REG_OCP                  0x0B
#define SX1278_REG_LNA                  0x0C
#define SX1278_REG_FIFO_ADDR_PTR        0x0D
#define SX1278_REG_FIFO_TX_BASE_ADDR    0x0E
#define SX1278_REG_FIFO_RX_BASE_ADDR    0x0F
#define SX1278_REG_FIFO_RX_CURRENT_ADDR 0x10
#define SX1278_REG_IRQ_FLAGS_MASK       0x11
#define SX1278_REG_IRQ_FLAGS            0x12
#define SX1278_REG_RX_NB_BYTES          0x13
#define SX1278_REG_MODEM_CONFIG_1       0x1D
#define SX1278_REG_MODEM_CONFIG_2       0x1E
#define SX1278_REG_SYMB_TIMEOUT_LSB     0x1F
#define SX1278_REG_PREAMBLE_MSB         0x20
#define SX1278_REG_PREAMBLE_LSB         0x21
#define SX1278_REG_PAYLOAD_LENGTH       0x22
#define SX1278_REG_MAX_PAYLOAD_LENGTH   0x23
#define SX1278_REG_MODEM_CONFIG_3       0x26
#define SX1278_REG_FREQ_ERROR_MSB       0x28
#define SX1278_REG_FREQ_ERROR_MID       0x29
#define SX1278_REG_FREQ_ERROR_LSB       0x2A
#define SX1278_REG_RSSI_WIDEBAND        0x2C
#define SX1278_REG_DETECTION_OPTIMIZE   0x31
#define SX1278_REG_DETECTION_THRESHOLD  0x37
#define SX1278_REG_SYNC_WORD            0x39
#define SX1278_REG_DIO_MAPPING_1        0x40
#define SX1278_REG_DIO_MAPPING_2        0x41
#define SX1278_REG_VERSION              0x42
#define SX1278_REG_PA_DAC               0x4D

/* Operating modes */
#define SX1278_MODE_SLEEP               0x00
#define SX1278_MODE_STDBY               0x01
#define SX1278_MODE_FSTX                0x02
#define SX1278_MODE_TX                  0x03
#define SX1278_MODE_FSRX                0x04
#define SX1278_MODE_RXCONTINUOUS        0x05
#define SX1278_MODE_RXSINGLE            0x06
#define SX1278_MODE_CAD                 0x07

#define SX1278_MODE_LORA                0x80
#define SX1278_MODE_FSK                 0x00

/* IRQ Flags */
#define SX1278_IRQ_RX_TIMEOUT           0x80
#define SX1278_IRQ_RX_DONE              0x40
#define SX1278_IRQ_PAYLOAD_CRC_ERROR    0x20
#define SX1278_IRQ_VALID_HEADER         0x10
#define SX1278_IRQ_TX_DONE              0x08
#define SX1278_IRQ_CAD_DONE             0x04
#define SX1278_IRQ_FHSS_CHANGE_CHANNEL  0x02
#define SX1278_IRQ_CAD_DETECTED         0x01

/* Bandwidth options */
typedef enum {
    SX1278_BW_7_8_KHZ   = 0,
    SX1278_BW_10_4_KHZ  = 1,
    SX1278_BW_15_6_KHZ  = 2,
    SX1278_BW_20_8_KHZ  = 3,
    SX1278_BW_31_25_KHZ = 4,
    SX1278_BW_41_7_KHZ  = 5,
    SX1278_BW_62_5_KHZ  = 6,
    SX1278_BW_125_KHZ   = 7,
    SX1278_BW_250_KHZ   = 8,
    SX1278_BW_500_KHZ   = 9
} SX1278_Bandwidth_t;

/* Spreading factor */
typedef enum {
    SX1278_SF_6  = 6,
    SX1278_SF_7  = 7,
    SX1278_SF_8  = 8,
    SX1278_SF_9  = 9,
    SX1278_SF_10 = 10,
    SX1278_SF_11 = 11,
    SX1278_SF_12 = 12
} SX1278_SpreadFactor_t;

/* Coding rate */
typedef enum {
    SX1278_CR_4_5 = 1,
    SX1278_CR_4_6 = 2,
    SX1278_CR_4_7 = 3,
    SX1278_CR_4_8 = 4
} SX1278_CodingRate_t;

/* Handle structure */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *nss_port;
    uint16_t nss_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef *dio0_port;
    uint16_t dio0_pin;
    
    uint32_t frequency;
    SX1278_Bandwidth_t bandwidth;
    SX1278_SpreadFactor_t sf;
    SX1278_CodingRate_t cr;
    uint8_t tx_power;
    
    volatile bool tx_done;
    volatile bool rx_done;
} SX1278_Handle_t;

/* Configuration structure */
typedef struct {
    uint32_t frequency;
    SX1278_Bandwidth_t bandwidth;
    SX1278_SpreadFactor_t spreading_factor;
    SX1278_CodingRate_t coding_rate;
    uint8_t tx_power;  // dBm, 2-17 or 20 (with PA_BOOST)
    uint16_t preamble_length;
    uint8_t sync_word;
} SX1278_Config_t;

/* Function prototypes */
HAL_StatusTypeDef SX1278_Init(SX1278_Handle_t *hdev, SPI_HandleTypeDef *hspi, 
                              GPIO_TypeDef *nss_port, uint16_t nss_pin,
                              GPIO_TypeDef *rst_port, uint16_t rst_pin,
                              GPIO_TypeDef *dio0_port, uint16_t dio0_pin);
HAL_StatusTypeDef SX1278_Configure(SX1278_Handle_t *hdev, SX1278_Config_t *config);
HAL_StatusTypeDef SX1278_Transmit(SX1278_Handle_t *hdev, uint8_t *data, uint8_t len);
HAL_StatusTypeDef SX1278_TransmitAsync(SX1278_Handle_t *hdev, uint8_t *data, uint8_t len);
bool SX1278_IsTxDone(SX1278_Handle_t *hdev);
void SX1278_HandleDIO0(SX1278_Handle_t *hdev);
HAL_StatusTypeDef SX1278_SetMode(SX1278_Handle_t *hdev, uint8_t mode);
uint8_t SX1278_ReadRegister(SX1278_Handle_t *hdev, uint8_t reg);
void SX1278_WriteRegister(SX1278_Handle_t *hdev, uint8_t reg, uint8_t value);

#endif /* SX1278_H */
