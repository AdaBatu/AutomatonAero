/**
 * @file sx1278.c
 * @brief SX1278 LoRa 433MHz Radio Driver implementation
 */
#include "sx1278.h"

#define SPI_TIMEOUT     100

/* Internal helper functions */
static void SX1278_NSS_Low(SX1278_Handle_t *hdev)
{
    HAL_GPIO_WritePin(hdev->nss_port, hdev->nss_pin, GPIO_PIN_RESET);
}

static void SX1278_NSS_High(SX1278_Handle_t *hdev)
{
    HAL_GPIO_WritePin(hdev->nss_port, hdev->nss_pin, GPIO_PIN_SET);
}

/* Read single register */
uint8_t SX1278_ReadRegister(SX1278_Handle_t *hdev, uint8_t reg)
{
    uint8_t tx[2] = { reg & 0x7F, 0x00 };  // Read: MSB = 0
    uint8_t rx[2];
    
    SX1278_NSS_Low(hdev);
    HAL_SPI_TransmitReceive(hdev->hspi, tx, rx, 2, SPI_TIMEOUT);
    SX1278_NSS_High(hdev);
    
    return rx[1];
}

/* Write single register */
void SX1278_WriteRegister(SX1278_Handle_t *hdev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg | 0x80, value };  // Write: MSB = 1
    
    SX1278_NSS_Low(hdev);
    HAL_SPI_Transmit(hdev->hspi, tx, 2, SPI_TIMEOUT);
    SX1278_NSS_High(hdev);
}

/* Write burst to FIFO */
static void SX1278_WriteFIFO(SX1278_Handle_t *hdev, uint8_t *data, uint8_t len)
{
    uint8_t reg = SX1278_REG_FIFO | 0x80;
    
    SX1278_NSS_Low(hdev);
    HAL_SPI_Transmit(hdev->hspi, &reg, 1, SPI_TIMEOUT);
    HAL_SPI_Transmit(hdev->hspi, data, len, SPI_TIMEOUT);
    SX1278_NSS_High(hdev);
}

/* Set operating mode */
HAL_StatusTypeDef SX1278_SetMode(SX1278_Handle_t *hdev, uint8_t mode)
{
    SX1278_WriteRegister(hdev, SX1278_REG_OP_MODE, SX1278_MODE_LORA | mode);
    return HAL_OK;
}

/* Set frequency */
static void SX1278_SetFrequency(SX1278_Handle_t *hdev, uint32_t freq)
{
    // Frequency = Fstep * Frf
    // Fstep = Fxosc / 2^19 = 32MHz / 524288 = 61.03515625 Hz
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    
    SX1278_WriteRegister(hdev, SX1278_REG_FRF_MSB, (frf >> 16) & 0xFF);
    SX1278_WriteRegister(hdev, SX1278_REG_FRF_MID, (frf >> 8) & 0xFF);
    SX1278_WriteRegister(hdev, SX1278_REG_FRF_LSB, frf & 0xFF);
    
    hdev->frequency = freq;
}

/* Initialize SX1278 */
HAL_StatusTypeDef SX1278_Init(SX1278_Handle_t *hdev, SPI_HandleTypeDef *hspi, 
                              GPIO_TypeDef *nss_port, uint16_t nss_pin,
                              GPIO_TypeDef *rst_port, uint16_t rst_pin,
                              GPIO_TypeDef *dio0_port, uint16_t dio0_pin)
{
    hdev->hspi = hspi;
    hdev->nss_port = nss_port;
    hdev->nss_pin = nss_pin;
    hdev->reset_port = rst_port;
    hdev->reset_pin = rst_pin;
    hdev->dio0_port = dio0_port;
    hdev->dio0_pin = dio0_pin;
    
    hdev->tx_done = false;
    hdev->rx_done = false;
    
    // Ensure NSS is high
    SX1278_NSS_High(hdev);
    
    // Hardware reset
    if (hdev->reset_port != NULL) {
        HAL_GPIO_WritePin(hdev->reset_port, hdev->reset_pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(hdev->reset_port, hdev->reset_pin, GPIO_PIN_SET);
        HAL_Delay(10);
    }
    
    // Check version register
    uint8_t version = SX1278_ReadRegister(hdev, SX1278_REG_VERSION);
    if (version != 0x12) {
        return HAL_ERROR;  // Not SX1278
    }
    
    // Set sleep mode to access LoRa registers
    SX1278_SetMode(hdev, SX1278_MODE_SLEEP);
    HAL_Delay(10);
    
    // Set LoRa mode
    SX1278_WriteRegister(hdev, SX1278_REG_OP_MODE, SX1278_MODE_LORA | SX1278_MODE_SLEEP);
    HAL_Delay(10);
    
    return HAL_OK;
}

/* Configure SX1278 for LoRa operation */
HAL_StatusTypeDef SX1278_Configure(SX1278_Handle_t *hdev, SX1278_Config_t *config)
{
    // Go to standby mode
    SX1278_SetMode(hdev, SX1278_MODE_STDBY);
    
    // Set frequency
    SX1278_SetFrequency(hdev, config->frequency);
    hdev->frequency = config->frequency;
    
    // Set TX power
    if (config->tx_power > 17) {
        // High power mode (PA_BOOST + PA_DAC)
        SX1278_WriteRegister(hdev, SX1278_REG_PA_CONFIG, 0x8F);  // PA_BOOST, max power
        SX1278_WriteRegister(hdev, SX1278_REG_PA_DAC, 0x87);     // Enable +20dBm
    } else {
        // Normal power mode
        uint8_t pa_config = 0x80 | (config->tx_power - 2);  // PA_BOOST
        SX1278_WriteRegister(hdev, SX1278_REG_PA_CONFIG, pa_config);
        SX1278_WriteRegister(hdev, SX1278_REG_PA_DAC, 0x84);  // Default
    }
    hdev->tx_power = config->tx_power;
    
    // Set bandwidth, coding rate, implicit header mode
    uint8_t modem_config1 = (config->bandwidth << 4) | (config->coding_rate << 1) | 0x00;  // Explicit header
    SX1278_WriteRegister(hdev, SX1278_REG_MODEM_CONFIG_1, modem_config1);
    hdev->bandwidth = config->bandwidth;
    hdev->cr = config->coding_rate;
    
    // Set spreading factor, CRC on, normal RX timeout
    uint8_t modem_config2 = (config->spreading_factor << 4) | 0x04;  // CRC on
    SX1278_WriteRegister(hdev, SX1278_REG_MODEM_CONFIG_2, modem_config2);
    hdev->sf = config->spreading_factor;
    
    // Low data rate optimize for SF11, SF12 with low bandwidth
    uint8_t modem_config3 = 0x00;
    if ((config->spreading_factor >= SX1278_SF_11) && (config->bandwidth <= SX1278_BW_125_KHZ)) {
        modem_config3 = 0x08;  // LowDataRateOptimize = 1
    }
    SX1278_WriteRegister(hdev, SX1278_REG_MODEM_CONFIG_3, modem_config3 | 0x04);  // AGC auto on
    
    // Detection optimize for SF6 or others
    if (config->spreading_factor == SX1278_SF_6) {
        SX1278_WriteRegister(hdev, SX1278_REG_DETECTION_OPTIMIZE, 0xC5);
        SX1278_WriteRegister(hdev, SX1278_REG_DETECTION_THRESHOLD, 0x0C);
    } else {
        SX1278_WriteRegister(hdev, SX1278_REG_DETECTION_OPTIMIZE, 0xC3);
        SX1278_WriteRegister(hdev, SX1278_REG_DETECTION_THRESHOLD, 0x0A);
    }
    
    // Set preamble length
    SX1278_WriteRegister(hdev, SX1278_REG_PREAMBLE_MSB, (config->preamble_length >> 8) & 0xFF);
    SX1278_WriteRegister(hdev, SX1278_REG_PREAMBLE_LSB, config->preamble_length & 0xFF);
    
    // Set sync word (0x12 for private networks, 0x34 for LoRaWAN)
    SX1278_WriteRegister(hdev, SX1278_REG_SYNC_WORD, config->sync_word);
    
    // Set FIFO base addresses
    SX1278_WriteRegister(hdev, SX1278_REG_FIFO_TX_BASE_ADDR, 0x00);
    SX1278_WriteRegister(hdev, SX1278_REG_FIFO_RX_BASE_ADDR, 0x00);
    
    // Map DIO0 to TxDone
    SX1278_WriteRegister(hdev, SX1278_REG_DIO_MAPPING_1, 0x40);
    
    return HAL_OK;
}

/* Transmit data (blocking) */
HAL_StatusTypeDef SX1278_Transmit(SX1278_Handle_t *hdev, uint8_t *data, uint8_t len)
{
    // Go to standby
    SX1278_SetMode(hdev, SX1278_MODE_STDBY);
    
    // Set FIFO pointer to TX base
    SX1278_WriteRegister(hdev, SX1278_REG_FIFO_ADDR_PTR, 0x00);
    
    // Write data to FIFO
    SX1278_WriteFIFO(hdev, data, len);
    
    // Set payload length
    SX1278_WriteRegister(hdev, SX1278_REG_PAYLOAD_LENGTH, len);
    
    // Clear IRQ flags
    SX1278_WriteRegister(hdev, SX1278_REG_IRQ_FLAGS, 0xFF);
    
    // Start transmission
    SX1278_SetMode(hdev, SX1278_MODE_TX);
    
    // Wait for TxDone
    uint32_t start = HAL_GetTick();
    while (!(SX1278_ReadRegister(hdev, SX1278_REG_IRQ_FLAGS) & SX1278_IRQ_TX_DONE)) {
        if (HAL_GetTick() - start > 5000) {  // 5 second timeout
            SX1278_SetMode(hdev, SX1278_MODE_STDBY);
            return HAL_TIMEOUT;
        }
    }
    
    // Clear TxDone flag
    SX1278_WriteRegister(hdev, SX1278_REG_IRQ_FLAGS, SX1278_IRQ_TX_DONE);
    
    // Return to standby
    SX1278_SetMode(hdev, SX1278_MODE_STDBY);
    
    return HAL_OK;
}

/* Transmit data (non-blocking) */
HAL_StatusTypeDef SX1278_TransmitAsync(SX1278_Handle_t *hdev, uint8_t *data, uint8_t len)
{
    hdev->tx_done = false;
    
    // Go to standby
    SX1278_SetMode(hdev, SX1278_MODE_STDBY);
    
    // Set FIFO pointer to TX base
    SX1278_WriteRegister(hdev, SX1278_REG_FIFO_ADDR_PTR, 0x00);
    
    // Write data to FIFO
    SX1278_WriteFIFO(hdev, data, len);
    
    // Set payload length
    SX1278_WriteRegister(hdev, SX1278_REG_PAYLOAD_LENGTH, len);
    
    // Clear IRQ flags
    SX1278_WriteRegister(hdev, SX1278_REG_IRQ_FLAGS, 0xFF);
    
    // Map DIO0 to TxDone
    SX1278_WriteRegister(hdev, SX1278_REG_DIO_MAPPING_1, 0x40);
    
    // Start transmission
    SX1278_SetMode(hdev, SX1278_MODE_TX);
    
    return HAL_OK;
}

/* Check if transmission is complete */
bool SX1278_IsTxDone(SX1278_Handle_t *hdev)
{
    if (hdev->tx_done) {
        return true;
    }
    
    // Check flag manually
    if (SX1278_ReadRegister(hdev, SX1278_REG_IRQ_FLAGS) & SX1278_IRQ_TX_DONE) {
        SX1278_WriteRegister(hdev, SX1278_REG_IRQ_FLAGS, SX1278_IRQ_TX_DONE);
        hdev->tx_done = true;
        SX1278_SetMode(hdev, SX1278_MODE_STDBY);
        return true;
    }
    
    return false;
}

/* Handle DIO0 interrupt (call from EXTI callback) */
void SX1278_HandleDIO0(SX1278_Handle_t *hdev)
{
    uint8_t irq_flags = SX1278_ReadRegister(hdev, SX1278_REG_IRQ_FLAGS);
    
    if (irq_flags & SX1278_IRQ_TX_DONE) {
        hdev->tx_done = true;
        SX1278_WriteRegister(hdev, SX1278_REG_IRQ_FLAGS, SX1278_IRQ_TX_DONE);
        SX1278_SetMode(hdev, SX1278_MODE_STDBY);
    }
    
    if (irq_flags & SX1278_IRQ_RX_DONE) {
        hdev->rx_done = true;
        SX1278_WriteRegister(hdev, SX1278_REG_IRQ_FLAGS, SX1278_IRQ_RX_DONE);
    }
}
