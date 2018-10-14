#include "app_util_platform.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"

#include <stdio.h>
#include <stdbool.h>


/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* Common addresses definition for MAX30102 */
#define MAX30102_ADDR 0x57

/* MAX30102 register addresses */
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

// General application timer settings.
#define APP_TIMER_PRESCALER             15    // Value of the RTC1 PRESCALER register.
#define APP_TIMER_OP_QUEUE_SIZE         3     // Size of timer operation queues.


/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
//
//const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;
static uint8_t m_sample_FIFO[6];

uint8_t uch_dummy; //what is this?

uint8_t num_available_samples;
uint8_t num_samples_to_read;

uint32_t RED_LED;
uint32_t IR_LED;

uint64_t millis = 0;

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
//static void rtc_handler()
//{
//  millis++;
//  NRF_LOG_INFO("millis : %d", millis);
//}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t *temp) {
  int i;
  RED_LED = 0;
  IR_LED = 0;

  //0x00 0x00 0x00 0x00 0x00 0x00 |
  for (i = 0; i < 2; i++)
    RED_LED = (RED_LED | temp[i]) << 8;
  RED_LED = (RED_LED | temp[i]);

  for (i = 3; i < 5; i++)
    IR_LED = (IR_LED | temp[i]) << 8;
  IR_LED = (IR_LED | temp[i]);

  NRF_LOG_INFO("data_handler[RED]: %d ", RED_LED);
  NRF_LOG_INFO("data_handler[IR]: %d ", IR_LED);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context) {
  switch (p_event->type) {
  case NRF_DRV_TWI_EVT_DONE:
    if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX) {
      data_handler(m_sample_FIFO);
    }
    m_xfer_done = true;
    break;
  default:
    break;
  }
}

static void read_max_data() {
  m_xfer_done = false;

  /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
  ret_code_t err_code = nrf_drv_twi_rx(&m_twi, MAX30102_ADDR, m_sample_FIFO, 6);
  APP_ERROR_CHECK(err_code);
}

bool writeRegister(uint8_t regNumber, uint8_t value) {
  uint8_t valueBuffer[2];
  valueBuffer[0] = regNumber;
  valueBuffer[1] = value;
  //int32_t toSend = (int32_t)value;
  uint32_t err_code = nrf_drv_twi_tx(&m_twi, MAX30102_ADDR, valueBuffer, sizeof(valueBuffer), false);
  nrf_delay_ms(10);
  return true;
}

uint8_t readRegister(uint8_t regNumber) {
  m_xfer_done = false;

  //uint8_t resultsWhoAmI;
  uint8_t whoAmIPointer = regNumber;
  nrf_drv_twi_tx(&m_twi, MAX30102_ADDR, &whoAmIPointer, 1, true);
  nrf_delay_ms(10);
  nrf_drv_twi_rx(&m_twi, MAX30102_ADDR, &m_sample, 1);
  nrf_delay_ms(10);
  NRF_LOG_INFO("TWI 0x%x: 0x%x.", regNumber, m_sample);
  NRF_LOG_FLUSH();
  return m_sample;  
}

uint8_t readRegister_FIFO(uint8_t regNumber) {
  m_xfer_done = false;
  uint8_t i;

  //uint8_t resultsWhoAmI;
  uint8_t whoAmIPointer = regNumber;
  nrf_drv_twi_tx(&m_twi, MAX30102_ADDR, &whoAmIPointer, 1, true);
  nrf_delay_ms(10);
  nrf_drv_twi_rx(&m_twi, MAX30102_ADDR, m_sample_FIFO, 1);
  nrf_delay_ms(10);
  for (i = 0; i < 6; i++) {
    NRF_LOG_INFO("TWI 0x%x: 0x%x.", regNumber, m_sample_FIFO[i]);
  }
  NRF_LOG_FLUSH();

  return true;
}

/**
 * @brief MAX30102 initialization.
 */
void max_twi_init(void) {
  uint8_t temp2;

  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_max30102_config = {
      .scl = 17,
      .sda = 13,
      .frequency = NRF_DRV_TWI_FREQ_100K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .clear_bus_init = false};

  err_code = nrf_drv_twi_init(&m_twi, &twi_max30102_config, twi_handler, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);

  writeRegister(REG_MODE_CONFIG, 0x40);

  writeRegister(REG_INTR_ENABLE_1, 0xc0); // INTR setting [0x02 1100 0000] A_FULL_EN, PPG_RDY_EN
  writeRegister(REG_INTR_ENABLE_2, 0x00);
  writeRegister(REG_FIFO_WR_PTR, 0x00); //FIFO_WR_PTR[4:0] 0x04
  writeRegister(REG_OVF_COUNTER, 0x00); //OVF_COUNTER[4:0] 0x05
  writeRegister(REG_FIFO_RD_PTR, 0x00); //FIFO_RD_PTR[4:0] 0x06
  writeRegister(REG_FIFO_CONFIG, 0x0f); //sample avg = 1, fifo rollover=false, fifo almost full = 17 [0x08 0000 1111]
  writeRegister(REG_MODE_CONFIG, 0x03); //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED [0x09 0000 0011]
  writeRegister(REG_SPO2_CONFIG, 0x27); // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)  [0x0A 0010 0111]
  writeRegister(REG_LED1_PA, 0x24);     //Choose value for ~ 7mA for LED1 [0x0C 0010 0100]
  writeRegister(REG_LED2_PA, 0x24);     // Choose value for ~ 7mA for LED2 [0x0D 0010 0100]
  writeRegister(REG_PILOT_PA, 0x7f);    // Choose value for ~ 25mA for Pilot LED [0x10 0111 1111]

  num_available_samples = readRegister(REG_FIFO_WR_PTR);
  temp2 = readRegister(REG_FIFO_RD_PTR);
  num_samples_to_read = num_available_samples - temp2;

  readRegister(REG_INTR_STATUS_1);
  readRegister(REG_INTR_STATUS_2);
  readRegister_FIFO(REG_FIFO_DATA);
}


/**
 * @brief Function for main application entry.
 */
int main(void) {
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  NRF_LOG_INFO("TWI sensor example started.3");
  NRF_LOG_FLUSH();
  max_twi_init(); //nrf_drv_twi_init() -> nrf_drv_twi_enable()
  

  //readRegister(REG_INTR_STATUS_1);
  //readRegister(REG_MODE_CONFIG);
  while (true) {
    nrf_delay_ms(100);
    
    do {  
      __WFE();
    } while (m_xfer_done == false);

    read_max_data();
    NRF_LOG_FLUSH();
  }
}
/** @} */