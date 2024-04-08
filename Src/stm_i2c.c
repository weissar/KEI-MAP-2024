#include <stm_i2c.h>

#ifndef __CC_ARM
#define __nop()  asm("nop")
#endif

// timeout can be defined outside this file (header or project symbols)
#ifndef I2C_MAX_TIMEOUT
#define I2C_MAX_TIMEOUT   10000
#endif

// local function definitions hidden from external use
static bool I2C_Start(void);
static bool I2C_Stop(void);
static bool I2C_Addr(uint8_t adr);
static void I2C_Reset(void);

/**
 * \fn bool InitI2C1(i2cSpeed)
 * \brief Initialize I2C1, pins, etc.
 * \remark Pin mapping to PB8 SCL and PB9 SDA - based on Nucleo64 board
 *
 * \param spd Predefind enum, equal to speed value, available 100k, 400k
 * \return Success - can be false for unknown speed
 */
bool InitI2C1(i2cSpeed spd)
{
  if ((spd != i2cSpeed100k) && (spd != i2cSpeed400k))
    return false;

  STM_SetPinGPIO(GPIOB, 8, ioPortAlternatePP);  // I2C CLK
  STM_SetAFGPIO(GPIOB, 8, 4);    // AF04 = I2C1_SCL

  STM_SetPinGPIO(GPIOB, 9, ioPortAlternateOC);  // I2C data
  STM_SetAFGPIO(GPIOB, 9, 4);    // AF04 = I2C1_SDA

  if (!(RCC->APB1ENR & RCC_APB1ENR_I2C1EN))
  {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
  }

  I2C_Reset();
  
  // configuration
  I2C1->CR1 = I2C_CR1_PE;         // enable peripheral, remainnig bits = 0
  I2C1->CR2 = 0;                  // clear all cfg. bits
  I2C1->CR2 &= ~ I2C_CR2_FREQ;    // clear bits FREQ[5:0]
  
  {
    int apbClk; //TODO implement in core ... STM_GetBusClock(busClockAPB1);
#ifdef STM32F411xE
    {
      int apb1div = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;  // u F411 je to bit 10..12

      if ((apb1div & 0x04) == 0) // highest bit from 3 == 0 ?
        apbClk = SystemCoreClock;  // x1, AHB = sysclock
      else
        apbClk = SystemCoreClock >> ((apb1div & 0x03) + 1);
    }
#else
#error I2C init speed - unsupported processor type !!
#endif

    int apbClkMhz = apbClk / 1000000;          // clock in MHz

    I2C1->CR2 = apbClkMhz;

    I2C1->CR1 = 0;                  // disable preipheral

    // inspired by Cube generated code
    I2C1->TRISE = (spd <= 100000U)
        ? (apbClkMhz + 1U) : (((apbClkMhz * 300U) / 1000U) + 1U);

#define I2C_SPEED_STANDARD(__PCLK__, __SPEED__)            (((((__PCLK__)/((__SPEED__) << 1U)) & I2C_CCR_CCR) < 4U)? 4U:((__PCLK__) / ((__SPEED__) << 1U)))
#define I2C_SPEED_FAST(__PCLK__, __SPEED__, __DUTYCYCLE__) (((__DUTYCYCLE__) == I2C_DUTYCYCLE_2)? ((__PCLK__) / ((__SPEED__) * 3U)) : (((__PCLK__) / ((__SPEED__) * 25U)) | I2C_DUTYCYCLE_16_9))
#define I2C_SPEED(__PCLK__, __SPEED__, __DUTYCYCLE__)      (((__SPEED__) <= 100000U)? (I2C_SPEED_STANDARD((__PCLK__), (__SPEED__))) : \
                                                                  ((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__)) & I2C_CCR_CCR) == 0U)? 1U : \
                                                                  ((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__))) | I2C_CCR_FS))
#define I2C_DUTYCYCLE_2                 ((uint32_t)0x00000000U)
#define I2C_DUTYCYCLE_16_9              I2C_CCR_DUTY

    I2C1->CCR = I2C_SPEED(apbClk, spd, I2C_DUTYCYCLE_2);
  }
  
  I2C1->CR1 |= I2C_CR1_ACK;       // enable ACK

  #define I2C_ADDRESSINGMODE_7BIT         ((uint32_t)0x00004000)
  I2C1->OAR1 = I2C_ADDRESSINGMODE_7BIT;   // dle Cube
  
  #define I2C_DUALADDRESS_DISABLE         ((uint32_t)0x00000000)
  #define I2C_DUALADDRESS_DISABLED                I2C_DUALADDRESS_DISABLE
  I2C1->OAR2 = I2C_DUALADDRESS_DISABLED;  // dle Cube
  // end Wizard settings
  
  I2C1->CR1 |= I2C_CR1_PE;        // enable peripheral
  return true;
}

/**
 * \fn void I2C_Reset(void)
 * \brief Perform SWRST operation on I2C1
 * \remark Module internal function
 */
static void I2C_Reset(void)
{
  uint16_t tout;

  I2C1->CR1 |= I2C_CR1_SWRST;   // reset peripheral signal
  for (tout = 100; tout; tout--)  // short delay
    __nop();
  I2C1->CR1 = 0;
}

static const uint16_t _timeoutI2C = I2C_MAX_TIMEOUT;

/**
 * \fn uint16_t I2C_sr(void)
 * \brief Read 16-bit status from SR1 and SR2
 * \remark Module internal function
 *
 * \return Compound value
 */
static __inline uint16_t I2C_sr(void) 
{
  uint16_t sr;

  sr  = I2C1->SR1;
  sr |= I2C1->SR2 << 16;
  return (sr);
}

/**
 * \fn bool I2C_Start(void)
 * \brief Perform I2C Start-Condition
 * \remark Module internal function
 *
 * \return Success or timeout failure
 */
static bool I2C_Start(void)
{
  uint16_t w = _timeoutI2C;
  
  I2C1->CR1 &= ~I2C_CR1_STOP;
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C_sr() & I2C_SR1_SB))      // wait for start condition generated
  {
    if (w)
      w--;
    else
      break;
  }
  
  return (w > 0);
}

/**
 * \fn bool I2C_Stop(void)
 * \brief Perform I2C Stop-Condition
 * \remark Module internal function
 *
 * \return Success or timeout failure
 */
static bool I2C_Stop(void)
{
  uint16_t w = _timeoutI2C;

  I2C1->CR1 |= I2C_CR1_STOP;
  while (I2C_sr() & (I2C_SR2_MSL << 16))         // Wait until MSL bit reset - switch from Master mode
  {
    if (w)
      w--;
    else
      break;
  }

  if (w == 0)         // fail process ?
  {
    I2C1->CR1 &= ~I2C_CR1_STOP;           // clear STOP-condition request
    return false;
  }
  else
    return true;
}

/**
 * \fn bool I2C_Addr(uint8_t)
 * \brief Send address byte to I2C1 bus
 * \remark Module internal function
 *
 * \param adr
 * \return Success or timeout failure
 */
static bool I2C_Addr(uint8_t adr)
{
  uint16_t w = _timeoutI2C;
  
  I2C1->DR = adr;
  while(!(I2C_sr() & I2C_SR1_ADDR))  // wait for sending completion
  {
    if (w)
      w--;
    else
      break;
  }
  
  return (w > 0);
}

/**
 * \fn bool I2C_Write(uint8_t)
 * \brief Send data byte to I2C1 bus, waits for TXE
 * \remark Module internal function
 *
 * \param val
 * \return Success or timeout failure
 */
static bool I2C_Write(uint8_t val)
{
  uint16_t w = _timeoutI2C;
  
  I2C1->DR = val;
  while (!(I2C_sr() & (I2C_SR1_BTF | I2C_SR1_TXE)))       // any of that bits
  {
    if (w)
      w--;
    else
      break;
  }
  
  return (w > 0);
}

/**
 * \fn uint8_t I2C_Read(bool)
 * \brief Read single data value (byte) from bus
 * \remark Module internal function
 *
 * \param ack Select if ACK bit will be set
 * \return Readed value
 */
static uint8_t I2C_Read(bool ack)
{
  uint16_t w = _timeoutI2C;

  // Enable/disable Master acknowledge
  if (ack) I2C1->CR1 |= I2C_CR1_ACK;
  else     I2C1->CR1 &= ~I2C_CR1_ACK;

  while (!(I2C_sr() & I2C_SR1_RXNE))
  {
    if (w)
      w--;
    else
      break;
  }
  
  return (I2C1->DR);
}

/**
 * \fn bool I2C1_WriteByte(uint8_t, uint8_t, uint8_t)
 * \brief Write single 8b value to I2C device to their register
 *
 * \param devAdr Device address, 7bit, left aligned
 * \param regAdr Register address
 * \param val Value to write
 * \return Every time OK
 */
bool I2C1_WriteByte(uint8_t devAdr, uint8_t regAdr, uint8_t val)
{ // 7-bit address, last bit R = 1, W = 0
  I2C_Start();
  I2C_Addr(devAdr & 0xfe);          // write
  I2C_Write(regAdr);                // address
  I2C_Write(val);                   // data
  I2C_Stop();

  return true;    //TODO check return states of all partial functions
}

/**
 * \fn bool I2C1_WriteBytes(uint8_t, uint8_t*, uint32_t)
 * \brief Transfer block of values to I2C device
 * \remark Starting register address can be first value in array,
 *         depends on connected I2C device
 *
 * \param devAdr Device address, 7bit, left aligned
 * \param pbuf Data pointer
 * \param len Number of bytes to transfer
 * \return Success of (all) data transfers
 */
bool I2C1_WriteBytes(uint8_t devAdr, uint8_t *pbuf, uint32_t len)
{
  bool bbResult = true;

  I2C_Start();
  I2C_Addr(devAdr & 0xfe);          // write
  for(; len; len--)
  {
    bbResult = bbResult && I2C_Write(*pbuf);
    pbuf++;
  }
  I2C_Stop();

  return bbResult;
}

/**
 * \fn bool I2C1_WriteBytesPre(uint8_t, uint8_t, uint8_t*, uint32_t)
 * \brief Special block transfer
 * \remark Used specially for OLED controllers
 *         Send preByte prior the data-array
 *
 * \param devAdr Device address, 7bit, left aligned
 * \param preByte Additionally byte sended as first value
 * \param pbuf Data pointer
 * \param len Number of bytes to transfer
 * \return Success of (all) data transfers
 */
bool I2C1_WriteBytesPre(uint8_t devAdr, uint8_t preByte, uint8_t *pbuf, uint32_t len)
{
  bool bbResult = true;

  I2C_Start();
  I2C_Addr(devAdr & 0xfe);                 // write
  bbResult = bbResult && I2C_Write(preByte);
  for(; len; len--)
  {
    bbResult = bbResult && I2C_Write(*pbuf);
    pbuf++;
  }
  I2C_Stop();

  return bbResult;
}

/**
 * \fn uint8_t I2C1_ReadByte(uint8_t, uint8_t)
 * \brief Read single byte from register in I2C device
 *
 * \param devAdr Device address, 7bit, left aligned
 * \param regAdr Register address
 * \return Readed value
 */
uint8_t I2C1_ReadByte(uint8_t devAdr, uint8_t regAdr)
{ // 7-bit address, last bit R = 1, W = 0
  uint8_t retval;

  I2C_Start();
  I2C_Addr(devAdr & 0xfe);        // write
  I2C_Write(regAdr);       // address of first register
  I2C_Start();
  I2C_Addr(devAdr | 1);    // read

  retval = I2C_Read(0);    // single read - generate nack

  I2C_Stop();
  return retval;
}

/**
 * \fn bool I2C1_ReadBytes(uint8_t, uint8_t, uint8_t*, uint32_t)
 * \brief Read sequence of byte from registers in I2C device
 *
 * \param devAdr Device address, 7bit, left aligned
 * \param regAdr First register address
 * \param pbuf Pointer to data buffer
 * \param len Required amount of readed bytes
 * \return Always true (in this moment)
 */
bool I2C1_ReadBytes(uint8_t devAdr, uint8_t regAdr, uint8_t *pbuf, uint32_t len)
{ // 7-bit address, last bit R = 1, W = 0
  I2C_Start();
  I2C_Addr(devAdr & 0xfe); // write
  I2C_Write(regAdr);       // address of first register
  I2C_Start();
  I2C_Addr(devAdr | 1);    // read
  for(; len; len--, pbuf++)
    *pbuf = I2C_Read(len > 1);    // for last read is false

  I2C_Stop();
  return true;
}

/**
 * \fn bool I2C1_CheckDeviceAddress(uint8_t)
 * \brief Check if device responds to address
 *
 * \param devAdr7 !!! right aligned address !!!
 * \return Success if device send ACK to address read sequence
 */
bool I2C1_CheckDeviceAddress(uint8_t devAdr7)
{
  I2C_Start();
  bool bb = I2C_Addr(devAdr7 << 1);   // 7-bit address up, LSB = 0
  I2C_Stop();

  return bb;
}
