#include "APA102_RGB.h"

#include <math.h>       // required for rainbow calculation

// https://developer.arm.com/documentation/100748/0620/Writing-Optimized-Code/Packing-data-structures
#pragma pack(1)
typedef struct rgb_struct
{
  uint8_t alpha;                            // bytes order for APA_102
  uint8_t b;
  uint8_t g;
  uint8_t r;
} RGB_LED;

#ifndef RAINBOW_COLORS_COUNT
#define RAINBOW_COLORS_COUNT  32
#endif  // RAINBOW_COLORS_COUNT

void RGB_Rainbow_32_Generate();

#ifndef APA102_LED_COUNT_MAX
#define APA102_LED_COUNT_MAX  10
#endif  // APA102_LED_COUNT_MAX

// array of rainbow values
//TODO generate as constant in compile time
static RGB_LED rainbow_led_array[RAINBOW_COLORS_COUNT];
// counter for rainbow effect
static int rainbow_pos = 0;

/**
 * \fn void RGB_Rainbow_Generate(void)
 * \brief Precalculate value for rainbow effect
 * \remark Module internal function
 * \remark Works well for 32 steps
 *
 */
static void RGB_Rainbow_Generate(void)
{
  // https://krazydad.com/tutorials/makecolors.php
  float frequency = .3;
  for (int i = 0; i < RAINBOW_COLORS_COUNT; ++i)
  {
    rainbow_led_array[i].r = sin(frequency * i + 0) * 127 + 128;
    rainbow_led_array[i].g = sin(frequency * i + 2) * 127 + 128;
    rainbow_led_array[i].b = sin(frequency * i + 4) * 127 + 128;
    // alpha not used yet
  }
}

// pure data to send to LEDs
// additionaly values for frame-start and frame-end values
static RGB_LED led_array[APA102_LED_COUNT_MAX + 2];
// starting value, can be reduced in "init"
static int led_count = APA102_LED_COUNT_MAX;    // first value = all

/**
 * \fn bool APA102_RainbowOffset(int)
 * \brief Generate rainbow for all LEDs with position shift
 *
 * \param offset Offset value to show
 * \return Success if offset lower than max-count
 */
bool APA102_RainbowOffset(int offset)
{
  if (offset > RAINBOW_COLORS_COUNT)
    return false;

  rainbow_pos = offset - 1;       // next si interne udela napred +1
  APA102_RainbowNext();

  return true;
}

/**
 * \fn int APA102_RainbowLength(void)
 * \brief Returns number of Rainbow steps
 *
 * \return Count of rainbow values
 */
int APA102_RainbowLength(void)
{
  return RAINBOW_COLORS_COUNT;
}

/**
 * \fn void APA102_RainbowNext(void)
 * \brief Fill LED buffer with rainbow content with internal rainbow-position
 *
 */
void APA102_RainbowNext(void)
{
  rainbow_pos++;
  if (rainbow_pos >= RAINBOW_COLORS_COUNT)
    rainbow_pos = 0;

  int xpos = rainbow_pos;
  for(int i = 1; i <= led_count; i++)
  {
    led_array[i].r = rainbow_led_array[xpos].r;
    led_array[i].g = rainbow_led_array[xpos].g;
    led_array[i].b = rainbow_led_array[xpos].b;

    xpos++;
    if (xpos >= RAINBOW_COLORS_COUNT)
      xpos = 0;
  }
}

/**
 * \fn bool APA102_GetRainbow(int, uint8_t*, uint8_t*, uint8_t*)
 * \brief Get precalculated rainbow value
 *
 * \param pos Position in rainbow table
 * \param pr Pointer to memory to store R value
 * \param pg Pointer to memory to store G value
 * \param pb Pointer to memory to store B value
 * \return Success if pos in range 0 and (Rainbow-count - 1)
 */
bool APA102_GetRainbow(int pos, uint8_t *pr, uint8_t *pg, uint8_t *pb)
{
  if ((pos < 0) || (pos >= RAINBOW_COLORS_COUNT))
    return false;

  *pr = rainbow_led_array[pos].r;
  *pg = rainbow_led_array[pos].g;
  *pb = rainbow_led_array[pos].b;

  return true;
}

/**
 * \fn void APA102_SetIntesity(uint8_t)
 * \brief Set intensity for all LEDs
 *
 * \param val Intensity in range 0-31 (used lower 5 bits)
 */
void APA102_SetIntesity(uint8_t val)
{
  val &= 0x1f;                              // only lower 5 bits is valid
  for (int i = 1; i <= led_count; i++)
    led_array[i].alpha = 0xE0 | val;        // 1110 0000 .... set top 3 bits to 1
}

/**
 * \fn bool APA102_LED(int, int, int, int)
 * \brief Set single LED values
 *
 * \param id LED number in range 0 and (led-counts - 1)
 * \param r R value
 * \param g G value
 * \param b B value
 * \return Success if id in valid range
 */
bool APA102_LED(int id, int r, int g, int b)
{
  if ((id < 0) || (id >= led_count))
    return false;

  id++;                                     // 0-based request, 1-based internally
  led_array[id].r = r;
  led_array[id].g = g;
  led_array[id].b = b;
  // alpha setted for all LEDs in special function

  return true;
}

/**
 * \fn int APA102_GetCount(void)
 * \brief Get count of LEDs (known from init)
 *
 * \return Number of LEDs
 */
int APA102_GetCount(void)
{
  return led_count;
}

// internal SPI pointer
static SPI_TypeDef *SPIx = NULL;

/**
 * \fn bool APA102_init(int)
 * \brief Initialize LED buffer, internal data and SPI fo communication
 * \remark Fixed using SPI1 and PA5 and PA7 - based on Nucleo64 pinout
 *
 * \param count Set available LEDs
 * \return Success if count between 1 and predefined COUNT_MAX
 */
bool APA102_init(int count)
{
  if ((count < 1) || (count > APA102_LED_COUNT_MAX))
    return false;

  led_count = count;
  *(uint32_t *)&led_array[0] = 0x00000000;              // start 32x 0
  *(uint32_t *)&led_array[led_count + 1] = 0xffffffff;  // stop 32x 1

  RGB_Rainbow_Generate();

  SPIx = SPI1;

  if (!(RCC->APB2ENR & RCC_APB2ENR_SPI1EN))
  {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
  }

  STM_SetPinGPIO(GPIOA, 5, ioPortAlternatePP);
  STM_SetAFGPIO(GPIOA, 5, 5);
  STM_SetPinGPIO(GPIOA, 7, ioPortAlternatePP);
  STM_SetAFGPIO(GPIOA, 7, 5);

  SPIx->CR1 = SPI_CR1_BR_1; // 010 = clk/8 - from APB1 (max. 50MHz), eg. 2M for 16M
  SPIx->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM; // work as master, 8-bit mode (DFF = 0)
  SPIx->CR1 |= 0;     // CPHA = 0, CPOL = 0 - see RM0383 pg. 555/836 (rev 1)
  SPIx->CR2 = 0;

  SPIx->CR1 |= SPI_CR1_SPE;
  return true;
}

/**
 * \fn void SPISend32(uint32_t)
 * \brief Send single 32b value to SPI (as 4 bytes)
 * \remark Module internal function
 *
 * \param val Value to write
 */
static void SPISend32(uint32_t val)
{
  uint8_t *dataPtr = (uint8_t*) &val;
  for (int i = 0; i < sizeof(uint32_t); i++)
  {
    while (!(SPIx->SR & SPI_SR_TXE) || (SPIx->SR & SPI_SR_BSY))
      ;

    SPIx->DR = *dataPtr;
    dataPtr++;
  }
}

/**
 * \fn bool APA102_Refresh(void)
 * \brief Send content of LED buffer via SPI
 *
 * \return Always true
 */
bool APA102_Refresh(void)
{
  for (int i = 0; i < (led_count + 2); i++)
  {
    SPISend32(*(uint32_t *)&led_array[i]);
  }

  return true;
}
