/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "CoreSPI.hpp"

SPIClass::SPIClass(Spi *_spi, uint32_t _id, uint32_t _defaultSS, void(*_initCb)(void)) :
  spi(_spi), id(_id), defaultSS(_defaultSS), initCb(_initCb), initialized(false)
{
  // Empty
}

void SPIClass::begin()
{
#if 0
  init();
  // NPCS control is left to the user

  // Default speed set to 4Mhz
  setClockDivider(BOARD_SPI_DEFAULT_SS, 21);
  setDataMode(BOARD_SPI_DEFAULT_SS, SPI_MODE0);
  setBitOrder(BOARD_SPI_DEFAULT_SS, MSBFIRST);
#endif // 0
}

void SPIClass::begin(uint8_t _pin)
{
#if 0
  init();

  uint32_t spiPin = BOARD_PIN_TO_SPI_PIN(_pin);
  PIO_Configure(
    g_aPinMap[spiPin].pPort,
    g_aPinMap[spiPin].ulPinType,
    g_aPinMap[spiPin].ulPin,
    g_aPinMap[spiPin].ulPinConfiguration);

  // Default speed set to 4Mhz
  setClockDivider(_pin, 21);
  setDataMode(_pin, SPI_MODE0);
  setBitOrder(_pin, MSBFIRST);
#endif // 0
}

void SPIClass::init()
{
#if 0
  if (initialized)
    return;
  interruptMode = 0;
  interruptSave = 0;
  interruptMask[0] = 0;
  interruptMask[1] = 0;
  interruptMask[2] = 0;
  interruptMask[3] = 0;
  initCb();
  SPI_Configure(spi, id, SPI_MR_MSTR | SPI_MR_PS | SPI_MR_MODFDIS);
  SPI_Enable(spi);
  initialized = true;
#endif // 0
}

#if 0
#ifndef interruptsStatus
#define interruptsStatus() __interruptsStatus()
static inline unsigned char __interruptsStatus(void) __attribute__((always_inline, unused));
static inline unsigned char __interruptsStatus(void)
{
  unsigned long primask, faultmask;

  asm volatile ("mrs %0, primask" : "=r" (primask));
  if (primask) return 0;

  asm volatile ("mrs %0, faultmask" : "=r" (faultmask));
  if (faultmask) return 0;

  return 1;
}
#endif // interruptsStatus
#endif // 0

void SPIClass::usingInterrupt(uint8_t interruptNumber)
{
#if 0
  uint8_t irestore;

  irestore = interruptsStatus();
  noInterrupts();
  if (interruptMode < 16)
    {
    if (interruptNumber > NUM_DIGITAL_PINS)
    {
      interruptMode = 16;
    }
    else
    {
      Pio *pio = g_aPinMap[interruptNumber].pPort;
      uint32_t mask = g_aPinMap[interruptNumber].ulPin;
      if (pio == PIOA) {
        interruptMode |= 1;
        interruptMask[0] |= mask;
      } else if (pio == PIOB) {
        interruptMode |= 2;
        interruptMask[1] |= mask;
      } else if (pio == PIOC) {
        interruptMode |= 4;
        interruptMask[2] |= mask;
      } else if (pio == PIOD) {
        interruptMode |= 8;
        interruptMask[3] |= mask;
      } else {
        interruptMode = 16;
      }
    }
  }
  if (irestore) interrupts();
#endif // 0
}

void SPIClass::beginTransaction(uint8_t pin, SPISettings settings)
{
#if 0
  uint8_t mode = interruptMode;
  if (mode > 0) {
    if (mode < 16) {
      if (mode & 1) PIOA->PIO_IDR = interruptMask[0];
      if (mode & 2) PIOB->PIO_IDR = interruptMask[1];
      if (mode & 4) PIOC->PIO_IDR = interruptMask[2];
      if (mode & 8) PIOD->PIO_IDR = interruptMask[3];
    } else {
      interruptSave = interruptsStatus();
      noInterrupts();
    }
  }
  uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(pin);
  bitOrder[ch] = settings.border;
  SPI_ConfigureNPCS(spi, ch, settings.config);
  //setBitOrder(pin, settings.border);
  //setDataMode(pin, settings.datamode);
  //setClockDivider(pin, settings.clockdiv);
#endif // 0
}

void SPIClass::endTransaction(void)
{
#if 0
  uint8_t mode = interruptMode;
  if (mode > 0) {
    if (mode < 16) {
      if (mode & 1) PIOA->PIO_IER = interruptMask[0];
      if (mode & 2) PIOB->PIO_IER = interruptMask[1];
      if (mode & 4) PIOC->PIO_IER = interruptMask[2];
      if (mode & 8) PIOD->PIO_IER = interruptMask[3];
    } else {
      if (interruptSave) interrupts();
    }
  }
#endif // 0
}

void SPIClass::end(uint8_t _pin)
{
#if 0
  uint32_t spiPin = BOARD_PIN_TO_SPI_PIN(_pin);
  // Setting the pin as INPUT will disconnect it from SPI peripheral
  pinMode(spiPin, INPUT);
#endif // 0
}

void SPIClass::end(void)
{
#if 0
  SPI_Disable(spi);
  initialized = false;
#endif // 0
}

void SPIClass::setBitOrder(uint8_t _pin, BitOrder _bitOrder)
{
#if 0
  uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(_pin);
  bitOrder[ch] = _bitOrder;
#endif // 0
}

void SPIClass::setDataMode(uint8_t _pin, uint8_t _mode)
{
#if 0
  uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(_pin);
  mode[ch] = _mode | SPI_CSR_CSAAT;
  // SPI_CSR_DLYBCT(1) keeps CS enabled for 32 MCLK after a completed
  // transfer. Some device needs that for working properly.
  SPI_ConfigureNPCS(spi, ch, mode[ch] | SPI_CSR_SCBR(divider[ch]) | SPI_CSR_DLYBCT(1));
#endif // 0
}

void SPIClass::setClockDivider(uint8_t _pin, uint8_t _divider)
{
#if 0
  uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(_pin);
  divider[ch] = _divider;
  // SPI_CSR_DLYBCT(1) keeps CS enabled for 32 MCLK after a completed
  // transfer. Some device needs that for working properly.
  SPI_ConfigureNPCS(spi, ch, mode[ch] | SPI_CSR_SCBR(divider[ch]) | SPI_CSR_DLYBCT(1));
#endif // 0
}

byte SPIClass::transfer(byte _pin, uint8_t _data, SPITransferMode _mode)
{
#if 0
  uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(_pin);
  // Reverse bit order
  if (bitOrder[ch] == LSBFIRST)
    _data = __REV(__RBIT(_data));
  uint32_t d = _data | SPI_PCS(ch);
  if (_mode == SPI_LAST)
    d |= SPI_TDR_LASTXFER;

  // SPI_Write(spi, _channel, _data);
  while ((spi->SPI_SR & SPI_SR_TDRE) == 0)
    ;
  spi->SPI_TDR = d;

  // return SPI_Read(spi);
  while ((spi->SPI_SR & SPI_SR_RDRF) == 0)
    ;
  d = spi->SPI_RDR;
  // Reverse bit order
  if (bitOrder[ch] == LSBFIRST)
    d = __REV(__RBIT(d));
  return d & 0xFF;
#endif // 0

  return 0;
}

void SPIClass::transfer(byte _pin, void *_buf, size_t _count, SPITransferMode _mode)
{
#if 0
  if (_count == 0)
    return;

  uint8_t *buffer = (uint8_t *)_buf;
  if (_count == 1) {
    *buffer = transfer(_pin, *buffer, _mode);
    return;
  }

  uint32_t ch = BOARD_PIN_TO_SPI_CHANNEL(_pin);
  bool reverse = (bitOrder[ch] == LSBFIRST);

  // Send the first byte
  uint32_t d = *buffer;
  if (reverse)
    d = __REV(__RBIT(d));
  while ((spi->SPI_SR & SPI_SR_TDRE) == 0)
    ;
  spi->SPI_TDR = d | SPI_PCS(ch);

  while (_count > 1) {
    // Prepare next byte
    d = *(buffer+1);
    if (reverse)
      d = __REV(__RBIT(d));
    if (_count == 2 && _mode == SPI_LAST)
      d |= SPI_TDR_LASTXFER;

    // Read transferred byte and send next one straight away
    while ((spi->SPI_SR & SPI_SR_RDRF) == 0)
      ;
    uint8_t r = spi->SPI_RDR;
    spi->SPI_TDR = d | SPI_PCS(ch);

    // Save read byte
    if (reverse)
      r = __REV(__RBIT(r));
    *buffer = r;
    buffer++;
    _count--;
  }

  // Receive the last transferred byte
  while ((spi->SPI_SR & SPI_SR_RDRF) == 0)
    ;
  uint8_t r = spi->SPI_RDR;
  if (reverse)
    r = __REV(__RBIT(r));
  *buffer = r;
#endif // 0
}

void SPIClass::attachInterrupt(void)
{
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt(void)
{
  // Should be disableInterrupt()
}

#if SPI_INTERFACES_COUNT > 0
static void SPI_0_Init(void)
{
#if 0
  PIO_Configure(
      g_aPinMap[PIN_SPI_MOSI].pPort,
      g_aPinMap[PIN_SPI_MOSI].ulPinType,
      g_aPinMap[PIN_SPI_MOSI].ulPin,
      g_aPinMap[PIN_SPI_MOSI].ulPinConfiguration);
  PIO_Configure(
      g_aPinMap[PIN_SPI_MISO].pPort,
      g_aPinMap[PIN_SPI_MISO].ulPinType,
      g_aPinMap[PIN_SPI_MISO].ulPin,
      g_aPinMap[PIN_SPI_MISO].ulPinConfiguration);
  PIO_Configure(
      g_aPinMap[PIN_SPI_SCK].pPort,
      g_aPinMap[PIN_SPI_SCK].ulPinType,
      g_aPinMap[PIN_SPI_SCK].ulPin,
      g_aPinMap[PIN_SPI_SCK].ulPinConfiguration);
#endif // 0
}

SPIClass SPI(SPI_INTERFACE, SPI_INTERFACE_ID, SS, SPI_0_Init);
#endif // SPI_INTERFACES_COUNT

