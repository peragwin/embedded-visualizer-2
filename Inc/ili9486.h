/*
 * main.c - ILI9486 demo for STM32F4.
 *
 * Copyright (C) 2016 Jan Havran
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ILI9486_H
#define __ILI9486_H

#include "stm32h7xx_hal.h"

#define SPI_PORT    1

#define GPIO_PORT   'C'
#define CE0_GPIO	8
#define RS_GPIO		9
#define DC_GPIO		10

// void init_stm32(void)
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;
// #ifndef CUSTOM_SPI
// 	SPI_InitTypeDef  SPI_InitStructure;
// #endif	// CUSTOM_SPI

// 	/* Enable peripheral clock for SPI1 */
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

// 	/* Enable peripheral clock for GPIO A */
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

// 	/* Common GPIO config */
// 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;

// 	/* Output GPIO config */
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	// output
// #ifndef CUSTOM_SPI
// 	GPIO_InitStructure.GPIO_Pin = RS_GPIO | DC_GPIO | CE0_GPIO;
// #else
// 	GPIO_InitStructure.GPIO_Pin = RS_GPIO | DC_GPIO | CE0_GPIO | SCLK_GPIO |
// 		MOSI_GPIO;
// #endif	// CUSTOM_SPI
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);

// #ifdef CUSTOM_SPI
// 	GPIO_ResetBits(GPIOA, MOSI_GPIO);
// 	GPIO_ResetBits(GPIOA, SCLK_GPIO);
// #endif	// CUSTOM_SPI
// 	GPIO_SetBits(GPIOA, CE0_GPIO);

// #ifndef CUSTOM_SPI
// 	/* SPI GPIO config */
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
// 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

// 	/* Init SCLK*/
// 	GPIO_InitStructure.GPIO_Pin = SCLK_GPIO;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);
// 	GPIO_PinAFConfig(GPIOA, SCLK_SOURCE, GPIO_AF_SPI1);

// 	/* Init MOSI */
// 	GPIO_InitStructure.GPIO_Pin =  MOSI_GPIO;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);
// 	GPIO_PinAFConfig(GPIOA, MOSI_SOURCE, GPIO_AF_SPI1);

// 	/* Init SPI */
// 	SPI_DeInit(SPI1);
// 	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
// 	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
// 	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
// 	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	//clock is low when idle
// 	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
// 	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
// 	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
// 	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
// 	SPI_Init(SPI1, &SPI_InitStructure);
  
// 	/* Enable SPI */
// 	SPI_Cmd(SPI1, ENABLE);
// #endif	// CUSTOM_SPI
// }

void ILI9486_Init(void);

uint16_t Color16(uint8_t r, uint8_t g, uint8_t b);

void ILI9486_DrawPixels(int x, int y, int w, int h, uint16_t *color);

void LCD_Printf(const char *str, int x, int y, uint16_t color);

#endif