/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

// DMAMUX1 - DMA1/DMA2 mappings for x7plus-wl wheel loader controller

// SPI DMA mappings for sensor interfaces
#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0 /* DMA1:37 */
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0 /* DMA1:38 */

#define DMAMAP_SPI4_RX    DMAMAP_DMA12_SPI4RX_0 /* DMA1:83 */
#define DMAMAP_SPI4_TX    DMAMAP_DMA12_SPI4TX_0 /* DMA1:84 */

// UART DMA mappings for wheel loader communication
// GPS1 (RTK GPS) - USART2
#define DMAMAP_USART2_RX  DMAMAP_DMA12_USART2RX_0 /* DMA1:35 */
#define DMAMAP_USART2_TX  DMAMAP_DMA12_USART2TX_0 /* DMA1:36 */

// TELEM1 (MAVLink) - USART1
#define DMAMAP_USART1_RX  DMAMAP_DMA12_USART1RX_0 /* DMA2:41 */
#define DMAMAP_USART1_TX  DMAMAP_DMA12_USART1TX_0 /* DMA2:42 */

// TELEM2 (NXT Controller 1) - UART4
#define DMAMAP_UART4_RX   DMAMAP_DMA12_UART4RX_0 /* DMA1:63 */
#define DMAMAP_UART4_TX   DMAMAP_DMA12_UART4TX_0 /* DMA1:64 */

// UART4 (NXT Controller 2) - USART6
#define DMAMAP_USART6_RX  DMAMAP_DMA12_USART6RX_0 /* DMA2:71 */
#define DMAMAP_USART6_TX  DMAMAP_DMA12_USART6TX_0 /* DMA2:72 */

// Additional UART for debug/expansion - UART8
#define DMAMAP_UART8_RX   DMAMAP_DMA12_UART8RX_0 /* DMA1:81 */
#define DMAMAP_UART8_TX   DMAMAP_DMA12_UART8TX_0 /* DMA1:82 */

// DMAMUX2 (BDMA) - Low-power domain peripherals
#define DMAMAP_SPI6_RX    DMAMAP_BDMA_SPI6_RX /* BDMA:11 */
#define DMAMAP_SPI6_TX    DMAMAP_BDMA_SPI6_TX /* BDMA:12 */

// I2C DMA mappings for sensor communication
#define DMAMAP_I2C1_RX    DMAMAP_DMA12_I2C1RX_0 /* DMA1:15 */
#define DMAMAP_I2C1_TX    DMAMAP_DMA12_I2C1TX_0 /* DMA1:16 */

#define DMAMAP_I2C2_RX    DMAMAP_DMA12_I2C2RX_0 /* DMA1:17 */
#define DMAMAP_I2C2_TX    DMAMAP_DMA12_I2C2TX_0 /* DMA1:18 */
