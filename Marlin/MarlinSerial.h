/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * MarlinSerial.h - Hardware serial library for Wiring
 * Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 *
 * Modified 28 September 2010 by Mark Sproul
 * Modified 14 February 2016 by Andreas Hardtung (added tx buffer)
*/

#ifndef MARLINSERIAL_H
#define MARLINSERIAL_H

#include "Stream.h"

#include "MarlinConfig.h"

#ifndef SERIAL_PORT
  #define SERIAL_PORT 0
#endif

// The presence of the UBRRH register is used to detect a UART.
#define UART_PRESENT(port) ((port == 0 && (defined(UBRRH) || defined(UBRR0H))) || \
                            (port == 1 && defined(UBRR1H)) || (port == 2 && defined(UBRR2H)) || \
                            (port == 3 && defined(UBRR3H)))

// These are macros to build serial port register names for the selected SERIAL_PORT (C preprocessor
// requires two levels of indirection to expand macro values properly)
#define SERIAL_REGNAME(registerbase,number,suffix) SERIAL_REGNAME_INTERNAL(registerbase,number,suffix)
#if SERIAL_PORT == 0 && (!defined(UBRR0H) || !defined(UDR0)) // use un-numbered registers if necessary
  #define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##suffix
#else
  #define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##number##suffix
#endif

// Registers used by MarlinSerial class (expanded depending on selected serial port)
#define T_PORT SERIAL_PORT
#define M_UCSRxA(T_PORT)           SERIAL_REGNAME(UCSR,T_PORT,A) // defines M_UCSRxA to be UCSRnA where n is the serial port number
#define M_UCSRxB(T_PORT)           SERIAL_REGNAME(UCSR,T_PORT,B)
#define M_RXENx(T_PORT)            SERIAL_REGNAME(RXEN,T_PORT,)
#define M_TXENx(T_PORT)            SERIAL_REGNAME(TXEN,T_PORT,)
#define M_TXCx(T_PORT)             SERIAL_REGNAME(TXC,T_PORT,)
#define M_RXCIEx(T_PORT)           SERIAL_REGNAME(RXCIE,T_PORT,)
#define M_UDREx(T_PORT)            SERIAL_REGNAME(UDRE,T_PORT,)
#define M_UDRIEx(T_PORT)           SERIAL_REGNAME(UDRIE,T_PORT,)
#define M_UDRx(T_PORT)             SERIAL_REGNAME(UDR,T_PORT,)
#define M_UBRRxH(T_PORT)           SERIAL_REGNAME(UBRR,T_PORT,H)
#define M_UBRRxL(T_PORT)           SERIAL_REGNAME(UBRR,T_PORT,L)
#define M_RXCx(T_PORT)             SERIAL_REGNAME(RXC,T_PORT,)
#define M_USARTx_RX_vect(T_PORT)   SERIAL_REGNAME(USART,T_PORT,_RX_vect)
#define M_U2Xx(T_PORT)             SERIAL_REGNAME(U2X,T_PORT,)
#define M_USARTx_UDRE_vect(T_PORT) SERIAL_REGNAME(USART,T_PORT,_UDRE_vect)

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0

// Define constants and variables for buffering serial data.
// Use only 0 or powers of 2 greater than 1
// : [0, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, ...]
#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
// 256 is the max TX buffer limit due to uint8_t head and tail.
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 32
#endif

#if !(defined(__AVR__) && defined(USBCON))

  #if RX_BUFFER_SIZE > 256
    typedef uint16_t ring_buffer_pos_t;
  #else
    typedef uint8_t ring_buffer_pos_t;
  #endif

  typedef struct {
    unsigned char buffer[RX_BUFFER_SIZE];
    volatile ring_buffer_pos_t head, tail;
  } ring_buffer_r;

  typedef struct {
    unsigned char buffer[TX_BUFFER_SIZE];
    volatile uint8_t head, tail;
  } ring_buffer_t;

  #if ENABLED(SERIAL_STATS_DROPPED_RX)
    extern uint8_t rx_dropped_bytes;  // TODO
  #endif

  #if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
    extern ring_buffer_pos_t rx_max_enqueued; // TODO
  #endif

  #if ENABLED(EMERGENCY_PARSER)
    extern bool killed_by_M112;
  #endif

  class MarlinSerial { //: public Stream
  class MarlinSerial : public Stream {

    public:
      MarlinSerial();
      void begin(const long);
      void end();
      int peek(void);
      int read(void);
      void flush(void);
      int available(void);
      void checkRx(void);
      size_t write(uint8_t c);
      int availableForWrite();
      void flushTX();
      void writeNoHandshake(const uint8_t c);

      #if ENABLED(SERIAL_STATS_DROPPED_RX)
        FORCE_INLINE uint32_t dropped() { return rx_dropped_bytes; }
      #endif

      #if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
        FORCE_INLINE ring_buffer_pos_t rxMaxEnqueued() { return rx_max_enqueued; }
      #endif

    private:
      void printNumber(unsigned long, const uint8_t);
      void printFloat(double, uint8_t);

      static ring_buffer_r rx_buffer;
      friend FORCE_INLINE void store_rxd_char(void);
      #if TX_BUFFER_SIZE > 0
        static ring_buffer_t tx_buffer;
        static bool _written;
        friend FORCE_INLINE void _tx_udr_empty_irq(void);
      #endif

    public:
      size_t write(const uint8_t *buffer, size_t size) { size_t count = size; while (count--) write(*buffer++); return size; }

  };

  extern MarlinSerial customizedSerial;

#endif // !(__AVR__ && USBCON)

// Use the UART for Bluetooth in AT90USB configurations
#if defined(__AVR__) && defined(USBCON) && ENABLED(BLUETOOTH)
  extern HardwareSerial bluetoothSerial;
#endif

#endif // MARLINSERIAL_H
