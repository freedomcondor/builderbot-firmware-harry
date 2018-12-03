/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
*/

#ifndef HUART_CONTROLLER_H
#define HUART_CONTROLLER_H

#include <inttypes.h>

#define SERIAL_BUFFER_SIZE 64

class CHUARTController // public CInputStream, public COutputStream { // BASIC! contains only ring buffer
{
public:
   struct SRingBuffer
   {
      uint8_t buffer[SERIAL_BUFFER_SIZE];
      volatile unsigned int head;
      volatile unsigned int tail;
   };

   static CHUARTController& instance() {
      return _hardware_serial;
   }

   void Begin(unsigned long);
   void End();
   virtual int Available(void);
   virtual int Peek(void);
   virtual uint8_t Read(void);
   virtual void Flush(void);

   virtual uint8_t Write(uint8_t);

private:
   SRingBuffer *_rx_buffer;
   SRingBuffer *_tx_buffer;
   volatile uint8_t *_ubrrh;
   volatile uint8_t *_ubrrl;
   volatile uint8_t *_ucsra;
   volatile uint8_t *_ucsrb;
   volatile uint8_t *_ucsrc;
   volatile uint8_t *_udr;
   uint8_t _rxen;
   uint8_t _txen;
   uint8_t _rxcie;
   uint8_t _udrie;
   uint8_t _u2x;
   bool transmitting;


private:
   
   /* singleton instance */
   static CHUARTController _hardware_serial;

   /* constructor */
   CHUARTController();
};

#endif
