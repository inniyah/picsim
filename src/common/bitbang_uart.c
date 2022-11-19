/* ########################################################################

   PICsimLab - PIC laboratory simulator

   ########################################################################

   Copyright (c) : 2020-2022  Luis Claudio Gamboa Lopes

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

   For e-mail suggestions :  lcgamboa@yahoo.com
   ######################################################################## */

#include "../../include/bitbang_uart.h"

#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define dprintf                                                                \
  if (1) {                                                                     \
  } else                                                                       \
    printf

void bb_uart_rst(bb_uart_t *bu) {
  bu->prx = 1;
  bu->insr = 0;
  bu->outsr = 1;
  bu->bcr = 0;
  bu->bcw = 0;
  bu->rxc = 0;
  bu->tcountr = 0;
  bu->tcountw = 0;
  bu->leds = 0;
  bu->data_recv = 0;
  bu->data_to_send = 0;
  dprintf("rst uart\n");
}

void bb_uart_init(bb_uart_t *bu) {
  bb_uart_rst(bu);
  bu->speed = 9600;
  bu->freq = 1000000L;
  bu->cycle_count = 100;
  dprintf("init uart\n");
}

void bb_uart_end(bb_uart_t *bu) {}

void bb_uart_set_clk_freq(bb_uart_t *bu, unsigned long freq) {
  bu->freq = freq;
  bu->cycle_count = bu->freq / bu->speed;
  dprintf("uart cycle_count %i \n", bu->cycle_count);
}

void bb_uart_set_speed(bb_uart_t *bu, unsigned int speed) {
  if (speed) {
    bu->speed = speed;
  } else {
    speed = 1;
  }
  bu->cycle_count = bu->freq / bu->speed;
  dprintf("uart cycle_count %i \n", bu->cycle_count);
}

void bb_uart_open(bb_uart_t *bu) {
  bu->dataw = 0xFF;
  bu->bcw = 1;
  bu->outsr = 0xFFFF;
  bu->tcountw = 0;
}

unsigned char bb_uart_io(bb_uart_t *bu, unsigned char rx) {

  // read rx
  if (bu->tcountr) {
    bu->tcountr++;

    if (rx)
      bu->rxc++;

    if (bu->tcountr >= bu->cycle_count) {
      bu->tcountr = 1;

      // printf ("bit=%i mean=%i\n", bu->bcr, bu->rxc);
      if (bu->rxc > (bu->cycle_count >> 1)) {
        bu->insr = (bu->insr >> 1) | 0x8000;
      } else {
        bu->insr = (bu->insr >> 1) & 0x7FFF;
      }
      bu->bcr++;
      bu->rxc = 0;

      if (bu->bcr > 8) // start+eight bits+ stop
      {
        dprintf("uart sr in 0x%04X  out 0x%04X\n", bu->insr, bu->outsr);
        dprintf("%c  0x%04X\n", bu->insr >> 8, bu->insr >> 8);

        bu->datar = (0x00FF & (bu->insr >> 8));
        bu->data_recv = 1;
        bu->tcountr = 0;
        bu->bcr = 0;
      }
    }
  } else {
    // start bit
    if ((bu->prx == 1) && (rx == 0)) // falling edge
    {
      // dprintf ("uart start bit \n");
      bu->tcountr = 1;
      bu->bcr = 0;
      bu->insr = 0;
      bu->rxc = 0;
      bu->leds |= 0x01;
    }
  }

  bu->prx = rx;

  // write tx

  if (!bu->bcw) {
    if (bu->data_to_send) {
      bu->data_to_send = 0;
      dprintf("uart data to send %c \n", bu->dataw);
      bu->leds |= 0x02;
      bu->bcw = 1;
      bu->outsr = (bu->dataw << 1) | 0xFE00;
      // printf ("0x%04X %02X\n", bu->outsr, bu->dataw);
      bu->tcountw = 0;
    }
  } else {
    bu->tcountw++;

    if (bu->tcountw >= (bu->cycle_count)) {
      bu->tcountw = 0;

      bu->outsr = (bu->outsr >> 1);
      bu->bcw++;
      if (bu->bcw > 10) {
        bu->bcw = 0;
      }
    }
  }

  return (bu->outsr & 0x01);
}

unsigned char bb_uart_transmitting(bb_uart_t *bu) { return (bu->bcw > 0); }

void bb_uart_send(bb_uart_t *bu, unsigned char data) {
  bu->dataw = data;
  bu->data_to_send = 1;
}

unsigned char bb_uart_data_available(bb_uart_t *bu) { return bu->data_recv; }

unsigned char bb_uart_recv(bb_uart_t *bu) {
  bu->data_recv = 0;
  return bu->datar;
}
