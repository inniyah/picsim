/* ########################################################################

   PICsim - PIC simulator

   ########################################################################

   Copyright (c) : 2019-2020  Luis Claudio Gamboa Lopes

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

#include <stdio.h>
#include "../../include/periferic18.h"
#include "../../include/picsim.h"

void p18_uart_rst(_pic* pic, int nser) {
    if (nser == 0) {
        pic->serial[0].serial_PIR = pic->P18map.PIR1;
        pic->serial[0].serial_PIE = pic->P18map.PIE1;
        pic->serial[0].RXIF_mask = 0x20;
        pic->serial[0].TXIF_mask = 0x10;
        pic->serial[0].serial_TXSTA = pic->P18map.TXSTA;
        pic->serial[0].serial_RCSTA = pic->P18map.RCSTA;
        pic->serial[0].serial_SPBRG = pic->P18map.SPBRG;
        pic->serial[0].serial_SPBRGH = pic->P18map.SPBRGH;
        pic->serial[0].serial_BAUDCTL = pic->P18map.BAUDCON;
        pic->serial[0].serial_RCREG = pic->P18map.RCREG;
        pic->serial[0].serial_TXREG = pic->P18map.TXREG;
        pic->serial[0].serial_TXREG_ADDR = sfr_addr(pic->P18map.TXREG);
        pic->serial[0].serial_RCSTA_ADDR = sfr_addr(pic->P18map.RCSTA);
        pic->serial[0].serial_RCREG_ADDR = sfr_addr(pic->P18map.RCREG);
        pic->serial[0].serial_TRIS_RX = &pic->ram[sfr_addr(pic->pins[pic->usart_rx[0] - 1].port) + 0x12];
        pic->serial[0].serial_TRIS_RX_MASK = (0x01 << pic->pins[pic->usart_rx[0] - 1].pord);

        pic->serial[0].s_open = 1;

        bb_uart_rst(&pic->serial[0].bbuart);
    }
}

void p18_uart_rst_2(_pic* pic, int nser) {
    // serial compat
    if (nser == 0) {
        pic->P18map.TXSTA = pic->P18map.TX1STA;
        pic->P18map.RCSTA = pic->P18map.RC1STA;
        pic->P18map.SPBRG = pic->P18map.SP1BRGL;
        pic->P18map.SPBRGH = pic->P18map.SP1BRGH;
        pic->P18map.RCREG = pic->P18map.RC1REG;
        pic->P18map.TXREG = pic->P18map.TX1REG;
        pic->P18map.BAUDCON = pic->P18map.BAUDCON1;
        p18_uart_rst(pic, nser);
    }
}

void p18_uart_rst_3(_pic* pic, int nser) {
    if (nser == 0) {
        pic->serial[0].serial_PIR = pic->P18map.PIR3;
        pic->serial[0].serial_PIE = pic->P18map.PIE3;
        pic->serial[0].RXIF_mask = 0x20;
        pic->serial[0].TXIF_mask = 0x10;
        pic->serial[0].serial_TXSTA = pic->P18map.TX1STA;
        pic->serial[0].serial_RCSTA = pic->P18map.RC1STA;
        pic->serial[0].serial_SPBRG = pic->P18map.SP1BRGL;
        pic->serial[0].serial_SPBRGH = pic->P18map.SPBRGH;
        pic->serial[0].serial_BAUDCTL = pic->P18map.BAUDCON1;
        pic->serial[0].serial_RCREG = pic->P18map.RC1REG;
        pic->serial[0].serial_TXREG = pic->P18map.TX1REG;
        pic->serial[0].serial_TXREG_ADDR = sfr_addr(pic->P18map.TX1REG);
        pic->serial[0].serial_RCSTA_ADDR = sfr_addr(pic->P18map.RC1STA);
        pic->serial[0].serial_RCREG_ADDR = sfr_addr(pic->P18map.RC1REG);
        pic->serial[0].serial_TRIS_RX = &pic->ram[sfr_addr(pic->pins[pic->usart_rx[0] - 1].port) - 0x05];
        pic->serial[0].serial_TRIS_RX_MASK = (0x01 << pic->pins[pic->usart_rx[0] - 1].pord);
        pic->serial[0].s_open = 1;
        bb_uart_rst(&pic->serial[0].bbuart);
    } else if (nser == 1) {
        pic->serial[1].serial_PIR = pic->P18map.PIR3;
        pic->serial[1].serial_PIE = pic->P18map.PIE3;
        pic->serial[1].RXIF_mask = 0x80;
        pic->serial[1].TXIF_mask = 0x40;
        pic->serial[1].serial_TXSTA = pic->P18map.TX2STA;
        pic->serial[1].serial_RCSTA = pic->P18map.RC2STA;
        pic->serial[1].serial_SPBRG = pic->P18map.SP2BRGL;  // TODO support to
                                                            // SP2BRGH
        pic->serial[1].serial_RCREG = pic->P18map.RC2REG;
        pic->serial[1].serial_TXREG = pic->P18map.TX2REG;
        pic->serial[1].serial_TXREG_ADDR = sfr_addr(pic->P18map.TX2REG);
        pic->serial[1].serial_RCSTA_ADDR = sfr_addr(pic->P18map.RC2STA);
        pic->serial[1].serial_RCREG_ADDR = sfr_addr(pic->P18map.RC2REG);
        pic->serial[1].serial_TRIS_RX = &pic->ram[sfr_addr(pic->pins[pic->usart_rx[1] - 1].port) - 0x05];
        pic->serial[1].serial_TRIS_RX_MASK = (0x01 << pic->pins[pic->usart_rx[1] - 1].pord);
        pic->serial[1].s_open = 1;
        bb_uart_rst(&pic->serial[1].bbuart);
    }
}

void p18_uart_rst_4(_pic* pic, int nser) {
    // serial compat
    if (nser == 0) {
        pic->P18map.TXSTA = pic->P18map.TXSTA1;
        pic->P18map.RCSTA = pic->P18map.RCSTA1;
        pic->P18map.SPBRG = pic->P18map.SPBRG1;
        pic->P18map.SPBRGH = pic->P18map.SPBRGH1;
        pic->P18map.RCREG = pic->P18map.RCREG1;
        pic->P18map.TXREG = pic->P18map.TXREG1;
        pic->P18map.BAUDCON = pic->P18map.BAUDCON1;
        p18_uart_rst(pic, nser);
    }
}

/*
void
p18_uart(void)
{
}
 */
