/* ########################################################################

   PICsim - PIC simulator

   ########################################################################

   Copyright (c) : 2008-2020  Luis Claudio Gamboa Lopes

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

#include "../../include/picsim.h"
#include"../p18/p18_periferic.h"
#include <string.h>

void
PIC18F452_map(void)
{
 memset(&pic->P18map,0,sizeof(P18map_t));

#ifdef ICSPDBG   
 pic->P18map.DEBUG = &pic->ram[0xFD4];
 pic->P18map.BDMSR2 = &pic->ram[0xFB9];
 pic->P18map.BDMSR1 = &pic->ram[0xFB8];
 pic->P18map.BDMSR0 = &pic->ram[0xFB7];
#endif 
 pic->P18map.WS = &pic->ram[0xF00];
 pic->P18map.STATUSS = &pic->ram[0xF01];
 pic->P18map.BSRS = &pic->ram[0xF02];
 
 pic->P18map.TMR3L = &pic->ram[0xFB2];
 pic->P18map.TMR3H = &pic->ram[0xFB3];
 pic->P18map.CCPR2L = &pic->ram[0xFBB];
 pic->P18map.CCPR2H = &pic->ram[0xFBC];
 pic->P18map.CCPR1L = &pic->ram[0xFBE];
 pic->P18map.CCPR1H = &pic->ram[0xFBF];
 pic->P18map.ADRESL = &pic->ram[0xFC3];
 pic->P18map.ADRESH = &pic->ram[0xFC4];
 pic->P18map.TMR1L = &pic->ram[0xFCE];
 pic->P18map.TMR1H = &pic->ram[0xFCF];
 pic->P18map.TMR0L = &pic->ram[0xFD6];
 pic->P18map.TMR0H = &pic->ram[0xFD7];
 pic->P18map.FSR2L = &pic->ram[0xFD9];
 pic->P18map.FSR2H = &pic->ram[0xFDA];
 pic->P18map.FSR1L = &pic->ram[0xFE1];
 pic->P18map.FSR1H = &pic->ram[0xFE2];
 pic->P18map.FSR0L = &pic->ram[0xFE9];
 pic->P18map.FSR0H = &pic->ram[0xFEA];
 pic->P18map.PRODL = &pic->ram[0xFF3];
 pic->P18map.PRODH = &pic->ram[0xFF4];
 pic->P18map.TBLPTRL = &pic->ram[0xFF6];
 pic->P18map.TBLPTRH = &pic->ram[0xFF7];
 pic->P18map.TBLPTRU = &pic->ram[0xFF8];
 pic->P18map.PCL = &pic->ram[0xFF9];
 pic->P18map.PCLATH = &pic->ram[0xFFA];
 pic->P18map.PCLATU = &pic->ram[0xFFB];
 pic->P18map.TOSL = &pic->ram[0xFFD];
 pic->P18map.TOSH = &pic->ram[0xFFE];
 pic->P18map.TOSU = &pic->ram[0xFFF];
 pic->P18map.PORTA = &pic->ram[0xF80];
 pic->P18map.PORTB = &pic->ram[0xF81];
 pic->P18map.PORTC = &pic->ram[0xF82];
 pic->P18map.PORTD = &pic->ram[0xF83];
 pic->P18map.PORTE = &pic->ram[0xF84];
 pic->P18map.LATA = &pic->ram[0xF89];
 pic->P18map.LATB = &pic->ram[0xF8A];
 pic->P18map.LATC = &pic->ram[0xF8B];
 pic->P18map.LATD = &pic->ram[0xF8C];
 pic->P18map.LATE = &pic->ram[0xF8D];
 pic->P18map.TRISA = &pic->ram[0xF92];
 pic->P18map.TRISB = &pic->ram[0xF93];
 pic->P18map.TRISC = &pic->ram[0xF94];
 pic->P18map.TRISD = &pic->ram[0xF95];
 pic->P18map.TRISE = &pic->ram[0xF96];
 pic->P18map.PIE1 = &pic->ram[0xF9D];
 pic->P18map.PIR1 = &pic->ram[0xF9E];
 pic->P18map.IPR1 = &pic->ram[0xF9F];
 pic->P18map.PIE2 = &pic->ram[0xFA0];
 pic->P18map.PIR2 = &pic->ram[0xFA1];
 pic->P18map.IPR2 = &pic->ram[0xFA2];
 pic->P18map.EECON1 = &pic->ram[0xFA6];
 pic->P18map.EECON2 = &pic->ram[0xFA7];
 pic->P18map.EEDATA = &pic->ram[0xFA8];
 pic->P18map.EEADR = &pic->ram[0xFA9];
 pic->P18map.RCSTA = &pic->ram[0xFAB];
 pic->P18map.TXSTA = &pic->ram[0xFAC];
 pic->P18map.TXREG = &pic->ram[0xFAD];
 pic->P18map.RCREG = &pic->ram[0xFAE];
 pic->P18map.SPBRG = &pic->ram[0xFAF];
 pic->P18map.T3CON = &pic->ram[0xFB1];
 pic->P18map.CCP2CON = &pic->ram[0xFBA];
 pic->P18map.CCP1CON = &pic->ram[0xFBD];
 pic->P18map.ADCON1 = &pic->ram[0xFC1];
 pic->P18map.ADCON0 = &pic->ram[0xFC2];
 pic->P18map.SSPCON2 = &pic->ram[0xFC5];
 pic->P18map.SSPCON1 = &pic->ram[0xFC6];
 pic->P18map.SSPSTAT = &pic->ram[0xFC7];
 pic->P18map.SSPADD = &pic->ram[0xFC8];
 pic->P18map.SSPBUF = &pic->ram[0xFC9];
 pic->P18map.T2CON = &pic->ram[0xFCA];
 pic->P18map.PR2 = &pic->ram[0xFCB];
 pic->P18map.TMR2 = &pic->ram[0xFCC];
 pic->P18map.T1CON = &pic->ram[0xFCD];
 pic->P18map.RCON = &pic->ram[0xFD0];
 pic->P18map.WDTCON = &pic->ram[0xFD1];
 //pic->P18map.LVDCON = &pic->ram[0xFD2];
 //pic->P18map.OSCCON = &pic->ram[0xFD3];
 pic->P18map.T0CON = &pic->ram[0xFD5];
 pic->P18map.STATUS = &pic->ram[0xFD8];
 pic->P18map.PLUSW2 = &pic->ram[0xFDB];
 pic->P18map.PREINC2 = &pic->ram[0xFDC];
 pic->P18map.POSTDEC2 = &pic->ram[0xFDD];
 pic->P18map.POSTINC2 = &pic->ram[0xFDE];
 pic->P18map.INDF2 = &pic->ram[0xFDF];
 pic->P18map.BSR = &pic->ram[0xFE0];
 pic->P18map.PLUSW1 = &pic->ram[0xFE3];
 pic->P18map.PREINC1 = &pic->ram[0xFE4];
 pic->P18map.POSTDEC1 = &pic->ram[0xFE5];
 pic->P18map.POSTINC1 = &pic->ram[0xFE6];
 pic->P18map.INDF1 = &pic->ram[0xFE7];
 pic->P18map.WREG = &pic->ram[0xFE8];
 pic->P18map.PLUSW0 = &pic->ram[0xFEB];
 pic->P18map.PREINC0 = &pic->ram[0xFEC];
 pic->P18map.POSTDEC0 = &pic->ram[0xFED];
 pic->P18map.POSTINC0 = &pic->ram[0xFEE];
 pic->P18map.INDF0 = &pic->ram[0xFEF];
 pic->P18map.INTCON3 = &pic->ram[0xFF0];
 pic->P18map.INTCON2 = &pic->ram[0xFF1];
 pic->P18map.INTCON = &pic->ram[0xFF2];
 pic->P18map.TABLAT = &pic->ram[0xFF5];
 pic->P18map.STKPTR = &pic->ram[0xFFC];
}

/*PINS 

01 - MCLR VPP 
02 - RA0 AN0 
03 - RA1 AN1 
04 - RA2 AN2 VREF- 
05 - RA3 AN3 VREF+ 
06 - RA4 T0CKI 
07 - RA5 AN4 SS LVDIN 
08 - RE0 RD AN5 
09 - RE1 WR AN6 
10 - RE2 CS AN7 
11 - VDD 
12 - VSS 
13 - OSC1 CLKI 
14 - OSC2 CLKO RA6 
15 - RC0 T1OSO T1CKI 
16 - RC1 T1OSI CCP2 
17 - RC2 CCP1 
18 - RC3 SCK SCL 
19 - RD0 PSP0 
20 - RD1 PSP1 
21 - RD2 PSP2 
22 - RD3 PSP3 
23 - RC4 SDI SDA 
24 - RC5 SDO 
25 - RC6 TX CK 
26 - RC7 RX DT 
27 - RD4 PSP4 
28 - RD5 PSP5 
29 - RD6 PSP6 
30 - RD7 PSP7 
31 - VSS 
32 - VDD 
33 - RB0 INT0 
34 - RB1 INT1 
35 - RB2 INT2 
36 - RB3 CCP2 
37 - RB4 
38 - RB5 PGM 
39 - RB6 PGC 
40 - RB7 PGD 
 */

/*
======================
Family=P18 
processor=PIC18F452
deviceid=0x420
 */

void
PIC18F452_reset(void)
{
 
 pic->pins[ 0].port = P_RST    ;pic->pins[ 0].pord = -1;
 pic->pins[ 1].port = pic->P18map.PORTA;pic->pins[ 1].pord = 0;
 pic->pins[ 2].port = pic->P18map.PORTA;pic->pins[ 2].pord = 1;
 pic->pins[ 3].port = pic->P18map.PORTA;pic->pins[ 3].pord = 2;
 pic->pins[ 4].port = pic->P18map.PORTA;pic->pins[ 4].pord = 3;
 pic->pins[ 5].port = pic->P18map.PORTA;pic->pins[ 5].pord = 4;
 pic->pins[ 6].port = pic->P18map.PORTA;pic->pins[ 6].pord = 5;
 pic->pins[ 7].port = pic->P18map.PORTE;pic->pins[ 7].pord = 0;
 pic->pins[ 8].port = pic->P18map.PORTE;pic->pins[ 8].pord = 1;
 pic->pins[ 9].port = pic->P18map.PORTE;pic->pins[ 9].pord = 2;
 pic->pins[10].port = P_VDD    ;pic->pins[10].pord = -1;pic->pins[10].value = 1;
 pic->pins[11].port = P_VSS    ;pic->pins[11].pord = -1;
 pic->pins[12].port = P_OSC    ;pic->pins[12].pord = -1;
 pic->pins[13].port = pic->P18map.PORTA;pic->pins[13].pord = 6;
 pic->pins[14].port = pic->P18map.PORTC;pic->pins[14].pord = 0;
 pic->pins[15].port = pic->P18map.PORTC;pic->pins[15].pord = 1;
 pic->pins[16].port = pic->P18map.PORTC;pic->pins[16].pord = 2;
 pic->pins[17].port = pic->P18map.PORTC;pic->pins[17].pord = 3;
 pic->pins[18].port = pic->P18map.PORTD;pic->pins[18].pord = 0;
 pic->pins[19].port = pic->P18map.PORTD;pic->pins[19].pord = 1;
 pic->pins[20].port = pic->P18map.PORTD;pic->pins[20].pord = 2;
 pic->pins[21].port = pic->P18map.PORTD;pic->pins[21].pord = 3;
 pic->pins[22].port = pic->P18map.PORTC;pic->pins[22].pord = 4;
 pic->pins[23].port = pic->P18map.PORTC;pic->pins[23].pord = 5;
 pic->pins[24].port = pic->P18map.PORTC;pic->pins[24].pord = 6;
 pic->pins[25].port = pic->P18map.PORTC;pic->pins[25].pord = 7;
 pic->pins[26].port = pic->P18map.PORTD;pic->pins[26].pord = 4;
 pic->pins[27].port = pic->P18map.PORTD;pic->pins[27].pord = 5;
 pic->pins[28].port = pic->P18map.PORTD;pic->pins[28].pord = 6;
 pic->pins[29].port = pic->P18map.PORTD;pic->pins[29].pord = 7;
 pic->pins[30].port = P_VSS    ;pic->pins[30].pord = -1;
 pic->pins[31].port = P_VDD    ;pic->pins[31].pord = -1;pic->pins[31].value = 1;
 pic->pins[32].port = pic->P18map.PORTB;pic->pins[32].pord = 0;
 pic->pins[33].port = pic->P18map.PORTB;pic->pins[33].pord = 1;
 pic->pins[34].port = pic->P18map.PORTB;pic->pins[34].pord = 2;
 pic->pins[35].port = pic->P18map.PORTB;pic->pins[35].pord = 3;
 pic->pins[36].port = pic->P18map.PORTB;pic->pins[36].pord = 4;
 pic->pins[37].port = pic->P18map.PORTB;pic->pins[37].pord = 5;
 pic->pins[38].port = pic->P18map.PORTB;pic->pins[38].pord = 6;
 pic->pins[39].port = pic->P18map.PORTB;pic->pins[39].pord = 7;
 
 pic->mclr = 1;

 pic->ccp[0] = 17;
 pic->ccp[1] = 16;//36

 pic->adc[0] = 2;
 pic->adc[1] = 3;
 pic->adc[2] = 4;
 pic->adc[3] = 5;
 pic->adc[4] = 7;
 pic->adc[5] = 8;
 pic->adc[6] = 9;
 pic->adc[7] = 10;

 pic->usart[0] = 26;
 pic->usart[1] = 25;

 pic->pgc = 39;
 pic->pgd = 40;

 pic->sck = 18;
 pic->sdo = 24;
 pic->sdi = 23;

 pic->t0cki = 6;
 pic->t1cki = 15;

 pic->int0 = 33;
 pic->int1 = 34;
 pic->int2 = 35;
 
 p18_tmr0_rst ();
 p18_tmr1_rst ();
 p18_tmr2_rst ();
 p18_tmr3_rst ();
 p18_adc_rst ();
 p18_wdt_rst ();
 p18_eeprom_rst ();
 p18_mssp_rst ();
 p18_int_pin_rst();
 p18_int_portb_rst();
 p18_uart_rst();
}
void
PIC18F452_periferic(void)
{
  p18_mssp ();
  p18_adc_2 ();
  p18_int_pin();
  p18_int_portb();
  p18_tmr0 ();
  p18_wdt ();
  p18_eeprom ();
  p18_tmr1 ();
  p18_tmr2 ();
  p18_tmr3 ();
  p18_uart ();
}

int
PIC18F452_getconf(unsigned int cfg)
{
 switch (cfg)
  {
  case CFG_MCLR:
   return 1; 
   break;
  case CFG_WDT:
   return ((pic->config[1] & 0x0100) > 0);
   break;
  case CFG_DEBUG:
   return ((pic->config[3] & 0x0080) == 0);
   break;
  }
 return 0;
}

void
PIC18F452_start(void)
{
 pic->ROMSIZE = 16384;
 pic->EEPROMSIZE = 256;
 pic->RAMSIZE = 4096;
 pic->PINCOUNT = 40;
 pic->IDSIZE = 8;
 pic->CONFIGSIZE = 7;
 pic->STACKSIZE = 31;
 pic->CCPCOUNT = 2;
 pic->ADCCOUNT = 8;
 pic->WDT_MS = 18;
 pic->reset = PIC18F452_reset;
 pic->mmap = PIC18F452_map;
 pic->getconf = PIC18F452_getconf;
 pic->periferic = PIC18F452_periferic;
}

