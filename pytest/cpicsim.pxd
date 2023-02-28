# cython: profile=False
# distutils: language = c++
# cython: embedsignature = True
# cython: language_level = 3

cdef extern from "picsim.h":
    cdef int PMAX

    cdef int PD_OUT
    cdef int PD_IN

    cdef int PT_POWER
    cdef int PT_DIGITAL
    cdef int PT_ANALOG
    cdef int PT_ANAREF
    cdef int PT_USART
    cdef int PT_NC

    cdef int P16
    cdef int P18
    cdef int P16E

    int P16F84A
    int P16F777
    int P16F18855
    int P18F452

    cdef int CFG_MCLR
    cdef int CFG_WDT
    cdef int CFG_DEBUG
    cdef int CFG_WDT_DIV

    cdef int PDIP
    cdef int QFN

    cdef int HEX_OK
    cdef int HEX_NFOUND
    cdef int HEX_CHKSUM
    cdef int HEX_NWRITE

    cdef int P_VDD
    cdef int P_VSS
    cdef int P_RST
    cdef int P_OSC
    cdef int P_USB
    cdef int P_NC

    ctypedef struct bb_uart_t:
        unsigned char prx
        unsigned short insr
        unsigned short outsr
        unsigned int bcr
        unsigned long tcountr
        unsigned int bcw
        unsigned long tcountw
        unsigned long speed
        unsigned int cycle_count
        unsigned int rxc
        unsigned int leds
        unsigned char datar
        unsigned char data_recv
        unsigned char dataw
        unsigned char data_to_send
        unsigned long freq

    void bb_uart_rst(bb_uart_t* bu)

    void bb_uart_init(bb_uart_t* bu)

    void bb_uart_end(bb_uart_t* bu)

    void bb_uart_set_clk_freq(bb_uart_t* bu, unsigned long freq)

    void bb_uart_set_speed(bb_uart_t* bu, unsigned int speed)

    unsigned char bb_uart_transmitting(bb_uart_t* bu)

    void bb_uart_send(bb_uart_t* bu, unsigned char data)

    unsigned char bb_uart_data_available(bb_uart_t* bu)

    unsigned char bb_uart_recv(bb_uart_t* bu)

    void bb_uart_open(bb_uart_t* bu)

    unsigned char bb_uart_io(bb_uart_t* bu, unsigned char rx)

    ctypedef struct P16map_t:
        unsigned char* OPTION_REG
        unsigned char* INDF
        unsigned char* STATUS
        unsigned char* FSR
        unsigned char* PCL
        unsigned char* PCLATH
        unsigned char* INTCON
        unsigned char* PIR1
        unsigned char* PIR2
        unsigned char* PIE1
        unsigned char* PIE2
        unsigned char* PORTA
        unsigned char* PORTB
        unsigned char* PORTC
        unsigned char* PORTD
        unsigned char* PORTE
        unsigned char* TRISA
        unsigned char* TRISB
        unsigned char* TRISC
        unsigned char* TRISD
        unsigned char* TRISE
        unsigned char* TXSTA
        unsigned char* RCSTA
        unsigned char* TXREG
        unsigned char* RCREG
        unsigned char* SPBRG
        unsigned char* SPBRGH
        unsigned char* TMR0
        unsigned char* T1CON
        unsigned char* TMR1L
        unsigned char* TMR1H
        unsigned char* CCP1CON
        unsigned char* CCP2CON
        unsigned char* CCP3CON
        unsigned char* CCPR1L
        unsigned char* CCPR1H
        unsigned char* CCPR2L
        unsigned char* CCPR2H
        unsigned char* CCPR3L
        unsigned char* CCPR3H
        unsigned char* TMR2
        unsigned char* T2CON
        unsigned char* PR2
        unsigned char* SSPCON
        unsigned char* SSPCON2
        unsigned char* SSPBUF
        unsigned char* SSPSTAT
        unsigned char* SSPADD
        unsigned char* ADCON0
        unsigned char* ADCON1
        unsigned char* ADCON2
        unsigned char* ADRESL
        unsigned char* ADRESH
        unsigned char* ANSEL
        unsigned char* ANSELH
        unsigned char* EECON1
        unsigned char* EECON2
        unsigned char* EEDATA
        unsigned char* EEADR
        unsigned char* EEDATH
        unsigned char* EEADRH
        unsigned char* BAUDCTL

    ctypedef struct P16Emap_t:
        unsigned char* INDF0
        unsigned char* INDF1
        unsigned char* PCL
        unsigned char* STATUS
        unsigned char* FSR0L
        unsigned char* FSR0H
        unsigned char* FSR1L
        unsigned char* FSR1H
        unsigned char* BSR
        unsigned char* WREG
        unsigned char* PCLATH
        unsigned char* INTCON
        unsigned char* STATUS_SHAD
        unsigned char* WREG_SHAD
        unsigned char* BSR_SHAD
        unsigned char* PCLATH_SHAD
        unsigned char* FSR0L_SHAD
        unsigned char* FSR0H_SHAD
        unsigned char* FSR1L_SHAD
        unsigned char* FSR1H_SHAD
        unsigned char* STKPTR
        unsigned char* TOSL
        unsigned char* TOSH
        unsigned char* OPTION_REG
        unsigned char* PIR0
        unsigned char* PIR1
        unsigned char* PIR2
        unsigned char* PIE0
        unsigned char* PIE1
        unsigned char* PIE2
        unsigned char* PORTA
        unsigned char* PORTB
        unsigned char* PORTC
        unsigned char* PORTD
        unsigned char* PORTE
        unsigned char* TRISA
        unsigned char* TRISB
        unsigned char* TRISC
        unsigned char* TRISD
        unsigned char* TRISE
        unsigned char* LATA
        unsigned char* LATB
        unsigned char* LATC
        unsigned char* LATD
        unsigned char* LATE
        unsigned char* ANSELA
        unsigned char* ANSELB
        unsigned char* ANSELC
        unsigned char* ANSELD
        unsigned char* ANSELE
        unsigned char* RC1REG
        unsigned char* TX1REG
        unsigned char* RC1STA
        unsigned char* TX1STA
        unsigned char* SP1BRGL
        unsigned char* SP1BRGH
        unsigned char* BAUDCON1
        unsigned char* TXSTA
        unsigned char* RCSTA
        unsigned char* TXREG
        unsigned char* RCREG
        unsigned char* TMR0
        unsigned char* T0CON0
        unsigned char* T0CON1
        unsigned char* TMR0L
        unsigned char* TMR0H
        unsigned char* T1CON
        unsigned char* TMR1L
        unsigned char* TMR1H
        unsigned char* CCP1CON
        unsigned char* CCP2CON
        unsigned char* CCPR1L
        unsigned char* CCPR1H
        unsigned char* CCPR2L
        unsigned char* CCPR2H
        unsigned char* T2CON
        unsigned char* TMR2
        unsigned char* PR2
        unsigned char* ADCON0
        unsigned char* ADCON1
        unsigned char* ADRESL
        unsigned char* ADRESH
        unsigned char* ADPCH
        unsigned char* WDTCON0
        unsigned char* WDTCON
        unsigned char* EECON2
        unsigned char* EECON1
        unsigned char* EEADRL
        unsigned char* EEADRH
        unsigned char* EEDATL
        unsigned char* EEDATH
        unsigned char* NVMCON1
        unsigned char* NVMCON2
        unsigned char* NVMDATL
        unsigned char* NVMDATH
        unsigned char* NVMADRL
        unsigned char* NVMADRH
        unsigned char* SSP1STAT
        unsigned char* SSP1CON2
        unsigned char* SSP1CON1
        unsigned char* SSP1BUF
        unsigned char* SSP1ADD
        unsigned char* IOCBP
        unsigned char* IOCBN
        unsigned char* IOCBF
        unsigned char* RA0PPS
        unsigned char* RA1PPS
        unsigned char* RA2PPS
        unsigned char* RA3PPS
        unsigned char* RA4PPS
        unsigned char* RA5PPS
        unsigned char* RA6PPS
        unsigned char* RA7PPS
        unsigned char* RB0PPS
        unsigned char* RB1PPS
        unsigned char* RB2PPS
        unsigned char* RB3PPS
        unsigned char* RB4PPS
        unsigned char* RB5PPS
        unsigned char* RB6PPS
        unsigned char* RB7PPS
        unsigned char* RC0PPS
        unsigned char* RC1PPS
        unsigned char* RC2PPS
        unsigned char* RC3PPS
        unsigned char* RC4PPS
        unsigned char* RC5PPS
        unsigned char* RC6PPS
        unsigned char* RC7PPS
        unsigned char* RD0PPS
        unsigned char* RD1PPS
        unsigned char* RD2PPS
        unsigned char* RD3PPS
        unsigned char* RD4PPS
        unsigned char* RD5PPS
        unsigned char* RD6PPS
        unsigned char* RD7PPS
        unsigned char* RE0PPS
        unsigned char* RE1PPS
        unsigned char* RE2PPS

    ctypedef struct P18map_t:
        unsigned char WS
        unsigned char STATUSS
        unsigned char BSRS
        unsigned char* STATUS
        unsigned char* PLUSW2
        unsigned char* PREINC2
        unsigned char* POSTDEC2
        unsigned char* POSTINC2
        unsigned char* INDF2
        unsigned char* BSR
        unsigned char* PLUSW1
        unsigned char* PREINC1
        unsigned char* POSTDEC1
        unsigned char* POSTINC1
        unsigned char* INDF1
        unsigned char* WREG
        unsigned char* PLUSW0
        unsigned char* PREINC0
        unsigned char* POSTDEC0
        unsigned char* POSTINC0
        unsigned char* INDF0
        unsigned char* INTCON3
        unsigned char* INTCON2
        unsigned char* INTCON
        unsigned char* TABLAT
        unsigned char* STKPTR
        unsigned char* FSR2L
        unsigned char* FSR2H
        unsigned char* FSR1L
        unsigned char* FSR1H
        unsigned char* FSR0L
        unsigned char* FSR0H
        unsigned char* PRODL
        unsigned char* PRODH
        unsigned char* TBLPTRL
        unsigned char* TBLPTRH
        unsigned char* TBLPTRU
        unsigned char* PCL
        unsigned char* PCLATH
        unsigned char* PCLATU
        unsigned char* TOSL
        unsigned char* TOSH
        unsigned char* TOSU
        unsigned char* RCON
        unsigned char* PIE0
        unsigned char* PIR0
        unsigned char* IPR0
        unsigned char* PIE1
        unsigned char* PIR1
        unsigned char* IPR1
        unsigned char* PIE2
        unsigned char* PIR2
        unsigned char* IPR2
        unsigned char* PIE3
        unsigned char* PIR3
        unsigned char* IPR3
        unsigned char* PIE4
        unsigned char* PIR4
        unsigned char* IPR4
        unsigned char* PIE5
        unsigned char* PIR5
        unsigned char* IPR5
        unsigned char* PIE6
        unsigned char* PIR6
        unsigned char* IPR6
        unsigned char* PORTA
        unsigned char* PORTB
        unsigned char* PORTC
        unsigned char* PORTD
        unsigned char* PORTE
        unsigned char* PORTF
        unsigned char* PORTG
        unsigned char* LATA
        unsigned char* LATB
        unsigned char* LATC
        unsigned char* LATD
        unsigned char* LATE
        unsigned char* LATF
        unsigned char* LATG
        unsigned char* TRISA
        unsigned char* TRISB
        unsigned char* TRISC
        unsigned char* TRISD
        unsigned char* TRISE
        unsigned char* TRISF
        unsigned char* TRISG
        unsigned char* ANSELA
        unsigned char* ANSELB
        unsigned char* ANSELC
        unsigned char* ANSELD
        unsigned char* ANSELE
        unsigned char* IOCBF
        unsigned char* IOCBN
        unsigned char* IOCBP
        unsigned char* ADCON2
        unsigned char* ADCON1
        unsigned char* ADCON0
        unsigned char* ADRESL
        unsigned char* ADRESH
        unsigned char* ADCON3
        unsigned char* ADPCH
        unsigned char* T0CON
        unsigned char* TMR0L
        unsigned char* TMR0H
        unsigned char* T0CON0
        unsigned char* T0CON1
        unsigned char* T1CON
        unsigned char* TMR1L
        unsigned char* TMR1H
        unsigned char* T1CLK
        unsigned char* T3CON
        unsigned char* TMR3L
        unsigned char* TMR3H
        unsigned char* CCP2CON
        unsigned char* CCP1CON
        unsigned char* CCPR2L
        unsigned char* CCPR2H
        unsigned char* CCPR1L
        unsigned char* CCPR1H
        unsigned char* SSPCON1
        unsigned char* SSPCON2
        unsigned char* SSPSTAT
        unsigned char* SSPADD
        unsigned char* SSPBUF
        unsigned char* SSP1CON1
        unsigned char* SSP1CON2
        unsigned char* SSP1STAT
        unsigned char* SSP1ADD
        unsigned char* SSP1BUF
        unsigned char* EECON1
        unsigned char* EECON2
        unsigned char* EEDATA
        unsigned char* EEADR
        unsigned char* EEADRH
        unsigned char* NVMCON1
        unsigned char* NVMCON2
        unsigned char* NVMDAT
        unsigned char* NVMADRL
        unsigned char* NVMADRH
        unsigned char* RCREG
        unsigned char* TXREG
        unsigned char* SPBRG
        unsigned char* SPBRGH
        unsigned char* RCSTA
        unsigned char* TXSTA
        unsigned char* BAUDCON
        unsigned char* RCREG1
        unsigned char* TXREG1
        unsigned char* SPBRG1
        unsigned char* SPBRGH1
        unsigned char* RCSTA1
        unsigned char* TXSTA1
        unsigned char* RCREG2
        unsigned char* TXREG2
        unsigned char* SPBRG2
        unsigned char* SPBRGH2
        unsigned char* RCSTA2
        unsigned char* TXSTA2
        unsigned char* RC1REG
        unsigned char* TX1REG
        unsigned char* SP1BRGL
        unsigned char* SP1BRGH
        unsigned char* RC1STA
        unsigned char* TX1STA
        unsigned char* BAUDCON1
        unsigned char* RC2REG
        unsigned char* TX2REG
        unsigned char* SP2BRGL
        unsigned char* SP2BRGH
        unsigned char* RC2STA
        unsigned char* TX2STA
        unsigned char* BAUDCON2
        unsigned char* T2CON
        unsigned char* PR2
        unsigned char* TMR2
        unsigned char* T2TMR
        unsigned char* T2PR
        unsigned char* WDTCON
        unsigned char* WDTCON0
        unsigned char* RA0PPS
        unsigned char* RA1PPS
        unsigned char* RA2PPS
        unsigned char* RA3PPS
        unsigned char* RA4PPS
        unsigned char* RA5PPS
        unsigned char* RA6PPS
        unsigned char* RA7PPS
        unsigned char* RB0PPS
        unsigned char* RB1PPS
        unsigned char* RB2PPS
        unsigned char* RB3PPS
        unsigned char* RB4PPS
        unsigned char* RB5PPS
        unsigned char* RB6PPS
        unsigned char* RB7PPS
        unsigned char* RC0PPS
        unsigned char* RC1PPS
        unsigned char* RC2PPS
        unsigned char* RC3PPS
        unsigned char* RC4PPS
        unsigned char* RC5PPS
        unsigned char* RC6PPS
        unsigned char* RC7PPS
        unsigned char* RD0PPS
        unsigned char* RD1PPS
        unsigned char* RD2PPS
        unsigned char* RD3PPS
        unsigned char* RD4PPS
        unsigned char* RD5PPS
        unsigned char* RD6PPS
        unsigned char* RD7PPS
        unsigned char* RE0PPS
        unsigned char* RE1PPS
        unsigned char* RE2PPS

    ctypedef struct picpin:
        unsigned char ptype
        unsigned char dir
        unsigned char value
        unsigned char lvalue
        char pord
        unsigned char* port
        float avalue
        unsigned char ovalue
        float oavalue
        unsigned char lsvalue

    ctypedef struct ccp_t:
        unsigned char pin
        unsigned char ovalue
        unsigned char cap_ps

    ctypedef struct _serial:
        unsigned char recb
        int serialc
        unsigned char txtemp[2]
        char txtc
        unsigned char RCREG2
        char SERIALDEVICE[100]
        unsigned int serialbaud
        float serialexbaud
        int serialfd
        int s_open
        int flowcontrol
        int ctspin
        int rtspin
        unsigned char buff[8192]
        int bc
        unsigned char* serial_TXSTA
        unsigned char* serial_PIR
        unsigned char* serial_PIE
        unsigned char RXIF_mask
        unsigned char TXIF_mask
        unsigned char* serial_RCSTA
        unsigned char* serial_SPBRG
        unsigned char* serial_SPBRGH
        unsigned char* serial_BAUDCTL
        unsigned char* serial_RCREG
        unsigned char* serial_TXREG
        unsigned short serial_TXREG_ADDR
        unsigned short serial_RCSTA_ADDR
        unsigned short serial_RCREG_ADDR
        unsigned char* serial_TRIS_RX
        unsigned char serial_TRIS_RX_MASK
        bb_uart_t bbuart

    unsigned char NO_IO[6]

    ctypedef void (*__pic_reset_ft)(_pic* pic)

    ctypedef void (*__pic_mmap_ft)(_pic* pic)

    ctypedef int (*__pic_getconf_ft)(_pic* pic, unsigned int)

    ctypedef void (*__pic_periferic_ft)(_pic* pic)

    ctypedef int (*__pic_interrupt_ft)(_pic* pic)

    ctypedef void (*__pic_stop_ft)(_pic* pic)

    cdef struct _pic:
        unsigned char print_ "print"
        unsigned int RAMSIZE
        unsigned int ROMSIZE
        unsigned int EEPROMSIZE
        unsigned int IDSIZE
        unsigned int CONFIGSIZE
        unsigned char STACKSIZE
        unsigned char PINCOUNT
        unsigned char CCPCOUNT
        unsigned char ADCCOUNT
        unsigned char USARTCOUNT
        unsigned char WDT_MS
        unsigned char family
        unsigned int processor
        unsigned short debugv[8]
        unsigned char* ram
        unsigned short* prog
        unsigned short* id
        unsigned char* eeprom
        unsigned short* config
        unsigned int pc
        unsigned int jpc
        unsigned short lram
        unsigned short rram
        unsigned int* stack
        unsigned char w
        unsigned char wdt
        unsigned char s2
        unsigned long long cycles
        float freq
        unsigned char sleep
        unsigned char pkg
        picpin* pins
        unsigned char mclr
        ccp_t* ccp
        unsigned char* adc
        unsigned char* usart_rx
        unsigned char* usart_tx
        unsigned char pgc
        unsigned char pgd
        unsigned char sck
        unsigned char sdo
        unsigned char sdi
        unsigned char t0cki
        unsigned char t1cki
        unsigned char intlv
        unsigned char ocd_pgca
        unsigned char ocd_pgc
        unsigned char debug
        unsigned char sstep
        unsigned char dbg
        unsigned char frst
        int cp0
        int cp1
        int cp2
        int cp2_
        int cp3
        int t2pr
        unsigned char t0cki_
        unsigned char t1cki_
        unsigned char int0
        unsigned char int1
        unsigned char int2
        unsigned char int0v
        unsigned char int1v
        unsigned char int2v
        unsigned char portbm
        float vcc
        int adcstep
        unsigned char adcon1
        float twdt
        int ee_wr
        unsigned char p16latch[5]
        unsigned char porta
        unsigned char portb
        unsigned char portc
        unsigned char portd
        unsigned char porte
        unsigned char portf
        unsigned char portg
        unsigned char trisa
        unsigned char trisb
        unsigned char trisc
        unsigned char trisd
        unsigned char trise
        unsigned char trisf
        unsigned char trisg
        unsigned char ioupdated
        unsigned char ssp_ck
        unsigned char ssp_sck
        unsigned char ssp_scka
        unsigned char sspsr
        unsigned char ssp_bit
        _serial serial[2]
        __pic_reset_ft reset
        __pic_mmap_ft mmap
        __pic_getconf_ft getconf
        __pic_periferic_ft periferic
        __pic_interrupt_ft interrupt
        __pic_stop_ft stop
        P16map_t P16map
        P16Emap_t P16Emap
        P18map_t P18map

    int pic_set_serial(_pic* pic_, int nser, const char* name, int flowcontrol, int ctspin, int rtspin)

    int pic_init(_pic* pic_, int processor, const char* fname, int leeprom, float freq)

    int pic_reset(_pic* pic_, int flags)

    void pic_erase_flash(_pic* pic_)

    void pic_step(_pic* pic_)

    void pic_end(_pic* pic_)

    unsigned char pic_get_pin(_pic* pic_, unsigned char pin)

    int pic_set_pin(_pic* pic_, unsigned char pin, unsigned char value)

    int pic_set_apin(_pic* pic_, unsigned char pin, float value)

    unsigned char pic_get_pin_type(_pic* pic_, unsigned char pin)

    unsigned char pic_get_pin_dir(_pic* pic_, unsigned char pin)

    unsigned char pic_get_pin_DOV(_pic* pic_, unsigned char pin)

    int pic_set_pin_DOV(_pic* pic_, unsigned char pin, unsigned char value)

    void pic_icsp_init(_pic* pic_)

    int pic_icsp(_pic* pic_)

    void pic_icsp_init18(_pic* pic_)

    int pic_icsp18(_pic* pic_)

    int read_ihx(_pic* pic_, const char* fname, int leeprom)

    int read_ihx_16e(_pic* pic_, const char* fname, int leeprom)

    int read_ihx_18(_pic* pic_, const char* fname, int leeprom)

    int write_ihx(_pic* pic_, const char* fname)

    int write_ihx16e(_pic* pic_, const char* fname)

    int write_ihx18(_pic* pic_, const char* fname)

    int getproclist(char list[][30], int size)

    unsigned int getprocbyname(const char* str)

    unsigned int getfprocbyname(const char* str)

    unsigned int getfprocbynumber(int proc)

    char* getnamebyproc(int proc, char* str)

    const char* getFSRname_16(unsigned int addr)

    const char* getFSRname_16E(unsigned int addr)

    const char* getFSRname_16E2(unsigned int addr)

    const char* getFSRname_18(unsigned int addr)

    const char* getPinName(_pic* pic, int pin, char* pname)

    ctypedef void (*_pic_desc_pic_desc_start_ft)(_pic* pic)

    ctypedef struct pic_desc:
        unsigned short ID
        char name[30]
        unsigned char family
        _pic_desc_pic_desc_start_ft start

    pic_desc PICS[25]

    int PIC_count

    void pic_register(pic_desc picd)
