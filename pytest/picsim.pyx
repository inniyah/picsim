# cython: profile=False
# cython: embedsignature = True
# cython: language_level = 3
# distutils: language = c++

# C-Import Cython Definitions

from libc.stdint cimport uint64_t, uint32_t, uint16_t, uint8_t, int64_t, int32_t, int16_t, int8_t
from libc.stdlib cimport calloc, malloc, free
from libc.string cimport memset, strcmp
from libc.stdio cimport printf

from libcpp cimport bool
from libcpp.memory cimport unique_ptr, shared_ptr, make_shared, allocator
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.utility cimport pair

from cpython cimport array
from cpython.ref cimport PyObject
from cpython.mem cimport PyMem_Malloc, PyMem_Realloc, PyMem_Free
from cpython.bytes cimport PyBytes_FromStringAndSize

from cython.operator cimport dereference as deref

from enum import IntEnum

# Import Python Modules

import array
import numbers

# C-Import Custom Cython Definitions

cimport cpicsim

P16F84A   = cpicsim.P16F84A
P16F777   = cpicsim.P16F777
P16F18855 = cpicsim.P16F18855
P18F452   = cpicsim.P18F452

def getprocbyname(name : str) -> int:
    return cpicsim.getprocbyname(name.encode('utf8'))

def getfprocbyname(name : str) -> int:
    return cpicsim.getfprocbyname(name.encode('utf8'))

def getproclist():
    proclist = []
    cdef char[30][30] plist
    pc = cpicsim.getproclist(plist, cpicsim.PMAX)
    cdef const char * pname
    for i in range(pc):
        pname = &plist[i][0]
        proclist.append(pname.decode('utf8'))
    return proclist

cdef class P16map_:
    cdef cpicsim.P16map_t * ptr
    def __cinit__(self):
        self.ptr = NULL
    @staticmethod
    cdef create(cpicsim.P16map_t * ptr):
        cdef P16map_ m = P16map_()
        m.ptr = ptr
        return m
    def getOPTION_REG(self) -> uint8_t:
        return self.ptr.OPTION_REG
    def getINDF(self) -> uint8_t:
        return self.ptr.INDF
    def getSTATUS(self) -> uint8_t:
        return self.ptr.STATUS
    def getFSR(self) -> uint8_t:
        return self.ptr.FSR
    def getPCL(self) -> uint8_t:
        return self.ptr.PCL
    def getPCLATH(self) -> uint8_t:
        return self.ptr.PCLATH
    def getINTCON(self) -> uint8_t:
        return self.ptr.INTCON
    def getPIR1(self) -> uint8_t:
        return self.ptr.PIR1
    def getPIR2(self) -> uint8_t:
        return self.ptr.PIR2
    def getPIE1(self) -> uint8_t:
        return self.ptr.PIE1
    def getPIE2(self) -> uint8_t:
        return self.ptr.PIE2
    def getPORTA(self) -> uint8_t:
        return self.ptr.PORTA
    def getPORTB(self) -> uint8_t:
        return self.ptr.PORTB
    def getPORTC(self) -> uint8_t:
        return self.ptr.PORTC
    def getPORTD(self) -> uint8_t:
        return self.ptr.PORTD
    def getPORTE(self) -> uint8_t:
        return self.ptr.PORTE
    def getTRISA(self) -> uint8_t:
        return self.ptr.TRISA
    def getTRISB(self) -> uint8_t:
        return self.ptr.TRISB
    def getTRISC(self) -> uint8_t:
        return self.ptr.TRISC
    def getTRISD(self) -> uint8_t:
        return self.ptr.TRISD
    def getTRISE(self) -> uint8_t:
        return self.ptr.TRISE
    def getTXSTA(self) -> uint8_t:
        return self.ptr.TXSTA
    def getRCSTA(self) -> uint8_t:
        return self.ptr.RCSTA
    def getTXREG(self) -> uint8_t:
        return self.ptr.TXREG
    def getRCREG(self) -> uint8_t:
        return self.ptr.RCREG
    def getSPBRG(self) -> uint8_t:
        return self.ptr.SPBRG
    def getSPBRGH(self) -> uint8_t:
        return self.ptr.SPBRGH
    def getTMR0(self) -> uint8_t:
        return self.ptr.TMR0
    def getT1CON(self) -> uint8_t:
        return self.ptr.T1CON
    def getTMR1L(self) -> uint8_t:
        return self.ptr.TMR1L
    def getTMR1H(self) -> uint8_t:
        return self.ptr.TMR1H
    def getCCP1CON(self) -> uint8_t:
        return self.ptr.CCP1CON
    def getCCP2CON(self) -> uint8_t:
        return self.ptr.CCP2CON
    def getCCP3CON(self) -> uint8_t:
        return self.ptr.CCP3CON
    def getCCPR1L(self) -> uint8_t:
        return self.ptr.CCPR1L
    def getCCPR1H(self) -> uint8_t:
        return self.ptr.CCPR1H
    def getCCPR2L(self) -> uint8_t:
        return self.ptr.CCPR2L
    def getCCPR2H(self) -> uint8_t:
        return self.ptr.CCPR2H
    def getCCPR3L(self) -> uint8_t:
        return self.ptr.CCPR3L
    def getCCPR3H(self) -> uint8_t:
        return self.ptr.CCPR3H
    def getTMR2(self) -> uint8_t:
        return self.ptr.TMR2
    def getT2CON(self) -> uint8_t:
        return self.ptr.T2CON
    def getPR2(self) -> uint8_t:
        return self.ptr.PR2
    def getSSPCON(self) -> uint8_t:
        return self.ptr.SSPCON
    def getSSPCON2(self) -> uint8_t:
        return self.ptr.SSPCON2
    def getSSPBUF(self) -> uint8_t:
        return self.ptr.SSPBUF
    def getSSPSTAT(self) -> uint8_t:
        return self.ptr.SSPSTAT
    def getSSPADD(self) -> uint8_t:
        return self.ptr.SSPADD
    def getADCON0(self) -> uint8_t:
        return self.ptr.ADCON0
    def getADCON1(self) -> uint8_t:
        return self.ptr.ADCON1
    def getADCON2(self) -> uint8_t:
        return self.ptr.ADCON2
    def getADRESL(self) -> uint8_t:
        return self.ptr.ADRESL
    def getADRESH(self) -> uint8_t:
        return self.ptr.ADRESH
    def getANSEL(self) -> uint8_t:
        return self.ptr.ANSEL
    def getANSELH(self) -> uint8_t:
        return self.ptr.ANSELH
    def getEECON1(self) -> uint8_t:
        return self.ptr.EECON1
    def getEECON2(self) -> uint8_t:
        return self.ptr.EECON2
    def getEEDATA(self) -> uint8_t:
        return self.ptr.EEDATA
    def getEEADR(self) -> uint8_t:
        return self.ptr.EEADR
    def getEEDATH(self) -> uint8_t:
        return self.ptr.EEDATH
    def getEEADRH(self) -> uint8_t:
        return self.ptr.EEADRH
    def getBAUDCTL(self) -> uint8_t:
        return self.ptr.BAUDCTL

cdef class P16Emap_:
    cdef cpicsim.P16Emap_t * ptr
    def __cinit__(self):
        self.ptr = NULL
    @staticmethod
    cdef create(cpicsim.P16Emap_t * ptr):
        cdef P16Emap_ m = P16Emap_()
        m.ptr = ptr
        return m
    def getINDF0(self) -> uint8_t:
        return self.ptr.INDF0
    def getINDF1(self) -> uint8_t:
        return self.ptr.INDF1
    def getPCL(self) -> uint8_t:
        return self.ptr.PCL
    def getSTATUS(self) -> uint8_t:
        return self.ptr.STATUS
    def getFSR0L(self) -> uint8_t:
        return self.ptr.FSR0L
    def getFSR0H(self) -> uint8_t:
        return self.ptr.FSR0H
    def getFSR1L(self) -> uint8_t:
        return self.ptr.FSR1L
    def getFSR1H(self) -> uint8_t:
        return self.ptr.FSR1H
    def getBSR(self) -> uint8_t:
        return self.ptr.BSR
    def getWREG(self) -> uint8_t:
        return self.ptr.WREG
    def getPCLATH(self) -> uint8_t:
        return self.ptr.PCLATH
    def getINTCON(self) -> uint8_t:
        return self.ptr.INTCON
    def getSTATUS_SHAD(self) -> uint8_t:
        return self.ptr.STATUS_SHAD
    def getWREG_SHAD(self) -> uint8_t:
        return self.ptr.WREG_SHAD
    def getBSR_SHAD(self) -> uint8_t:
        return self.ptr.BSR_SHAD
    def getPCLATH_SHAD(self) -> uint8_t:
        return self.ptr.PCLATH_SHAD
    def getFSR0L_SHAD(self) -> uint8_t:
        return self.ptr.FSR0L_SHAD
    def getFSR0H_SHAD(self) -> uint8_t:
        return self.ptr.FSR0H_SHAD
    def getFSR1L_SHAD(self) -> uint8_t:
        return self.ptr.FSR1L_SHAD
    def getFSR1H_SHAD(self) -> uint8_t:
        return self.ptr.FSR1H_SHAD
    def getSTKPTR(self) -> uint8_t:
        return self.ptr.STKPTR
    def getTOSL(self) -> uint8_t:
        return self.ptr.TOSL
    def getTOSH(self) -> uint8_t:
        return self.ptr.TOSH
    def getOPTION_REG(self) -> uint8_t:
        return self.ptr.OPTION_REG
    def getPIR0(self) -> uint8_t:
        return self.ptr.PIR0
    def getPIR1(self) -> uint8_t:
        return self.ptr.PIR1
    def getPIR2(self) -> uint8_t:
        return self.ptr.PIR2
    def getPIE0(self) -> uint8_t:
        return self.ptr.PIE0
    def getPIE1(self) -> uint8_t:
        return self.ptr.PIE1
    def getPIE2(self) -> uint8_t:
        return self.ptr.PIE2
    def getPORTA(self) -> uint8_t:
        return self.ptr.PORTA
    def getPORTB(self) -> uint8_t:
        return self.ptr.PORTB
    def getPORTC(self) -> uint8_t:
        return self.ptr.PORTC
    def getPORTD(self) -> uint8_t:
        return self.ptr.PORTD
    def getPORTE(self) -> uint8_t:
        return self.ptr.PORTE
    def getTRISA(self) -> uint8_t:
        return self.ptr.TRISA
    def getTRISB(self) -> uint8_t:
        return self.ptr.TRISB
    def getTRISC(self) -> uint8_t:
        return self.ptr.TRISC
    def getTRISD(self) -> uint8_t:
        return self.ptr.TRISD
    def getTRISE(self) -> uint8_t:
        return self.ptr.TRISE
    def getLATA(self) -> uint8_t:
        return self.ptr.LATA
    def getLATB(self) -> uint8_t:
        return self.ptr.LATB
    def getLATC(self) -> uint8_t:
        return self.ptr.LATC
    def getLATD(self) -> uint8_t:
        return self.ptr.LATD
    def getLATE(self) -> uint8_t:
        return self.ptr.LATE
    def getANSELA(self) -> uint8_t:
        return self.ptr.ANSELA
    def getANSELB(self) -> uint8_t:
        return self.ptr.ANSELB
    def getANSELC(self) -> uint8_t:
        return self.ptr.ANSELC
    def getANSELD(self) -> uint8_t:
        return self.ptr.ANSELD
    def getANSELE(self) -> uint8_t:
        return self.ptr.ANSELE
    def getRC1REG(self) -> uint8_t:
        return self.ptr.RC1REG
    def getTX1REG(self) -> uint8_t:
        return self.ptr.TX1REG
    def getRC1STA(self) -> uint8_t:
        return self.ptr.RC1STA
    def getTX1STA(self) -> uint8_t:
        return self.ptr.TX1STA
    def getSP1BRGL(self) -> uint8_t:
        return self.ptr.SP1BRGL
    def getSP1BRGH(self) -> uint8_t:
        return self.ptr.SP1BRGH
    def getBAUDCON1(self) -> uint8_t:
        return self.ptr.BAUDCON1
    def getTXSTA(self) -> uint8_t:
        return self.ptr.TXSTA
    def getRCSTA(self) -> uint8_t:
        return self.ptr.RCSTA
    def getTXREG(self) -> uint8_t:
        return self.ptr.TXREG
    def getRCREG(self) -> uint8_t:
        return self.ptr.RCREG
    def getTMR0(self) -> uint8_t:
        return self.ptr.TMR0
    def getT0CON0(self) -> uint8_t:
        return self.ptr.T0CON0
    def getT0CON1(self) -> uint8_t:
        return self.ptr.T0CON1
    def getTMR0L(self) -> uint8_t:
        return self.ptr.TMR0L
    def getTMR0H(self) -> uint8_t:
        return self.ptr.TMR0H
    def getT1CON(self) -> uint8_t:
        return self.ptr.T1CON
    def getTMR1L(self) -> uint8_t:
        return self.ptr.TMR1L
    def getTMR1H(self) -> uint8_t:
        return self.ptr.TMR1H
    def getCCP1CON(self) -> uint8_t:
        return self.ptr.CCP1CON
    def getCCP2CON(self) -> uint8_t:
        return self.ptr.CCP2CON
    def getCCPR1L(self) -> uint8_t:
        return self.ptr.CCPR1L
    def getCCPR1H(self) -> uint8_t:
        return self.ptr.CCPR1H
    def getCCPR2L(self) -> uint8_t:
        return self.ptr.CCPR2L
    def getCCPR2H(self) -> uint8_t:
        return self.ptr.CCPR2H
    def getT2CON(self) -> uint8_t:
        return self.ptr.T2CON
    def getTMR2(self) -> uint8_t:
        return self.ptr.TMR2
    def getPR2(self) -> uint8_t:
        return self.ptr.PR2
    def getADCON0(self) -> uint8_t:
        return self.ptr.ADCON0
    def getADCON1(self) -> uint8_t:
        return self.ptr.ADCON1
    def getADRESL(self) -> uint8_t:
        return self.ptr.ADRESL
    def getADRESH(self) -> uint8_t:
        return self.ptr.ADRESH
    def getADPCH(self) -> uint8_t:
        return self.ptr.ADPCH
    def getWDTCON0(self) -> uint8_t:
        return self.ptr.WDTCON0
    def getWDTCON(self) -> uint8_t:
        return self.ptr.WDTCON
    def getEECON2(self) -> uint8_t:
        return self.ptr.EECON2
    def getEECON1(self) -> uint8_t:
        return self.ptr.EECON1
    def getEEADRL(self) -> uint8_t:
        return self.ptr.EEADRL
    def getEEADRH(self) -> uint8_t:
        return self.ptr.EEADRH
    def getEEDATL(self) -> uint8_t:
        return self.ptr.EEDATL
    def getEEDATH(self) -> uint8_t:
        return self.ptr.EEDATH
    def getNVMCON1(self) -> uint8_t:
        return self.ptr.NVMCON1
    def getNVMCON2(self) -> uint8_t:
        return self.ptr.NVMCON2
    def getNVMDATL(self) -> uint8_t:
        return self.ptr.NVMDATL
    def getNVMDATH(self) -> uint8_t:
        return self.ptr.NVMDATH
    def getNVMADRL(self) -> uint8_t:
        return self.ptr.NVMADRL
    def getNVMADRH(self) -> uint8_t:
        return self.ptr.NVMADRH
    def getSSP1STAT(self) -> uint8_t:
        return self.ptr.SSP1STAT
    def getSSP1CON2(self) -> uint8_t:
        return self.ptr.SSP1CON2
    def getSSP1CON1(self) -> uint8_t:
        return self.ptr.SSP1CON1
    def getSSP1BUF(self) -> uint8_t:
        return self.ptr.SSP1BUF
    def getSSP1ADD(self) -> uint8_t:
        return self.ptr.SSP1ADD
    def getIOCBP(self) -> uint8_t:
        return self.ptr.IOCBP
    def getIOCBN(self) -> uint8_t:
        return self.ptr.IOCBN
    def getIOCBF(self) -> uint8_t:
        return self.ptr.IOCBF
    def getRA0PPS(self) -> uint8_t:
        return self.ptr.RA0PPS
    def getRA1PPS(self) -> uint8_t:
        return self.ptr.RA1PPS
    def getRA2PPS(self) -> uint8_t:
        return self.ptr.RA2PPS
    def getRA3PPS(self) -> uint8_t:
        return self.ptr.RA3PPS
    def getRA4PPS(self) -> uint8_t:
        return self.ptr.RA4PPS
    def getRA5PPS(self) -> uint8_t:
        return self.ptr.RA5PPS
    def getRA6PPS(self) -> uint8_t:
        return self.ptr.RA6PPS
    def getRA7PPS(self) -> uint8_t:
        return self.ptr.RA7PPS
    def getRB0PPS(self) -> uint8_t:
        return self.ptr.RB0PPS
    def getRB1PPS(self) -> uint8_t:
        return self.ptr.RB1PPS
    def getRB2PPS(self) -> uint8_t:
        return self.ptr.RB2PPS
    def getRB3PPS(self) -> uint8_t:
        return self.ptr.RB3PPS
    def getRB4PPS(self) -> uint8_t:
        return self.ptr.RB4PPS
    def getRB5PPS(self) -> uint8_t:
        return self.ptr.RB5PPS
    def getRB6PPS(self) -> uint8_t:
        return self.ptr.RB6PPS
    def getRB7PPS(self) -> uint8_t:
        return self.ptr.RB7PPS
    def getRC0PPS(self) -> uint8_t:
        return self.ptr.RC0PPS
    def getRC1PPS(self) -> uint8_t:
        return self.ptr.RC1PPS
    def getRC2PPS(self) -> uint8_t:
        return self.ptr.RC2PPS
    def getRC3PPS(self) -> uint8_t:
        return self.ptr.RC3PPS
    def getRC4PPS(self) -> uint8_t:
        return self.ptr.RC4PPS
    def getRC5PPS(self) -> uint8_t:
        return self.ptr.RC5PPS
    def getRC6PPS(self) -> uint8_t:
        return self.ptr.RC6PPS
    def getRC7PPS(self) -> uint8_t:
        return self.ptr.RC7PPS
    def getRD0PPS(self) -> uint8_t:
        return self.ptr.RD0PPS
    def getRD1PPS(self) -> uint8_t:
        return self.ptr.RD1PPS
    def getRD2PPS(self) -> uint8_t:
        return self.ptr.RD2PPS
    def getRD3PPS(self) -> uint8_t:
        return self.ptr.RD3PPS
    def getRD4PPS(self) -> uint8_t:
        return self.ptr.RD4PPS
    def getRD5PPS(self) -> uint8_t:
        return self.ptr.RD5PPS
    def getRD6PPS(self) -> uint8_t:
        return self.ptr.RD6PPS
    def getRD7PPS(self) -> uint8_t:
        return self.ptr.RD7PPS
    def getRE0PPS(self) -> uint8_t:
        return self.ptr.RE0PPS
    def getRE1PPS(self) -> uint8_t:
        return self.ptr.RE1PPS
    def getRE2PPS(self) -> uint8_t:
        return self.ptr.RE2PPS

cdef class P18map_:
    cdef cpicsim.P18map_t * ptr
    def __cinit__(self):
        self.ptr = NULL
    @staticmethod
    cdef create(cpicsim.P18map_t * ptr):
        cdef P18map_ m = P18map_()
        m.ptr = ptr
        return m
    def getWS(self) -> uint8_t:
        return self.ptr.WS
    def getSTATUSS(self) -> uint8_t:
        return self.ptr.STATUSS
    def getBSRS(self) -> uint8_t:
        return self.ptr.BSRS
    def getSTATUS(self) -> uint8_t:
        return self.ptr.STATUS
    def getPLUSW2(self) -> uint8_t:
        return self.ptr.PLUSW2
    def getPREINC2(self) -> uint8_t:
        return self.ptr.PREINC2
    def getPOSTDEC2(self) -> uint8_t:
        return self.ptr.POSTDEC2
    def getPOSTINC2(self) -> uint8_t:
        return self.ptr.POSTINC2
    def getINDF2(self) -> uint8_t:
        return self.ptr.INDF2
    def getBSR(self) -> uint8_t:
        return self.ptr.BSR
    def getPLUSW1(self) -> uint8_t:
        return self.ptr.PLUSW1
    def getPREINC1(self) -> uint8_t:
        return self.ptr.PREINC1
    def getPOSTDEC1(self) -> uint8_t:
        return self.ptr.POSTDEC1
    def getPOSTINC1(self) -> uint8_t:
        return self.ptr.POSTINC1
    def getINDF1(self) -> uint8_t:
        return self.ptr.INDF1
    def getWREG(self) -> uint8_t:
        return self.ptr.WREG
    def getPLUSW0(self) -> uint8_t:
        return self.ptr.PLUSW0
    def getPREINC0(self) -> uint8_t:
        return self.ptr.PREINC0
    def getPOSTDEC0(self) -> uint8_t:
        return self.ptr.POSTDEC0
    def getPOSTINC0(self) -> uint8_t:
        return self.ptr.POSTINC0
    def getINDF0(self) -> uint8_t:
        return self.ptr.INDF0
    def getINTCON3(self) -> uint8_t:
        return self.ptr.INTCON3
    def getINTCON2(self) -> uint8_t:
        return self.ptr.INTCON2
    def getINTCON(self) -> uint8_t:
        return self.ptr.INTCON
    def getTABLAT(self) -> uint8_t:
        return self.ptr.TABLAT
    def getSTKPTR(self) -> uint8_t:
        return self.ptr.STKPTR
    def getFSR2L(self) -> uint8_t:
        return self.ptr.FSR2L
    def getFSR2H(self) -> uint8_t:
        return self.ptr.FSR2H
    def getFSR1L(self) -> uint8_t:
        return self.ptr.FSR1L
    def getFSR1H(self) -> uint8_t:
        return self.ptr.FSR1H
    def getFSR0L(self) -> uint8_t:
        return self.ptr.FSR0L
    def getFSR0H(self) -> uint8_t:
        return self.ptr.FSR0H
    def getPRODL(self) -> uint8_t:
        return self.ptr.PRODL
    def getPRODH(self) -> uint8_t:
        return self.ptr.PRODH
    def getTBLPTRL(self) -> uint8_t:
        return self.ptr.TBLPTRL
    def getTBLPTRH(self) -> uint8_t:
        return self.ptr.TBLPTRH
    def getTBLPTRU(self) -> uint8_t:
        return self.ptr.TBLPTRU
    def getPCL(self) -> uint8_t:
        return self.ptr.PCL
    def getPCLATH(self) -> uint8_t:
        return self.ptr.PCLATH
    def getPCLATU(self) -> uint8_t:
        return self.ptr.PCLATU
    def getTOSL(self) -> uint8_t:
        return self.ptr.TOSL
    def getTOSH(self) -> uint8_t:
        return self.ptr.TOSH
    def getTOSU(self) -> uint8_t:
        return self.ptr.TOSU
    def getRCON(self) -> uint8_t:
        return self.ptr.RCON
    def getPIE0(self) -> uint8_t:
        return self.ptr.PIE0
    def getPIR0(self) -> uint8_t:
        return self.ptr.PIR0
    def getIPR0(self) -> uint8_t:
        return self.ptr.IPR0
    def getPIE1(self) -> uint8_t:
        return self.ptr.PIE1
    def getPIR1(self) -> uint8_t:
        return self.ptr.PIR1
    def getIPR1(self) -> uint8_t:
        return self.ptr.IPR1
    def getPIE2(self) -> uint8_t:
        return self.ptr.PIE2
    def getPIR2(self) -> uint8_t:
        return self.ptr.PIR2
    def getIPR2(self) -> uint8_t:
        return self.ptr.IPR2
    def getPIE3(self) -> uint8_t:
        return self.ptr.PIE3
    def getPIR3(self) -> uint8_t:
        return self.ptr.PIR3
    def getIPR3(self) -> uint8_t:
        return self.ptr.IPR3
    def getPIE4(self) -> uint8_t:
        return self.ptr.PIE4
    def getPIR4(self) -> uint8_t:
        return self.ptr.PIR4
    def getIPR4(self) -> uint8_t:
        return self.ptr.IPR4
    def getPIE5(self) -> uint8_t:
        return self.ptr.PIE5
    def getPIR5(self) -> uint8_t:
        return self.ptr.PIR5
    def getIPR5(self) -> uint8_t:
        return self.ptr.IPR5
    def getPIE6(self) -> uint8_t:
        return self.ptr.PIE6
    def getPIR6(self) -> uint8_t:
        return self.ptr.PIR6
    def getIPR6(self) -> uint8_t:
        return self.ptr.IPR6
    def getPORTA(self) -> uint8_t:
        return self.ptr.PORTA
    def getPORTB(self) -> uint8_t:
        return self.ptr.PORTB
    def getPORTC(self) -> uint8_t:
        return self.ptr.PORTC
    def getPORTD(self) -> uint8_t:
        return self.ptr.PORTD
    def getPORTE(self) -> uint8_t:
        return self.ptr.PORTE
    def getPORTF(self) -> uint8_t:
        return self.ptr.PORTF
    def getPORTG(self) -> uint8_t:
        return self.ptr.PORTG
    def getLATA(self) -> uint8_t:
        return self.ptr.LATA
    def getLATB(self) -> uint8_t:
        return self.ptr.LATB
    def getLATC(self) -> uint8_t:
        return self.ptr.LATC
    def getLATD(self) -> uint8_t:
        return self.ptr.LATD
    def getLATE(self) -> uint8_t:
        return self.ptr.LATE
    def getLATF(self) -> uint8_t:
        return self.ptr.LATF
    def getLATG(self) -> uint8_t:
        return self.ptr.LATG
    def getTRISA(self) -> uint8_t:
        return self.ptr.TRISA
    def getTRISB(self) -> uint8_t:
        return self.ptr.TRISB
    def getTRISC(self) -> uint8_t:
        return self.ptr.TRISC
    def getTRISD(self) -> uint8_t:
        return self.ptr.TRISD
    def getTRISE(self) -> uint8_t:
        return self.ptr.TRISE
    def getTRISF(self) -> uint8_t:
        return self.ptr.TRISF
    def getTRISG(self) -> uint8_t:
        return self.ptr.TRISG
    def getANSELA(self) -> uint8_t:
        return self.ptr.ANSELA
    def getANSELB(self) -> uint8_t:
        return self.ptr.ANSELB
    def getANSELC(self) -> uint8_t:
        return self.ptr.ANSELC
    def getANSELD(self) -> uint8_t:
        return self.ptr.ANSELD
    def getANSELE(self) -> uint8_t:
        return self.ptr.ANSELE
    def getIOCBF(self) -> uint8_t:
        return self.ptr.IOCBF
    def getIOCBN(self) -> uint8_t:
        return self.ptr.IOCBN
    def getIOCBP(self) -> uint8_t:
        return self.ptr.IOCBP
    def getADCON2(self) -> uint8_t:
        return self.ptr.ADCON2
    def getADCON1(self) -> uint8_t:
        return self.ptr.ADCON1
    def getADCON0(self) -> uint8_t:
        return self.ptr.ADCON0
    def getADRESL(self) -> uint8_t:
        return self.ptr.ADRESL
    def getADRESH(self) -> uint8_t:
        return self.ptr.ADRESH
    def getADCON3(self) -> uint8_t:
        return self.ptr.ADCON3
    def getADPCH(self) -> uint8_t:
        return self.ptr.ADPCH
    def getT0CON(self) -> uint8_t:
        return self.ptr.T0CON
    def getTMR0L(self) -> uint8_t:
        return self.ptr.TMR0L
    def getTMR0H(self) -> uint8_t:
        return self.ptr.TMR0H
    def getT0CON0(self) -> uint8_t:
        return self.ptr.T0CON0
    def getT0CON1(self) -> uint8_t:
        return self.ptr.T0CON1
    def getT1CON(self) -> uint8_t:
        return self.ptr.T1CON
    def getTMR1L(self) -> uint8_t:
        return self.ptr.TMR1L
    def getTMR1H(self) -> uint8_t:
        return self.ptr.TMR1H
    def getT1CLK(self) -> uint8_t:
        return self.ptr.T1CLK
    def getT3CON(self) -> uint8_t:
        return self.ptr.T3CON
    def getTMR3L(self) -> uint8_t:
        return self.ptr.TMR3L
    def getTMR3H(self) -> uint8_t:
        return self.ptr.TMR3H
    def getCCP2CON(self) -> uint8_t:
        return self.ptr.CCP2CON
    def getCCP1CON(self) -> uint8_t:
        return self.ptr.CCP1CON
    def getCCPR2L(self) -> uint8_t:
        return self.ptr.CCPR2L
    def getCCPR2H(self) -> uint8_t:
        return self.ptr.CCPR2H
    def getCCPR1L(self) -> uint8_t:
        return self.ptr.CCPR1L
    def getCCPR1H(self) -> uint8_t:
        return self.ptr.CCPR1H
    def getSSPCON1(self) -> uint8_t:
        return self.ptr.SSPCON1
    def getSSPCON2(self) -> uint8_t:
        return self.ptr.SSPCON2
    def getSSPSTAT(self) -> uint8_t:
        return self.ptr.SSPSTAT
    def getSSPADD(self) -> uint8_t:
        return self.ptr.SSPADD
    def getSSPBUF(self) -> uint8_t:
        return self.ptr.SSPBUF
    def getSSP1CON1(self) -> uint8_t:
        return self.ptr.SSP1CON1
    def getSSP1CON2(self) -> uint8_t:
        return self.ptr.SSP1CON2
    def getSSP1STAT(self) -> uint8_t:
        return self.ptr.SSP1STAT
    def getSSP1ADD(self) -> uint8_t:
        return self.ptr.SSP1ADD
    def getSSP1BUF(self) -> uint8_t:
        return self.ptr.SSP1BUF
    def getEECON1(self) -> uint8_t:
        return self.ptr.EECON1
    def getEECON2(self) -> uint8_t:
        return self.ptr.EECON2
    def getEEDATA(self) -> uint8_t:
        return self.ptr.EEDATA
    def getEEADR(self) -> uint8_t:
        return self.ptr.EEADR
    def getEEADRH(self) -> uint8_t:
        return self.ptr.EEADRH
    def getNVMCON1(self) -> uint8_t:
        return self.ptr.NVMCON1
    def getNVMCON2(self) -> uint8_t:
        return self.ptr.NVMCON2
    def getNVMDAT(self) -> uint8_t:
        return self.ptr.NVMDAT
    def getNVMADRL(self) -> uint8_t:
        return self.ptr.NVMADRL
    def getNVMADRH(self) -> uint8_t:
        return self.ptr.NVMADRH
    def getRCREG(self) -> uint8_t:
        return self.ptr.RCREG
    def getTXREG(self) -> uint8_t:
        return self.ptr.TXREG
    def getSPBRG(self) -> uint8_t:
        return self.ptr.SPBRG
    def getSPBRGH(self) -> uint8_t:
        return self.ptr.SPBRGH
    def getRCSTA(self) -> uint8_t:
        return self.ptr.RCSTA
    def getTXSTA(self) -> uint8_t:
        return self.ptr.TXSTA
    def getBAUDCON(self) -> uint8_t:
        return self.ptr.BAUDCON
    def getRCREG1(self) -> uint8_t:
        return self.ptr.RCREG1
    def getTXREG1(self) -> uint8_t:
        return self.ptr.TXREG1
    def getSPBRG1(self) -> uint8_t:
        return self.ptr.SPBRG1
    def getSPBRGH1(self) -> uint8_t:
        return self.ptr.SPBRGH1
    def getRCSTA1(self) -> uint8_t:
        return self.ptr.RCSTA1
    def getTXSTA1(self) -> uint8_t:
        return self.ptr.TXSTA1
    def getRCREG2(self) -> uint8_t:
        return self.ptr.RCREG2
    def getTXREG2(self) -> uint8_t:
        return self.ptr.TXREG2
    def getSPBRG2(self) -> uint8_t:
        return self.ptr.SPBRG2
    def getSPBRGH2(self) -> uint8_t:
        return self.ptr.SPBRGH2
    def getRCSTA2(self) -> uint8_t:
        return self.ptr.RCSTA2
    def getTXSTA2(self) -> uint8_t:
        return self.ptr.TXSTA2
    def getRC1REG(self) -> uint8_t:
        return self.ptr.RC1REG
    def getTX1REG(self) -> uint8_t:
        return self.ptr.TX1REG
    def getSP1BRGL(self) -> uint8_t:
        return self.ptr.SP1BRGL
    def getSP1BRGH(self) -> uint8_t:
        return self.ptr.SP1BRGH
    def getRC1STA(self) -> uint8_t:
        return self.ptr.RC1STA
    def getTX1STA(self) -> uint8_t:
        return self.ptr.TX1STA
    def getBAUDCON1(self) -> uint8_t:
        return self.ptr.BAUDCON1
    def getRC2REG(self) -> uint8_t:
        return self.ptr.RC2REG
    def getTX2REG(self) -> uint8_t:
        return self.ptr.TX2REG
    def getSP2BRGL(self) -> uint8_t:
        return self.ptr.SP2BRGL
    def getSP2BRGH(self) -> uint8_t:
        return self.ptr.SP2BRGH
    def getRC2STA(self) -> uint8_t:
        return self.ptr.RC2STA
    def getTX2STA(self) -> uint8_t:
        return self.ptr.TX2STA
    def getBAUDCON2(self) -> uint8_t:
        return self.ptr.BAUDCON2
    def getT2CON(self) -> uint8_t:
        return self.ptr.T2CON
    def getPR2(self) -> uint8_t:
        return self.ptr.PR2
    def getTMR2(self) -> uint8_t:
        return self.ptr.TMR2
    def getT2TMR(self) -> uint8_t:
        return self.ptr.T2TMR
    def getT2PR(self) -> uint8_t:
        return self.ptr.T2PR
    def getWDTCON(self) -> uint8_t:
        return self.ptr.WDTCON
    def getWDTCON0(self) -> uint8_t:
        return self.ptr.WDTCON0
    def getRA0PPS(self) -> uint8_t:
        return self.ptr.RA0PPS
    def getRA1PPS(self) -> uint8_t:
        return self.ptr.RA1PPS
    def getRA2PPS(self) -> uint8_t:
        return self.ptr.RA2PPS
    def getRA3PPS(self) -> uint8_t:
        return self.ptr.RA3PPS
    def getRA4PPS(self) -> uint8_t:
        return self.ptr.RA4PPS
    def getRA5PPS(self) -> uint8_t:
        return self.ptr.RA5PPS
    def getRA6PPS(self) -> uint8_t:
        return self.ptr.RA6PPS
    def getRA7PPS(self) -> uint8_t:
        return self.ptr.RA7PPS
    def getRB0PPS(self) -> uint8_t:
        return self.ptr.RB0PPS
    def getRB1PPS(self) -> uint8_t:
        return self.ptr.RB1PPS
    def getRB2PPS(self) -> uint8_t:
        return self.ptr.RB2PPS
    def getRB3PPS(self) -> uint8_t:
        return self.ptr.RB3PPS
    def getRB4PPS(self) -> uint8_t:
        return self.ptr.RB4PPS
    def getRB5PPS(self) -> uint8_t:
        return self.ptr.RB5PPS
    def getRB6PPS(self) -> uint8_t:
        return self.ptr.RB6PPS
    def getRB7PPS(self) -> uint8_t:
        return self.ptr.RB7PPS
    def getRC0PPS(self) -> uint8_t:
        return self.ptr.RC0PPS
    def getRC1PPS(self) -> uint8_t:
        return self.ptr.RC1PPS
    def getRC2PPS(self) -> uint8_t:
        return self.ptr.RC2PPS
    def getRC3PPS(self) -> uint8_t:
        return self.ptr.RC3PPS
    def getRC4PPS(self) -> uint8_t:
        return self.ptr.RC4PPS
    def getRC5PPS(self) -> uint8_t:
        return self.ptr.RC5PPS
    def getRC6PPS(self) -> uint8_t:
        return self.ptr.RC6PPS
    def getRC7PPS(self) -> uint8_t:
        return self.ptr.RC7PPS
    def getRD0PPS(self) -> uint8_t:
        return self.ptr.RD0PPS
    def getRD1PPS(self) -> uint8_t:
        return self.ptr.RD1PPS
    def getRD2PPS(self) -> uint8_t:
        return self.ptr.RD2PPS
    def getRD3PPS(self) -> uint8_t:
        return self.ptr.RD3PPS
    def getRD4PPS(self) -> uint8_t:
        return self.ptr.RD4PPS
    def getRD5PPS(self) -> uint8_t:
        return self.ptr.RD5PPS
    def getRD6PPS(self) -> uint8_t:
        return self.ptr.RD6PPS
    def getRD7PPS(self) -> uint8_t:
        return self.ptr.RD7PPS
    def getRE0PPS(self) -> uint8_t:
        return self.ptr.RE0PPS
    def getRE1PPS(self) -> uint8_t:
        return self.ptr.RE1PPS
    def getRE2PPS(self) -> uint8_t:
        return self.ptr.RE2PPS

cdef class bb_uart_:
    cdef cpicsim.bb_uart_t * ptr
    def __cinit__(self):
        self.ptr = NULL
    @staticmethod
    cdef create(cpicsim.bb_uart_t * ptr):
        cdef bb_uart_ u = bb_uart_()
        u.ptr = ptr
        return u
    def getPrx(self) -> uint8_t:
        return self.ptr.prx
    def getInsr(self) -> uint16_t:
        return self.ptr.insr
    def getOutsr(self) -> uint16_t:
        return self.ptr.outsr
    def getBcr(self) -> uint32_t:
        return self.ptr.bcr
    def getTcountr(self) -> uint32_t:
        return self.ptr.tcountr
    def getBcw(self) -> uint32_t:
        return self.ptr.bcw
    def getTcountw(self) -> uint32_t:
        return self.ptr.tcountw
    def getSpeed(self) -> uint32_t:
        return self.ptr.speed
    def getCycleCount(self) -> uint32_t:
        return self.ptr.cycle_count
    def getRxc(self) -> uint32_t:
        return self.ptr.rxc
    def getLeds(self) -> uint32_t:
        return self.ptr.leds
    def getDataR(self) -> uint8_t:
        return self.ptr.datar
    def getDataRecv(self) -> uint8_t:
        return self.ptr.data_recv
    def getDataW(self) -> uint8_t:
        return self.ptr.dataw
    def getDataToSend(self) -> uint8_t:
        return self.ptr.data_to_send
    def getFreq(self) -> uint32_t:
        return self.ptr.freq

cdef class serial_:
    cdef cpicsim._serial * ptr
    def __cinit__(self):
        self.ptr = NULL
    @staticmethod
    cdef create(cpicsim._serial * ptr):
        cdef serial_ s = serial_()
        s.ptr = ptr
        return s
    def getRecb(self) -> uint8_t:
        return self.ptr.recb
    def getSerialc(self) -> int:
        return self.ptr.serialc
    def getSerialBaud(self) -> uint32_t:
        return self.ptr.serialbaud
    def getSerialExBaud(self) -> float:
        return self.ptr.serialexbaud
    def getSerialFd(self) -> int:
        return self.ptr.serialfd
    def getSOpen(self) -> int:
        return self.ptr.s_open
    def getFlowControl(self) -> int:
        return self.ptr.flowcontrol
    def getCtSpin(self) -> int:
        return self.ptr.ctspin
    def getRtSpin(self) -> int:
        return self.ptr.rtspin
    def getBc(self) -> int:
        return self.ptr.bc
    def getRxIFMask(self) -> uint8_t:
        return self.ptr.RXIF_mask
    def getTxIFMask(self) -> uint8_t:
        return self.ptr.TXIF_mask
    def getSerialTXSTA(self) -> uint8_t:
        return self.ptr.serial_TXSTA
    def getSerialPIR(self) -> uint8_t:
        return self.ptr.serial_PIR
    def getSerialPIE(self) -> uint8_t:
        return self.ptr.serial_PIE
    def getSerialRCSTA(self) -> uint8_t:
        return self.ptr.serial_RCSTA
    def getSerialSPBRG(self) -> uint8_t:
        return self.ptr.serial_SPBRG
    def getSerialSPBRGH(self) -> uint8_t:
        return self.ptr.serial_SPBRGH
    def getSerialBAUDCTL(self) -> uint8_t:
        return self.ptr.serial_BAUDCTL
    def getSerialRCREG(self) -> uint8_t:
        return self.ptr.serial_RCREG
    def getSerialTXREG(self) -> uint8_t:
        return self.ptr.serial_TXREG
    def getSerialTXREGADDR(self) -> uint16_t:
        return self.ptr.serial_TXREG_ADDR
    def getSerialRCSTAADDR(self) -> uint16_t:
        return self.ptr.serial_RCSTA_ADDR
    def getSerialRCREGADDR(self) -> uint16_t:
        return self.ptr.serial_RCREG_ADDR
    def getSerialTRISRX(self) -> uint8_t:
        return self.ptr.serial_TRIS_RX
    def getSerialTRISRXMASK(self) -> uint8_t:
        return self.ptr.serial_TRIS_RX_MASK
    def getBbUART(self) -> bb_uart_:
        u = bb_uart_.create(&self.ptr.bbuart)
        return u

cdef class pic_:
    cdef cpicsim._pic pic
    def __cinit__(self):
        pass
    def __dealloc__(self):
        pass
    def set_serial(self, nser : int, name : str, flowcontrol : int, ctspin : int, rtspin : int) -> int:
        return cpicsim.pic_set_serial(&self.pic, nser, name.encode('utf8'), flowcontrol, ctspin, rtspin)
    def init(self, processor : int, fname : str, leeprom : int, freq : float) -> int:
        return cpicsim.pic_init(&self.pic, processor, fname.encode('utf8'), leeprom, freq)
    def reset(self, flags : int) -> int:
        return cpicsim.pic_reset(&self.pic, flags)
    def step(self):
        cpicsim.pic_step(&self.pic)
    def end(self):
        cpicsim.pic_end(&self.pic)
    def get_pin(self, pin : uint8_t) -> uint8_t:
        return cpicsim.pic_get_pin(&self.pic, pin)
    def set_pin(self, pin : uint8_t, value : uint8_t) -> int:
        return cpicsim.pic_set_pin(&self.pic, pin, value)

    @property
    def print(self):
         return True if self.pic.print_ else False
    @print.setter
    def print(self, value : bint):
        self.pic.print_ = 1 if value else 0

class pic(pic_):
    def __init__(self):
        pass
    def __del__(self):
        pass
