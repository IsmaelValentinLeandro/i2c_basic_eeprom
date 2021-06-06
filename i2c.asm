;----------------------------------------------------------
; Code Produced by the Proton Compiler. Ver 3.5.4.7
; Copyright Rosetta Technologies/Crownhill Associates Ltd
; Written by Les Johnson. August 2012
;----------------------------------------------------------
;
 LIST  P = 18F452, F = INHX32, W = 2, X = ON, R = DEC, MM = ON, N = 0, C = 255, T=ON
TOSU equ 0X0FFF
TOSLH equ 0X0FFE
TOSH equ 0X0FFE
TOSL equ 0X0FFD
STKPTR equ 0X0FFC
PCLATU equ 0X0FFB
PCLATH equ 0X0FFA
PCL equ 0X0FF9
TBLPTRLHH equ 0X0FF8
TBLPTRU equ 0X0FF8
TBLPTRLH equ 0X0FF7
TBLPTRH equ 0X0FF7
TBLPTRL equ 0X0FF6
TABLAT equ 0X0FF5
PRODLH equ 0X0FF4
PRODH equ 0X0FF4
PRODL equ 0X0FF3
INTCON equ 0X0FF2
INTCON1 equ 0X0FF2
INTCON2 equ 0X0FF1
INTCON3 equ 0X0FF0
INDF0 equ 0X0FEF
POSTINC0 equ 0X0FEE
POSTDEC0 equ 0X0FED
PREINC0 equ 0X0FEC
PLUSW0 equ 0X0FEB
FSR0LH equ 0X0FEA
FSR0H equ 0X0FEA
FSR0L equ 0X0FE9
WREG equ 0X0FE8
INDF1 equ 0X0FE7
POSTINC1 equ 0X0FE6
POSTDEC1 equ 0X0FE5
PREINC1 equ 0X0FE4
PLUSW1 equ 0X0FE3
FSR1LH equ 0X0FE2
FSR1H equ 0X0FE2
FSR1L equ 0X0FE1
BSR equ 0X0FE0
INDF2 equ 0X0FDF
POSTINC2 equ 0X0FDE
POSTDEC2 equ 0X0FDD
PREINC2 equ 0X0FDC
PLUSW2 equ 0X0FDB
FSR2LH equ 0X0FDA
FSR2H equ 0X0FDA
FSR2L equ 0X0FD9
STATUS equ 0X0FD8
TMR0LH equ 0X0FD7
TMR0H equ 0X0FD7
TMR0L equ 0X0FD6
T0CON equ 0X0FD5
OSCCON equ 0X0FD3
LVDCON equ 0X0FD2
HLVDCON equ 0X0FD2
WDTCON equ 0X0FD1
RCON equ 0X0FD0
TMR1LH equ 0X0FCF
TMR1H equ 0X0FCF
TMR1L equ 0X0FCE
T1CON equ 0X0FCD
TMR2 equ 0X0FCC
PR2 equ 0X0FCB
T2CON equ 0X0FCA
SSPBUF equ 0X0FC9
SSPADD equ 0X0FC8
SSPSTAT equ 0X0FC7
SSPCON1 equ 0X0FC6
SSPCON2 equ 0X0FC5
ADRESLH equ 0X0FC4
ADRESH equ 0X0FC4
ADRESL equ 0X0FC3
ADCON0 equ 0X0FC2
ADCON1 equ 0X0FC1
CCPR1LH equ 0X0FBF
CCPR1H equ 0X0FBF
CCPR1L equ 0X0FBE
CCP1CON equ 0X0FBD
CCPR2LH equ 0X0FBC
CCPR2H equ 0X0FBC
CCPR2L equ 0X0FBB
CCP2CON equ 0X0FBA
TMR3LH equ 0X0FB3
TMR3H equ 0X0FB3
TMR3L equ 0X0FB2
T3CON equ 0X0FB1
SPBRG equ 0X0FAF
RCREG equ 0X0FAE
TXREG equ 0X0FAD
TXSTA equ 0X0FAC
RCSTA equ 0X0FAB
EEADR equ 0X0FA9
EEDATL equ 0X0FA8
EEDATA equ 0X0FA8
EECON2 equ 0X0FA7
EECON1 equ 0X0FA6
IPR2 equ 0X0FA2
PIR2 equ 0X0FA1
PIE2 equ 0X0FA0
IPR1 equ 0X0F9F
PIR1 equ 0X0F9E
PIE1 equ 0X0F9D
TRISE equ 0X0F96
TRISD equ 0X0F95
TRISC equ 0X0F94
TRISB equ 0X0F93
TRISA equ 0X0F92
LATE equ 0X0F8D
LATD equ 0X0F8C
LATC equ 0X0F8B
LATB equ 0X0F8A
LATA equ 0X0F89
PORTE equ 0X0F84
PORTD equ 0X0F83
PORTC equ 0X0F82
PORTB equ 0X0F81
PORTA equ 0X0F80
_I2C_SCL_PORT=TRISC
_I2C_SCL_PIN=3
_I2C_SDA_PORT=TRISC
_I2C_SDA_PIN=4
STKFUL=7
STKUNF=6
GIE=7
GIEH=7
PEIE=6
GIEL=6
TMR0IE=5
T0IE=5
INT0IE=4
INT0E=4
RBIE=3
TMR0IF=2
T0IF=2
INT0IF=1
INT0F=1
RBIF=0
NOT_RBPU=7
RBPU=7
INTEDG0=6
INTEDG1=5
INTEDG2=4
TMR0IP=2
T0IP=2
RBIP=0
INT2IP=7
INT1IP=6
INT2IE=4
INT1IE=3
INT2IF=1
INT1IF=0
N=4
OV=3
Z=2
DC=1
C=0
TMR0ON=7
T08BIT=6
T0CS=5
T0SE=4
PSA=3
T0PS2=2
T0PS1=1
T0PS0=0
SCS=0
IRVST=5
LVDEN=4
LVDL3=3
LVDL2=2
LVDL1=1
LVDL0=0
SWDTE=0
SWDTEN=0
IPEN=7
NOT_RI=4
RI=4
NOT_TO=3
TO=3
NOT_PD=2
PD=2
NOT_POR=1
POR=1
NOT_BOR=0
BOR=0
RD16=7
PP_RD16=7
T1CKPS1=5
T1CKPS0=4
T1OSCEN=3
NOT_T1SYNC=2
T1SYNC=2
T1INSYNC=2
TMR1CS=1
TMR1ON=0
TOUTPS3=6
TOUTPS2=5
TOUTPS1=4
TOUTPS0=3
TMR2ON=2
PP_TMR2ON=2
T2CKPS1=1
PP_T2CKPS1=1
T2CKPS0=0
PP_T2CKPS0=0
SMP=7
CKE=6
I2C_DAT=5
NOT_A=5
NOT_ADDRESS=5
D_A=5
DATA_ADDRESS=5
I2C_STOP=4
I2C_START=3
I2C_READ=2
NOT_W=2
NOT_WRITE=2
R_W=2
PP_R_W=2
READ_WRITE=2
UA=1
BF=0
WCOL=7
SSPOV=6
SSPEN=5
CKP=4
SSPM3=3
SSPM2=2
SSPM1=1
SSPM0=0
GCEN=7
ACKSTAT=6
ACKDT=5
PP_ACKDT=5
ACKEN=4
PP_ACKEN=4
RCEN=3
PP_RCEN=3
PEN=2
PP_PEN=2
RSEN=1
PP_RSEN=1
SEN=0
PP_SEN=0
ADCS1=7
ADCS0=6
CHS2=5
CHS1=4
CHS0=3
GO=2
NOT_DONE=2
DONE=2
GO_DONE=2
PP_GO_DONE=2
ADON=0
PP_ADON=0
ADFM=7
ADCS2=6
PCFG3=3
PCFG2=2
PCFG1=1
PCFG0=0
DC1B1=5
CCP1X=5
DC1B0=4
CCP1Y=4
CCP1M3=3
CCP1M2=2
CCP1M1=1
CCP1M0=0
DC2B1=5
CCP2X=5
DC2B0=4
CCP2Y=4
CCP2M3=3
CCP2M2=2
CCP2M1=1
CCP2M0=0
RD16=7
PP_RD16=7
T3CCP2=6
PP_T3CCP2=6
T3CKPS1=5
T3CKPS0=4
T3CCP1=3
PP_T3CCP1=3
NOT_T3SYNC=2
T3SYNC=2
T3INSYNC=2
TMR3CS=1
TMR3ON=0
CSRC=7
TX9=6
NOT_TX8=6
TX8_9=6
TXEN=5
SYNC=4
BRGH=2
TRMT=1
TX9D=0
TXD8=0
SPEN=7
RX9=6
RC9=6
NOT_RC8=6
RC8_9=6
SREN=5
CREN=4
PP_CREN=4
ADDEN=3
FERR=2
OERR=1
PP_OERR=1
RX9D=0
RCD8=0
EEIP=4
BCLIP=3
LVDIP=2
TMR3IP=1
CCP2IP=0
EEIF=4
BCLIF=3
LVDIF=2
TMR3IF=1
CCP2IF=0
EEIE=4
BCLIE=3
LVDIE=2
TMR3IE=1
CCP2IE=0
PSPIP=7
ADIP=6
RCIP=5
TXIP=4
SSPIP=3
CCP1IP=2
TMR2IP=1
TMR1IP=0
PSPIF=7
ADIF=6
RCIF=5
PP_RCIF=5
TXIF=4
PP_TXIF=4
SSPIF=3
CCP1IF=2
TMR2IF=1
TMR1IF=0
PSPIE=7
ADIE=6
RCIE=5
TXIE=4
SSPIE=3
CCP1IE=2
TMR2IE=1
TMR1IE=0
IBF=7
OBF=6
IBOV=5
PSPMODE=4
TRISE2=2
TRISE1=1
TRISE0=0
EEPGD=7
PP_EEPGD=7
CFGS=6
FREE=4
WRERR=3
PP_WRERR=3
WREN=2
PP_WREN=2
WR=1
PP_WR=1
RD=0
PP_RD=0
RA0=0
AN0=0
RA1=1
AN1=1
RA2=2
AN2=2
VREFM=2
RA3=3
AN3=3
VREFP=3
RA4=4
T0CKI=4
RA5=5
AN4=5
SS=5
LVDIN=5
RA6=6
OSC2=6
CLKO=6
RB0=0
INT0=0
RB1=1
INT1=1
RB2=2
INT2=2
RB3=3
CCP2A=3
RB4=4
RB5=5
RB6=6
RB7=7
RC0=0
T1OSO=0
T1CKI=0
RC1=1
T1OSI=1
CCP2=1
RC2=2
CCP1=2
RC3=3
SCK=3
SCL=3
RC4=4
SDI=4
SDA=4
RC5=5
SDO=5
RC6=6
TX=6
CK=6
RC7=7
RX=7
RD0=0
PP_RD0=0
PSP0=0
RD1=1
PP_RD1=1
PSP1=1
RD2=2
PP_RD2=2
PSP2=2
RD3=3
PP_RD3=3
PSP3=3
RD4=4
PP_RD4=4
PSP4=4
RD5=5
PP_RD5=5
PSP5=5
RD6=6
PP_RD6=6
PSP6=6
RD7=7
PP_RD7=7
PSP7=7
RE0=0
RD=0
PP_RD=0
AN5=0
RE1=1
WR=1
PP_WR=1
AN6=1
RE2=2
CS=2
AN7=2
  __MAXRAM 0XFFF
  __BADRAM 0X600-0XF7F
  __BADRAM 0XF85-0XF88
  __BADRAM 0XF8E-0XF91
  __BADRAM 0XF97-0XF9C
  __BADRAM 0XFA3-0XFA5
  __BADRAM 0XFAA
  __BADRAM 0XFB4-0XFB9
OSCS_ON_1 equ 0XDF
OSCS_OFF_1 equ 0XFF
LP_OSC_1 equ 0XF8
XT_OSC_1 equ 0XF9
HS_OSC_1 equ 0XFA
RC_OSC_1 equ 0XFB
EC_OSC_1 equ 0XFC
ECIO_OSC_1 equ 0XFD
HSPLL_OSC_1 equ 0XFE
RCIO_OSC_1 equ 0XFF
BOR_ON_2 equ 0XFF
BOR_OFF_2 equ 0XFD
PWRT_OFF_2 equ 0XFF
PWRT_ON_2 equ 0XFE
BORV_20_2 equ 0XFF
BORV_27_2 equ 0XFB
BORV_42_2 equ 0XF7
BORV_45_2 equ 0XF3
WDT_ON_2 equ 0XFF
WDT_OFF_2 equ 0XFE
WDTPS_128_2 equ 0XFF
WDTPS_64_2 equ 0XFD
WDTPS_32_2 equ 0XFB
WDTPS_16_2 equ 0XF9
WDTPS_8_2 equ 0XF7
WDTPS_4_2 equ 0XF5
WDTPS_2_2 equ 0XF3
WDTPS_1_2 equ 0XF1
CCP2MX_ON_3 equ 0XFF
CCP2MX_OFF_3 equ 0XFE
STVR_ON_4 equ 0XFF
STVR_OFF_4 equ 0XFE
LVP_ON_4 equ 0XFF
LVP_OFF_4 equ 0XFB
DEBUG_ON_4 equ 0X7F
DEBUG_OFF_4 equ 0XFF
CP0_ON_5 equ 0XFE
CP0_OFF_5 equ 0XFF
CP1_ON_5 equ 0XFD
CP1_OFF_5 equ 0XFF
CP2_ON_5 equ 0XFB
CP2_OFF_5 equ 0XFF
CP3_ON_5 equ 0XF7
CP3_OFF_5 equ 0XFF
CPB_ON_5 equ 0XBF
CPB_OFF_5 equ 0XFF
CPD_ON_5 equ 0X7F
CPD_OFF_5 equ 0XFF
WRT0_ON_6 equ 0XFE
WRT0_OFF_6 equ 0XFF
WRT1_ON_6 equ 0XFD
WRT1_OFF_6 equ 0XFF
WRT2_ON_6 equ 0XFB
WRT2_OFF_6 equ 0XFF
WRT3_ON_6 equ 0XF7
WRT3_OFF_6 equ 0XFF
WRTC_ON_6 equ 0XDF
WRTC_OFF_6 equ 0XFF
WRTB_ON_6 equ 0XBF
WRTB_OFF_6 equ 0XFF
WRTD_ON_6 equ 0X7F
WRTD_OFF_6 equ 0XFF
EBTR0_ON_7 equ 0XFE
EBTR0_OFF_7 equ 0XFF
EBTR1_ON_7 equ 0XFD
EBTR1_OFF_7 equ 0XFF
EBTR2_ON_7 equ 0XFB
EBTR2_OFF_7 equ 0XFF
EBTR3_ON_7 equ 0XF7
EBTR3_OFF_7 equ 0XFF
EBTRB_ON_7 equ 0XBF
EBTRB_OFF_7 equ 0XFF
config1l equ 0X300000
config1h equ 0X300001
config2l equ 0X300002
config2h equ 0X300003
config3l equ 0X300004
config3h equ 0X300005
config4l equ 0X300006
config4h equ 0X300007
config5l equ 0X300008
config5h equ 0X300009
config6l equ 0X30000A
config6h equ 0X30000B
config7l equ 0X30000C
config7h equ 0X30000D
DEVID1 equ 0X3FFFFE
DEVID2 equ 0X3FFFFF
IDLOC0 equ 0X200000
__IDLOC0 equ 0X200000
IDLOC1 equ 0X200001
__IDLOC1 equ 0X200001
IDLOC2 equ 0X200002
__IDLOC2 equ 0X200002
IDLOC3 equ 0X200003
__IDLOC3 equ 0X200003
IDLOC4 equ 0X200004
__IDLOC4 equ 0X200004
IDLOC5 equ 0X200005
__IDLOC5 equ 0X200005
IDLOC6 equ 0X200006
__IDLOC6 equ 0X200006
IDLOC7 equ 0X200007
__IDLOC7 equ 0X200007
#define __18F452 1
#define XTAL 4
#define _CORE 16
#define _MAXRAM 1536
#define _RAM_END 1536
#define _MAXMEM 0X8000
#define _ADC 8
#define _ADC_RES 10
#define _EEPROM 256
#define RAM_BANKS 6
#define _USART 1
#define _USB 0
#define _USB#RAM_START 0
#define _FLASH 1
#define _CWRITE_BLOCK 8
#define BANK0_START 128
#define BANK0_END 255
#define BANK1_START 256
#define BANK1_END 511
#define BANK2_START 512
#define BANK2_END 767
#define BANK3_START 768
#define BANK3_END 1023
#define BANK4_START 1024
#define BANK4_END 1279
#define BANK5_START 1280
#define BANK5_END 1535
#define bankA_Start 0
#define bankA_End 127
#define _SYSTEM_VARIABLE_COUNT 20
ram_bank = 0
#define LCD#DTPORT PORTB
#define LCD#DTPIN 4
#define LCD#RSPORT PORTB
#define LCD#RSPIN 2
#define LCD#ENPORT PORTB
#define LCD#ENPIN 3
#define LCD#TYPE 0
#define LCD#INTERFACE 4
#define LCD#LINES 2
#define clrw clrf WREG
#define negw negf WREG
#define skpc btfss STATUS,0
#define skpnc btfsc STATUS,0
#define clrc bcf STATUS,0
#define setc bsf STATUS,0
#define skpz btfss STATUS,2
#define skpnz btfsc STATUS,2
#define clrz bcf STATUS,2
#define setz bsf STATUS,2
MOVFW macro pVarin
    movf pVarin,W
    endm
rlf macro pVarin,pDestination
    rlcf pVarin,pDestination
    endm
rrf macro pVarin,pDestination
    rrcf pVarin,pDestination
    endm
jump macro pLabel
    goto pLabel
    endm
f@call macro pDestination
if (pDestination < 1)
    call pDestination
else
if (pDestination > $)
    call pDestination
else
if (pDestination < ($ - 0X03FF))
    call pDestination
else
    rcall pDestination
endif
endif
endif
    endm
f@jump macro pDestination
ifdef watchdog_req
if ($ == pDestination)
    clrwdt
endif
endif
if (pDestination < 1)
    goto pDestination
else
if ((pDestination) > $)
    goto pDestination
else
if ((pDestination) < ($ - 0X03FF))
    goto pDestination
else
    bra pDestination
endif
endif
endif
    endm
ifdef watchdog_req
    chk@slf macro pDestination
if ($ == pDestination)
    clrwdt
endif
    endm
endif
g@oto macro pDestination
if (pDestination < 1)
    btfsc STATUS,OV
    goto pDestination
else
if (pDestination > $)
    btfsc STATUS,OV
    goto pDestination
else
if (pDestination < ($ - 127))
    btfsc STATUS,OV
    goto pDestination
else
    bov pDestination
endif
endif
endif
    endm
go@to macro pDestination
if (pDestination < 1)
    goto pDestination
else
if (pDestination > $)
    goto pDestination
else
if (pDestination < ($ - 0X03FF))
    goto pDestination
else
    bra pDestination
endif
endif
endif
    endm
s@b macro pVarin
if ((pVarin > bankA_End) & (pVarin < 0X0F80))
if ((pVarin & 0X0F00) != (ram_bank << 8))
    movlb high (pVarin)
    ram_bank = (pVarin >> 8)
endif
endif
    endm
r@b macro
if(ram_bank != 0)
    movlb 0
    ram_bank = 0
endif
    endm
wreg_byte macro pByteOut
    movff WREG,pByteOut
    endm
wreg_bit macro pVarOut,pBitout
    s@b pVarOut
    btfsc WREG,0
    bsf pVarOut,pBitout
    btfss WREG,0
    bcf pVarOut,pBitout
    r@b
    endm
wreg_word macro pWordOut
    movff WREG,pWordOut
    movlw 0
    movff WREG,pWordOut+1
    endm
wreg_dword macro pDwordOut
    movff WREG,pDwordOut
    movlw 0
    movff WREG,pDwordOut+3
    movff WREG,pDwordOut+2
    movff WREG,pDwordOut+1
    endm
num_SFR macro pNumIn,pSFROut
    movlw pNumIn
    movwf pSFROut
    endm
NUM16_SFR macro pNumIn,pSFROut
    movlw (pNumIn & 255)
    movwf pSFROut
    movlw ((pNumIn >> 8) & 255)
    movwf pSFROut + 1
    endm
byte_wreg macro pByteIn
    movff pByteIn,WREG
    endm
num_wreg macro pNumIn
    movlw (pNumIn & 255)
    endm
num_byte macro pNumIn,pByteOut
    movlw (pNumIn & 255)
    movff WREG,pByteOut
    endm
num_bit macro pNumIn,pVarOut,pBitout
    s@b pVarOut
if((pNumIn & 1) == 1)
    bsf pVarOut,pBitout
else
    bcf pVarOut,pBitout
endif
    r@b
    endm
num_word macro pNumIn,pWordOut
ifdef _USELFSR
if(pWordOut == FSR0L)
    lfsr 0,pNumIn
    exitm
endif
if(pWordOut == FSR1L)
    lfsr 1,pNumIn
    exitm
endif
if(pWordOut == FSR2L)
    lfsr 2,pNumIn
    exitm
endif
endif
    s@b pWordOut
    movlw (pNumIn & 255)
    movwf pWordOut
    s@b pWordOut+1
    movlw high (pNumIn)
    movwf pWordOut+1
    r@b
    endm
num_dword macro pNumIn,pDwordOut
    s@b pDwordOut
    movlw low (pNumIn)
    movwf pDwordOut
    s@b pDwordOut+1
    movlw high (pNumIn)
    movwf pDwordOut+1
    s@b pDwordOut+2
    movlw ((pNumIn >> 16) & 255)
    movwf pDwordOut+2
    s@b pDwordOut+3
    movlw ((pNumIn >> 24) & 255)
    movwf pDwordOut+3
    r@b
    endm
bit_wreg macro pVarin,pBitIn
    s@b pVarin
    clrw
    btfsc pVarin,pBitIn
    movlw 1
    r@b
    endm
bit_byte macro pVarin,pBitIn,pByteOut
    s@b pVarin
    clrw
    btfsc pVarin,pBitIn
    movlw 1
    s@b pByteOut
    movwf pByteOut
    r@b
    endm
bit_bit macro pVarin,pBitIn,pVarOut,pBitout
if ((pVarin & 0X0F00) == (pVarOut & 0X0F00))
    s@b pVarin
    btfsc pVarin,pBitIn
    bsf pVarOut,pBitout
    btfss pVarin,pBitIn
    bcf pVarOut,pBitout
else
if ((pVarin <= bankA_End) | (pVarin >= 0X0F80))
    s@b pVarOut
    btfsc pVarin,pBitIn
    bsf pVarOut,pBitout
    btfss pVarin,pBitIn
    bcf pVarOut,pBitout
else
if ((pVarOut <= bankA_End) | (pVarOut >= 0X0F80))
    s@b pVarin
    btfsc pVarin,pBitIn
    bsf pVarOut,pBitout
    btfss pVarin,pBitIn
    bcf pVarOut,pBitout
else
    s@b pVarin
    clrdc
    btfsc pVarin,pBitIn
    setdc
    s@b pVarOut
    skpndc
    bsf pVarOut,pBitout
    skpdc
    bcf pVarOut,pBitout
endif
endif
endif
    r@b
    endm
bit_word macro pVarin,pBitIn,pWordOut
    s@b pWordOut+1
    clrf pWordOut+1
    bit_byte pVarin,pBitIn,pWordOut
    endm
bit_dword macro pVarin,pBitIn,pDwordOut
    s@b pDwordOut+3
    clrf pDwordOut+3
    s@b pDwordOut+2
    clrf pDwordOut+2
    s@b pDwordOut+1
    clrf pDwordOut+1
    bit_byte pVarin,pBitIn,pDwordOut
    endm
word_wreg macro pWordIn
    byte_wreg pWordIn
    endm
word_byte macro pWordIn,pByteOut
    byte_byte pWordIn,pByteOut
    endm
word_bit macro pWordIn,pVarOut,pBitout
    byte_bit pWordIn, pVarOut, pBitout
    endm
word_word macro pWordIn,pWordOut
    movff pWordIn+1,pWordOut+1
    movff pWordIn,pWordOut
    endm
word_dword macro pWordIn,pDwordOut
    movlw 0
    movff WREG,pDwordOut+3
    movff WREG,pDwordOut+2
    word_word pWordIn,pDwordOut
    endm
byte_byte macro pByteIn,pByteOut
    movff pByteIn,pByteOut
    endm
byte_word macro pByteIn,pWordOut
    movlw 0
    movff WREG,pWordOut+1
    byte_byte pByteIn,pWordOut
    endm
byte_dword macro pByteIn,pDwordOut
    movlw 0
    movff WREG,pDwordOut+3
    movff WREG,pDwordOut+2
    movff WREG,pDwordOut+1
    byte_byte pByteIn,pDwordOut
    endm
byte_bit macro pByteIn,pVarOut,pBitout
if ((pByteIn & 0X0F00) == (pVarOut & 0X0F00))
    s@b pByteIn
    btfsc pByteIn,0
    bsf pVarOut,pBitout
    btfss pByteIn,0
    bcf pVarOut,pBitout
else
if ((pByteIn <= bankA_End) | (pByteIn >= 0X0F80))
    s@b pVarOut
    btfsc pByteIn,0
    bsf pVarOut,pBitout
    btfss pByteIn,0
    bcf pVarOut,pBitout
else
if ((pVarOut <= bankA_End) | (pVarOut >= 0X0F80))
    s@b pByteIn
    btfsc pByteIn,0
    bsf pVarOut,pBitout
    btfss pByteIn,0
    bcf pVarOut,pBitout
else
    s@b pByteIn
    rrf pByteIn,W
    s@b pVarOut
    skpnc
    bsf pVarOut,pBitout
    skpc
    bcf pVarOut,pBitout
endif
endif
endif
    r@b
    endm
dword_wreg macro pDwordIn
    byte_wreg pDwordIn
    endm
dword_byte macro pDwordIn,pByteOut
    byte_byte pDwordIn,pByteOut
    endm
dword_word macro pDwordIn,pWordOut
    movff pDwordIn+1,pWordOut+1
    movff pDwordIn,pWordOut
    endm
dword_dword macro pDwordIn,pDwordOut
    movff pDwordIn+3,pDwordOut+3
    movff pDwordIn+2,pDwordOut+2
    movff pDwordIn+1,pDwordOut+1
    movff pDwordIn,pDwordOut
    endm
dword_bit macro pDwordIn,pVarOut,pBitout
    byte_bit pDwordIn,pVarOut,pBitout
    endm
num_float macro pNumIn,pFloatOut
    num_byte pNumIn,pFloatOut+3
    num_byte ((pNumIn >> 8) & 255),pFloatOut+2
    num_byte ((pNumIn >> 16) & 255),pFloatOut+1
    num_byte ((pNumIn >> 24) & 255),pFloatOut
    endm
wreg_float macro pFloatOut
    call INT08@TOFL32
    movff PP_AARG,pFloatOut
    movff PP_AARGH,pFloatOut+1
    movff PP_AARGHH,pFloatOut+2
    movff PP_AARGHHH,pFloatOut+3
    endm
bit_float macro pVarin,pBitIn,pFloatOut
    bit_wreg pVarin,pBitIn
    call INT08@TOFL32
    movff PP_AARG,pFloatOut
    movff PP_AARGH,pFloatOut+1
    movff PP_AARGHH,pFloatOut+2
    movff PP_AARGHHH,pFloatOut+3
    endm
byte_float macro pByteIn,pFloatOut
    byte_wreg pByteIn
    call INT08@TOFL32
    movff PP_AARG,pFloatOut
    movff PP_AARGH,pFloatOut+1
    movff PP_AARGHH,pFloatOut+2
    movff PP_AARGHHH,pFloatOut+3
    endm
word_float macro pWordIn,pFloatOut
    byte_byte pWordIn,PP_AARG
    byte_byte pWordIn+1,PP_AARGH
    call INT16@TOFL32
    movff PP_AARG,pFloatOut
    movff PP_AARGH,pFloatOut+1
    movff PP_AARGHH,pFloatOut+2
    movff PP_AARGHHH,pFloatOut+3
    endm
dword_float macro pDwordIn,pFloatOut
    movff pDwordIn,PP_AARG
    movff pDwordIn+1,PP_AARGH
    movff pDwordIn+2,PP_AARGHH
    movff pDwordIn+3,PP_AARGHHH
    call INT32@TOFL32
    movff PP_AARG,pFloatOut
    movff PP_AARGH,pFloatOut+1
    movff PP_AARGHH,pFloatOut+2
    movff PP_AARGHHH,pFloatOut+3
    endm
float_float macro pFloatIn,pFloatOut
    movff pFloatIn,pFloatOut
    movff pFloatIn+1,pFloatOut+1
    movff pFloatIn+2,pFloatOut+2
    movff pFloatIn+3,pFloatOut+3
    endm
float_wreg macro pFloatIn
    float_float pFloatIn,PP_AARG
    call FL32@TOINT32
    endm
float_bit macro pFloatIn,pVarOut,pBitout
    float_float pFloatIn,PP_AARG
    call FL32@TOINT32
    wreg_bit pVarOut,pBitout
    endm
float_byte macro pFloatIn,pByteOut
    float_float pFloatIn,PP_AARG
    call FL32@TOINT32
    wreg_byte pByteOut
    endm
float_word macro pFloatIn,pWordOut
    float_float pFloatIn,PP_AARG
    call FL32@TOINT32
    movff PP_AARGHHH,pWordOut
    movff PP_AARGHH,pWordOut+1
    endm
float_dword macro pFloatIn,pDwordOut
    float_float pFloatIn,PP_AARG
    call FL32@TOINT32
    movff PP_AARGHHH,pDwordOut
    movff PP_AARGHH,pDwordOut+1
    movff PP_AARGH,pDwordOut+2
    movff PP_AARG,pDwordOut+3
    endm
num_FSR0 macro pNumIn
    lfsr 0,pNumIn
    endm
num_FSR1 macro pNumIn
    lfsr 1,pNumIn
    endm
num_FSR2 macro pNumIn
    lfsr 2,pNumIn
    endm
label_word macro pLabelIn,pWordOut
    movlw (pLabelIn & 255)
    movff WREG, pWordOut
    movlw ((pLabelIn >> 8) & 255)
    movff WREG, pWordOut+1
    endm
label_pointer macro pLabelIn
    movlw (pLabelIn & 255)
    movwf TBLPTRL
    movlw ((pLabelIn >> 8) & 255)
    movwf TBLPTRH
    movlw ((pLabelIn >> 16) & 255)
    movwf TBLPTRU
    endm
wreg_sword macro pWordOut
    movff WREG,pWordOut
    s@b pWordOut+1
    clrf pWordOut+1
    btfsc WREG,7
    decf pWordOut+1,F
    r@b
    endm
wreg_sdword macro pDwordOut
    movff WREG,pDwordOut
    s@b pDwordOut
    movlw 0
    btfsc pDwordOut,7
    movlw 255
    movff WREG,pDwordOut+1
    movff WREG,pDwordOut+2
    movff WREG,pDwordOut+3
    r@b
    endm
byte_sword macro pByteIn,pWordOut
    movff pByteIn,pWordOut
    s@b pByteIn
    movlw 0
    btfsc pByteIn,7
    movlw 255
    movff WREG,pWordOut+1
    r@b
    endm
byte_sdword macro pByteIn,pDwordOut
    movff pByteIn,pDwordOut
    s@b pByteIn
    movlw 0
    btfsc pByteIn,7
    movlw 255
    movff WREG,pDwordOut+1
    movff WREG,pDwordOut+2
    movff WREG,pDwordOut+3
    r@b
    endm
word_sdword macro pWordIn,pDwordOut
    movff pWordIn,pDwordOut
    movff pWordIn+1,pDwordOut+1
    s@b pWordIn+1
    movlw 0
    btfsc pWordIn+1,7
    movlw 255
    movff WREG,pDwordOut+2
    movff WREG,pDwordOut+3
    r@b
    endm
BPF = 0
BPFH = 1
GEN = 2
GEN2 = 3
GEN2H = 4
GEN4 = 5
GEN4H = 6
GENH = 7
GPR = 8
PP0 = 9
PP0H = 10
PP1 = 11
PP1H = 12
PP2 = 13
PP2H = 14
PP3 = 15
PP3H = 16
PP4 = 17
PP4H = 18
PP5 = 19
VarWrite = 20
VarWriteH = 21
VarRead = 22
VarReadH = 23
Address = 24
AddressH = 25
Array = 26
variable Array#0=26,Array#1=27,Array#2=28,Array#3=29
variable Array#4=30,Array#5=31,Array#6=32,Array#7=33
variable Array#8=34,Array#9=35
#define __LCD_RSPORT PORTB
#define __LCD_ENPORT PORTB
#define __LCD_DTPORT PORTB
#define SCL PORTC,3
#define SDA PORTC,4
#define __XTAL 4
#define __LCD_DATAUS 75
#define __LCD_RSPIN 2
#define __LCD_ENPIN 3
#define __LCD_DTPIN 4
#define __LCD_INTERFACE 4
#define __LCD_LINES 2
#define __LCD_TYPE 0
proton#code#start
    org 0
    goto proton#main#start
    org 8
LCD@CLS
    movlw 128
    movwf 1,0
CLS
    movlw 254
    call _BYTE__SEND_
    movlw 1
    call _BYTE__SEND_
    movlw 117
    movwf 10,0
    movlw 48
    goto __DELAY_US_W_
LCD@CRS
    clrf 1,0
    bsf 1,7,0
CURS@
    movwf 19,0
    movlw 254
    call _BYTE__SEND_
    movf 19,W,0
    goto _BYTE__SEND_
OUT@DEC
    bcf 0,3,0
    movf 6,W,0
    btfsc 4056,2,0
    bsf 0,3,0
    movlw 5
    movwf 5,0
    movlw 39
    movwf 12,0
    movlw 16
    rcall D@DIG
    movlw 3
    movwf 12,0
    movlw 232
    rcall D@DIG
    clrf 12,0
    movlw 100
    rcall D@DIG
    clrf 12,0
    movlw 10
    rcall D@DIG
    movf 13,W,0
    bra SEND@IT
D@DIG
    movwf 11,0
    movf 14,W,0
    movwf 10,0
    movf 13,W,0
    movwf 9,0
    call __DIVIDE_U1616_
    movf 9,W,0
SEND@IT
    movwf 9,0
    dcfsnz 5,F,0
    bcf 0,3,0
    movf 6,W,0
    bz $ + 8
    subwf 5,W,0
    btfsc 4056,0,0
    bra EX@SEND@IT
    movf 9,W,0
    btfss 4056,2,0
    bcf 0,3,0
    btfsc 0,3,0
    bra EX@SEND@IT
    addlw 48
    goto _BYTE__SEND_
EX@SEND@IT
    return
PRINT@MSTR
    clrf EECON1,0
    bsf EECON1,PP_EEPGD,0
PRT@MTL1
    tblrd*+
    movf 4085,W,0
    bz PRT@MTEXT
    rcall PRINT
    bra PRT@MTL1
PRT@MTEXT
    return
PRINT
    movwf 16,0
    bcf LATB,3,0
    nop
    bcf LATB,2,0
    bcf TRISB,3,0
    bcf TRISB,2,0
    movlw 15
    andwf TRISB,F,0
    movf 16,W,0
    btfsc 0,1,0
    goto PRT@1
    movlw 58
    movwf 10,0
    movlw 152
    call __DELAY_US_W_
    movlw 51
    movwf 15,0
    rcall PR@LP
    movlw 19
    movwf 10,0
    movlw 136
    call __DELAY_US_W_
    rcall PR@LP
    movlw 100
    call __DELAY_US_
    rcall PR@LP
    movlw 100
    call __DELAY_US_
    movlw 34
    movwf 15,0
    rcall PR@LP
    movlw 40
    rcall PR@CM
    movlw 12
    rcall PR@CM
    movlw 6
    rcall PR@CM
    bsf 0,1,0
    movf 16,W,0
    bra PRT@1
PR@CM
    bsf 0,0,0
PRT@1
    movwf 15,0
    btfss 0,0,0
    bra PR@CC
    bcf LATB,2,0
    sublw 3
    bnc PR@SD
    rcall PR@SD
    movlw 7
    movwf 10,0
    movlw 208
    call __DELAY_US_W_
    return
PR@CC
    bsf 0,0,0
    sublw 254
    bz PR@EX
    bsf LATB,2,0
PR@SD
    btfss 0,0,0
PR@LP
    bcf 0,0,0
    bsf LATB,3,0
    movlw 15
    andwf PORTB,F,0
    movf 15,W,0
    andlw 240
    iorwf PORTB,F,0
    bcf LATB,3,0
    swapf 15,F,0
    btfsc 0,0,0
    bra PR@LP
    movlw 75
    call __DELAY_US_
PR@EX
    movf 16,W,0
    return
_BYTE__SEND_
    btfsc 1,7,0
    goto PRINT
I2CIN2
    bsf 0,4,0
    btfss 0,4,0
I2CIN
    bcf 0,4,0
    btfsc 0,5,0
    bra I2CIN@I
    bsf 0,5,0
    rcall I2C@STT
    bc I2C@STP
I2CIN@I
    movlw 8
    movwf 12,0
I2CIN@LP
    rcall I2C@GB
    rlcf 11,F,0
    decfsz 12,F,0
    bra I2CIN@LP
    btfss 0,4,0
    rcall I2C@DL
    rcall I2C@CH
    rcall I2C@CL
    btfsc 0,4,0
    rcall I2C@STP
    rcall I2C@DH
    movf 11,W,0
    bcf 4056,0,0
    return
I2COUT2
    bsf 0,4,0
    btfss 0,4,0
I2COUT
    bcf 0,4,0
    btfsc 0,6,0
    bra I2C@OUTC
    andlw 254
    movwf 8,0
    bsf 0,6,0
    bcf 4056,0,0
    return
I2C@OUTC
    btfsc 0,7,0
    bra I2COUT@I
    movwf 15,0
    bsf 0,7,0
    rcall I2C@STT
    bc I2C@STP
    movf 15,W,0
I2COUT@I
    rcall I2C@OB
    bc I2C@STP
    btfss 0,4,0
    return
I2C@STP
    rcall I2C@DL
    rcall I2C@CH
    bcf 0,6,0
    bcf 0,7,0
    bcf 0,5,0
    rcall I2C@DH
    return
I2C@DH
    setf 4066,0
    incfsz 2,W,0
    movwf 4065,0
    bsf 4065,4,0
    movf 7,W,0
I2C@HH
    iorwf 4068,F,0
    bra I2C@LH
I2C@DL
    setf 4066,0
    movff 2,4065
    comf 7,W,0
I2C@WL
    andwf 4070,F,0
    bsf 4065,4,0
    andwf 4068,F,0
I2C@LH
    return
I2C@STT
    rcall I2C@DH
    rcall I2C@CH
    rcall I2C@DL
    rcall I2C@CL
    movf 8,W,0
    btfsc 0,5,0
    iorlw 1
I2C@OB
    movwf 11,0
    movlw 8
    movwf 12,0
I2COUT@LP
    rlcf 11,F,0
    btfsc 4056,0,0
    rcall I2C@DH
    btfss 4056,0,0
    rcall I2C@DL
    rcall I2C@CH
    rcall I2C@CL
    decfsz 12,F,0
    bra I2COUT@LP
    rcall I2C@DH
I2C@GB
    rcall I2C@CH
    setf 4066,0
    movff 2,4065
    movf 7,W,0
    andwf 4071,W,0
    addlw 255
I2C@CL
    setf 4066,0
    movff 3,4065
    comf 4,W,0
    bra I2C@WL
I2C@CH
    setf 4066,0
I2C@HD
    incfsz 3,W,0
    movwf 4065,0
    bsf 4065,4,0
    movf 4,W,0
    bra I2C@HH
__DELAY_MS_
    clrf 12,0
__DELAY_MS_W_
    movwf 11,0
DLY@P
    movlw 255
    addwf 11,F,0
    addwfc 12,F,0
    bra $ + 2
    btfss 4056,0,0
    return
    movlw 3
    movwf 10,0
    movlw 222
    rcall __DELAY_US_W_
    bra DLY@P
__DELAY_US_
    clrf 10,0
__DELAY_US_W_
    addlw 233
    movwf 9,0
    movlw 252
    bnc $ + 12
    nop
    nop
    addwf 9,F,0
    bc $ - 4
    nop
    addwf 9,F,0
    decf 10,F,0
    bc $ - 12
    btfsc 9,0,0
    bra $ + 2
    btfss 9,1,0
    bra $ + 6
    bra $ + 2
    nop
    return
__DIVIDE_U1616_
    clrf 14,0
    clrf 13,0
__DIVIDE_INT_U1616_
    movlw 16
    movwf 4083,0
DV@LP
    rlcf 10,W,0
    rlcf 13,F,0
    rlcf 14,F,0
    movf 11,W,0
    subwf 13,W,0
    movf 12,W,0
    subwfb 14,W,0
    bnc D@K
    movf 11,W,0
    subwf 13,F,0
    movf 12,W,0
    subwfb 14,F,0
    bsf 4056,0,0
D@K
    rlcf 9,F,0
    rlcf 10,F,0
    decfsz 4083,F,0
    bra DV@LP
    movf 9,W,0
    return
STR@LB1
    db 85,78,69,77,65,84,32,66,66,71,0
proton#main#start
    clrf BPF,0
    movlb 0
F2_SOF equ $ ; I2C.PRP
F2_EOF equ $ ; I2C.PRP
F1_SOF equ $ ; I2C.BAS
F1_000004 equ $ ; IN [I2C.BAS] DELAYMS 20
    movlw 20
    f@call __DELAY_MS_
F1_000006 equ $ ; IN [I2C.BAS] ALL_DIGITAL TRUE
    movlw 7
    movwf ADCON1,0
F1_000025 equ $ ; IN [I2C.BAS] ADDRESS = 20
    clrf AddressH,0
    movlw 20
    movwf Address,0
F1_000026 equ $ ; IN [I2C.BAS] VARWRITE = 150
    clrf VarWriteH,0
    movlw 150
    movwf VarWrite,0
loop
F1_000029 equ $ ; IN [I2C.BAS] CLS
    f@call LCD@CLS
F1_000030 equ $ ; IN [I2C.BAS] VARREAD = 0
    clrf VarReadH,0
    clrf VarRead,0
F1_000031 equ $ ; IN [I2C.BAS] PRINT AT 1, 1, "UNEMAT BBG"
    movlw 128
    movwf BPFH,0
    f@call LCD@CRS
    movlw ((STR@LB1 >> 8) & 255)
    movwf TBLPTRLH,0
    movlw (STR@LB1 & 255)
    movwf TBLPTRL,0
    f@call PRINT@MSTR
F1_000032 equ $ ; IN [I2C.BAS] I2CIN SDA, SCL, %10100001, ADDRESS,[VARWRITE]
    movlw PORTC
    movwf GEN,0
    movlw 16
    movwf GENH,0
    movlw PORTC
    movwf GEN2,0
    movlw 8
    movwf GEN2H,0
    movlw 161
    f@call I2COUT
    movf AddressH,W,0
    f@call I2COUT
    movf Address,W,0
    f@call I2COUT
    f@call I2CIN
    movwf VarWriteH,0
    f@call I2CIN2
    movwf VarWrite,0
F1_000033 equ $ ; IN [I2C.BAS] DELAYMS 500
    movlw 1
    movwf PP1H,0
    movlw 244
    f@call __DELAY_MS_W_
F1_000034 equ $ ; IN [I2C.BAS] I2CIN SDA, SCL, %10100000, ADDRESS,[VARREAD]
    movlw PORTC
    movwf GEN,0
    movlw 16
    movwf GENH,0
    movlw PORTC
    movwf GEN2,0
    movlw 8
    movwf GEN2H,0
    movlw 160
    f@call I2COUT
    movf AddressH,W,0
    f@call I2COUT
    movf Address,W,0
    f@call I2COUT
    f@call I2CIN
    movwf VarReadH,0
    f@call I2CIN2
    movwf VarRead,0
F1_000035 equ $ ; IN [I2C.BAS] PRINT AT 2, 1, DEC VARREAD
    movlw 128
    movwf BPFH,0
    movlw 192
    f@call LCD@CRS
    clrf GEN4H,0
    movff VarReadH,PP2H
    movff VarRead,PP2
    f@call OUT@DEC
F1_000036 equ $ ; IN [I2C.BAS] DELAYMS 500
    movlw 1
    movwf PP1H,0
    movlw 244
    f@call __DELAY_MS_W_
F1_000037 equ $ ; IN [I2C.BAS] GOTO LOOP
    f@jump loop
F1_EOF equ $ ; I2C.BAS
PB@LB2
    bra PB@LB2
__EOF
config DEBUG=off
config WDT=off
config OSC=HS
config OSCS=off
config PWRT=on
config BOR=on
config BORV=27
config WDTPS=128
config CCP2MUX=on
config STVR=off
config LVP=off
config CP0=off
config CP1=off
config CP2=off
config CP3=off
config CPB=off
config CPD=off
config WRT0=off
config WRT1=off
config WRT2=off
config WRT3=off
config WRTB=off
config WRTC=off
config WRTD=off
config EBTR0=off
config EBTR1=off
config EBTR2=off
config EBTR3=off
config EBTRB=off
    end
