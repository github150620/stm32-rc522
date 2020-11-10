

// Table 5: MFRC522 Registers Overview
// Page 0: Command and Status
#define     Reserved00            0x00    
#define     CommandReg            0x01    
#define     ComIEnReg             0x02    
#define     DivlEnReg             0x03    
#define     ComIrqReg             0x04    
#define     DivIrqReg             0x05
#define     ErrorReg              0x06    
#define     Status1Reg            0x07    
#define     Status2Reg            0x08    
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved0F            0x0F
// Page 1: Command    
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved1A            0x1A
#define     Reserved1B            0x1B
#define     MifareReg             0x1C
#define     Reserved1D            0x1D
#define     Reserved1E            0x1E
#define     SerialSpeedReg        0x1F
// Page 2: CFG    
#define     Reserved20            0x20  
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     Reserved23            0x23
#define     ModWidthReg           0x24
#define     Reserved25            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsCfgReg            0x28
#define     ModGsCfgReg           0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
// Page 3: TestRegister    
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39  
#define     TestDAC2Reg           0x3A   
#define     TestADCReg            0x3B   
#define     Reserved3C            0x3C   
#define     Reserved3D            0x3D   
#define     Reserved3E            0x3E   
#define     Reserved3F		  	    0x3F

// Table 16:
#define    TxIRq         0x40
#define    RxIRq         0x20
#define    IdleIRq       0x10
#define    HiAlertIRq    0x08
#define    LoAlertIRq    0x04
#define    ErrIRq        0x02
#define    TimerIRq      0x01

// Table 19: Description of ErrorReg bits
#define    WrErr         0x80
#define    TempErr       0x40
//         -             0x20
#define    BufferOvfl    0x10
#define    CollErr       0x08
#define    CRCErr        0x04
#define    ParityErr     0x02
#define    ProtocolErr   0x01


// Table 149: Command overview
#define    Idle              0x00
#define    Mem               0x01
#define    GenerateRandmID   0x02
#define    CalcCRC           0x03
#define    Transmit          0x04
#define    NoCmdChange       0x07
#define    Receive           0x08
#define    Transceive        0x0C
#define    MFAuthent         0x0E
#define    SoftReset         0x0F

// depend on GPIO and SPI
void RC522_Reset      (void);
u8   RC522_Read       (u8 addr);
void RC522_Write      (u8 addr, u8 value);

// depend on read and write
void RC522_SetRegBits    (u8 reg, u8 mask);
void RC522_ClearRegBits  (u8 reg, u8 mask);

// 
void RC522_Init             (void);
void RC522_Config_ISO14443A (void);
void RC522_AntennaOff       (void);
void RC522_AntennaOn        (void);


