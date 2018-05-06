#include "ADS124X.h"
#include <SPI.h>
#include <Arduino.h>

// ADS1247 pinout
int CS, DRDY, RESET_, START;

// ADS1247 Variables
const int ADResetPulse = 1;          // ADC reset (min. 4x244 = 976 ns, low), 244 ns = 1/4,096 MHz;
const int ADResetDelay = 1;          // ADC delay after reset in ms min 0.6 ms
int mux1settings;
int a =1;

// Save user temperary variables
byte _mux0;
byte _mux1;
byte _sys0;
byte _idac0;
byte _idac1;
byte _gain;

#pragma region Commands

#pragma region ADS 1247, 1248 commands

#define NOP			 0xff
#define NOR			 0x00
#define WAKEUP		 0X00
#define SLEEP	     0X02
#define SYNC		 0X04
#define REST	     0X06    /// double
#define RDATAC		 0X14
#define WRITE	     0X40
#define READ	     0X20
#define RDATA		 0X12
#define RESET		 0x06    /// double
#define STOPDATACONT 0x16
#define SYSOCAL		 0X60
#define SYSGCAL		 0X61
#define SELFOCAL	 0x62

#pragma endregion ADS 1247, 1248 commands

#pragma region  ADS 1247 and ADS1248 Register Map

#define MUX0    0x00
#define VBIAS   0x01
#define MUX1    0x02
#define SYS0    0x03
#define OFC0    0x04
#define OFC1    0x05
#define OFC2    0x06
#define FSC0    0x07
#define FSC1    0x08
#define FSC2    0x09
#define IDAC0   0x0A
#define IDAC1   0x0B
#define GPIOCFG 0x0C
#define GPIODIR 0x0D
#define GPIODAT 0x0E

#pragma endregion  ADS 1247 and ADS1248 Register Map

#pragma region  Multiplexer Control Register 0 Register - MUX0
// Burn-out Detect Current Source Registe
#define BOOFF   0B00000000
#define BO5U    0B01000000
#define BO2U    0B10000000
#define BO10u   0B11000000
// Multiplexer Selection - ADC Positive Input
#define AINP0   0B00000000
#define AINP1   0B00001000
#define AINP2   0B00010000
#define AINP3   0B00011000
#define AINP4   0B00100000   // ADS1248
#define AINP5   0B00101000   // ADS1248
#define AINP6   0B00110000   // ADS1248
#define AINP7   0B00111000   // ADS1248
// Multiplexer Selection - ADC Negative Input
#define AINN0   0B00000000
#define AINN1   0B00000001
#define AINN2   0B00000010
#define AINN3   0B00000011
#define AINN4   0B00000100   // ADS1248
#define AINN5   0B00000101   // ADS1248
#define AINN6   0B00000110   // ADS1248
#define AINN7   0B00000111   // ADS1248
#pragma endregion MUX0

#pragma region Bias Voltage Register

#define VBIASA3DIS  0B00000000
#define VBIASA3EN   0B00001000
#define VBIASA2DIS  0B00000000
#define VBIASA2EN   0B00000100
#define VBIASA1DIS  0B00000000
#define VBIASA1EN   0B00000010
#define VBIASA0DIS  0B00000000
#define VBIASA0EN   0B00000001

#pragma endregion Bias Voltage Register

#pragma region MULTIPLEX CONTROL REGISTER 1 - MUX1

#define CLKSTATINT    0B00000000  // CLOCK STATUS
#define CLKSTATEXT    0B10000000
#define VREFCONOFF    0B00000000  // INTERNAL REFERENCE CONTROL
#define VREFCONON     0B00100000
#define VREFCONCON    0B01000000
#define REFSELT0      0B00000000  // REFERENCE SELECT CONTROL
#define REFSELT1      0B00001000
#define REFSELTINT    0B00010000
#define REFSELTINT0   0B00011000
#define MUXCALNOR     0B00000000  // SYSTEM MONITOR CONTROL
#define MUXCALOFFSET  0B00000001
#define MUXCALGAIN    0B00000010
#define MUXCALTEMP    0B00000011
#define MUXCALREF1    0B00000100
#define MUXCALREF0    0B00000101
#define MUXCALANAL    0B00000110
#define MUXCALDIGI    0B00000111

#pragma endregion MULTIPLEX CONTROL REGISTER 1 - MUX1

#pragma region SYSTEM CONTROL REGISTER 0 - SYS0

#define PGA1    0B00000000  // GAIN SETTING FOR PGA
#define PGA2    0B00010000
#define PGA4    0B00100000
#define PGA8    0B00110000
#define PGA16   0B01000000
#define PGA32   0B01010000
#define PGA64   0B01100000
#define PGA128  0B01110000
#define DR5     0B00000000  // DATA OUTPUT RATE SETTING
#define DR10    0B00000001
#define DR20    0B00000010
#define DR40    0B00000011
#define DR80    0B00000100
#define DR160   0B00000101
#define DR320   0B00000110
#define DR640   0B00000111
#define DR1K    0B00001000
#define DR2K    0B00001001

#pragma endregion SYSTEM CONTROL REGISTER 0

#pragma region IDAC CONTROL REGISTER 0 - IDAC0

#define DRDYONLY  0B00000000  // DATA READY MODE SETTING
#define DRDYREADY 0B00001000
#define IDACOFF   0B00000000  // IDAC EXCITATION CURRENT MAGNITUDE
#define IDAC50    0B00000001
#define IDAC100   0B00000010
#define IDAC250   0B00000011
#define IDAC500   0B00000100
#define IDAC750   0B00000101
#define IDAC1K    0B00000110
#define IDAC1K5   0B00000111

#pragma endregion IDAC CONTROL REGISTER 0 - IDAC0

#pragma region IDAC CONTROL REGISTER 1 - IDAC1

#define I1DIR0    0B00000000  // IDAC EXCITATION CURRENT OUTPUT 1
#define I1DIR1    0B00010000
#define I1DIR2    0B00100000
#define I1DIR3    0B00110000
#define I1DIR4    0B01000000
#define I1DIR5    0B01010000
#define I1DIR6    0B01100000
#define I1DIR7    0B01110000
#define IEXC11    0B10000000
#define IEXC12    0B10010000
#define DISCONEC  0B11000000
#define I2DIR0    0B00000000  // IDAC EXCITATION CURRENT OUTPUT 2
#define I2DIR1    0B00000001
#define I2DIR2    0B00000010
#define I2DIR3    0B00000011
#define I2DIR4    0B00000100
#define I2DIR5    0B00000101
#define I2DIR6    0B00000110
#define I2DIR7    0B00000111
#define IEXC21    0B00001000
#define IEXC22    0B00001001
#define DISCONEC2  0B00001100

#pragma endregion IDAC CONTROL REGISTER 1 - IDAC1

#pragma endregion Commands

#pragma region Basic functions

void ADS::PinMap(int cs, int drdy, int reset, int start)
{
	//CS, DRDY, RESET, START = cs, drdy, reset, start;
	CS = cs;
	DRDY=drdy;
	RESET_=reset;
	START = start;
}

void ADS::Initialize()
{
	// Arduino ADC interface (wire accordingly)
	// inverse means that when pin is low relavent function is active e.g.:
	// DRDY low means ADC has data to send
	pint(CS, 0);
	pint(DRDY, 1);
	pinc(CS, 1);
	if (START != 99)	// if start pin is used
	{
		pint(START, 0);	// PIN TYPE OUTPUT
		pinc(START, 1); // PIN LOGIC STATE 1
	}
	if (RESET != 99)	// if reset pin is used
	{
		pint(RESET, 0); // PIN TYPE OUTPUT
		pinc(RESET, 1); // PIN LOGIC STATE 1
	}
	
	setSPI();
}

void ADS::EndCOM()
{
	spiEnd();
}

void ADS::Setup(byte mux1, byte sys0, byte mux0, byte idac0, byte idac1) // set up basic operation
{
	_mux0 = mux0; _mux1 = mux1; _sys0 = sys0; _idac0 = idac0; _idac1 = idac1; 
	_gain = sys0>>4;
	
	pinc(CS, 0);
	wait(1);
	spiWrite(RESET);
	wait(ADResetDelay);
	spiWrite(STOPDATACONT);
	wait(210);
	
	spiWrite(WRITE | MUX1);	// mux1      ( write command )
	spiWrite(MUX1 ^ SYS0);	// sys0      ( write command )
	spiWrite(mux1);			// user input
	spiWrite(sys0);			// user input
	
	spiWrite(WRITE | MUX0);	// mux0      ( write command )
	spiWrite(NOR);			// no register
	spiWrite(mux0);			// user input
	
	spiWrite(WRITE | IDAC0);
	spiWrite(NOR);			// no register
	spiWrite(idac0);		// user input
	
	spiWrite(WRITE | IDAC1);
	spiWrite(NOR);			// no register
	spiWrite(idac1);		// user input
	
	wait(1);
	pinc(CS, 1);
}

void ADS::ResetHW()
{
	if (RESET_ != 99)
	{
		pinc(RESET_, 0);
		wait(ADResetPulse);
		pinc(RESET_, 1);
		wait(ADResetDelay);
	}
}

void ADS::ResetSW()
{
	pinc(CS, 0);
	spiWrite(RESET);
	pinc(CS, 1);
}

void ADS::Start()
{
	if (START != 99)
	{
		pinc(START, 0);
		wait(10);
		pinc(START, 1);
	}

}

int ADS::Command(int settings, ...)
{
	va_list ap;
	int x;
	int comm = 0;
	va_start(ap, settings); //Requires the last fixed parameter (to get the address)
	for (x=0; x<settings; x++)
	{
		comm = comm | va_arg(ap, int); //Requires the type to cast to.
	}
	va_end(ap);
	return comm;
}

int32_t ADS::GetData()
{   
	while(pinState(DRDY) == 1){;} // wait for data to be ready
	pinc(CS, 0);
	wait(1);
	spiWrite(RDATA);
	int32_t data;
	for (int i=0; i<3; i++)
	{
		data |= spiRead(NOP);
		if (i == 0 | i == 1)
		{ data <<= 8;}
	}
	pinc(CS, 1);
	return (int32_t)data;
	
}

int32_t ADS::GetReg(int bytes, byte reg)
{
	pinc(CS, 0);
	wait(1);
	spiWrite(READ | reg);
	spiWrite(NOR);
	int32_t data;
	for (int i=0; i<bytes; i++)
	{
		data = spiRead(NOP);
	}
	pinc(CS, 1);
	return data;
}

void ADS::SetReg(int reg, byte val)
{
	if (reg == SYS0)
	{
		_gain= val<<4;
	}
	pinc(CS, 0);
	wait(1);
	
	spiWrite(WRITE | reg);
	spiWrite(NOR);
	spiWrite(val);
	
	wait(1);
	pinc(CS, 1);
}

// check mux 1 datasheet
void ADS::SysOffSetCal()
{
	pinc(CS, 0);
	//            flex        flex
	SetReg(MUX1, VREFCONON | REFSELT0 | REFSELTINT | MUXCALOFFSET);
	wait(1);
	pinc(CS, 1);
	while(pinState(DRDY) == 1){;}
	
	SetReg(IDAC1, DISCONEC | DISCONEC);
	wait(1);
	pinc(CS, 1);
	while(pinState(DRDY) == 1){;}
	
	pinc(CS, 0);
	wait(1);
	spiWrite(SYSOCAL);
	wait(10);
	//SetReg(MUX1, _mux1);
	pinc(CS, 1);
	while(pinState(DRDY) == 1){;}
	
	pinc(CS, 0);
	SetReg(MUX1, _mux1);  // change back to usr's settings
	wait(1);
	pinc(CS, 1);
	while(pinState(DRDY) == 1){;}
	
	pinc(CS, 0);
	SetReg(IDAC1, _idac1);  // change back to usr's settings
	wait(1);
	pinc(CS, 1);
	while(pinState(DRDY) == 1){;}
}

// check mux 1 datasheet    // make more flexable for user
void ADS::SysGainCal()
{
	pinc(CS, 0);
	//
	SetReg(MUX1, VREFCONON | REFSELT0 | MUXCALGAIN);
	wait(1);
	pinc(CS, 1);
	while(pinState(DRDY) == 1){;}
	
	pinc(CS, 0);
	spiWrite(SYSGCAL);
	pinc(CS, 1);
	while(pinState(DRDY) == 1){;}
	
	pinc(CS, 0);
	SetReg(MUX1, _mux1);  // change back to usr's settings
	wait(1);
	pinc(CS, 1);
	
	while(pinState(DRDY) == 1){;}
}

// check mux 1 datasheet
void ADS::SelfOCal()
{
	pinc(CS, 0);
	SetReg(MUX1, VREFCONON | REFSELT0 | MUXCALOFFSET);
	wait(1);
	pinc(CS, 1);
	while(pinState(DRDY) == 1){;}
	
	pinc(CS, 0);
	spiWrite(SELFOCAL);
	wait(1);
	pinc(CS, 1);
	while(pinState(DRDY) == 1){;}
	
	pinc(CS, 0);
	SetReg(MUX1, _mux1);  // change back to usr's settings
	wait(1);
	pinc(CS, 1);
	while(pinState(DRDY) == 1){;}
		
}

#pragma endregion Basic functions

#pragma region Useful functions

double ADS::pt100_conv(int32_t x,  double correction, double Rref, int gain)
{
	if (gain=0)
	{
		int gain = _gain;
	}
	
	double data = x;
	data =((data * Rref)/(8388607 * gain)) + correction;
	
	long double a,b,c,delta, A=(3.9083/1000), B=(-5.775/10000000), C=(-4.183/1000000000000);
	int R0=100;
	a = B*R0;
	b = (R0*A);
	c = (R0-data);
	delta=(b*b)-(4*c*a);
	
	return double((-b + sqrt(delta))/(2*a));
}

void ADS::SetupInternalTemp()
{
	Setup(0x33, 0x00, 0x00,0x00,0x00);
}

double ADS::inTemp(int scale)
{
	double data = GetData();
	double mv = (data*2048)/8388607;
	if ( scale == 1 )
	{
		return ((mv - 118)/0.405) + 25;
	}
	
	if (scale == 2)
	{
		return ((mv - 118)/0.405) + 25 + 273.15;
	}
}

int32_t ADS::OffSetReg()
{
	int32_t data;
	data |= GetReg(1, 0x06);
	data <<= 8;
	data |= GetReg(1, 0x05);
	data <<= 8;
	data |= GetReg(1, 0x04);
	
	return data;
}
//   5626610  / 4194304
int32_t ADS::FullScaleReg()
{
	int32_t data;
	data |= GetReg(1, 0x09);
	data <<= 8;
	data |= GetReg(1, 0x08);
	data <<= 8;
	data |= GetReg(1, 0x07);
	
	return data;
}

int ADS::Gain()
{
	return (_gain);
}

#pragma endregion Useful functions

#pragma region Private functions

void ADS::pinc(int pin, int lvl)
{
	if (lvl == 1)
	{
		digitalWrite(pin, HIGH);
	}
	if (lvl == 0)
	{
		digitalWrite(pin, LOW);
	}
}

void ADS::pint(int pin, int type)
{
	if (type == 1)
	{
		pinMode(pin, INPUT);
	}
	if (type == 0)
	{
		pinMode(pin, OUTPUT);
	}
}

void ADS::wait(int time)
{
	delay(time);
}

void ADS::setSPI()
{
	// Arduino SPI settings (change first line according to IC specifications)
	// defines SPI communication type and initializes module
	SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1)); // SPI setup SPI communication
	wait(200);          // SPI delay for initialization
	SPI.begin();         // SPI Module 'wake up'
}

void ADS::spiWrite(byte x)
{
	SPI.transfer(x);
}

byte ADS::spiRead(byte x)
{
	return SPI.transfer(x);
}

void ADS::spiEnd()
{
	SPI.end();
	SPI.endTransaction();
}

bool ADS::pinState(byte pin)
{
	if (digitalRead( pin ) == true) { return true; }
	if (digitalRead( pin ) == false){ return false;	}
}
#pragma endregion Private functions

