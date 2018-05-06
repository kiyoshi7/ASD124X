// ADS.h
#ifndef ADS124X_H
#define ADS124X_H

#include <stdarg.h>


class ADS124X
{
public:
	void PinMap(int cs, int drdy, int reset, int start); // set operation pins if no start/reset pin write 99
	void Initialize(); // start spi and prepare pins
	void EndCOM();
	void Setup(	byte mux1,	// set up basic operation
				byte sys0,
				byte mux0,
				byte idac0,
				byte idac1);
	void ResetHW();	   // if there is a reset pin use it in operation
	void ResetSW();	   // software reset
	void Start();	   // if there is a start pin use it in operation
	int Command(int settings, ...); //  number of settings Variadic function ie. use as many args as needed
	
	int32_t GetData(); //  number of bytes.
	int32_t GetReg(int bytes, byte reg);  //  number of bytes, register to be read (eg: RDATA).
	int32_t OffSetReg(); // gets the offset register value
	int32_t FullScaleReg(); // gets the full-scale calibration value from register
	
	void SetReg(int reg, byte val);  //register and bytes to write
	int Gain();						 // returns value of gain
	void SysOffSetCal();			 // Orders system offset Calibration.
	void SysGainCal();				 // 
	void SelfOCal();
	
	
	// useful functions
	double pt100_conv(int32_t x,  double correction, double Rref, int gain);	// transform raw data to pt100, correction offset ie 0.79586, set gain to 0 if autodetected is to be used
	void SetupInternalTemp();
	double inTemp(int scale); // 0 = F (not implemented); 1 = C; 2 = K(not implemented will return in C)


private:	
	// for pic portability
	void pinc(int pin, int lvl); // digitalwrite()
	void pint(int pin, int type);// pinmode - 0 out, 1 in
	void wait(int time);		 // delay()
	void setSPI();
	void spiWrite(byte x);
	byte spiRead(byte x);
	void spiEnd();	
	bool pinState(byte pin);
};


#endif
