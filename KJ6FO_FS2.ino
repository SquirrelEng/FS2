//
//
// Flying Squirrel #2 flight computer software
//
// Author Don Gibson KJ6FO

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
#define DEBUGMESSAGES 0
#define DEBUGSCHEDULE 0
int MsgType = 1; // DEBUG

// Libraries
#include <OneWire.h>
#include <TinyGPS.h>
#include <si5351.h>
#include <KJ6FOWSPR.h>

//#include <rs_commonKJ6FO.h>
//#include <intKJ6FO.h>
#include <string.h>



//#define CALIBRATIONADJUST  -18627L // Measured from calibration program.  Proto board 1
#define CALIBRATIONADJUST  -7800L // Measured from calibration program. FS2 Flight Hardware Board #2

// Mode defines

//
// WSPR
//
#define WSPR_DEFAULT_FREQ       14095600UL + 1400UL  //Hz  Base(Dial) Freq plus audio offset to band pass bottom edge.
#define WSPR_DEFAULT_FREQ_ADJ         40L // 40Hz up from bottom edge of 200hz passband 
#define WSPR_TONE_SPACING       14648+5   // ~1.4648 Hz + 5 to handle interger rounding.
#define WSPR_TONE_DIVISOR       100       // Divisor to bring the number to 1.46-ish
#define WSPR_DELAY              682667UL     // Delay nS value for WSPR .682667s  minus time it takes to change frequency (Measured varies by CPU board, but 5 seems to work weel for most)
#define WSPR_POWER_DBM				13    // 13 dbm = 20 Milliwatts          
#define WSPRType1 1  // Should be  enums, but having compiler issues
#define WSPRType2 2
#define WSPRType3 3


//
// FSQ
//
#define FSQ_DEFAULT_FREQ        (14097000UL + 1350UL - 80UL)    // HZ Base freq is 1350 Hz higher than dial freq in USB  //Board2
#define FSQ_TONE_SPACING        879          // ~8.79 Hz
#define FSQ_TONE_DIVISOR        1            // Divisor of 1 will leave TONE_SPACING unchanged
#define FSQ_2_DELAY             500000UL         // nS Delay value for 2 baud FSQ
#define FSQ_3_DELAY             333000UL         // nS Delay value for 3 baud FSQ
#define FSQ_4_5_DELAY           222000UL         //nS Delay value for 4.5 baud FSQ
#define FSQ_6_DELAY             167000UL         // nS Delay value for 6 baud FSQ


//
// Hardware defines
//
#define XMITLED_PIN             13
#define RADIOPOWER_PIN			11
#define GPSPOWER_PIN			10
#define DS18B20PIN				12		
#define OFF 0
#define ON 1



//
// Global Class instantiations
//
Si5351 si5351;
KJ6FOWSPR jtencode;
TinyGPS gps;
OneWire  ds(DS18B20PIN);  // on pin A3 (a 4.7K resistor is necessary)

//
// Global variables
//

char MyCallsign[] = "MYCALL/B";

uint8_t RadioPower_dbm = WSPR_POWER_DBM; 
#define TXBUFFERSIZE 255
uint8_t tx_buffer[TXBUFFERSIZE];


//
// Global GPS Info
//
// Current values of time and position etc.
long LastLat = -1.0;  // Last reported Lat
long LastLon = -1.0;  // Last reported Long
long CurrLat = 0.0;  // Current Lat
long CurrLon = 0.0;  // Current Long
long CurrAlt = 0;    // Current Altitude
char CurrentGridSquare[7];  // 6 digit grid square
float CurrentTemp;
volatile int CurrFix = 0; //  Fix number (i.e. # of fixes from GPS)
volatile int CurrDays = 0; //  # of Days
volatile int CurrHours = 0; //  # of Hours
volatile int CurrMinutes = 0;
volatile int CurrSeconds = 0;
int LastSecond = -1;   // used by time debug messages in loop() 

//
//Battery Voltage  & Power management
//
// The Voltage threshold value we have to have to stay running.
#define CUTOFFVOLTAGE 2.8

// The treshold voltage value we need to get started up from a boot.
#define TURNONVOLTAGE 2.9

float CurrentVolts = 0; // Measured battery voltage
bool bRadioIsOn = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// CODE ////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

//
//  Setup() Lets get going!!!
//
void setup()
{
	// Set up serial port
	Serial.begin(9600); // Baudrate determined by GPS module

	// Enable internal reference so we can measure battery voltage.
	analogReference(INTERNAL);// Use internal 1.1v reference for analog pins.

	// Use the Arduino's on-board LED as a keying indicator.
	pinMode(XMITLED_PIN, OUTPUT);
	XmitIndicator(OFF);

	// Init Power pins
	pinMode(RADIOPOWER_PIN, OUTPUT);
	RadioPower(OFF);
	pinMode(GPSPOWER_PIN, OUTPUT);
	GPSPower(OFF);



	// We may be restarting from a cold dead battery that is begining to charge from the solar cells at first light.
	// Wait until we have built up enough voltage for a fix to succeed and get stuff running.  If the voltage is too low
	// the GPS will kill the battery before getting a fix and possibly cause reboot again in an near endless cycle.
	GetCurrentVolts(); // First battery Reading is always high. Read it and move on
	delay(1000);
	GetCurrentVolts(); // This will be a more accurate reading.
	while (CurrentVolts < TURNONVOLTAGE)  // Just loop here until we have enough power to get going.
	{
		GPSPower(OFF); // Make sure GPS Stays off, it may have tried to start and the MCU crashed leaving it on.
		GetCurrentVolts();
		delay(10000); // 10 sec
	}

	// Get our initial position and get the time clock up and running.
	GetFix(20);
}


//
// The Loop()
//
void loop()
{

	// Only process once each second
	if (CurrSeconds != LastSecond)
	{
		LastSecond = CurrSeconds;

		Serial.print(CurrMinutes);Serial.print(":");Serial.println(CurrSeconds);

		if (CurrSeconds == 59) // get a voltage reading
		{
			GetCurrentVolts(); // Get a voltage reading
		}

		if (CurrentVolts > CUTOFFVOLTAGE) // Do we have enough power to do stuff?
		{
			if (CurrSeconds == 0)  // Top of the minute
			{

				// Process the schedule
				int SchedMinute = CurrMinutes % 30; // # of minutes into schedule (30 Min repeating schedule)


#if DEBUGSCHEDULE
				//DEBUG SCHEDULE
				//  Cycles through modes one per time slot, faster to debug than waiting on the flight schedule 
				//
				if (CurrMinutes % 2 == 0)
				{
					if (MsgType == 1)
					{
						MsgType++;
						XmitWSPR(WSPRType2, true);
					}
					else if(MsgType==2)
					{
						MsgType++;
						XmitWSPR(WSPRType3, false);
					}
					else
					{
						GetFix(10);
					}
				}
				else
				{
					MsgType = 1;
					XmitFSQ(FSQ_3_DELAY,true);
				}
#else
				switch (SchedMinute)
				{
					// FSQ 3
					case 1:
					case 17:
					{
						XmitFSQ(FSQ_3_DELAY,true);
					}
					break;

					// WSPR Type 2
					case 2:
					case 18:
					{
						
						XmitWSPR(WSPRType2);
					}
					break;

					// WSPR Type 3
					case 4:
					case 20:
					{
						XmitWSPR(WSPRType3);
					}
					break;

					//

					// Long GPS fix time
					case 10: // One per schedule period, run the GPS longer so it has time to update Ephemeris/Almanac Data from sats. This will keep the GPS running faster
					{
						GetFix(40);
					}
					break;

					// All the rest
					default:
					{
						if (CurrMinutes % 2 == 0)  // Even minutes
						{
							GetFix(10); // Quick Fix.
							
						}
						else // odd minutes
						{
							XmitFSQ(FSQ_6_DELAY,false);
						}
					}
					break;
				}
#endif
			}
		}
	}
}


//
//  Encode and send FSQ telemetry.
//
void XmitFSQ(unsigned long BaudRate,bool bLeaveRadioOn)
{

	StartRadio();  // Init Radio.
	delay(1000);

	// Clear out the transmit buffer
	memset(tx_buffer, 0, TXBUFFERSIZE);

	// KJ6FO/B FS#2 FFFFF HHH:MM 999999.9m -99.9999,-999.9999 -99.99c 9.99v  // Message format

	// Update Temp 
	GetCurrentTemp();

	// Compose Telemetry message
	String Message = "[fs2.txt]hab ";
	Message.concat(CurrFix);
	Message.concat(" ");
	Message.concat(CurrDays);
	Message.concat(":");
	if (CurrHours < 10)
	{
		Message.concat("0"); //leading zero
	}
	Message.concat(CurrHours);
	Message.concat(":");
	if (CurrMinutes < 10)
	{
		Message.concat("0"); //leading zero
	}
	Message.concat(CurrMinutes);
	Message.concat(" ");
	Message.concat(CurrAlt);
	Message.concat("m ");
	Message += String(CurrLat / 1000000.0, 6);
	Message.concat(",");
	Message += String(CurrLon / 1000000.0, 6);
	Message.concat(" ");
	Message.concat(CurrentTemp);
	Message.concat("c ");
	Message += String(CurrentVolts, 2);
	Message.concat("v     \b"); // Blank spaces to clean up end of string on receiver display, due to slow squelch and extraneous chars. \b is EOT char

	// Encode the message into FSQ Symbols
	uint8_t symbol_count = jtencode.fsq_dir_encode(MyCallsign, "allcall", '#', Message.c_str(), tx_buffer);

	SendTelemetry(FSQ_DEFAULT_FREQ,tx_buffer, symbol_count,FSQ_TONE_SPACING, FSQ_TONE_DIVISOR, BaudRate, true);

	if (!bLeaveRadioOn)
	{
		RadioPower(OFF);  //Power down Radio
	}
	
}

//
//  Send WSPR reports.
//
void XmitWSPR(int MessageType)
{
	bool bLeaveRadioOn = false;
	StartRadio(); // Init Radio.
	delay(1000); // WSPR starts at 1 sec into minute. + ?a little extra to allow for GPS errors. Dont want to start too early.

	GetCurrentTemp(); // required for Frequency temp adjustments

	uint8_t symbol_count = WSPR_SYMBOL_COUNT; // From the library defines

	// Clear out the transmit buffer
	memset(tx_buffer, 0, TXBUFFERSIZE);

	// Encode message type
	switch (MessageType)
	{
		// Not using WSPR Type 1 message
		//case WSPRType1:
		//	//Serial.println("WSPR Type 1");
		//	//jtencode.wspr_encode(MyCall, MyGridSquare, RadioPower_dbm, tx_buffer);  // Type 1
		//	break;

	case WSPRType2:
	{
		Serial.println("WSPR Type 2");
		jtencode.wspr_encodeType2(MyCallsign, RadioPower_dbm, tx_buffer); // Type 2		
		bLeaveRadioOn = true; // A type 3 will follow so leave radio running and keep warm.
	}
	break;

	case WSPRType3:
	{
		Serial.println("WSPR Type 3");
		jtencode.wspr_encodeType3(MyCallsign, CurrentGridSquare, RadioPower_dbm, tx_buffer);  // Type 3
	}
	break;
	}

	SendTelemetry(WSPR_DEFAULT_FREQ + WSPR_DEFAULT_FREQ_ADJ ,tx_buffer, symbol_count, WSPR_TONE_SPACING, WSPR_TONE_DIVISOR,WSPR_DELAY, false);

	if (!bLeaveRadioOn)
	{
		RadioPower(OFF);  //Power down Radio
	}
}

//
// Xmit the encoded telemetry symbols
//
void SendTelemetry(unsigned long XmitFrequency, uint8_t *tx_buffer, int symbol_count, uint16_t tone_spacing, uint16_t tone_divisor, unsigned long tone_delay, bool bTuningGuide)
{
	unsigned long millicount = 0L; // DEBUG

	// Start Transmitting.
	XmitIndicator(ON);  // Turn on the ON AIR light.
	si5351.output_enable(SI5351_CLK0, 1);  // Radio xmit enabled. 

	uint64_t AdjustedXmitFreq = (XmitFrequency * SI5351_FREQ_MULT);  //Convert to 100th Hz
	AdjustedXmitFreq -= CalcFreqDriftDelta(CurrentTemp); //adjust for temp

	// Radio oscillator drifts up in freq as ambient temp cools down. Need time to fine tune signal on Rx.
	if (bTuningGuide) // Xmit tuning guide?
	{

		// Xmit tuning guide tone for FSQ
		Serial.println("Tune Start.");
		si5351.set_freq(AdjustedXmitFreq, SI5351_CLK0);
		delay(5000);  // 5 Seconds.

	}

	// Now transmit the channel symbols
	for (int i = 0; i < symbol_count; i++)
	{
		unsigned long starttime = micros(); // Start time
		si5351.set_freq(AdjustedXmitFreq + ((tx_buffer[i] * tone_spacing)/ tone_divisor), SI5351_CLK0);

		while ((micros() - starttime) < tone_delay) {}; // Spin until time is up
		
	}

	// Turn off the radio outout
	si5351.output_enable(SI5351_CLK0, 0);
	XmitIndicator(OFF);  // Turn off xmit light

}


//
// Set up interrupts for timer
//
// reference http://www.instructables.com/id/Arduino-Timer-Interrupts/
//
void SyncClockInterrupts()
{
	cli();//stop interrupts

	//set timer1 interrupt at 1Hz
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1 = 0;//initialize counter value to 0
			  // set compare match register for 1hz increments
	//OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)  16Mhz Based systems
	OCR1A = 7812;// = (8*10^6) / (1*1024) - 1 (must be <65536)   8Mhz Based systems

				  // turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS10 and CS12 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	sei();//allow interrupts
}

//
// One second (1Hz) clock tick ISR
//
ISR(TIMER1_COMPA_vect)   
{
	CurrSeconds++; //Bump up Seconds
	if (CurrSeconds == 60) 
	{
		CurrSeconds = 0;
		CurrMinutes++;
		if (CurrMinutes == 60)
		{
			CurrMinutes = 0;
			CurrHours++;
			if (CurrHours > 24)
			{
				CurrDays++;
			}
		}
	}
}




//
// Get position altitude andtime fix from GPS
//
// nFixes should be 45 or less.
void GetFix(int nFixes)
{
#if DEBUGMESSAGES
	Serial.println("GetFix()");
#endif 

	// Turn on GPS
	GPSPower(ON);

	// Time vars
	int year;
	byte month;
	byte day;
	byte hour;
	byte minute;
	byte second = 0;
	byte hundredths;


	unsigned long fix_age = TinyGPS::GPS_INVALID_AGE;
	int lastsecond  = -1;
	int countseconds = -1;
	
	// Vars for timout detection
	int LastCurrSeconds = CurrSeconds;
	int TimeoutSeconds = 0;

	while (fix_age == TinyGPS::GPS_INVALID_AGE || countseconds < nFixes)
	{
		// Check the running system clock for timeout.
		if (LastCurrSeconds != CurrSeconds)
		{
			LastCurrSeconds = CurrSeconds;
			TimeoutSeconds++;
			if (TimeoutSeconds >= 120) // 2 mins without success
			{
				GPSPower(OFF);
				Serial.println("Timeout.");  //DEBUG
				return;  // Bailout
			}
		}


		//Get GPS Data
		while (Serial.available() > 0)
		{
			char ch = Serial.read();
			//Serial.print(ch);  //DEBUG Only
			gps.encode(ch);
		}

		// Decode the time
		gps.crack_datetime(&year, &month, &day,
			&hour, &minute, &second, &hundredths, &fix_age);


		if (fix_age != TinyGPS::GPS_INVALID_AGE)  // We have a valid fix
		{
			if (lastsecond != second)  // The seconds changed
			{
				lastsecond = second;
				countseconds++;         // Bump count

				Serial.print("Cnt=");Serial.print(countseconds);  Serial.print(" Sec=");Serial.println(second);  //DEBUG

			}
		}
	}


	SyncClockInterrupts(); // Refresh Clock interval to sync with GPS in case of drift.	

	// 4th Dimention is time.
	CurrSeconds = second;	// Update seconds
	CurrMinutes = minute;	// Update minute
	CurrHours = hour;		// Update hour
	CurrDays = day;			// Update day

	// 3D cooridinates
	gps.get_position(&CurrLat, &CurrLon);
	CurrAlt = gps.altitude() / 100; // Centimeters to meters
	Calc6DigitGridSquare(CurrentGridSquare, CurrLat / 1000000.0, CurrLon / 1000000.0);

#if DEBUGMESSAGES
	//Serial.print("Min=");Serial.println(CurrMinutes);
	Serial.print(CurrLat);Serial.print(",");Serial.println(CurrLon);
	Serial.print("A=");Serial.println(CurrAlt);
	Serial.print("H=");Serial.println(gps.hdop());
	Serial.print("G=");Serial.println(CurrentGridSquare);
#endif

	CurrFix++; // Bump Fix count

	// Turn GPS Off
	GPSPower(OFF);
}


// GridSquare6Digits should be 7 bytes long to allow for null terminator.
void Calc6DigitGridSquare(char *GridSquare6Digits, double lat, double lon)
{
	int o1, o2, o3;
	int a1, a2, a3;
	double remainder;

	// longitude
	remainder = lon + 180.0;
	o1 = (int)(remainder / 20.0);
	remainder = remainder - (double)o1 * 20.0;
	o2 = (int)(remainder / 2.0);
	remainder = remainder - 2.0 * (double)o2;
	o3 = (int)(12.0 * remainder);

	// latitude
	remainder = lat + 90.0;
	a1 = (int)(remainder / 10.0);
	remainder = remainder - (double)a1 * 10.0;
	a2 = (int)(remainder);
	remainder = remainder - (double)a2;
	a3 = (int)(24.0 * remainder);
	GridSquare6Digits[0] = (char)o1 + 'A';
	GridSquare6Digits[1] = (char)a1 + 'A';
	GridSquare6Digits[2] = (char)o2 + '0';
	GridSquare6Digits[3] = (char)a2 + '0';
	GridSquare6Digits[4] = (char)o3 + 'A';
	GridSquare6Digits[5] = (char)a3 + 'A';
	GridSquare6Digits[6] = (char)0;
}

// Read the temp DS18B20 sensor
void GetCurrentTemp()
{
	byte i;
	byte present = 0;
	byte data[9];
	float celsius;

	ds.reset();				// Reset wakes up the bus for a command.
	ds.write(0xCC);			//Skip ROM Command. Just activate the only deviceon the line 
	ds.write(0x44, 1);        // Start conversion, with parasite power on at the end. Result goes to scratchpad memory

	delay(1000);     // Wait for the conversion to take place.  750ms is the spec'ed max time needed, give it a little bit more time.

					 // Read the scratchpad memory
	present = ds.reset();
	ds.write(0xCC);         // Skip ROM
	ds.write(0xBE);         // Read Scratchpad

							// 9 Bytes of data should be ready to read, so get it.
	for (i = 0; i < 9; i++)
	{
		data[i] = ds.read();
	}


	// Convert the first two bytes to an 16bit int.
	int16_t raw = (data[1] << 8) | data[0];

	// Convert to a float value.
	CurrentTemp = (float)raw / 16.0;  // Degrees C
}


//
// Computes predicted frequency drift due to temperature changes
// Returns 100th of Hz
//
int CalcFreqDriftDelta(float TempC)
{

	Serial.print("C=");Serial.println(TempC);
	float TempCSquared = TempC * TempC;

	// Formula derrived from curve fit from test data. Polynomial curve order of 2;

	//  Formula: -3.132026502�10-3�x2�- 5.491572347 x�+ 136.3524712
	double fy = (-0.003132026502*TempCSquared) - (5.491572347*TempC) + 136.3524712;  //Predicted Freq shift (HZ)
	Serial.print("Calc drift=");Serial.println(fy);

	
	int iy = fy * SI5351_FREQ_MULT;  // Convert to 100th Hz
	Serial.print("Drift Delta(hz)=");Serial.println(fy);

	return iy;

}


//
//  Power Management etc.
//

//
// Get the current battery voltage
//
void GetCurrentVolts()
{
	// Reading the voltage from a 5:1 voltage divider using internal 1.1 v reference.
	// Chips vary as well as the resistors, so hand tune the scale value to match an
	// actual voltage as mesured with a volt meter.
	int v = analogRead(0);
	CurrentVolts = (v / 1023.0) * 6.35; // Scale hand tweaked to get calibrated result matching volt meter.
#if DEBUGMESSAGES
	Serial.print("V=");Serial.println(CurrentVolts);
#endif
}

//
// Get the radio board set up and ready to transmit
//
void StartRadio()
{

	if (!bRadioIsOn)  // Only start radio if it is off.
	{
		RadioPower(ON);
		delay(50); // the radio needs a slight "Warm up" time, after power up

		//si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);  //Etherkit Boards
		si5351.init(SI5351_CRYSTAL_LOAD_10PF, 0, 0); // Adafruit boards

													 // Set CLK0 output
		si5351.set_correction(CALIBRATIONADJUST); // Measured value from calibration program
		si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
		si5351.set_freq(WSPR_DEFAULT_FREQ, SI5351_CLK0); // Get radio on freq. Use WSPR freq for now.
		si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
		si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially
		Serial.println("si5351 Started");
	}
	else // Nothing to do, leave radio alone.
	{
		Serial.println("si5351 already started.");
	}
	
	
}

//
// Turn Radio Power switch on/off.  
//
void RadioPower(int OnOff)
{
	Serial.print("R PWR ");Serial.println(OnOff);
	digitalWrite(RADIOPOWER_PIN, OnOff);
	bRadioIsOn = OnOff;
}

//
// ON THE AIR indicator LED
//
void XmitIndicator(int OnOff)
{
	digitalWrite(XMITLED_PIN, OnOff);
}

//
// Turn GPS power switch on/off.
//
void GPSPower(int OnOff)
{

	digitalWrite(GPSPOWER_PIN, OnOff);
}
