

#include <Arduino.h>
#include <i2c_t3.h>
#include <INA226.h>
#include <EEPROM.h>
#include <FlexCAN.h>
#include <NMEA2000_teensy.h>
#include <NMEA2000_CAN.h>       // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include "kalman.h"

INA226 ina;  // I2C current shunt monitor and bus voltage 

#define EEPROM_ADR_CONFIG 0 // eeprom address for config params
#define BatteryInstance 1
#define DCInstance 1
#define UNKNOWN 0
#define ATREST 1
#define CHARGING 2
#define LOAD 3

float startSoC = 0;
float estsoc = 0;
double BatteryWH;
double FullBatteryWH;
int state = ATREST;
int laststate = UNKNOWN;
unsigned long laststatechange = 0;
//static float estCapacity;
static unsigned long lastreading = 0;
int output_level = 0;
static float Peukert = 1.250;

// custom settings to store in non-volatile memory
// N.B. change the MAGIC when ever tConfig is modified
struct tConfig {
	uint16_t Magic; //test if eeprom initialized
	char Version[8];
	char Name[10];
	uint8_t DeviceInstance;
	uint32_t UniqueNumber;			// Hardware Serial Number
	uint16_t  HeartbeatInterval;	// N * 0.01s, valid range 1 to 655.32s, 0 = disabled
	tN2kBatType BatteryType;			// 0 flooded, 1 Gel , 2 AGM
	tN2kBatChem BatteryChem;
	uint8_t BatteryCapacity;
	uint8_t EstimatedBatteryCapacity;
	tN2kBatNomVolt NominalVoltage;			// 6, 12, 24, 32, 36, 42, and 48 Volts.
	double LowVoltage;	  // Voltage at 20% charge	
	double HalfVoltage;   // Voltage at 50% charge
	double FullyChargedOCVoltage;
	double TemperatureCoefficient;	// 0%/?C ? 5%/?C.
	double PeukertExponent;		//  For lead-acid batteries, the value of the Peukert constant is in the range of 1.10 ? 1.25
	double ChargeEfficiencyFactor;	// between 5% and 100%
	double FullyChargedVoltage;		
	double FullyChargedCurrent;		// Amps
	long FullyChargedTime;			// seconds
	float ShuntResistance;		// A/mV 100A/100mV
	float MaxExpectedCurrent;
};

tConfig config;

// Default config 
#define MAGIC 24150  // random number chosen to detect tConfig version stored in NV memory
const tConfig defConfig PROGMEM = {
	MAGIC,
	"1.00.00", 		//version
	"Battery", 		//name
	1,				//DeviceInstance
	1,				//UniqueNumber
	0,			//HeartbeatInterval *0.01s - range 0 to 655.32s 
	N2kDCbt_Flooded,	// BatteryType
	N2kDCbc_LeadAcid,   // BatteryChem
	220,				// Amp-Hours
	220,				// est capacity
	N2kDCbnv_12v,		// NominalVoltage
	11.8,		// LowVoltage 20% DEAD
	12.12,		// Voltage at 50% charge
	12.75,		// Full charge OC voltage
	0.5,			//TemperatureCoefficient
	1.250,			// PeukertExponent
	0.75,			//ChargeEfficiencyFactor
	14.4,			// FullyChargedVoltage;
	0.7,			// FullyChargedCurrent;
	3600,			// FullyChargedTime  
	0.0010045,				// ShuntResistance A/mV
	81.9175				// MaxExpectedCurrent can't get any more out of shunt R
};

const tNMEA2000::tProductInformation BatteryMonitorProductInformation PROGMEM={
	1301,                       // N2kVersion
	100,                        // Manufacturer's product code
	"Battery monitor",			// Manufacturer's Model ID
	"1.0.0.0 (2016-11-07)",     // Manufacturer's Software version code
	"1.0.0.0 (2016-11-03)",     // Manufacturer's Model version
	"00000001",                 // Manufacturer's Model serial code
	0,                          // CertificationLevel
	1                           // LoadEquivalency
};                                      

const char BatteryMonitorManufacturerInformation[] PROGMEM = "Mr Bubble";
const char BatteryMonitorInstallationDescription1[] PROGMEM = "Battery Monitor";
const char BatteryMonitorInstallationDescription2[] PROGMEM = "";

 
// PGNReceive[] = { 126464L ,126992L, 126208L };
// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM = { 126996L, 126998L, 127508L ,127513L, 60928L, 65240L,0 };

static unsigned long slowloop = 1;
#define BatUpdatePeriod 1000 //1.0s
#define SlowUpdatePeriod 60 //60s
#define FastUpdatePeriod 30 //  50=20 ms
int samples = 0; // debug count number of samples in BatUpdatePeriod
unsigned long FastUpdate = millis();
unsigned long BattUpdated = millis();
unsigned char SID = 1;
double voltage = 0;
double avgVolt = 0; 
double avgCurrent = 0;
double current = 0;
double ahcharge = 0;
double ahload = 0;
//static double temperature = 0;
double RippleVoltage = 0;
//double BatteryCaseTemperature;
double StateOfCharge;
double ChargeEfficiencyFactor;
double EstimatedBatteryCapacity;
float TimeRemaining = -1; //unknown, minutes
const double sec2Hr = 0.00027777777;  //Convert watt-sec to Watt-Hrs
double power;
double bCharge;  //power variable (watt-hours) total effective charge to and from the battery
float LowVoltage;  //low battery value
float absorbCtr, chargeCtr, disChargeCtr, eqCtr, voltCount, ampCount, voltRatio, ampRatio;
float absorbTimeOut = 60 * 60 * 3.0;//3 hours in seconds
float eqTimeOut = 60 * 60 * 3.0;  //3 Hours in seconds

float RippleHigh, RippleLow;
// voltage kalman
#define PROCESS_NOISE 0.0000001   // q process noise covariance
#define SENSOR_NOISE 0.00001 // r measurement noise covariance
#define INITIAL_Q 500.0  // p estimation error covariance
// current kalman
double Q = 0.000001;
double R = 0.0001;// 0.2;
double P = 1000;
uint8_t FullBatteryAh;
//kalman_state filter;
Kalman vFilter(PROCESS_NOISE, SENSOR_NOISE, INITIAL_Q, 10);
Kalman aFilter(Q, R, P, 0);

// I2C read current sensor
void ReadINA226()
{
	voltage = ina.readBusVoltage();
	current = ina.readShuntCurrent();
	avgVolt = vFilter.getFilteredValue(voltage);
	avgCurrent = aFilter.getFilteredValue(current);
	if (voltage > RippleHigh) { RippleHigh = voltage; }
	if (voltage < RippleLow) { RippleLow = voltage; }
	RippleVoltage = RippleHigh - RippleLow;
}

void setup() {
	delay(500);
	pinMode(LED_BUILTIN, OUTPUT);    // LED
	digitalWrite(LED_BUILTIN, LOW);  // LED off
	Serial.begin(115200);
	Blink(2, 200);
	// read stored device settings
	ReadConfig();
	if (config.Magic != MAGIC) {
		Serial.println("Writing EEPROM changes.");
		InitializeEEPROM();
	}

	Serial.println("Starting.. letting other stuff boot up.");
	delay(5000); // let other stuff boot up so the power consumption is minimized

	//DumpConfig();
	//INA226 current shunt sensor, I2C address is 0x40
	ina.begin();
	ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_588US, INA226_SHUNT_CONV_TIME_8244US, INA226_MODE_SHUNT_BUS_CONT);
	ina.calibrate(config.ShuntResistance, config.MaxExpectedCurrent); // Calibrate INA226. Rshunt = 0.001 ohm, Max expected current = 80A
	// Set Product information
	NMEA2000.SetProductInformation(&BatteryMonitorProductInformation );
	// Set Configuration information
	NMEA2000.SetProgmemConfigurationInformation(BatteryMonitorManufacturerInformation, BatteryMonitorInstallationDescription1, BatteryMonitorInstallationDescription2);
	// Set device information
	NMEA2000.SetDeviceInformation(1,      // Unique number. Use e.g. Serial number.
		170,    // Device function=Battery. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
		35,     // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
		2046    // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
		);
	NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,62);
	NMEA2000.EnableForward(false);        // Disable all msg forwarding to USB (=Serial)
	NMEA2000.ExtendTransmitMessages(TransmitMessages);
	NMEA2000.SetN2kCANMsgBufSize(4);        
	NMEA2000.Open();
	
	Peukert = config.PeukertExponent;
	ChargeEfficiencyFactor = config.ChargeEfficiencyFactor;
	//EstimatedBatteryCapacity = config.EstimatedBatteryCapacity;

	FullBatteryAh = config.BatteryCapacity;
	FullBatteryWH = (float)FullBatteryAh*config.FullyChargedOCVoltage;

	//Read current and voltage over next 60ms
	ReadINA226(); RippleLow = RippleHigh = voltage;
	delay(10); ReadINA226(); delay(10); ReadINA226(); delay(10); ReadINA226(); delay(10); ReadINA226(); ReadINA226(); delay(10); ReadINA226(); 
	
	// assume system has been off for a while and this is a good O.C. voltage, temp would be good here
	// force an OC Soc calculation
	startSoC = 100 - ((config.FullyChargedOCVoltage - avgVolt) * 80.808); //SoC - 20%=11.76v, 50%=12.12v, 80%=12.5v, 100%=12.75v
	StateOfCharge = startSoC;
	BatteryWH = FullBatteryWH * startSoC/100; //adjust kWh
	bCharge = BatteryWH;// * avgVolt; 	//init charge 95%  full battery 
	EstimatedBatteryCapacity = BatteryWH / avgVolt; // Ah
	debugPrint();
	Print();
	delay(3000);
	SendBatteryConfig();
	lastreading = millis(); // init countcoulombs()
	FastUpdate = millis();
	BattUpdated = millis();
}

void loop() {	
	// fast loop timer
	if (FastUpdate + FastUpdatePeriod < millis()) {
		FastUpdate = millis();
		ReadINA226();
		samples++;	
		//countcoulombs();
	}
	// slow loop 1000ms
	if (BattUpdated + BatUpdatePeriod < millis()) {
		BattUpdated = millis();
		SendBattery(SID, avgVolt, avgCurrent, 0);
		calcPower();
		calculatestate();
		//estsoc = getsoc();  // SoC Ah delta from power on
		StateOfCharge = 100 * bCharge / FullBatteryWH; // W/h based SoC	
		if (StateOfCharge > 100) { StateOfCharge = 100; }
		if (StateOfCharge < 0) { StateOfCharge = 0; }

		RippleVoltage = RippleHigh - RippleLow;
		RippleLow = RippleHigh = avgVolt;
		SendN2kDCStatus(StateOfCharge, TimeRemaining, RippleVoltage);
		Print();
		
		slowloop++;
		if (slowloop > SlowUpdatePeriod) { slowloop = 1; VerySlowLoop(); }
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		samples = 0;
		SID++; if (SID > 254) { SID = 1; }
	}
	NMEA2000.ParseMessages();
}

// 60s
void VerySlowLoop()
{
	debugPrint();
	SendBatteryConfig();
	NMEA2000.SendIsoAddressClaim();
}

//Calc power
void calcPower() {
	double power = avgVolt * avgCurrent;  //Units are watt-seconds
	if (power <= 0) {
		bCharge += power * sec2Hr;
	}
	else {
		bCharge += power * sec2Hr * ChargeEfficiencyFactor;
	}
	if (bCharge < 0) {bCharge = 0;}
	if (bCharge > FullBatteryWH) {bCharge = FullBatteryWH;}
}

//fudge factors
#define ChargeCoulombicEfficiency  0.6
#define DischargeCoulombicEfficiency  0.5

//// return an estimated SoC based on last real reading and coloumb count
//double getsoc() {
//	if (state == CHARGING) {
//		return (double)startSoC / 100 - (((ahload - ahcharge) / BatteryWH));// *((100 - pow(1.05, StateOfCharge))*ChargeCoulombicEfficiency + 40) / 100);//* 0.94)
//	}
//	else {
//		return (double)startSoC / 100 - (((ahload - ahcharge) / BatteryWH));// *(2 - ((100 - StateOfCharge*(DischargeCoulombicEfficiency / 5)*DischargeCoulombicEfficiency) / 100)));//* 0.92)
//	}
//}


//void countcoulombs() {
//	if (avgCurrent == 0) {
//		return;
//	}
//	int interval = millis() - lastreading;
//
//	if (avgCurrent < 0) { // load
//		ahload += (pow(-avgCurrent, Peukert)  * ((double)interval / 3600000)); // milliseconds in an hour
//	}
//	else { // charge
//		ahcharge += (avgCurrent *  ChargeEfficiencyFactor * ((double)interval / 3600000)); // milliseconds in an hour
//	}
//	lastreading = millis();
//}


void calculatestate() {
	if (-0.1 < avgCurrent && avgCurrent < 0.1) {
		state = ATREST;
	}
	else if (avgCurrent < 0) {
		state = LOAD;

	}
	else if (avgCurrent > 0) {
		state = CHARGING;
	}

	if (state != laststate) {
		laststatechange = millis();
		laststate = state;
	}
}

void Print() {
	Serial.print(" Samples: "); Serial.print(samples); Serial.println(" s/s");
	Serial.print(" "); Serial.print(avgVolt); Serial.println("V");
	Serial.print(" "); Serial.print(avgCurrent); Serial.println("A");
	Serial.print(" "); Serial.print(avgVolt * avgCurrent); Serial.println(" W");
	Serial.print(" "); Serial.print(StateOfCharge, 2); Serial.println("%");
	Serial.print(" STATE: ");
	switch (state)
	{
		case UNKNOWN: Serial.println("UNKNOWN");
			break;
		case ATREST: Serial.println("ATREST");
			break;
		case CHARGING: Serial.println("CHARGING");
			break;
		case LOAD: Serial.println("LOAD");
			break;
		default:
			break;
	}
	Serial.println();
}

void debugPrint()
{
	Serial.print("SID:"); Serial.println(SID);
	Serial.print(" V:"); Serial.println(voltage);
	Serial.print(" A:"); Serial.println(current);
	Serial.print(" Ah: "); Serial.print(FullBatteryAh); Serial.println(" Ah");
	//Serial.print(" eAh:"); Serial.println(EstimatedBatteryCapacity);
	Serial.print(" Wh: "); Serial.print(FullBatteryAh*config.FullyChargedOCVoltage); Serial.println(" Wh");
	Serial.print(" Adj Wh: "); Serial.print(BatteryWH); Serial.println(" Wh");
	Serial.print(" Est Ah: "); Serial.print(bCharge / avgVolt); Serial.println(" Ah remaining");
	Serial.print(" CCe: "); Serial.print(((100 - pow(1.05, StateOfCharge))*ChargeCoulombicEfficiency + 40)); Serial.println("%");
	Serial.print(" DCe: "); Serial.print(2 - ((100 - StateOfCharge*(DischargeCoulombicEfficiency / 5)*DischargeCoulombicEfficiency) / 100)); Serial.println("%");
	Serial.print(" Ripple: "); Serial.print(RippleVoltage); Serial.println(" vpp");
	//Serial.print(" ahload: "); Serial.println(ahload, 4);
	//Serial.print(" ahcharge: "); Serial.println(ahcharge, 4);
	Serial.print(" OCsoc: "); Serial.print(startSoC, 2); Serial.println("% SoC Start- from OC voltage at power on");
	//Serial.print(" estsoc: "); Serial.print(estsoc * 100, 4); Serial.println("% SoC - Ah delta from power on");
	Serial.print(" bCharge: "); Serial.print(bCharge, 4); Serial.println(" W/h");
	if (ina.isMathOverflow()) { Serial.println("OVERFLOW"); }
	if (ina.isAlert()) { Serial.println("ALERT"); }
}

// if OC for T reset SoC
void calculatesoc() {
	if (state == ATREST) {
		if (
			(laststate == LOAD && (laststatechange + 300000 < millis())) ||
			(laststate == CHARGING && (laststatechange + 3600000 < millis()))
		) {			
			// voltage based soc
			startSoC = 100 - ((config.FullyChargedOCVoltage - avgVolt) * 80.808); //100 - ((12.80 - voltage) * 100);
			BatteryWH = (int)(BatteryWH * startSoC / StateOfCharge);
			//Serial.println("estsoc:");
			//Serial.println(estsoc);
			//Serial.println("real soc:");
			//Serial.println(soc);
			//Serial.println("ahload:");
			//Serial.println(ahload);
			//Serial.println("ahcharge:");
			//Serial.println(ahcharge);
			//Serial.println("new capacity:");
			//Serial.println(capacity);
			//Serial.println("");
			ahload = 0;
			ahcharge = 0;
		}
	}
	if (state == LOAD && (laststatechange + 300000 < millis()))
	{
		if (avgVolt < config.HalfVoltage) Serial.println("LOW BATTERY");
		if (avgVolt < config.LowVoltage) Serial.println("CRITICAL BATTERY");
	}
}


void DumpConfig()
{
	Serial.printf("BatteryCapacity:\t%d\n", config.BatteryCapacity);
	Serial.printf("BatteryChem:\t%d\n", config.BatteryChem);
	Serial.printf("BatteryType:\t%d\n", config.BatteryType);
	Serial.printf("ChargeEfficiencyFactor:\t%d\n", config.ChargeEfficiencyFactor);
	Serial.printf("DeviceInstance:\t%d\n", config.DeviceInstance);
	Serial.printf("FullyChargedCurrent:\t%f\n", config.FullyChargedCurrent);
	Serial.printf("FullyChargedTime:\t%f\n", config.FullyChargedTime);
	Serial.printf("FullyChargedVoltage:\t%f\n", config.FullyChargedVoltage);
	Serial.printf("HeartbeatInterval:\t%d\n", config.HeartbeatInterval);
	Serial.printf("Name:\t%s\n", config.Name);
	Serial.printf("NominalVoltage:\t%f\n", config.NominalVoltage);
	Serial.printf("PeukertExponent:\t%f\n", config.PeukertExponent);
	Serial.printf("ShuntResistance:\t%f\n", config.ShuntResistance);
}



// PGN to transmit unchanging battery configuration data
void SendBatteryConfig() {
	tN2kMsg N2kMsg;
	SetN2kBatConf(N2kMsg, BatteryInstance, config.BatteryType, N2kDCES_No, config.NominalVoltage, config.BatteryChem, AhToCoulomb(config.BatteryCapacity), config.TemperatureCoefficient, config.PeukertExponent, config.ChargeEfficiencyFactor);
	NMEA2000.SendMsg(N2kMsg);
}

//PGN to transmit slowly changing Battery Data
void SendBattery(unsigned char Instance, float BatteryVoltage, float BatteryCurrent, double BatteryTemperature) {
	tN2kMsg N2kMsg;
	SetN2kDCBatStatus(N2kMsg, BatteryInstance, BatteryVoltage, BatteryCurrent, BatteryTemperature, SID);
	NMEA2000.SendMsg(N2kMsg);
}

// PGN to transmit slowly changing DC and Battery Data
void SendN2kDCStatus(unsigned char StateOfCharge, long TimeRemaining, double RippleVoltage) {
	tN2kMsg N2kMsg;
	SetN2kDCStatus(N2kMsg, SID, DCInstance, N2kDCt_Battery, StateOfCharge, 0, TimeRemaining, RippleVoltage);
	NMEA2000.SendMsg(N2kMsg);
}

// LED blinker
// count flashes in duration ms
void Blink(int count, unsigned long duration)
{
	unsigned long d = duration / count;
	for (int counter = 0; counter < count; counter++) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(d / 2);
		digitalWrite(LED_BUILTIN, LOW);
		delay(d / 2);
	}
}

//Load From EEPROM 
void ReadConfig()
{
	EEPROM.get(EEPROM_ADR_CONFIG, config);
}

//Write to EEPROM - Teensy non-volatile area size is 2048 bytes  100,000 cycles
void UpdateConfig()
{
	EEPROM.put(EEPROM_ADR_CONFIG, config);
	Blink(5, 2000);
}

void InitializeEEPROM()
{
	//for(unsigned int i = 0; i < EEPROM.length(); i++) EEPROM.write(i, 0);	//EEPROM clear
	config = defConfig;
	UpdateConfig();
	Blink(10, 3000);
}
