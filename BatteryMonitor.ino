

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
#define BULK 4
#define ABSORB 5
#define FLOAT 6

float soc = 0;
float estsoc = 0;
float capacity = 220;
int state = UNKNOWN;
int laststate = UNKNOWN;
long laststatechange = 0;
float ahcharge = 0;
float ahload = 0;
float estCapacity;
long lastreading = 0;
int output_level = 0;
float peukert_c = 1.250;
float charge_c = 1.2;

// custom settings to store in non-volatile memory
// N.B. change the MAGIC when ever tConfig is modified
#define MAGIC 24140  // random number chosen to detect tConfig version stored in NV memory
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
	uint8_t ChargeEfficiencyFactor;	// between 5% and 100%
	double FullyChargedVoltage;		
	double FullyChargedCurrent;		// Amps
	long FullyChargedTime;			// seconds
	/*The ?Fully Charged Voltage? indicates the value voltage at which the battery is
	considered fully charged if the battery voltage remains above this value and the battery current
	remains below the ?Fully Charged Current? for the amount of time defined by the ?Fully 	Charged Time? parameter.
	*/
	float ShuntResistance;		// A/mV 100A/100mV
	float MaxExpectedCurrent;
};

tConfig config;

// Default config 
const tConfig defConfig PROGMEM = {
	MAGIC,
	"1.00.00", 		//version
	"Battery", 		//name
	1,				//DeviceInstance
	1,				//UniqueNumber
	0,			//HeartbeatInterval *0.01s - range 0 to 655.32s 
	N2kDCbt_Flooded,	// BatteryType
	N2kDCbc_LeadAcid,   // BatteryChem
	200,				// Amp-Hours
	200,				// est capacity
	N2kDCbnv_12v,		// NominalVoltage
	11.8,		// LowVoltage
	12.12,		// Voltage at 50% charge
	12.75,		// Full charge OC voltage
	0.5,			//TemperatureCoefficient
	1.250,			// PeukertExponent
	75,				//ChargeEfficiencyFactor
	14.4,			// FullyChargedVoltage;
	0.7,			// FullyChargedCurrent;
	3600,			// FullyChargedTime  
	0.0010045,				// ShuntResistance A/mV
	81.9175				// MaxExpectedCurrent
};

const tProductInformation BatteryMonitorProductInformation PROGMEM={
	1301,                       // N2kVersion
	100,                        // Manufacturer's product code
	"Battery monitor",			// Manufacturer's Model ID
	"1.0.0.0 (2016-11-07)",     // Manufacturer's Software version code
	"1.0.0.0 (2016-11-03)",     // Manufacturer's Model version
	"00000001",                 // Manufacturer's Model serial code
	0,                          // CertificationLevel
	1                           // LoadEquivalency
};                                      

const tNMEA2000::tProgmemConfigurationInformation BatteryMonitorConfigurationInformation PROGMEM={
	"Mr Bubble", // Manufacturer information
	"Battery Monitor", // Installation description1
	"" // Installation description2
};
 
//uint32_t PGNReceive[] = { 126464L ,126992L, 126208L };
//uint32_t PGNSend[] = { 126996L, 126998L, 127508L ,127513L ,59392L, 59904L, 60928L, 65240L };


// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM = { 126996L, 126998L, 127508L ,127513L, 60928L, 65240L,0 };


long slowloop = 1;
long veryslowloop = 0;
#define BatUpdatePeriod 1000 //1.0s
#define SlowUpdatePeriod 60 //60s
#define FastUpdatePeriod 50// 20 ms
static unsigned long FastUpdate = millis();
static unsigned long BattUpdated = millis();
//static unsigned long ConfigUpdated = millis()-2000; // ConfigUpdated = millis() - ConfigUpdatePeriod;
static unsigned char SID = 1;
static double voltage = 0;
static double avgVolt = 0; 
static double avgCurrent = 0;
static double current = 0;
//static double temperature = 0;
static double RippleVoltage = 0;
//static double BatteryCaseTemperature;
static double StateOfCharge;
//static uint8_t  TimeRemainingFloor;  // default set to 50%
static double TimeRemaining; //minutes

float sec2Hr = 1.0 / (3600.0);  //Convert watt-sec to Watt-Hrs
float amps_reverse, power, watts;
float bCharge;  //power variable (watt-hours) total effective charge to and from the battery
float LowVoltage;  //low battery value
float absorbCtr, chargeCtr, disChargeCtr, eqCtr, voltCount, ampCount, voltRatio, ampRatio;
int bStatus;  //Battery status, 0-6 states

float absorbTimeOut = 60 * 60 * 3.0;//3 hours in seconds
float eqTimeOut = 60 * 60 * 3.0;  //3 Hours in seconds

float RippleHigh, RippleLow;

#define PROCESS_NOISE 0.0000001   // q process noise covariance
#define SENSOR_NOISE 0.00001 // r measurement noise covariance
#define INITIAL_Q 500.0  // p estimation error covariance

int samples = 0;
double Q = 0.000001;
double R = 0.0001;// 0.2;
double P = 1000;

//kalman_state filter;
Kalman vFilter(PROCESS_NOISE, SENSOR_NOISE, INITIAL_Q, 10);
Kalman aFilter(Q, R, P, 0);

// I2C read current sensor
void ReadINA226()
{
	voltage = ina.readBusVoltage();
	current = ina.readShuntCurrent();
}

void setup() {
	
	pinMode(LED_BUILTIN, OUTPUT);    // LED
	digitalWrite(LED_BUILTIN, LOW);  // LED off
	Serial.begin(115200);
	delay(500);
	Blink(2, 200);
	// read stored device settings
	ReadConfig();
	if (config.Magic != MAGIC) {
		InitializeEEPROM();
	}
	DumpConfig();
	//INA226 current shunt sensor, I2C address is 0x40
	ina.begin();
	ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_588US, INA226_SHUNT_CONV_TIME_8244US, INA226_MODE_SHUNT_BUS_CONT);
	//Serial.print("MaxExpectedCurrent:"); Serial.println(config.MaxExpectedCurrent);
	ina.calibrate(config.ShuntResistance, config.MaxExpectedCurrent); // Calibrate INA226. Rshunt = 0.001 ohm, Max expected current = 80A
	//Serial.print("CAL:"); Serial.println(ina.readCal());
	//Serial.print("currentLSB:"); Serial.println(ina.currentLSB,12);
	//NMEA2000.SetHeartbeatInterval(0);
	// Set Product information
	NMEA2000.SetProductInformation(&BatteryMonitorProductInformation );
	// Set Configuration information
	NMEA2000.SetProgmemConfigurationInformation(&BatteryMonitorConfigurationInformation );
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
	capacity = config.BatteryCapacity;
	bCharge = config.BatteryCapacity * 0.95; //init charge 95%  full battery 

	absorbCtr = 0;
	eqCtr = 0;
	chargeCtr = 60 * 9;
	disChargeCtr = 0;
	//bLow = 0.5;  //Low battery warning when hattery 50% charged
	//bStatus = 7;//This a an error value must be updated in Calculate bStatus
	ReadINA226();
	RippleLow = RippleHigh = voltage; 
	avgVolt = vFilter.getFilteredValue(voltage);
	avgCurrent = aFilter.getFilteredValue(current);
	estCapacity = config.EstimatedBatteryCapacity;
	SendBatteryConfig();
	lastreading = millis();
}

void loop() {	
	
	// fast loop timer
	if (FastUpdate + FastUpdatePeriod < millis()) {
		FastUpdate = millis();
		ReadINA226();
		avgVolt = vFilter.getFilteredValue(voltage);
		avgCurrent = aFilter.getFilteredValue(current);
		if (voltage > RippleHigh) {
			RippleHigh = voltage;
		}
		if (voltage < RippleLow) {
			RippleLow = voltage;
		}
		samples++;	
		countcoulombs(-avgCurrent);
		//Serial.print(voltage);
		//Serial.print("\t"); Serial.println(avgVolt);//Serial.print(","); 
		//Serial.print(current,4);
		//Serial.print("\t"); Serial.println(avgCurrent,4);

	}
	if (BattUpdated + BatUpdatePeriod < millis()) {
		BattUpdated = millis();
		RippleVoltage = RippleHigh - RippleLow;
		RippleLow = RippleHigh = avgVolt;

		
		//calcbStatus();
		calculatestate(avgCurrent);
		calcPower(avgVolt, avgCurrent);
		//calculatesoc();
		//displayStatus();
		//outputdebug();
		estsoc = getsoc();
		soc = 100 - ((config.FullyChargedOCVoltage - avgVolt) * 80.808);

		//capacity = (int)(capacity * soc / estsoc);


		StateOfCharge = 100 * bCharge / config.BatteryCapacity;
		

		SendBattery(SID, avgVolt, avgCurrent, 0);
		SendN2kDCStatus(StateOfCharge, TimeRemaining, RippleVoltage);
		Print();
		SID++; if (SID > 254) { SID = 1; }
		slowloop++;
		if (slowloop > SlowUpdatePeriod) { slowloop = 1; SlowLoop(); }
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		samples = 0;
	}
	NMEA2000.ParseMessages();
}

void SlowLoop()
{
	Serial.println("\nISO");
	SendBatteryConfig();
	NMEA2000.SendIsoAddressClaim();
}


//Calc power
void calcPower(double voltage, double current) {
	power = voltage * current;  //Units are watt-seconds
	if (power <= 0) {
		bCharge = bCharge + (power * sec2Hr);
	}
	else {
		bCharge = bCharge + (power * sec2Hr * (config.ChargeEfficiencyFactor / 100));
	}
	if (bCharge < 0) { bCharge = 0; }
}

// return an estimated SoC based on last real reading and coloumb count
float getsoc() {
	return (float)soc - ((ahload - ahcharge) / config.BatteryCapacity);
}


void countcoulombs(double current) {
	//Serial.print("d."); Serial.println(pow(current, peukert_c),8);
	if (current == 0) {
		return;
	}
	int interval = millis() - lastreading;

	if (current > 0) { // load
		ahload += (pow(current, peukert_c)  * interval / 3600000); // milliseconds in an hour
		TimeRemaining = (bCharge / current) * 60;// bCharge / power;
	}
	else { // charge
		ahcharge += (current *  charge_c * interval / 3600000); // milliseconds in an hour
		TimeRemaining = 0;
		//ahcharge += (current *  (config.ChargeEfficiencyFactor / 100) * interval / 3600000); // milliseconds in an hour
	}

	lastreading = millis();
}


void calculatestate(double current) {
	if (-0.1 < current && current < 0.1) {
		state = ATREST;
	}
	else if (current < 0) {
		state = LOAD;

	}
	else if (current > 0) {
		state = CHARGING;
	}

	if (state != laststate) {
		laststatechange = millis();
		laststate = state;
	}
}

void Print() {
	Serial.print("SID:"); Serial.println(SID);
	Serial.print(" Ah:"); Serial.println(config.BatteryCapacity);
	Serial.print(" eAh:"); Serial.println(config.EstimatedBatteryCapacity);
	Serial.print(" V:"); Serial.println(voltage);
	Serial.print(" kV"); Serial.println(avgVolt);
	Serial.print(" A:"); Serial.println(current);
	Serial.print(" kA:"); Serial.println(avgCurrent);
	Serial.print(" W:"); Serial.println(voltage*current);
	Serial.print(" R: "); Serial.println(RippleVoltage);
	Serial.print(" S: "); Serial.println(samples);
	Serial.print(" W: "); Serial.println(power);
	Serial.print("ahload:");	Serial.println(ahload,8);
	Serial.print("ahcharge:");	Serial.println(ahcharge,8);
	Serial.print(" soc: "); Serial.println(soc);
	Serial.print(" estsoc: "); Serial.println(estsoc);
	Serial.print(" capacity: "); Serial.println(capacity);

	Serial.print(" StateOfCharge: "); Serial.print(StateOfCharge,2); Serial.println("%");
	Serial.print(" bCharge: "); Serial.print(bCharge, 4); Serial.println(" W/h");
	Serial.print(" TimeRemaining: "); Serial.print(TimeRemaining*60,0); Serial.print("s ");	Serial.print(TimeRemaining, 2); Serial.print("m ");	Serial.print(TimeRemaining/60, 1); Serial.println("h ");
		
	if (bCharge < config.BatteryCapacity*voltage) {
		Serial.print(StateOfCharge, 1);
		Serial.println("%");
	}
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
		case BULK: Serial.println("BULK");
			break;
		case ABSORB: Serial.println("ABSORB");
			break;
		case FLOAT: Serial.println("FLOAT");
			break;
		default:
			break;
	}
	Serial.println();
	if (ina.isMathOverflow()) { Serial.println("OVERFLOW"); }
	if (ina.isAlert()) { Serial.println("ALERT"); }
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


//
void calculatesoc() {
	if (state == ATREST) {
		if ((laststate == LOAD && (laststatechange + 300000 < millis())) ||
			laststate == CHARGING && (laststatechange + 3600000 < millis())) {
			int estsoc = getsoc();
			soc = 100 - ((config.FullyChargedOCVoltage - avgVolt) * 80.808); //100 - ((12.80 - voltage) * 100);
			capacity = (int)(capacity * soc / estsoc);
			Serial.println("estsoc:");
			Serial.println(estsoc);
			Serial.println("real soc:");
			Serial.println(soc);
			Serial.println("ahload:");
			Serial.println(ahload);
			Serial.println("ahcharge:");
			Serial.println(ahcharge);
			Serial.println("new capacity:");
			Serial.println(capacity);
			Serial.println("");
			ahload = 0;
			ahcharge = 0;
		}
	}
	if (state == LOAD && avgVolt < config.LowVoltage && (laststatechange + 300000 < millis())) 
	{
		Serial.println("CRITICAL BATTERY");
	}
	if (state == LOAD && avgVolt < config.HalfVoltage && (laststatechange + 300000 < millis()))
	{
		Serial.println("LOW BATTERY");
	}
}



//void calcbStatus() {
//	bStatus = 0; //This charge/discharge
//	if (current >= 0)
//		chargeCtr = chargeCtr + 1;
//	if (current < 0) {
//		disChargeCtr = disChargeCtr + 1;
//	}
//
//	if (absorbCtr < 0)
//		absorbCtr = 0;
//	if (voltage > 14.15) {
//		bStatus = ATREST;
//		absorbCtr = absorbCtr + 1;
//	}
//	if (voltage > 13.65 && absorbCtr > absorbTimeOut)
//		bStatus = CHARGING;
//}


void outputdebug() {

	Serial.print(soc);
	Serial.print(",");
	Serial.print(getsoc());
	Serial.println("");
	Serial.flush();
}
//
//void displayStatus() {
//	Serial.println("");
//	watts = voltage * current;
//	Serial.print("Battery State: "); Serial.println(bCharge);
//	if (bCharge < config.BatteryCapacity) {
//		Serial.print(100 * bCharge / config.BatteryCapacity, 1);
//		Serial.println("%");
//	}
//	else {
//		Serial.println("Full ");
//		bCharge = config.BatteryCapacity;
//	}
//	switch (bStatus) {
//	case 0:
//		if(current >= 0)
//			Serial.print("Charge   ");
//		else
//			Serial.print("Discharge "); 
//		Serial.print(watts, 1);
//		Serial.print("W ");
//		break;
//	case 1:
//		Serial.println("Absorb   ");
//		break;
//	case 2:
//		Serial.println("Float    ");
//		break;
//	default:
//		Serial.println("Error    ");
//	}
//	if (bCharge < bCharge * bLow) {//Battery Low blink
//
//			Serial.println("Low");
//	}
//
//	if (voltage > 14.6 && eqCtr < eqTimeOut) {
//		Serial.print("Equalize ");
//		eqCtr = eqCtr + 1;
//	}
//	Serial.println("");
//}

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
