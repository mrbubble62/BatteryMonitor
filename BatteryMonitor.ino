#include <i2c_t3.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <FlexCAN.h>
#include <NMEA2000_teensy.h>
#include <NMEA2000_CAN.h>       // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <INA226.h>

INA226 ina;  // I2C current shunt monitor and bus voltage 

#define EEPROM_ADR_CONFIG 0 // eeprom address for config params
#define BatteryInstance 1
#define DCInstance 1
#define MAGIC 24131  // random number chosen to detect tConfig version stored in NV memory



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
	tN2kBatNomVolt NominalVoltage;			// 6, 12, 24, 32, 36, 42, and 48 Volts.
	double TemperatureCoefficient;	// 0%/?C ? 5%/?C.
	double PeukertExponent;		// 10-15 PeukertExponentx10 For lead-acid batteries, the value of the Peukert constant is in the range of 1.10 ? 1.25
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
	2,				//DeviceInstance
	1,				//UniqueNumber
	0,			//HeartbeatInterval *0.01s - range 0 to 655.32s 
	N2kDCbt_Flooded,	// BatteryType
	N2kDCbc_LeadAcid,   // BatteryChem
	110,				// Amp-Hours
	N2kDCbnv_12v,		// NominalVoltage
	0.5,			//TemperatureCoefficient
	1.250,			// PeukertExponent x10
	75,				//ChargeEfficiencyFactor
	14.4,			// FullyChargedVoltage;
	0.7,			// FullyChargedCurrent;
	3600,			// FullyChargedTime  
	0.0010045,				// ShuntResistance A/mV
	80				// MaxExpectedCurrent
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
 
uint32_t PGNReceive[] = { 126464L ,126992L, 126208L };
uint32_t PGNSend[] = { 126996L, 126998L, 127508L ,127513L ,59392L, 59904L, 60928L, 65240L };

#define BatUpdatePeriod 680 //0.68s
#define ConfigUpdatePeriod 60050 // 68s

static unsigned long BattUpdated = millis();
static unsigned long ConfigUpdated = millis()-2000; // ConfigUpdated = millis() - ConfigUpdatePeriod;
static unsigned char SID = 1;
static double voltage = 0;
static double current = 0;
static double temperature = 0;
static double RippleVoltage = 0;
static double BatteryCaseTemperature;
static unsigned char StateOfCharge;
static uint8_t  TimeRemainingFloor;  // default set to 50%
static long TimeRemaining; //minutes

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
	//DumpConfig();
	//INA226 current shunt sensor, I2C address is 0x40
	ina.begin();
	ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_588US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
	ina.calibrate(config.ShuntResistance, config.MaxExpectedCurrent); // Calibrate INA226. Rshunt = 0.001 ohm, Max expected current = 80A

	NMEA2000.SetHeartbeatInterval(0);
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
	NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,44);
	NMEA2000.EnableForward(false);        // Disable all msg forwarding to USB (=Serial)
	NMEA2000.SetN2kCANMsgBufSize(4);        
	NMEA2000.Open();
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

void loop() {	
	// fast loop timer
	if (BattUpdated + BatUpdatePeriod < millis()) {
		BattUpdated = millis();
		ReadINA226();
		//Serial.print(voltage);
		//Serial.print(":");
		//Serial.print(current);
		//Serial.println();
		SendBattery(1,voltage, current, temperature);
		//SendN2kDCStatus(StateOfCharge, TimeRemaining, RippleVoltage);
		Blink(1, 2);
	}
	//slow loop timer
//	if (ConfigUpdated + ConfigUpdatePeriod < millis()) {
//		if (config.HeartbeatInterval != NMEA2000.GetHeartbeatInterval()) {
//			config.HeartbeatInterval = NMEA2000.GetHeartbeatInterval();
//			UpdateConfig();
//		}
//		ConfigUpdated = millis();
//		SendBatteryConfig();
//		
//	}
	NMEA2000.ParseMessages();
}

// PGN to transmit unchanging battery configuration data
void SendBatteryConfig() {
	tN2kMsg N2kMsg;
	SetN2kBatConf(N2kMsg, BatteryInstance, config.BatteryType, N2kDCES_No, config.NominalVoltage, config.BatteryChem, AhToCoulomb(110), 53, config.PeukertExponent, config.ChargeEfficiencyFactor);
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
