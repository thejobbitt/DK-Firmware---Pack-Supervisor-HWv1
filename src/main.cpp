// DK Supervisor PCB production code for Teensy 3.2
// Compiled with VS Code and PlatformIO

#define VERSION 0123   // EXAMPLE: 5215 = 52th week of 2015

// ****************************************************************************CONFIGURATION
/*
 * Cell Type
 * Cell types are in celltypes.h, if you need to add a cell do so in this file
 * you can find the info in the cells datasheet.
 */
  //#define LG_MH1
  #define LG_MJ1
  //#define SANYO_NCR18650B

/**
 * Pack Settings
 * 
 */
  #define NUMBER_OF_BLOCK_MANAGERS  20  
  #define NUMBER_OF_S_CELLS         40  // number of block managers x 2
  #define NUMBER_OF_P_CELLS         30  // default: 10 (each block has 10 per clamp)

  // ---------SERVER ADDRESS History
  // 1 and 9 => reserved for lab use
  // 60 used for Kents Beetle
  // DONT use 0 or 255! ---does not work for manchester encoding!
  #define SERVER_ADDRESS 60  

/**
 * DK Supervisor
 * 
 */
  #define DK_SUPER_HW_V1
  //#define DK_SUPER_HW_V2

  #ifdef DK_SUPER_HW_V1
    #define RF24
  #endif
  #ifdef DK_SUPER_HW_V2
    #define SUBG
  #endif

/**
 * Chargers 
 * 
 */
  // No Control charger
    #define CHARGER_RELAY_ON_OFF
  // Analog style 0 to 5v
    #define CHARGER_ANALOG_DAC2
    #ifdef CHARGER_ANALOG_DAC2
      #define ANALOG_ON   5 // fully on voltage
      #define ANALOG_OFF  2 // fully off voltage
    #endif
  // 72v CAN Bus Charger
    //#define CHARGER_MYBLUESKY_S2500

/**
 * Displays
 * 
 */
  // SPI display
    //#define DISPLAY_SPI
  // KOSO Meter Fuel Gauge
    //#define DISPLAY_KOSO_DL03

// **************************************************************************** //
//                ALL BASIC CONFIGURATIONS ARE DONE ABOVE                       //
//      DO NOT EDIT ANYTHING BELOW UNLESS YOU KNOW WHAT YOU ARE DOING           //
// **************************************************************************** //

// ****************************************************************************INCLUDES
#include <Arduino.h>
#include <RHReliableDatagram.h>     // https://github.com/andrewbierer/RadioString
#ifdef RF24
  #include <RH_NRF24.h>             // https://github.com/PaulStoffregen/RadioHead
#endif
#include <FlexCAN.h>                // https://github.com/teachop/FlexCAN_Library
#include <SPI.h>                    // https://github.com/PaulStoffregen/SPI
#include <EEPROM.h>                 // https://github.com/PaulStoffregen/EEPROM
#include <elapsedMillis.h>          // https://github.com/pfeerick/elapsedMillis

#include <NCP18.h>                  // NTC to 12bit ADC Reference
#include <celltypes.h>              // Cell Specifcations

// ****************************************************************************APPLICATION VAR
#pragma region APP
  // app states
#define DK_SLEEP    0
#define DK_PAUSE    1
#define DK_CHARGE   2
#define DK_DRIVE    3
uint8_t dkMode = DK_SLEEP; // default to sleep

  // fault states
#define ERROR_NOFAULT         0
#define ERROR_OVERTEMP        1
#define ERROR_UNDERTEMP       2
#define ERROR_OVERVOLT_BLOCK  3
#define ERROR_OVERVOLT_CELL   4
#define ERROR_UNDERVOLT_BLOCK 5
#define ERROR_UNDERVOLT_CELL  6
#define ERROR_DEADV_BLOCK     7
#define ERROR_DEADV_CELL      8
uint8_t  dkError = ERROR_NOFAULT; // default to no faults

bool relay_charge_state = 0;
bool relay_drive_state  = 0;
const int pwm_freq = 40000;
const int FULLPWMRANGE = 1000;
  // structures
struct celltypes cell;

const float HYSTERESIS = 0.4;    // provide hysteresis to prevent relay chatter and oscillation
int sensorValue = 0;        // value read from the ADC input

float Vnominal = NUMBER_OF_S_CELLS * cell.volt_Nominal; // Nominal pack voltage with cells at Vnom

float Vpack = Vnominal;  // start with nominal battery voltage
float gSOCv = Vnominal;   // init with a nominal value

// output SOC signal from op amp PWM - digital DAC outputs
  // 60-80V Vpack ==> 0-100% for now, VOLTAGE IS NOT A GOOD FUEL GAUAGE, but ...
  //  120-160 Vpack ==> 0-100% for now...OK till we have something better.
  float TempV = Vnominal;    // temp pack Volts for SOC computations
  int SOCv;     // SOC based on voltage
  const int MAX_PWM = 1023;
  //const int MIN_PWM = 1;

  // serial ouput
elapsedMillis serialDelayTimer  = 0;    // limits the serial out put to make it more readable
uint16_t      serialDelay       = 1000; // in milliseconds

// debug serial out is not delayed
  //#define CHARGER_DEBUG
  //#define CAN_DEBUG
bool PRINT = 0;   // 0 = no debug print
bool VerbosePrintBLOCKDATA = PRINT;  // turn on (1) or off verbose prints
bool VerbosePrintSUPERDATA = PRINT;  // turn on (1) or off verbose prints for supervisor
bool VerbosePrintEEDATA = PRINT;  // turn on (1) or off verbose prints
bool VerbosePrintCommsDATA = PRINT ;

  // software real time clock vars
unsigned long currentmicros = 0;
unsigned long nextmicros = 0;
unsigned long interval = 1000278UL; // about 1000000 uS (increase constant to slow clock)
int seconds = 0;                    // at reset, set clock to 0:00:00
int minutes = 0;
int hours = 0;                      // at reset, set clock to 0:00:00
uint16_t gCharge_Timer = 0;         // default to zero for delay timer
int ModeTimer = 0;                  // use for off timer = 10 mins get set later
byte T_MODECHECK = 10;              // update historical vars every minute for a 10 min running average
const int T_HISTORYCHECK = 1;       // update historical vars every 1 secs
int HistoryTimer = T_HISTORYCHECK;
#pragma endregion APP

// ****************************************************************************WATCHDOG SETUP
#pragma region WD
/**
 * @brief Watchdog Timer
 * Set one second (it's adjustable with WDOG_T register) watchdog enable  by redefining 
 * the startup_early_hook which by default disables the COP (TURN OFF WHEN DEBUGGING). 
 * Must be before void setup();
 */

#define WATCHDOG_MILLISECONDS 2000

extern "C" void startup_early_hook(void);

void startup_early_hook(void) {
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  WDOG_TOVALH = WATCHDOG_MILLISECONDS >> 16;
  WDOG_TOVALL = WATCHDOG_MILLISECONDS & 0xFFFF;
  WDOG_PRESC = 0;
  WDOG_STCTRLH = WDOG_STCTRLH_WDOGEN;
  for (int i=0; i<8192; i++) asm("nop");
}

/**
 * @brief Watchdog Reset
 * Needs to be reset before watch dog reaches "WATCHDOG_MILLISECONDS"
 */
void watchdogReset() {
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

#pragma endregion WD

// ****************************************************************************DK BOARD SETUP
#pragma region DKBOARDSETUP
#ifdef DK_SUPER_HW_V1
    // switch pins
  #define SW1_LEARN_BLKS_PIN  2
    // led pins
  #define LED1_RED_PIN        15
  #define LED1_GREEN_PIN      16
  #define LED2_RED_PIN        7
  #define LED2_GREEN_PIN      8
  //#define LED_ONBOARD         13  //temp use of led on teensy    (ALSO used spi clock)
    // relay pins
  #define RELAYDR1_CHARGE_PIN 5
  #define RELAYDR2_DRIVE_PIN  6
    // user input pins
  #define INPUT_CHARGE_PIN    22
  #define INPUT_KEYSWITCH_PIN 23
    // input pins
  #define PACK_VOLTAGE_PIN              A0  // Schematic(VSCALED-PACK), 1.4v at 100VDC, 2.4V at 170VDC
  #define SUPERVISOR_TEMP_PIN           A3  // Onboard supervisor NTC (previously NTCambient)
  #define CURRENT_SENSOR_CHARGE_PIN     A11 // Schematic(ICHG), PCB(CON8, CH)
  #define CURRENT_SENSOR_DISCHARGE_PIN  A5  // Schematic(IDISCHG), PCB(CON8, DIS)  
  #define VBALANCE                      A10 // Schematic(TP VBAL), Currently unused
    // output pins
  #define OUTPUT_LIMP_PIN     18
  #define DAC_OUT_FUEL_GAUGE  A14
    // PWM Pins for op amp digital DACs (note: 10 bits PWM same setup as Analog DAC output)
  #define PWM1_DAC1           20
  #define PWM2_DAC2           21
    // SPI pins
  #define SPI0_SCLK           13
  #define SPI0_MISO           12
  #define SPI0_MOSI           11
  #define SPI0_CS             10
  #define SPI0_CE             9
    // unused pins
  #define UNUSEDA12           A12
  #define UNUSEDA13           A13
  #define UNUSEDA24           24
  #define UNUSEDA25           25
  #define UNUSEDA26           26
  #define UNUSEDA27           27
  #define UNUSEDA28           28
  #define UNUSEDA29           29
  #define UNUSEDA30           30
  #define UNUSEDA31           31
  #define UNUSEDA32           32
  #define UNUSEDA33           33
#endif

#define VREF (3.266)                    // ADC reference voltage (= power supply)
#define VINPUT (2.171)                  // ADC input voltage from resistive divider to VREF
#define ADCMAX (65535)                  // maximum possible reading from ADC
#define EXPECTED (ADCMAX*(VINPUT/VREF)) // expected ADC reading
#define SAMPLES (200)                   // how many samples to use for ADC read - 200 seemed to work best   
#define ADC_RESOLUTION (12)             // ADC resolution in bits, usable from 10-13 on this chip
#define DAC_RESOLUTION (10)             // DAC resolution in bits, usable from 0-12 on this chip (same setup as PWM outputs)

/**************************************************************************/
/*!
	@brief  initialization of the microcontroller.
*/
/**************************************************************************/
void initBoard(){
  // leds
  pinMode(LED1_GREEN_PIN, OUTPUT);
  pinMode(LED1_RED_PIN, OUTPUT);
  pinMode(LED2_GREEN_PIN, OUTPUT);
  pinMode(LED2_RED_PIN, OUTPUT);

  // relays
  pinMode(RELAYDR1_CHARGE_PIN, OUTPUT);
  pinMode(RELAYDR2_DRIVE_PIN, OUTPUT);

  // inputs
  pinMode(SUPERVISOR_TEMP_PIN, INPUT);

  // User inputs
  pinMode(INPUT_CHARGE_PIN, INPUT);
  pinMode(INPUT_KEYSWITCH_PIN, INPUT);
  pinMode(SW1_LEARN_BLKS_PIN, INPUT);

  // outputs
  pinMode(PWM1_DAC1, OUTPUT);
  pinMode(PWM2_DAC2, OUTPUT);
  pinMode(OUTPUT_LIMP_PIN, OUTPUT);

  // Unused I/O make digital output for low impedance and EMI-resistance
  pinMode(A12, INPUT_PULLUP);
  pinMode(A13, INPUT_PULLUP);   
  pinMode(UNUSEDA24, OUTPUT);
  pinMode(UNUSEDA25, OUTPUT);   
  pinMode(UNUSEDA26, OUTPUT);   
  pinMode(UNUSEDA27, OUTPUT);   
  pinMode(UNUSEDA28, OUTPUT);   
  pinMode(UNUSEDA29, OUTPUT);   
  pinMode(UNUSEDA30, OUTPUT);   
  pinMode(UNUSEDA31, OUTPUT);   
  pinMode(UNUSEDA32, OUTPUT);   
  pinMode(UNUSEDA33, OUTPUT);

  // configure the ADC - Teensy PWM runs at 23kHz, DAC value is 0 to 1023
  analogWriteFrequency(PWM1_DAC1, pwm_freq); 
  analogWriteResolution(DAC_RESOLUTION);

  analogReference(EXTERNAL);      // set analog reference to ext ref
  analogReadRes(ADC_RESOLUTION);  // Teensy 3.0: set ADC resolution to this many bits
}

#pragma endregion DKBOARDSETUP

// ****************************************************************************COMMS SETUP
#pragma region COMMS
uint8_t gBlockNumber = 1;  // default to first  block, can be 1-40 blocks NOTE there is no number 0 block
uint8_t gValidPacket = gBlockNumber;  // this var checks what incoming packet is valid
uint8_t gPacketTimer = 1;   // number of loops before next block in line is polled - start with 20 loops or about 800 msecs. Typ is 200 msec aver block time

//uint8_t Main_Loops = 15;   // at power up use 1 main loop for packet timing to wake up blocks, then change to n loops (start with 12) to insure communication
uint8_t Main_Loops = 1;   // at power up use 1 main loop for packet timing to wake up blocks, then change to n loops (start with 12) to insure communication
uint8_t gBlockNumCommFault = 0;   // This var holds the last block to not communicate to Pack Supervisor

// Single instance of the radio driver
#define DK_MESSAGE_LNGTH 16  //DK uses 16 byte packet
RH_NRF24 driver(SPI0_CE, SPI0_CS); // for Teensy 3.x SS and CE lines
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);
// Dont put this on the stack:
//uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];    // buffer = 24 bytes long is max RH packet
uint8_t buf[DK_MESSAGE_LNGTH];    // buffer = 16 bytes long is max DK packet

// comms vars
uint16_t CommsFaults = 0;
bool Disconnected_Block = 0;
uint16_t Disconnected_BlockNum = 0;   // BLock 0 not used - START at Block 1
float Highest_Vcell;
float Lowest_Vcell;
uint16_t Highest_Tcell;
uint16_t Lowest_Tcell;
float Temp1 ;
float Temp2 ;
uint16_t Temp3 ;
uint16_t Temp4 ;
float Hist_Highest_Vcell ;        //  from all blocks of the highest average cell voltage
float Hist_Lowest_Vcell ;
float  Hist_Highest_Tcell;     // taken from all blocks - the average of the highest cell temperature
float  Hist_Lowest_Tcell;
unsigned long gdebugRXtime = 0;       // get to get current time from millis()
bool Goodcomms = false;      // default to NO until good packet is received

// put data from block into a struct - MUST be identical to struct in block
struct DATA {
  float sCellV_Hiside;       // high voltage side cells of dkblock
  uint16_t sThottest;        // hottest NTC counts (lower is hotter)
  uint16_t sTcoldest;        // coldtest NTC counts
  float sCellV_Loside ;      // Low voltage side cells
  float schecksum;                // checksum for data integrity
  //bool sCharge;
};
DATA data;

/**
 * @brief Read RF24 radio FIFO's and get block data
 * 
 */
void GetBlockData(){
  //uint8_t PAYLOAD = 16;     // bytes, use for comparing payload, comms will ignore data if wrong PL
  //float floatmatch = 0.000600; // +/- this amount to allow for matching float math on receive
  //float tempa;    //  float tempb;   //  float tempc;
  uint8_t len = sizeof(buf);
  uint8_t from;

  // Wait here for a message addressed to us from the client
  if(manager.recvfromAck(buf, &len, &from)){
      // we are here becase we 1)sent a short ACK-SYNC packet and 2)received block data 
      Serial.print("      RX BLK# ");       Serial.print(from);
      Serial.print(" Length");       Serial.print(" = ");       Serial.print(len);
      Serial.print("bytes  TX-RX-TX-RX time = ");  
      Serial.print(millis() - gdebugRXtime); Serial.print(" msecs");
      Serial.println();
     // if (VerbosePrintCommsDATA){
        Serial.print(((DATA*)buf) -> sCellV_Hiside ); Serial.print(" VDC Hi Cell   ");
        Serial.print(((DATA*)buf) -> sThottest); Serial.print(" Hot NTC counts   ");
        Serial.print(((DATA*)buf) -> sTcoldest); Serial.print(" Cold NTC counts   ");
        Serial.print(((DATA*)buf) -> sCellV_Loside ); Serial.print(" VDC Lo Cell   ");
        Serial.print(((DATA*)buf) -> schecksum ); Serial.println(" Checksum from BLOCK");
      //}
      Goodcomms = true;
    }
}

#pragma endregion COMMS

// ****************************************************************************LEARN BLOCKS
#pragma region LEARN BLOCKS
const byte LEARN_TIMEOUT = 2;       // learn mode timeout after 2 minutes after power up
//uint8_t blockNum[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // the array of 10 blocks for FIDO, this will have to be user configurable
uint8_t blockNum[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Change for Kents Bug
bool LearnBlockSwitch = 1;         // switch used to determine if block values should be zeroed

// block array for block 'awareness'
const uint8_t COMM_TIMEOUT = 255;     // max 4+ minute comm timeout - then go to sleep
//uint8_t Block_Comm_Timer[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Block_Comm_Timer[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };    // Kents bug change

//const float VPACK_HI_CHG_LIMIT = 60.0;    // do not allow charger to raise pack > 4.20V cell x 20 cells = 84VDC
// const float Vpack_HV_Run_Limit = 64.0; // Confirm with Jeb
//   const float VPACK_LO_RUN_LIMIT = 52.0;  // do not allow to run pack below 2.80V per cell x 20 cells = 58V
// ship with the following values ...but check with JEb first...
//const float VPACK_HI_CHG_LIMIT = 85.0;    // do not allow charger to raise pack > 4.25V cell x 20 cells = 85VDC
const float VPACK_HI_CHG_LIMIT = NUMBER_OF_S_CELLS * cell.volt_HVD;    // do not allow charger to raise pack > 4.25V cell x # of cells in series
//const float Vpack_HV_Run_Limit = 90.0; // Confirm with Jeb
//const float VPACK_LO_RUN_LIMIT = 58.0;  // do not allow to run pack below 2.80V per cell x 20 cells = 58V
// current control
uint16_t gTempPot;       // current potentiomenter
float gAmps;              // amps plus and minus through LEM sensor

    
// declare global vars
//const float EEPROM_CHG_SENSOR_OFFSET = 3094.8;  //3103;       // save offset from 2.50V at this address (0-4096 counts) - start with 2.500V exactly
const float EEPROM_CHG_SENSOR_OFFSET = 3094.77;  //3103;       // save offset from 2.50V at this address (0-4096 counts) - start with 2.500V exactly
uint16_t EEPROM_DISCH_SENSOR_OFFSET = EEPROM_CHG_SENSOR_OFFSET;       // save offset from 2.50V at this address (0-4096 counts) - start with 2.500V exactly
float LEM_Offset = EEPROM_CHG_SENSOR_OFFSET; // starting value



bool LEARNBLOCKS = false;   // Learn blocks is turned on by "CONNECT" switch
byte tempx = 0;
#pragma endregion LEARN BLOCKS

// ****************************************************************************RELAY ON OFF CHARGER
#pragma region ON OFF CHARGER
#ifdef CHARGER_RELAY_ON_OFF


#endif
#pragma endregion ON OFF CHARGER

// ****************************************************************************ANALOG CHARGER
#pragma region ANALOG CHARGER
#ifdef CHARGER_ANALOG_DAC2
  const byte CHARGER_CONTROL = PWM2_DAC2;      // 0-2-5V output for charger control near balance (FIDO is 2-5V = 0-100% linear charge)

#endif
#pragma endregion ANALOG CHARGER

// ****************************************************************************BLUE MY SKY CHARGER
#pragma region BMS S2500 CHARGER
#ifdef CHARGER_MYBLUESKY_S2500
    // states
  #define BMS_DO_NOT_CHARGE    0
  #define BMS_TRICKLE          1
  #define BMS_BULK_FULL        2
  #define BMS_BULK_HALF        3
  #define BMS_BULK_QUARTER     4
  #define BMS_TOP_UP           5
  #define BMS_PAUSE            6
  #define BMS_COMPLETE         7
  uint8_t bmsChargeStatus    = BMS_DO_NOT_CHARGE;
    // charging variables
  const uint16_t bmsVoltage             = 830;    // max charge voltage 830 = 83.0V
  const uint16_t bmsTrickleCurrent      = 10;     // trickle current 10 = 1.0A
  const uint16_t bmsMaxCurrent          = 200;    // max charge current 200 = 20.0A
  const uint16_t bmsEndCurrent          = 1;      // end charge current 1 = 0.1A
  static float bmsVoltageStatus         = 0;      // charger output
  static float bmsCurrentStatus         = 0;      // charger output
  elapsedMillis bmsTopUpCounter;                  // counter for top up cycle if necessary
  const uint8_t bmsChargeDelay          = 24;     // time delay between charge attempts in hours
    // can bus variables
  const uint16_t Tx_msg_interval = 1000;   // CAN Bus message rate
  elapsedMillis Tx_counter;
  static CAN_message_t rxMsg;
  const uint32_t BMS_TO_CHARGER = 0x1806E5F4;
  const uint32_t CHARGER_BROADCAST = 0x18FF50E5;
  
  /**************************************************************************/
  /*!
  	@brief  initialization of the Blue My Sky CAN BUS Charger
  */
  /**************************************************************************/
  void initBmsCAN(void){
    Can0.begin(250000);                         // join CAN at 250Kbps
    CAN_filter_t Filter;                        // CAN filter struct
    Filter.id = 0;                              // dont filter any Rx messages
    Filter.ext = 1;                             // set filter to Rx extended
    Filter.rtr = 0;                             // remote requests off
    for(uint8_t MBnum=0; MBnum<16; MBnum++) {   // set filter on all mailboxes
      Can0.setFilter(Filter,MBnum);
    }
  }
  
  /**************************************************************************/
  /*!
  	@brief Blue My Sky Charger CAN Send
    @param id destination id
    @param cV adjust voltage
    @param cC adjust current
    @param on_off turn charger on or off
  */
  /**************************************************************************/
  void bmsCANSend(uint32_t id, uint16_t  cV, uint16_t cC, bool on_off){
    CAN_message_t hold;
    hold.id = id;                                                    // set id
    hold.ext = 1;                                                    // set message as extended
    hold.rtr = 0;                                                    // set remote off
    hold.len = 5;                                                    // message length in bytes
    hold.buf[0] = (uint8_t)((cV & 0xFF00) >> 8);                     // split the two bytes of voltage                               
    hold.buf[1] = (uint8_t)(cV & 0x00FF);
    hold.buf[2] = (uint8_t)((cC & 0xFF00) >> 8);                     // split the two bytes of current
    hold.buf[3] = (uint8_t)(cC & 0x00FF);
    if(on_off == 1) hold.buf[4] = 0x00;                              // 00 is on
    else hold.buf[4] = 0x01;                                         // send to the bus
    Can0.write(hold);
  
    #ifdef CAN_DEBUG
    Serial.print("Tx - ");
    Serial.print(hold.id, HEX);   Serial.print(" ");
    Serial.print(hold.len, HEX);  Serial.print(" ");
    for(int i = 0; i<hold.len; i++){
      Serial.print(hold.buf[i], HEX); Serial.print(" ");
    }
    Serial.println();
    #endif
  }
  
  /**************************************************************************/
  /*!
  	@brief Blue My Sky Charger CAN Receive - Receives data from can bus, 
    returns 1 for message received and 0 for no message data is passed by 
    struct reference
    @param CAN_message_t incomming message
  */
  /**************************************************************************/
  bool bmsCANReceive(struct CAN_message_t &hold){
    if(Can0.available()) {
      Can0.read(hold);
  
      #ifdef CAN_DEBUG
      Serial.print("Rx - ");
      Serial.print(hold.id, HEX); Serial.print(" ");
      Serial.print(hold.len, HEX); Serial.print(" ");
      for(int i = 0; i<hold.len; i++){
        Serial.print(hold.buf[i], HEX); Serial.print(" ");
      }
      Serial.println();
      #endif
  
      return 1;
    }
  
    return 0;
  }
  
  /**************************************************************************/
  /*!
  	@brief Blue My Sky Charger Manager - Manages all communication when called
  */
  /**************************************************************************/
  void blueMySkyChargerManager(){
    // CAN bus controlled charger with a min output of 1A and a Max of 20A @ 240VAC
    // send message at set speed (1 per sec default)
    if(Tx_counter >= Tx_msg_interval) {
      Tx_counter = 0;                                   // reset counter
      bool rxFlag = 0;
      rxFlag = bmsCANReceive(rxMsg);                    // check for messages
      // if message, convert hex bytes into float
      if(rxFlag == 1 && rxMsg.id == CHARGER_BROADCAST){
        bmsVoltageStatus = ((uint8_t)(rxMsg.buf[0]) <<8 | (uint8_t)(rxMsg.buf[1]));
        bmsVoltageStatus = bmsVoltageStatus/10;
        bmsCurrentStatus = ((uint8_t)(rxMsg.buf[2]) <<8 | (uint8_t)(rxMsg.buf[3]));
        bmsCurrentStatus = bmsCurrentStatus/10;
        // charger data serial output for debug
        #ifdef CHARGER_DEBUG
          Serial.print(F("Charger Output: ")); Serial.print(bmsVoltageStatus, 1); Serial.print("V ");
          Serial.print(bmsCurrentStatus, 1); Serial.println("A ");
          Serial.print(F("DK_PS Readings: ")); Serial.print(Vpack, 1); Serial.print("V ");
          Serial.print(gAmps, 1); Serial.print("A ");
          switch (bmsChargeStatus){
            case BMS_DO_NOT_CHARGE:
              Serial.println(F("DO NOT CHARGE"));
              break;
            case BMS_TRICKLE:
              Serial.println(F("TRICKLE"));
              break;
            case BMS_BULK_FULL:
              Serial.println(F("BULK FULL"));
              break;
            case BMS_BULK_HALF:
              Serial.println(F("BULK HALF"));
              break;  
            case BMS_BULK_QUARTER:
              Serial.println(F("BULK QUARTER"));
              break;
            case BMS_TOP_UP:
              Serial.println(F("TOPPING UP"));
              break;
            case BMS_COMPLETE:
              Serial.println(F("COMPLETE"));
              break;
          }
        #endif
      }
      // outside of safe operating range? DO NOT CHARGE
      #ifdef CHARGER_DEBUG
        Serial.print(F("High Temp  ADC: ")); Serial.println(Hist_Highest_Tcell);
        Serial.print(F("Low  Temp  ADC: ")); Serial.println(Hist_Lowest_Tcell);
        Serial.print(F("High Volt Cell: ")); Serial.println(Hist_Highest_Vcell);
        Serial.print(F("Low  Volt Cell: ")); Serial.println(Hist_Lowest_Vcell);
      #endif
      if(Hist_Lowest_Tcell >= cell.temp_under_chargeSoftCut || Hist_Highest_Tcell <= cell.temp_over_chargeSoftCut ||
      Hist_Lowest_Vcell <= cell.volt_low_chargeCut || Hist_Highest_Vcell >= cell.volt_high_chargeCut){
        bmsCANSend(BMS_TO_CHARGER, 0, 0, OFF);
        bmsChargeStatus = BMS_DO_NOT_CHARGE;
      }
      // inside of safe operating range? CHARGE
      else{
        // check for very low voltages, if not too low trickle charge
        if(Hist_Lowest_Vcell <= cell.volt_chargeTrickle) {
          bmsChargeStatus = BMS_TRICKLE;
        }
        // check for bulk charge voltages, adjust current based on cell temperature
        if(Hist_Lowest_Vcell > cell.volt_chargeTrickle && Hist_Highest_Vcell < cell.volt_chargeBulk) {
          if(Hist_Highest_Tcell >  cell.temp_chargeTaper_1) bmsChargeStatus = BMS_BULK_FULL;
          if(Hist_Highest_Tcell <= cell.temp_chargeTaper_1) bmsChargeStatus = BMS_BULK_HALF;
          if(Hist_Highest_Tcell <= cell.temp_chargeTaper_2) bmsChargeStatus = BMS_BULK_QUARTER;
        }
        // check for end charge voltages, top end trickle charge
        if(Hist_Highest_Vcell > cell.volt_chargeBulk && Hist_Highest_Vcell < cell.volt_chargeOff) {
          bmsChargeStatus = BMS_TOP_UP;
        }
        // if any cell over Charge off V, pause charger
        if(Hist_Highest_Vcell > cell.volt_chargeOff) {
          bmsChargeStatus = BMS_PAUSE;
        }
        // if all cells between balance V and charge off voltage, turn off charger
        if(Hist_Lowest_Vcell > cell.volt_Balance && Hist_Highest_Vcell < cell.volt_chargeOff) {
          bmsChargeStatus = BMS_COMPLETE;
        }
        // if statements above set charge status, switch statement sends data to charger
        switch (bmsChargeStatus) {
              case BMS_TRICKLE:
                bmsCANSend(BMS_TO_CHARGER, bmsVoltage, bmsTrickleCurrent, ON);
                break;
              case BMS_BULK_FULL:
                bmsCANSend(BMS_TO_CHARGER, bmsVoltage, bmsMaxCurrent, ON);
                break;
              case BMS_BULK_HALF:
                bmsCANSend(BMS_TO_CHARGER, bmsVoltage, (bmsMaxCurrent / 2), ON);
                break;
              case BMS_BULK_QUARTER:
                bmsCANSend(BMS_TO_CHARGER, bmsVoltage, (bmsMaxCurrent / 4), ON);
                break;
              case BMS_TOP_UP:
                // cycles charger on and off because charger resolution is not able to provide under 1A
                if(bmsTopUpCounter < 25000){
                  bmsCANSend(BMS_TO_CHARGER, bmsVoltage, bmsEndCurrent, ON);
                }
                else{
                  bmsCANSend(BMS_TO_CHARGER, bmsVoltage, bmsEndCurrent, OFF);
                }
                if(bmsTopUpCounter > 60000){
                  bmsTopUpCounter = 0;
                }
                break;
              case BMS_PAUSE:
                bmsCANSend(BMS_TO_CHARGER, bmsVoltage, bmsEndCurrent, OFF);
                break;
              case BMS_COMPLETE:
                bmsCANSend(BMS_TO_CHARGER, bmsVoltage, bmsEndCurrent, OFF);
                gCharge_Timer = bmsChargeDelay;
                break;
              default:
                bmsCANSend(BMS_TO_CHARGER, bmsVoltage, bmsEndCurrent, OFF);
                break;
        }
      }
    }
  }
#endif
#pragma endregion BMS S2500 CHARGER

// ****************************************************************************KOSO METER
#pragma region KOSO DL03
#ifdef KOSO_DL03
    // defines
  // setup DAC output for KOSO Speedo Fuel gauge  (5/3.3 = compensates for PWM testing done with Funct gen)
  #define KOSO_NO_BARS    100   // 1% PWM  = (5 / 3.3) * 0.01 * 1024 = 15  (0 bars > 4.32)
  #define KOSO_ONE_BARS   153   // 14% PWM = (5 / 3.3) * 0.14 * 1024 = 217 (1 bars < 4.07V)
  #define KOSO_TWO_BARS   225   // 18% PWM = (5 / 3.3) * 0.18 * 1024 = 279 (2 bars < 3.7V)
  #define KOSO_THREE_BARS 400   // 22% PWM = (5 / 3.3) * 0.22 * 1024 = 341 (3 bars < 3.2V)
  #define KOSO_FOUR_BARS  707   // 26% PWM = (5 / 3.3) * 0.26 * 1024 = 404 (4 bars < 2.3V)

  /**************************************************************************/
  /*!
  	@brief Voltage based KOSO DL03 Manager, Outputs on CON7 (Fuel Gauge) Pin 4
  */
  /**************************************************************************/
  void kosoDL03Manager(){
  if (SOCv >= 20) analogWrite(DAC_OUT_FUEL_GAUGE, KOSO_ONE_BARS);    // Need equiv of 14% PWM for 1 bars
  else            analogWrite(DAC_OUT_FUEL_GAUGE, KOSO_NO_BARS);     // No bars give *some* signal
  if (SOCv >= 40) analogWrite(DAC_OUT_FUEL_GAUGE, KOSO_TWO_BARS);    // Need equiv of 18% PWM for 2 bars
  if (SOCv >= 60) analogWrite(DAC_OUT_FUEL_GAUGE, KOSO_THREE_BARS);  // Need equiv of 22% PWM for 3 bars
  if (SOCv >= 80) analogWrite(DAC_OUT_FUEL_GAUGE, KOSO_FOUR_BARS);   // Need equiv of 26% PWM for all 4 bars
  }
#endif
#pragma endregion KOSO DL03


// ****************************************************************************APPLICATION SETUP
void setup() {
  // Setup Serial Comms
  Serial.begin(115200);
  // delay(2000);

  // Setup microcontroller
  initBoard();

  // nRF24 2.4Mhz packet comms
  if (!manager.init())
  {
    Serial.println("Comms init failed");
    manager.init();     // and try again if not the first time
  }
  else   Serial.println("Comms init success");

  analogWrite(CHARGER_CONTROL, 0);     // prog CHG current to 0

  Serial.println("1: Green LED1 ");
    watchdogReset();  // reset the watchdog timer

  digitalWrite(LED1_GREEN_PIN, HIGH);   delay(500);  // LED on for 1 second
  Serial.println("1: then go RED ");
  digitalWrite(LED1_GREEN_PIN, LOW);
   watchdogReset();  // reset the watchdog timer
 digitalWrite(LED1_RED_PIN, HIGH);   delay(500);  // LED on for 1 second
  Serial.println("2: Green LED2");
  watchdogReset();  // reset the watchdog timer
  digitalWrite(LED2_GREEN_PIN, HIGH);   delay(500);  // LED on for 1 second
  Serial.println("3: Now go RED ");
  digitalWrite(LED2_GREEN_PIN, LOW);
  watchdogReset();  // reset the watchdog timer
  digitalWrite(LED2_RED_PIN, HIGH);   delay(500);  // LED on for 1 second
  // wait for slow human to get serial capture running
  digitalWrite(LED1_GREEN_PIN, LOW); //leds all off
  digitalWrite(LED1_RED_PIN, LOW);
  digitalWrite(LED2_GREEN_PIN, LOW);
  digitalWrite(LED2_RED_PIN, LOW);

  // save all vars until blocks are awake and comms are established
  Highest_Vcell = cell.volt_Nominal;        // load  vars with nominal values
  Lowest_Vcell = cell.volt_Nominal;
  Highest_Tcell = NTC_AMBIENT;
  Lowest_Tcell = NTC_AMBIENT;
  Hist_Highest_Vcell = Highest_Vcell;   // running average of all blocks highest cell v
  Hist_Lowest_Vcell = Lowest_Vcell;
  Hist_Highest_Tcell = NTC_AMBIENT;     // running ave of ALL blocks highest cell temperature (seed with nominal 25C)
  Hist_Lowest_Tcell = NTC_AMBIENT;     // seed with nom 25C


  // Retreive EEPROM contents into RAM for Block comms - in other words...
  // "retreived blocks that pack supervisor learned was connected"

  int EE_address = 0;
  byte EE_value;          // block number that is stored in EE_address
 // for (EE_address = 0; EE_address < 10; EE_address++)   
  for (EE_address = 0; EE_address < NUMBER_OF_BLOCK_MANAGERS; EE_address++)   // change from 10 to 20 blocks for Kents bug
  {
    EE_value = EEPROM.read(EE_address);   // read for print debug
    blockNum[EE_address] = EE_value;      // READ EEprom at power up
    Serial.print("READ from EEprom address:  ");  Serial.print(EE_address);   Serial.print(" = Block#: ");  Serial.println(EE_value);
   Serial.print("READ from EEprom address:  ");  Serial.print(blockNum[EE_address]);   Serial.print(" = Block#: ");  Serial.println(EE_value);
//    Serial.print("READ from EEprom address:  ");  Serial.print(EE_address);   Serial.print(" = Block#: ");  Serial.println(EE_value);
//    Serial.print("READ from EEprom address:  ");  Serial.print(EE_address);   Serial.print(" = Block#: ");  Serial.println(EE_value);
//    Serial.print("READ from EEprom address:  ");  Serial.print(EE_address);   Serial.print(" = Block#: ");  Serial.println(EE_value);
//    Serial.print("READ from EEprom address:  ");  Serial.print(EE_address);   Serial.print(" = Block#: ");  Serial.println(EE_value);
//    Serial.print("READ from EEprom address:  ");  Serial.print(EE_address);   Serial.print(" = Block#: ");  Serial.println(EE_value);
//    Serial.print("READ from EEprom address:  ");  Serial.print(EE_address);   Serial.print(" = Block#: ");  Serial.println(EE_value);
//    Serial.print("READ from EEprom address:  ");  Serial.print(EE_address);   Serial.print(" = Block#: ");  Serial.println(EE_value);
  }



  // default RF24 manager settings are 2000ms timeout and 3 retries. Timeout can be 2x to avoid collisions. These are too long for COP, and we don't want
  // COP reset in comm routine in case the program hangs in that code... (*just* barely works at 110ms/3 retries, and 50ms/9)
 // manager.setTimeout(2000); //these were production values
 // manager.setRetries(3);
 // manager.setTimeout(10);   // these are debug values
 // manager.setRetries(1);    // 1 retry is 2 tries total
    manager.setTimeout(20);   // these are final values for fastest comms
    manager.setRetries(0);    // 0 = 1 try only

  //TIM ADD SPEED AND BPS and all other RF24 settings here and inBLOCK CODE!!!!!!!!!!!!
  //manager.setPayloadSize(12);     // 2 floats (4x2 bytes) = 8bytes +  2 + 2 = 12 bytes MUST AGREE WITH CLIENTS!!


  Serial.println("Start Pack Supervisor now... ");
  Serial.print(" Pack Address: ");   Serial.println(SERVER_ADDRESS);

  
  

  // only turn LIMP mode off at power cycle 
    digitalWrite(OUTPUT_LIMP_PIN, LOW);
    Serial.println("'Limp Mode'  is off because DK is being reset ");


}

// ****************************************************************************APPLICATION MAIN
void loop() {

  if (VerbosePrintSUPERDATA){
    Serial.print("Firmware Ver: ");  Serial.print(VERSION);
    Serial.print("  Server Address: ");  Serial.print(SERVER_ADDRESS);
    Serial.print("  PS Mode: ");  Serial.println(dkMode);
  }
  
  watchdogReset();  // reset the watchdog timer (times out in 1 sec so make sure loop is under about 500-600msec)

 

  // make clock tick tock
  currentmicros = micros(); // read the time.
  bool Comm_Flag = 0;
  bool Print_Flag = 0;

  // when 1000 millisecond clock ticks, run real time clock
  if ((currentmicros - nextmicros) >= interval)  // if 1000000 microseconds have gone by...
  {
    nextmicros = nextmicros + interval; // update for the next comparison
    seconds = seconds + 1;              // and run real time clock
    if ((seconds == 0) || (seconds == 15) || (seconds == 30) || (seconds == 45)) Print_Flag = 1;                     // 1 = ok to print to serial console (teensy Monitor) 0 = off
    Comm_Flag = 1;      
    if ((seconds > 15) && (minutes >= 1)) Main_Loops = 15;  // after 1.5 min of wakeup comms, go to actually getting data from blocks               
    if (HistoryTimer > 0) HistoryTimer --;    // run history timer every second

    if (seconds == 60)
    {
      seconds = 0;
      minutes = minutes + 1;
      //     if (ModeTimer > 0) ModeTimer = ModeTimer-1;
      if (minutes == 60)
      {
        minutes = 0;
        hours = hours + 1;
        if (gCharge_Timer > 0) gCharge_Timer --;
      }
      if (hours == 24) hours = 0;
    }
 

  }


       if (Print_Flag){  // once per minute
            Serial.println();
          Serial.print("Secs = ");  Serial.println(seconds);          // debug printout to pc (open serial monitor window)
          Serial.print("Mins = ");  Serial.println(minutes);
          Serial.print("Hrs = ");  Serial.println(hours);
          Serial.print("  millisecond counter: ");  Serial.println(millis());
          Serial.print(" Historical Avg Hottest Tcell: ");     Serial.print(Hist_Highest_Tcell, 0);  Serial.print(" ADC counts ");
       }
 


  // Learn mode switch routine...check if learn == on or off
  //tempx ++;
  tempx = tempx + digitalRead(SW1_LEARN_BLKS_PIN);       // read the input pin
  if ((tempx > 0) && (hours == 0) && (minutes < LEARN_TIMEOUT))   // only allow learn blocks function in first 2 minutes after power applied
  {
    if (tempx > 5)
    {
      tempx = 10;
      if (minutes < 1){
        LEARNBLOCKS = true;    // turn on first minute, allow to be turned off after that for time or full EE
      }
      if (LearnBlockSwitch == true){
        blockNum[0] = blockNum[1] = blockNum[2] = blockNum[3] = blockNum[4] = blockNum[5] = blockNum[6] = blockNum[7] = blockNum[8] = blockNum[9] = 0;
        if (NUMBER_OF_BLOCK_MANAGERS > 10){
          blockNum[10] = blockNum[11] = blockNum[12] = blockNum[13] = blockNum[14] = blockNum[15] = blockNum[16] = blockNum[17] = blockNum[18] = blockNum[19] = 0;
        }
      
      //is 4/18/2019
        for (tempx = 0; tempx < NUMBER_OF_BLOCK_MANAGERS; tempx++){
        Serial.print(" ZERO Block Addr: "); Serial.print(tempx+1); Serial.print(" with the value: ");  Serial.println(blockNum[tempx]);
  //        blockNum[tempx] = 0;  // first time through, zero all block array locations, so they can be populated by the learn blocks feature
        }
        LearnBlockSwitch = false ;
        
      }

    }
    else if (seconds > 55)  tempx = 0;     // if switch is not pressed, 0 out var after 5 seconds

  }
  else
  {
    LEARNBLOCKS = false;
  }

  if (VerbosePrintSUPERDATA)
 {
    Serial.println();
    Serial.print("    Learn Blocks is ON/OFF: ");  Serial.println(LEARNBLOCKS);
    Serial.println("  These Block numbers populate these memory locations: ");
    Serial.print("EEprom location 0 =  "); Serial.print("Block Addr "); Serial.println(blockNum[0]);
    Serial.print("EEprom location 1 =  "); Serial.print("Block Addr "); Serial.println(blockNum[1]);
    Serial.print("EEprom location 2 =  "); Serial.print("Block Addr "); Serial.println(blockNum[2]);
    Serial.print("EEprom location 3 =  "); Serial.print("Block Addr "); Serial.println(blockNum[3]);
    Serial.print("EEprom location 4 =  "); Serial.print("Block Addr "); Serial.println(blockNum[4]);
    Serial.print("EEprom location 5 =  "); Serial.print("Block Addr "); Serial.println(blockNum[5]);
    Serial.print("EEprom location 6 =  "); Serial.print("Block Addr "); Serial.println(blockNum[6]);
    Serial.print("EEprom location 7 =  "); Serial.print("Block Addr "); Serial.println(blockNum[7]);
    Serial.print("EEprom location 8 =  "); Serial.print("Block Addr "); Serial.println(blockNum[8]);
    Serial.print("EEprom location 9 =  "); Serial.print("Block Addr ");Serial.println(blockNum[9]);
    Serial.print("EEprom location 10 =  "); Serial.print("Block Addr ");Serial.println(blockNum[10]);
    Serial.print("EEprom location 11 =  "); Serial.print("Block Addr ");Serial.println(blockNum[11]);
    Serial.print("EEprom location 12 =  "); Serial.print("Block Addr ");Serial.println(blockNum[12]);
    Serial.print("EEprom location 13 =  "); Serial.print("Block Addr ");Serial.println(blockNum[13]);
    Serial.print("EEprom location 14 =  "); Serial.print("Block Addr ");Serial.println(blockNum[14]);
    Serial.print("EEprom location 15 =  "); Serial.print("Block Addr ");Serial.println(blockNum[15]);
    Serial.print("EEprom location 16 =  "); Serial.print("Block Addr ");Serial.println(blockNum[16]);
    Serial.print("EEprom location 17 =  "); Serial.print("Block Addr ");Serial.println(blockNum[17]);
    Serial.print("EEprom location 18 =  "); Serial.print("Block Addr ");Serial.println(blockNum[18]);
    Serial.print("EEprom location 19 =  "); Serial.print("Block Addr ");Serial.println(blockNum[19]);

    Serial.println();
  //}

  byte tempbl = 0  ;
  //byte TWO_MINUTES = 30;      // 240 seconds (4 min)
  byte TWO_MINUTES = 120;      // 120 seconds (2 min)
  //bool Disconnected_Block;   // start with no fault
  
  //if (VerbosePrintSUPERDATA){
      Serial.println("  These Block numbers contain these timeout values: ");
      while ( tempbl < NUMBER_OF_BLOCK_MANAGERS )
         {
           Serial.print("Block: "); Serial.print(blockNum[tempbl]); Serial.print(" = "); Serial.println(Block_Comm_Timer[tempbl]);
           //tempbl++;
           // run comm disconnect timer on all blocks here - count to "TWO_MINUTES" and stop
           if (Comm_Flag) 
           {
              if (Block_Comm_Timer[tempbl] < COMM_TIMEOUT) Block_Comm_Timer[tempbl] ++;     // run timer here, gets zero'd in comm routine
              if (Block_Comm_Timer[tempbl] >= TWO_MINUTES) {
                Disconnected_Block = true;                       //  yes at least one block is disconnected
                Disconnected_BlockNum = blockNum[tempbl];
               }
           }
          tempbl++;
        }
      Serial.println();
      if (Disconnected_Block) {
        Serial.print("Block: "); Serial.print(Disconnected_BlockNum); Serial.print(" = "); Serial.println("DISCONNECTED");
        }
       Serial.println();
  //}
  bool Tempdisc = false;
  tempbl = 0  ;
  // Now check all blocks for comms in "TWO_MINUTES", when timers are reset, clear Disconnect switches
    while ( tempbl < NUMBER_OF_BLOCK_MANAGERS )
     {
          if (Block_Comm_Timer[tempbl] > TWO_MINUTES) {
            Tempdisc = true;                       //  yes at least one block is disconnected
              //if (VerbosePrintSUPERDATA){ 
              Serial.print("Block DISCONNECT.... ");Serial.println("Block DISCONNECT");}
          //}
      tempbl++;
    }
    
    if (Tempdisc == false) {
       Disconnected_Block = false;                       //  no block is disconnected
       Disconnected_BlockNum = 0;
      //if (VerbosePrintSUPERDATA){
          Serial.print("All Blocks connected....");
          Serial.println("All Blocks connected");
      }
    }
    if (VerbosePrintSUPERDATA) Serial.println();


  // Write new learned block values to EEPROM
  // start writing and reading from the first byte (address 0) of the EEPROM
  int EE_address = 0;
  byte EE_value;

  //if ((LEARNBLOCKS == ON) && (blockNum[0]) )   // Make sure we are in learn mode and all blocks are written
  if ((LEARNBLOCKS == true) && (seconds == 0))   // Write EE once/sec when we are in learn mode
  {
    for (EE_address = 0; EE_address < 10; EE_address++)
    {
      EEPROM.write(EE_address, blockNum[EE_address]);
      EE_value = EEPROM.read(EE_address);
      Serial.print("Once/min save Block#: "); Serial.print(EE_value); Serial.print("to this EEprom address:  "); Serial.println(EE_address);
      Serial.print("Once/min save Block#: "); Serial.print(EE_value); Serial.print("to this EEprom address:  "); Serial.println(EE_address);

    }
    if (blockNum[0]) LEARNBLOCKS = false;   // if last block is saved to EE, turn off Learn blocks
  }

  if (LEARNBLOCKS == true)     // turn on RED leds
  {
    digitalWrite(LED2_RED_PIN, HIGH);  //on
    digitalWrite(LED1_RED_PIN, HIGH);  //on
  }
  else    // turn off red leds unle3ss
  {
    if (blockNum[0] == 0)
    {
      digitalWrite(LED2_RED_PIN, HIGH);  // turn on LED2=RED if all Blocks not programmed
      digitalWrite(LED1_RED_PIN, LOW);
    }
    else
    {
      digitalWrite(LED2_RED_PIN, LOW);  //off
      digitalWrite(LED1_RED_PIN, LOW);  //off
    }
  }
  // measure ambient NTC - start with all LEDs red. If NTC gets shorted, make them green
  //          digitalWrite(LED1_RED_PIN,HIGH);
  //          digitalWrite(LED2_RED_PIN,HIGH);
  float datSum = 0;  // reset our accumulated sum of input values to zero
  long n = 0;            // count of how many readings so far
  double x1 = 0;

  for (int i = 0; i < SAMPLES; i++) {
    x1 = analogRead(SUPERVISOR_TEMP_PIN);
    if ((i < 10 ) || (i > 150)) goto throwaway_a;  // throw away the first 10 and last 50 samples
    datSum += x1;
    n++;
throwaway_a:  ;
  } //end ntc loop


  float datAvg = (datSum) / n;    // find the mean
  int Tambient = datAvg;      // save ambient

  if (Print_Flag){
      Serial.print(SUPERVISOR_TEMP_PIN);  Serial.print(" :");
      Serial.print("  NTC ambient avg value: ");     Serial.print(Tambient);  Serial.println ();
      if (datAvg < 100)
      {
        Serial.print("NTC possibly shorted ");    Serial.println ();
      }
  }
  
  // measure charge current input
  datSum = 0;  // reset our accumulated sum of input values to zero
  n = 0;            // count of how many readings so far
  double x2 = 0;
  bool polarity;
  //float LEM_Offset = EEPROM_CHG_SENSOR_OFFSET; // starting value
  uint8_t near0 = 15;  // near zero amps so cal sensor value

  for (int i = 0; i < SAMPLES; i++) {
    x2 = analogRead(CURRENT_SENSOR_CHARGE_PIN);
    if ((i < 10 ) || (i > 150)) goto throwaway_i;  // throw away the first 10 and last 50 samples
    datSum += x2;
    n++;
throwaway_i:   ;
  } //end chg current loop
  
  datAvg = datSum / n;  // find the mean

   if (Print_Flag){
       Serial.print(CURRENT_SENSOR_CHARGE_PIN);
      if (datAvg < 50)
      {
        Serial.print("Ichg -> Error - Input is shorted or no signal ");
      }
      Serial.print("Chan: "); Serial.print(CURRENT_SENSOR_CHARGE_PIN);
      Serial.print(" ICHG avg value: ");     Serial.println(datAvg, 2);
   }
   
  if ((dkMode != DK_CHARGE) && (dkMode != DK_DRIVE)) // calibrate sensor when not charging or driving
  {
    if ((datAvg > (LEM_Offset-near0)) && (datAvg < LEM_Offset+near0)) LEM_Offset = datAvg;    // cal when close to 2.5V which is LEM offset at 0 amps
  }

  // Sensor is offset at 2.50V from ground, so subract offset from signal to get actual current
  if (datAvg >= EEPROM_CHG_SENSOR_OFFSET)
  {
    datAvg = datAvg - LEM_Offset;
    polarity = 1;
  }
  else
  {
    datAvg = LEM_Offset - datAvg;
    polarity = 0;
  }


  // Theoretical sensitivity for the LEM HO 100S = 8mv/amp. At 12 bit resolution each bit = 3.3V/4096 = 0.8057mV.
  // so each bit = .8057/8 = .1007 amps. Or 9.929696 counts per amp.
  // so each bit = .8057/16 = .0504 amps. Or 19.859 counts per amp.

  float TempAmps;
 //     TempAmps = (datAvg / 10.3);    // scale current to amps
       TempAmps = (datAvg / 10.1);    // scale current to amps
        
      gAmps = (TempAmps + (gAmps * 9)) / 10.00; // average for noise reduction


//  if (Print_Flag){
      Serial.print(" Amps avg value: ");
      if (polarity)  Serial.print("+");
      else Serial.print("-");
      Serial.print(gAmps, 3);
      Serial.println(" (POS => charge and NEG => discharge)");
      Serial.println();
 // }

// debug
     Serial.print(" ICHG avg value: ");     Serial.println(datAvg, 2);
     //debug

  // measure discharge current
  datSum = 0;  // reset our accumulated sum of input values to zero
  n = 0;            // count of how many readings so far
  double x3 = 0;

  for (int i = 0; i < SAMPLES; i++) {
    x3 = analogRead(CURRENT_SENSOR_DISCHARGE_PIN);
    datSum += x3;
    n++;
  } //end loop
  datAvg = (1.0 * datSum) / n;  // find the mean

  if (Print_Flag){
      Serial.print(CURRENT_SENSOR_DISCHARGE_PIN);
      Serial.print(" IDISCHG avg value: ");     Serial.print(datAvg, 2);
      if (datAvg < 50)
      {
        Serial.print("IDISCHG -> Error - Input is shorted or no signal ");
      }
      Serial.println ();
  }
  digitalWrite(LED1_GREEN_PIN, HIGH);  //on
  
  // read and scale Pack voltage input
  delay(10);
  datSum = 0;  // reset our accumulated sum of input values to zero
  n = 0;            // count of how many readings so far
  double x4 = 0;

  for (int i = 0; i < SAMPLES; i++) {  
    x4 = analogRead(PACK_VOLTAGE_PIN);
    if ((i < 10 ) || (i > 160)) goto throwaway_c;  // throw away the first 10 and last 40 samples
    datSum += x4;
    n++;
throwaway_c:  ;
  } //end loop

  digitalWrite(LED1_GREEN_PIN, LOW);  //off

  datAvg = datSum / n;  // find the mean


  float Volts;    // local var for maths

  if (Vnominal != NUMBER_OF_S_CELLS * cell.volt_Nominal)    // right now this is only 20S pack
  {
     // Volts = datAvg * 0.0575;    // low voltage calibrate
     // Volts = datAvg * 0.0565;    // 84.1 low voltage calibrate
    //  Volts = datAvg * 0.056679;    // 84.26 low voltage calibrate
      Volts = datAvg * 0.056680;    //  calibrate at 85V for HVD accuracy
  }
  else if (Vnominal == NUMBER_OF_S_CELLS * cell.volt_Nominal)
  {
       Volts = datAvg * 0.057420;    //  calibrate at 150 and 100V for HVD accuracy
  }
 
//  Volts = Volts + Vpack + Vpack;      // sum and average for noise reduction over less than a second
//  Vpack = Volts / 3;
  Volts = Volts + Vpack * 99;      // sum and average for noise reduction for many seconds of time
  Vpack = Volts / 100;



    Serial.print(" Vpack: ");     Serial.print(Vpack,1);  Serial.print(" VDC"); Serial.println ();
 //    Serial.print(" Volts: ");     Serial.print(Volts,1);  Serial.print(" VDC"); Serial.println ();
 //    Serial.print(" datAvg: ");     Serial.print(datAvg);  Serial.print(" data average"); Serial.println ();
 
  
  // now do fine calibration on signal
  //Vpack = Vpack - 0.010;    // add offset to signal

 
  if (Print_Flag){   // print data only when block data is in
     Serial.print(PACK_VOLTAGE_PIN);
     Serial.print(" Vpack: ");     Serial.print(Vpack,1);  Serial.print(" VDC"); Serial.println ();
  }


  // Make logic determinations for relays = ON or OFF, and use hysteresis to prevent relay oscillation

 const float Vpack_HVD = ((cell.volt_HVD + 0.02) * NUMBER_OF_S_CELLS) ;       // eg 4.21+.02=4.23 x 20 = 84.6 (.02 is headroom to balance (remember high cellV will open relay too)
  int Vpack_LVD = cell.volt_LVD * NUMBER_OF_S_CELLS;         // eg 2.9 x 20 = 58
  int Vpack_HV_Run_Limit = Vpack_HVD * 1.2;  // Make high voltage run limit 10% higher than HVD (be careful at high voltage like Kent's bug)
  int Vpack_Lo_Run_Limit = Vpack_LVD;

  relay_charge_state = false;      // default to relay off
  //  pack check for charger relay and cell check for charger relay
  //      if ((Vpack > Vpack_HVD) || (Hist_Highest_Vcell > Vcell_HVD_Spec)) relay_charge_state = OFF;                  // if Vbat is too high, turn off charger
  //      else if ((Hist_Highest_Vcell < (Vcell_HVD_Spec - HYSTERESIS)) && (Vpack < (Vpack_HVD - HYSTERESIS))) relay_charge_state = ON;  // else now is ok so turn back on
   
      if ((Hist_Highest_Vcell < (cell.volt_HVD )) && (Vpack < (Vpack_HVD)))    //4.21 if LG
      {
        if ((digitalRead(INPUT_CHARGE_PIN) == 0))
        {
          Serial.print(" Charge input IS ON ");
          Serial.print(" Charge Timer =  ");  Serial.print(gCharge_Timer); Serial.println(" hrs");
    
          if (gCharge_Timer == 0) relay_charge_state = true;      // if Charge input is activated turn on Charge relay
          // open charge relays for one day if all Cells are in Vbalance
          if (Hist_Lowest_Vcell > cell.volt_Balance)                                // 4.11 if LG
          {
            // shut down relay, start 1 day timer
            gCharge_Timer = 24;   //24 hours
            relay_charge_state = false;
            if (Print_Flag){
            Serial.println(" Charge relay now turns OFF for 24 hours = cells are ALL in balance ");
            }
          }
          else //reset charge timer if Vcell < 4V
          {
            if (Hist_Lowest_Vcell < cell.volt_chargeBulk) {
              gCharge_Timer = 0;
              Serial.println(" Charge timer reset to 0 hrs because Vcell lowest discharged below 4V ");
            }
          }
        }
        else
        {
          if (Print_Flag){
          Serial.println(" Charge relay IS OFF because charge input is OFF ");
          }
          gCharge_Timer = 0;   // reset 24 hour timer because charger is disconnected
        }
      }
      else
      {
        // display whihc is over voltge, pack or cell
        if (Hist_Highest_Vcell >= cell.volt_HVD ) {
          Serial.print(Hist_Highest_Vcell); Serial.print("V is > or = "); Serial.print(cell.volt_HVD);
          Serial.println("**** FAULT - Cell V is too high to initiate charger now.... ");
        }
       if (Vpack >= Vpack_HVD) {
        Serial.print(Vpack); Serial.print("is >>");Serial.print(Vpack_HVD);
        Serial.println("**** FAULT - Pack V is too high to initiate charger now.... ");
       }
        
        // Open charge relay if cells or pack go over limits
        Serial.println("**** FAULT - Cell or pack voltage is too high to initiate charger now.... ");
         // relay is shut relay, start 1 day timer
            gCharge_Timer = 24;   //24 hours
          if (Print_Flag){ Serial.print(" Charge Timer =  ");  Serial.print(gCharge_Timer); Serial.println(" hrs");
          }
      }
   
  relay_drive_state = false;   // default to relay off
  // Pack check for motor relay and cell check for motor relay
  if ((Vpack > Vpack_HV_Run_Limit) || (Vpack < Vpack_Lo_Run_Limit)){
    if (Print_Flag) Serial.println(" Drive relay IS OFF because Vpack is too LOW or too HIGH");
  }
  else{
    if((Vpack < (Vpack_HV_Run_Limit - HYSTERESIS)) && (Hist_Lowest_Vcell > cell.volt_LVD)){ // check pack and cell specs
      if ((digitalRead(INPUT_CHARGE_PIN) != 0) && (digitalRead(INPUT_KEYSWITCH_PIN) == 0)){
        relay_drive_state = true;   // if pack ok, and charger is off (or disconnected) turn on motor control
        if (Print_Flag) Serial.println(" Drive relay IS ON because KEYSWITCH is ON AND CHARGE input is OFF ");
      }
      else if(Print_Flag){
        Serial.println(" Drive relay IS OFF because KEYSWITCH is OFF or CHARGE input is ON, but packV and cellV are within limits ");
      }
    }
    else{
      Serial.println(" Drive relay IS OFF because Vpack or Vcell is too LOW - SO CHARGE PACK ");
      dkError = ERROR_UNDERVOLT_CELL ;         
    }
  }

  // new 7/19/2019
  // Cell temperature tests - OVERTEMP and UNDERTEMP for both charging and discharging
  // Overtemp first

  if (Hist_Highest_Tcell < cell.temp_over_driveRelayCut) {  // lower counts are hotter
      relay_drive_state = false;               
      Serial.println(" ***** FAULT - Drive relay is OFF because cell OVERTEMP ");
      Serial.print(" ***** FAULT - Overtemp Cell temp counts = "); Serial.println(Hist_Highest_Tcell);
      Serial.print(" *** High temp limit = "); Serial.println(cell.temp_over_driveRelayCut);
  }
  
  if (Hist_Highest_Tcell < cell.temp_over_chargeRelayCut) {     // lower is hotter with NTC thermistor
      relay_charge_state = false;
      Serial.println(" ***** FAULT - Charge relay is OFF because cell OVERTEMP ");
      Serial.print(" ***** FAULT - Overtemp Cell temp counts = "); Serial.println(Hist_Highest_Tcell);
      Serial.print(" *** High temp limit = "); Serial.println(cell.temp_over_chargeRelayCut);
      } 
    
  // undertemp second
  if (Hist_Lowest_Tcell > cell.temp_under_driveRelayCut) {
     relay_drive_state = false;               
     Serial.println(" ***** FAULT - Drive relay is OFF because cell UNDERTEMP ");
     Serial.print(" ***** FAULT - Undertemp Cell temp counts = "); Serial.print(Hist_Lowest_Tcell);
     Serial.print(" Low temp limit = "); Serial.println(cell.temp_under_driveRelayCut);
    }
    
   if (Hist_Lowest_Tcell > cell.temp_under_chargeRelayCut) {  // higher is colder with NTC
     relay_charge_state = false;
     Serial.println(" ***** FAULT - Charge relay is OFF because cell UNDERTEMP ");
     Serial.print(" ***** FAULT - Cell temp counts = "); Serial.print(Hist_Lowest_Tcell);
     Serial.print(" Low temp limit = "); Serial.println(cell.temp_under_chargeRelayCut);
    } 
    

  // new 7/19/2019




  
  // cell temperature check, make limp mode => 60C, relay open at 63C
  //if (Hist_Highest_Tcell < NTC_63C) {
  //    relay_drive_state = OFF;               // cell >63C so turn off motor and charger relays
  //    relay_charge_state = OFF;
  //    Serial.println(" ***** Charge AND Drive relay ARE OFF because cell OVERTEMP ");
  //}
  //else // cells not hot
  
    if (Print_Flag){
      Serial.print(" Charger HVD: ");     Serial.print(Vpack_HVD, 2);  Serial.print("VDC"); Serial.println ();
      Serial.print(" Motor LVD: ");     Serial.print(Vpack_LVD, 1);  Serial.print("VDC"); Serial.println ();
      Serial.print(" Vpack_HV_Run_Limit: ");     Serial.print(Vpack_HV_Run_Limit, 1);  Serial.print("VDC"); Serial.println ();
      Serial.print(" Vpack_Lo_Run_Limit: ");     Serial.print(Vpack_Lo_Run_Limit, 1);  Serial.print("VDC"); Serial.println ();
      Serial.println ();
    }

  // if undervolt cell AND Comms timeout (= comms error), wait until no comms error to turn on Drive
  if ((Disconnected_Block == true) && (dkError == ERROR_UNDERVOLT_CELL)) {
     if (dkError == ERROR_UNDERVOLT_CELL) relay_drive_state = false; 
       if (VerbosePrintSUPERDATA) Serial.println(" Drive relay is OFF because cell UNDERVOLT/Cell Disconnect ");
     //if (dkError == OVERVOLT_CELL) relay_charge_state = OFF; 
  }


  //   electrically switch relays on/off

  //if ((relay_charge_state == ON) && (digitalRead(INPUT_CHARGE_PIN == LOW)))    // Charger input goes low when charger is conn
  if ((relay_charge_state == true) )    // Charger input goes low when charger is conn
  {
    digitalWrite(RELAYDR1_CHARGE_PIN, HIGH);
      if (VerbosePrintSUPERDATA) Serial.println("Charger relay K1 is closed");
    dkMode = DK_CHARGE;
  }
  else {
    dkMode = DK_PAUSE;
    digitalWrite(RELAYDR1_CHARGE_PIN, LOW);
      if (VerbosePrintSUPERDATA) Serial.println("Charger relay K1 is open");
  }



  if ((relay_drive_state == true))   // keyswitch input goes lo when turned on
  {
    digitalWrite(RELAYDR2_DRIVE_PIN, HIGH);
      if (VerbosePrintSUPERDATA) Serial.println(" Motor Relay K2 is closed");
    dkMode = DK_DRIVE;
  }
  else {
    digitalWrite(RELAYDR2_DRIVE_PIN, LOW);
      if (VerbosePrintSUPERDATA) Serial.println("Motor Relay K2 is open");

  }


  // control charger current with DAC2
  float TargetI;        //Amps
  const int FULL_CHARGE_RATE = 1060 / 2;  // full charge rate = 5V out for Elcon charger control
  uint16_t FullChargeCurrent = 30;      // 30 amps full chg for FIDO
  const int STARTING_CHARGE_RATE = 1023 / 5; // zero charge rate to start - below 2V is 0 charge rate for Elcon PFC charger
  //const float BEGIN_BALANCEV = 4.0;
  float BalanceChargeCurrent = 0.20;  // 200ma charge currrent for balance, cell balancers are running at 250ma
  //Vcell_Balance

  // 100% charge rate to 4VPC, 100ma from there on,  default to 50% charge rate
 // if ((Hist_Highest_Vcell < Vcell_Balance) && (Vpack < Vpack_HVD )) TargetI = FullChargeCurrent; // Full charge rate to 90% or 4.000V
  if ((Hist_Highest_Vcell < cell.volt_Balance) && (Vpack < (cell.volt_Balance * NUMBER_OF_S_CELLS))) TargetI = FullChargeCurrent; // Full charge rate to 90% or 4.000V
  else
  {
    TargetI = BalanceChargeCurrent;                     // 200ma charge rate from there up to keep cells balanced
  }
      if (Print_Flag){  
      Serial.print(" Target Charge amps: ");    Serial.print(TargetI, 2);
      Serial.print(" Actual Charge amps: ");    Serial.println(gAmps, 2);
      }
    
    
  // init current pot
  if (dkMode != DK_CHARGE) gTempPot = STARTING_CHARGE_RATE;
  else //charge control fix Apr 2017
  {
    if ((gAmps > TargetI) && (gTempPot > 0)) gTempPot--;
    if ((gAmps < TargetI) && (gTempPot < FULL_CHARGE_RATE)) gTempPot++;
    // do nothing if they are equal
  }
  analogWrite(CHARGER_CONTROL, gTempPot);     // PWM2_DAC2 = DAC2 == CHG control 3-5V = 0-100%
    //charge control fix Apr 2017
  
  if (Print_Flag){ Serial.print(" Charger control setting: ");    Serial.print(gTempPot);  Serial.println(" out of 1024: ");}

  
    
  


  // Average SOCv over minutes (improvement) - Jul 12, 2017
  if ((relay_drive_state == false) && (relay_charge_state == false)) gSOCv = Vpack/2 ;           // both motor and charger relays are off == init SOC == Vpack

 
  // new 7/16/19
//   if ((relay_drive_state == OFF) && (relay_charge_state == OFF)) gSOCv = ((Vpack - Vpack_LVD)*2) ;           // both motor and charger relays are off == init SOC == Vpack
   gSOCv = ((Vpack - Vpack_LVD)*2) ;   
   //gSOCv = (Vpack + (gSOCv * 999)) / 1000.0; // average for noise reduction 
   if (Vpack >= Vpack_LVD) TempV = gSOCv;    // pack is (basically) dead at 2.95V per cell
   else TempV - 0;
   // new 7/16/19
   
   //if (Print_Flag)
   {   // print data only when block data is in
    Serial.print(" Vbat derived SOC: ");    Serial.print(gSOCv,0);  Serial.println("%  ......................... ");
       Serial.print(Vpack); Serial.print("   Vpack/Vpack LVD:   ");  Serial.println(Vpack_LVD);
   }
  
  //gSOCv = (Vpack + (gSOCv * 999)) / 1000.0; // average for noise reduction 
  
 // if (gSOCv >= Vpack_LVD) TempV = gSOCv - Vpack_LVD;    // pack is (basically) dead at 2.95V per cell
 // else TempV = 0;
  // Average SOCv over minutes (improvement) -  Jul 12, 2017
  //    Serial.print(" Tempv: ");    Serial.print(TempV,0);  Serial.println();

  //if (Print_Flag){   // print data only when block data is in
    // only turn LIMP mode off at power cycle or keyswitch cycle (limp latch) 
  //TempV = TempV * 50;  // for fido

  TempV = (TempV/100) * 1024; // PWM ==> 50% SOC = 0.5*1024
  
  // limit check...
  //if (Vpack < 59) TempV = 0;
  if (Vpack < Vpack_LVD) TempV = 0;   // change for kents bug
  
  if (TempV > MAX_PWM) TempV = MAX_PWM;    // 10 bit resolutioin - 1024 Max
  analogWrite(PWM1_DAC1, TempV);     // PWM1_DAC1 ==DAC1 == SOC output
  // analogWrite(PWM2_DAC2, TempV);     // PWM2_DAC2 = DAC2 == CHG control
 
  
  SOCv = (TempV / 1024) * 100;
  //if (VerbosePrintSUPERDATA)
  {
    Serial.print(" PWM1_DAC1 Counts (SOC): ");     Serial.print(TempV, 0);
    Serial.print(" SOC: ");     Serial.print(SOCv);  Serial.println ("%");
  }
    
  

  // Give "limp mode" signal (+3.3 logic high) at hot cells(57C)
  //if ((TempV < 200) || (Hist_Highest_Tcell < NTC_60C))
  if ((TempV < 200) || (Hist_Highest_Tcell < NTC_57C))
  {
    //digitalWrite(OUTPUT_LIMP_PIN, HIGH);
    if (seconds > 10) digitalWrite(OUTPUT_LIMP_PIN, HIGH); //do well after startup to give time for vpack to settle
  }
    else if (digitalRead(INPUT_KEYSWITCH_PIN) != 0)
  { 
    digitalWrite(OUTPUT_LIMP_PIN, LOW);
  }
  if (Goodcomms){   // print data only when block data is in
  //if (VerbosePrintSUPERDATA)
    {
      if (digitalRead(OUTPUT_LIMP_PIN) == 0) Serial.println("'Limp Mode' is off because of because SOC > 20% and cell temp < 60C");
      else {    
        if (Hist_Highest_Tcell < NTC_57C) Serial.println("'Limp Mode'  is on because Cell temp is over 60C");
        else  Serial.println("'Limp Mode'  is on because SOC is at or under 20% - and keyswitch has not reset this condition");
      }
      // only turn LIMP mode off at power cycle or keyswitch cycle (improvement limp latch) Jul 12, 2017
   }
  }
 // Serial.println();





  // nRF24 2.4Mhz packet comms - send ACK SYNC packet from PS then receive block data


   // new 7/11/2019 - change freq for each block - each block gets its own freq channel
   driver.setChannel(gBlockNumber);     // change channel on each block to equal it's block address
   //Serial.print(" driver.setChannel(gBlockNumber) =  "); Serial.println(gBlockNumber); 
   // new 7/11/2019


// new Mar 2019 debug

  // put data TO block into a struct - NEW 3/21/2019
  //**************debug Metrics
        gdebugRXtime = millis();       // get current time
  //**************debug Metrics
  struct dataStruct{
    //float stempValue1;   // want to send floats in the future? //int stempValue2;     // want to send ints in the future
    //unsigned long sSyncTX;  // sync time from P.S. to Block 
    // new 7/10
    //uint16_t BlockNumber;
    uint16_t BlockNumber = gBlockNumber;
    }ACKSyncpacket;
    
    uint8_t from;

 
    // Send rolling block number to ACK SYNC each block in turn
    // new 7/10/2019
    // if Packet timer has timed out or if valid packet received, increment to next block
    if (gPacketTimer > Main_Loops) {
      // new 7/16
      Serial.println("****");
      Serial.print("**** COMMS fault - BLOCK -  "); Serial.println(gBlockNumber); // print block error--> did not comms!
      Serial.println("****");
      gBlockNumCommFault = gBlockNumber;
      // new 7/16
      gBlockNumber++;
      gPacketTimer = 1;
    }
    else if (gValidPacket == ACKSyncpacket.BlockNumber) gBlockNumber++;
    
    gPacketTimer++;
    // new 7/10/2019
 
    if ( ACKSyncpacket.BlockNumber > NUMBER_OF_BLOCK_MANAGERS)  
    {
      // new 7/16
      Serial.print("!!!! This block not talking =  "); Serial.println(gBlockNumCommFault); // print block error--> did not comms!
      //Serial.print("!!!! Comms fault =  "); Serial.println(gBlockCommsFault); // print block error--> did not comms!

      // new 7/16
      gBlockNumber = 1; // always start with bl 1 not 0!
    }

    // new 7/10/2019
 //   Serial.print(" ACKSyncpacket.BlockNumber =  "); Serial.println(ACKSyncpacket.BlockNumber ); 
 //   Serial.print(" gPacketTimer=  "); Serial.println(gPacketTimer); 
    ACKSyncpacket.BlockNumber = gBlockNumber ; // make current block number global
    
    // new 7/10/2019
    
    // accurate syncronising here - every 50msecs
 
    // send ACK TRIGGER to each Block, one at a time to make them send Block data
    manager.sendtoWait((uint8_t*)&ACKSyncpacket, sizeof(ACKSyncpacket), RH_BROADCAST_ADDRESS);  // WORKS faster because broadcast doesn't wait for ACK!!
    
 //debug
 //delay(20);  // ALLOW 50 msecs between block receptions (main takes ~20ms)
  
  Goodcomms = false;    // default is there is no new data
// Check if block data is in yet  // <available> function code is in RH_NRF24.cpp file
  if (manager.available()){
 
    GetBlockData();
  }
  else {      
    if (VerbosePrintCommsDATA){
    Serial.print("No packet this time = ");
    Serial.println(millis() - gdebugRXtime); Serial.print(" msecs");
    }
  }

  

  //bool GoodComms = false;
  uint8_t PAYLOAD = 16;     // bytes, use for comparing payload, comms will ignore data if wrong PL
  float floatmatch = 0.000600; // +/- this amount to allow for matching float math on receive
  uint8_t len = sizeof(buf);
 // uint8_t from;
  float tempa;    // 4 bytes
  float tempb;
  float tempc;



 if (Goodcomms == false) goto badcomm;    // get out right now don't do checksum or payload checks




//debug
//delay(10);
delay(1);

      // filter for wrong payload length
      if (len != PAYLOAD)  
      {
        if (VerbosePrintCommsDATA) {
          Serial.print(" WRONG data length, is : should be  ");
          Serial.print(len);    Serial.print(PAYLOAD);
           Serial.println(".. so do not use this bad data!!!!!!");
          
        }
        goto badcomm;     // check for correct payload, get out if not
      }

      // filter for bad checksum
         Temp1 = (((DATA*)buf) -> sCellV_Hiside );
        Temp2 = (((DATA*)buf) -> sCellV_Loside );
        Temp3 = (((DATA*)buf) -> sThottest );
        Temp4 = (((DATA*)buf) -> sTcoldest );
        tempc = (((DATA*)buf) -> schecksum );

      tempa = Temp1 + Temp2 + Temp3 + Temp4;
      if (VerbosePrintCommsDATA){
        Serial.print(" Calculated checksum is = "); Serial.println(tempa,6); Serial.print(" Received checksum is = "); Serial.println(tempc,6);
      }
      tempb = tempa + floatmatch;    
      tempa = tempa - floatmatch;
      
      if (VerbosePrintSUPERDATA){
        Serial.print(" Checksum window to allow var for FP math is:   "); Serial.print(tempa,6); Serial.print(" low and high   "); Serial.println(tempb,6);
      }
      if ((tempc <= tempb) && (tempc >= tempa))     // allow data if checksum is within 500ppm accuracy
      {
           if (VerbosePrintSUPERDATA) Serial.print(" Checksum does agree with client!!!!!!!!!!");  
      }
      else
      {
         if (VerbosePrintCommsDATA){
            Serial.print(" Checksum does NOT agree with client"); 
            Serial.print(" Bad comms so no save BAD DATA"); 
        }
        
        goto badcomm;     // bad data = no save
      }

       // GoodComms = true;
       //  Serial.println(" Use and save this good data..."); 

       // new 7/10/2019
       gValidPacket = gBlockNumber;
       gPacketTimer = 0;
       // new 7/10/2019
 

badcomm:   

      // learn blocks to connect to here...if in LEARN BLOCKS mode
       //  Save this block if not already saved
      bool This_Block_Saved = false;
      if ((LEARNBLOCKS == true))
      {
        uint16_t x = NUMBER_OF_BLOCK_MANAGERS;   // number of blocks (+ 1 for zero)
        digitalWrite(LED2_RED_PIN, HIGH);  //on
        digitalWrite(LED1_RED_PIN, HIGH);  //on
     
        while ( x > 0 )
        {
          if (from == blockNum[x - 1])    // first check = has BLOCK already been saved?
          {
           if (VerbosePrintSUPERDATA){
            Serial.print("THIS BLOCK IS ALREADY SAVED IN LOCATION: "); Serial.println(x - 1);
            Serial.print("from: "); Serial.print(from); Serial.print("blockNum: "); Serial.println(x - 1);
           }
           This_Block_Saved = true;
           // zero comm timer for this block
           Block_Comm_Timer[x] = 0;
        }
          x--;
        }
        x = NUMBER_OF_BLOCK_MANAGERS;
        while ( x > 0 )
        {
          if (This_Block_Saved != true)      // if not saved, save BLOCK to zero'd location
          {
            // not saved so store this block in an unused or zerod location
            if (blockNum[x - 1] == 0)
            {
              blockNum[x - 1] = from;
              Serial.print("success saving Block#: "); Serial.print(from);
              Serial.print(" into RAM location: "); Serial.println(x - 1);
              goto exitsaveblock;
            }
          }
          x--;
        }
      }
      else // LEARN mode is off, so check incoming block number with saved blocks
      {
        int x = NUMBER_OF_BLOCK_MANAGERS;
        while ( x > 0 )
        {
          if (from == blockNum[x - 1])    // check if BLOCK already been saved?
          {
            if (VerbosePrintSUPERDATA){
            Serial.print("This block is ALREADY SAVED in EE location: "); Serial.println(x - 1);
            }
            This_Block_Saved = true;
            // zero comm timer for this block
            Block_Comm_Timer[x-1] = 0;
          }
          x--;
        }
      }

exitsaveblock:
      // Serial.print("Blocknum Saved: "); Serial.println(This_Block_Saved);
      // removed learn mode (Block_Saved) test for Kents' bug 'cause last of the RF24 comm units
      // Used Goodcomms instead  //once you have good comms with known blocks ...use that good data (until then use defaults from startup)
      //if (This_Block_Saved)    // if data is sent by a DKblock in this pack, read the data
      if (Goodcomms)
      {
        if (VerbosePrintCommsDATA)
        {
          Serial.print(" Do the math on this GOOD data: ");
          Serial.print(((DATA*)buf) -> sCellV_Hiside ); Serial.print(" VDC Hi Cell   ");
          Serial.print(((DATA*)buf) -> sThottest); Serial.print(" Hot NTC counts   ");
          Serial.print(((DATA*)buf) -> sTcoldest); Serial.print(" Cold NTC counts   ");
          Serial.print(((DATA*)buf) -> sCellV_Loside ); Serial.print(" VDC Lo Cell   ");
          Serial.print(((DATA*)buf) -> schecksum ); Serial.println(" CHECKSUM");
        }
        Temp1 = (((DATA*)buf) -> sCellV_Hiside );
        Temp2 = (((DATA*)buf) -> sCellV_Loside );
        Temp3 = (((DATA*)buf) -> sThottest );
        Temp4 = (((DATA*)buf) -> sTcoldest );
        if (VerbosePrintCommsDATA){
        Serial.println(" High resolution mode values: ");
        Serial.print(Temp1, 3); Serial.print(" VDC Hi Cell   ");
        Serial.print(Temp2, 3); Serial.println(" VDC Lo Cell");
        }
      }

    if (CommsFaults > 100) {
      CommsFaults = 0;
      Serial.print("Comms faults = "); Serial.print(CommsFaults);
      // since we have high comms faults, reint hardware
      manager.init();
      SPI.begin();  
    }


  // end nRF24 2.4Mhz packet comms
  digitalWrite(LED2_GREEN_PIN, LOW);  //off
  digitalWrite(LED2_RED_PIN, LOW);  //off


  //once you have good comms with known blocks ...use that good data (until then use defaults from startup)
  if (Goodcomms)
  {
    // find highest/lowest Vcell - peak detector
    if (Temp1 >= Temp2)
    {
      // reaction time too slow...speed up by less averaging  -data is better now...
      if (Temp1 > Highest_Vcell) Highest_Vcell = ((Highest_Vcell + Temp1) / 2) ;  // average with last reading
      if (Temp2 < Lowest_Vcell) Lowest_Vcell = ((Lowest_Vcell + Temp2) / 2) ;
    }
    else {
      if (Temp2 > Highest_Vcell) Highest_Vcell = ((Highest_Vcell + Temp2) / 2) ;  // insure that one bad byte does not change the value much
      if (Temp1 < Lowest_Vcell) Lowest_Vcell = ((Lowest_Vcell + Temp1) / 2) ;
    }
    if (Comm_Flag){  // once per minute
      Serial.print(" Highest Vcell: ");     Serial.print(Highest_Vcell, 3);  Serial.print(" VDC");
      Serial.print(" Lowest Vcell: ");     Serial.print(Lowest_Vcell, 3);  Serial.print(" VDC"); Serial.println ();
    }

    // find highest and lowest Tcell --- hotter is lower counts
    if ((Highest_Tcell > Temp3) || ( Highest_Tcell == 0)) Highest_Tcell = ((Highest_Tcell + Highest_Tcell + Highest_Tcell + Temp3) / 4 );      //hotter
    if (Lowest_Tcell < Temp4) Lowest_Tcell = ((Lowest_Tcell + Lowest_Tcell + Lowest_Tcell + Temp4) / 4);
    //    if (Lowest_Tcell < Temp4) Lowest_Tcell = Temp4;
    if (Comm_Flag){  // once per minute
      Serial.print(" Hottest Tcell: ");     Serial.print(Highest_Tcell, 1);  Serial.print(" ADC counts");
      Serial.print(" Coolest Tcell: ");     Serial.print(Lowest_Tcell, 1);  Serial.print(" ADC counts"); Serial.println ();
    }
    // Running average of cell parameters:
    if (HistoryTimer == 0) {
      HistoryTimer = T_HISTORYCHECK;    // set timer for next check
      //take a voltage sample and make running average over last n sec
      Hist_Highest_Vcell = (Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell ) / 3;
      Hist_Lowest_Vcell = (Lowest_Vcell + Hist_Lowest_Vcell + Hist_Lowest_Vcell) / 3;
      Hist_Highest_Tcell = (Highest_Tcell +  Hist_Highest_Tcell + Hist_Highest_Tcell) / 3;
      Hist_Lowest_Tcell = (Lowest_Tcell + Hist_Lowest_Tcell + Hist_Lowest_Tcell) / 3;
      // reaction time too slow...speed up by less averaging  -data is better now...
      //Hist_Highest_Vcell = (Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell + Hist_Highest_Vcell) / 11;
      //... 
      // reset the peak detectors (discharge the software capacitor)
      if (seconds > 10) {   // well after powerup
        //  if (Highest_Vcell > Vcell_Nominal_Spec) Highest_Vcell = (Highest_Vcell - 0.001);
        //   if (Lowest_Vcell < Vcell_Nominal_Spec) Lowest_Vcell = (Lowest_Vcell + 0.001);
        Highest_Vcell = (Highest_Vcell - 0.001);     // discharge away from peak
        Lowest_Vcell = (Lowest_Vcell + 0.001);       // discharge away from peak
        if (Highest_Tcell < NTC_AMBIENT)  Highest_Tcell ++;    // hottest goes toward colder
        if (Lowest_Tcell > NTC_AMBIENT) Lowest_Tcell --;    // coldest goes toward hotter
      }
    }
    if (Comm_Flag){  // once per minute
      Serial.print(" Historical Avg Highest Vcell: ");     Serial.print(Hist_Highest_Vcell, 3);  Serial.print(" VDC ");
      Serial.print(" Historical Avg Lowest Vcell: ");     Serial.print(Hist_Lowest_Vcell, 3);  Serial.print(" VDC"); Serial.println ();
      Serial.print(" Historical Avg Hottest Tcell: ");     Serial.print(Hist_Highest_Tcell, 0);  Serial.print(" ADC counts ");
      Serial.print(" Historical Avg Coolest Tcell: ");     Serial.print(Hist_Lowest_Tcell, 0);  Serial.print(" ADC counts"); Serial.println ();
    }
  }


//badcomm:       // bad comm bytes end up here...

  #ifdef KOSO_DL03
    kosoDL03Manager();
  #endif

  // Block awareness: check to see which blocks are talking, over a 100 sec window

  watchdogReset();  // wd resets in 1 sec

  //debug
    // delay(30);  // ALLOW about 50 msecs between block transmits/receptions (main takes ~20ms)
    delay(20);  // ALLOW about 50 msecs between block transmits/receptions (main takes ~20ms)
}
