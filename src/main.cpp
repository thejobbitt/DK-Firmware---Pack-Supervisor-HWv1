// Compile for Teensy 3.1, 96Mhz overclock, fastest code
// DK Supervisor PCB production code for Teensy 3.1 3.2   7/8/15  T Economu
// Compiled with VS Code and PlatformIO
//

// Date       Who     New Ver   WAS/IS changes
//
// TBE  6/16/2016  Activate CONNECT input  to be a "LEARN BLOCKS" function
// TBE  6/29/2016 Add 3-5VPMW for charge control on DAC2 output, Add Limp Mode output to A4-NC testpoint (PIN19)
// 7/7/2016   TE      2116    Added checksum for data comms to ensure good quality data, improved bogus bytes. Also
//                            added COP watchdog timer       
//  7/11/2016 TBE     2816    Added limp mode for Cell temp >= 60C and open relay for >= 63C per Jeb     
//  7/13/2016 TBE     2816    Changed Vbalance from 4.110 to 4.100 for LG MJ1 cell, calibrated Vpack and Icharge
//                            Also Need to order charger to test!
//  3/27/2018 TBE     1319    Sends ACK-sync packet from PS to block, so that blocks send one at a time
//  4/4/2018                  Made new function GetBlockData()
//  4/5/2019  TBE     1419    Integrate with production code, tested with 8 blocks in sync! Also added back in 4/17/2017 fix for 2-5VPWM DAC charge control not updating during charge mode
//  4/16/2019 TBE     1619    Named code - Working_supervisor_1619, changed to 144V/40 cell nominal system for run/learn code.
//                    1619-1  Added back in Keyword "Jul 12, 2017" Latched limp mode when SOC < 20%. Reset when Pack Supervisor power is cycled. Also was no SOCv averaging so Fuel Gauge jumped around a lot. Averaged value so steadier .
//                            And SOCv and TempV properly initialized, Vpack is calibrated for 150V and checked at 100V to be within 0.025% with Fluke 45
//                    1619-2  LEARN_TIMEOUT was 4 min after power up is 2 min
//  7/11/2019 TBE     2819    Breakthrough on comms throughput to help with interference between block comms. Each block gets its own freq channel (equals its block address).
//                            Also changed No_Of_Cells from 20 to 40 for KEnt's bug
//  7/18/2019 TBE     2919-1  Found bug in Avago/Broadcom data sheet for ACPL buffer amp, making oscillator at 100-150Vbat. Filter caps was 100pf is 0.1uf, 
//  7/19/2019 TBE     2919-2  Code did cell overtemp at 63C in discharge only. Rewrote over/under temperature cell protection for LG MJ1 cell in 
//                            and discharge modes (yes they are different)
//  7/23/2019 TBE     2919-3  Tested code for all 20 blocks in Kents bug with actually his hardware. 3.3 seconds comms for all 20 blcoks.
//                            For Kents Bug Server is address 60 and blocks are numbered 1-20   
//  7/24/2019 TBE     2919-4  Calibrated current sensor LEM HO-100S, calibrated SOC output                               
//  7/26/2019 TBE     3019    Filtered Vpack more, reacts too fast (but is very accurate)! Verified better than 1% accuracy at 100V and 160V. Note on next Version to put bigger X7R cap on Vpack resistor
//  8/1/2019          3019    Shipped Kent's bug units - 20 BM PCB and 1 PS PCB with Server Address = 60 and Block address = 1-20

#define VERSION 0123   // 5215 = 52th week of 2015

// ****************************************************************************CONFIGURATION
// Cell Type
  //#define LG_MH1
  #define LG_MJ1
  //#define SANYO_NCR18650B

// Pack defines
  #define NUMBER_OF_BLOCK_MANAGERS 40

// Define the radio
  #define RF24
  //#define SUBG

// Define the charger
  // No Control charger
    #define CHARGER_ON_OFF
  // Analog style 0 to 5v
    //#define CHARGER_ANALOG_DAC2
    #ifdef CHARGER_ANALOG_DAC2
      #define ANALOG_ON   5 // fully on voltage
      #define ANALOG_OFF  2 // fully off voltage
    #endif
  // 72v CAN Bus Charger
    //#define MYBLUESKY_S2500

    
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


// ****************************************************************************APPLICATION SETUP
#pragma region APP
  // app states
#define DK_SLEEP    0
#define DK_PAUSE    1
#define DK_CHARGE   2
#define DK_DRIVE    3
uint8_t dkMode = DK_SLEEP;  // default to sleep

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

#pragma endregion APP

void GetBlockData(void);

// ---------SERVER ADDRESS History
//#define SERVER_ADDRESS 2 -6 used for FIDO deliveries in 2017
// DONT use 0 or 255! ---does not work for manchester encoding!
//#define SERVER_ADDRESS 1= not used
#define SERVER_ADDRESS 60  // 1 and 9 => reserved for lab use
// new 7/11/2019
#define NUMofDKBLOCKS 20  // number of dk blocks in series in this pack
//#define NUMofDKBLOCKS 2  // number of dk blocks in series in this pack
// new 7/11/2019

// new 7/10/2019
uint8_t gBlockNumber = 1;  // default to first  block, can be 1-40 blocks NOTE there is no number 0 block
uint8_t gValidPacket = gBlockNumber;  // this var checks what incoming packet is valid
uint8_t gPacketTimer = 1;   // number of loops before next block in line is polled - start with 20 loops or about 800 msecs. Typ is 200 msec aver block time
//new 7/11/2019
//uint8_t Main_Loops = 15;   // at power up use 1 main loop for packet timing to wake up blocks, then change to n loops (start with 12) to insure communication
uint8_t Main_Loops = 1;   // at power up use 1 main loop for packet timing to wake up blocks, then change to n loops (start with 12) to insure communication
uint8_t gBlockNumCommFault = 0;   // This var holds the last block to not communicate to Pack Supervisor
//new 7/11/2019


// Single instance of the radio driver
#define DK_MESSAGE_LNGTH 16  //DK uses 16 byte packet
RH_NRF24 driver(9, 10); // for Teensy 3.x SS and CE lines
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);
// Dont put this on the stack:
//uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];    // buffer = 24 bytes long is max RH packet
uint8_t buf[DK_MESSAGE_LNGTH];    // buffer = 16 bytes long is max DK packet

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


const bool YES = 1;
const bool NO = 0;
const bool ON = 1;
const bool OFF = 0;
bool Goodcomms = NO;      // default to NO until good packet is received
  
// nRF24 2.4Mhz packet comms


// declare debug printouts
bool PRINT = 0;   // 0 = no debug print
bool VerbosePrintBLOCKDATA = PRINT;  // turn on (1) or off verbose prints
bool VerbosePrintSUPERDATA = PRINT;  // turn on (1) or off verbose prints for supervisor
bool VerbosePrintEEDATA = PRINT;  // turn on (1) or off verbose prints
bool VerbosePrintCommsDATA = PRINT ;

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

#define VREF (3.266)         // ADC reference voltage (= power supply)
#define VINPUT (2.171)       // ADC input voltage from resistive divider to VREF
#define ADCMAX (65535)       // maximum possible reading from ADC
#define EXPECTED (ADCMAX*(VINPUT/VREF))     // expected ADC reading
#define SAMPLES (200)      // how many samples to use for ADC read - 200 seemed to work best   
#define ADC_RESOLUTION (12)  // ADC resolution in bits, usable from 10-13 on this chip
#define DAC_RESOLUTION (10)  // DAC resolution in bits, usable from 0-12 on this chip (same setup as PWM outputs)
// ****************************************************************************WATCHDOG SETUP
#pragma region WD
  // prototypes
void WatchdogReset(void);
#pragma endregion WD

// ****************************************************************************DIGITAL IO SETUP
#pragma region DIO
  // switch pins
#define SW1_LEARN_BLKS_PIN  2
  // led pins
#define LED1_RED_PIN        15
#define LED1_GREEN_PIN      16
#define LED2_RED_PIN        7
#define LED2_GREEN_PIN      8
  // relay pins
#define RELAYDR1_CHARGE_PIN 5
#define RELAYDR2_DRIVE_PIN  6
  // user input pins
#define INPUT_CHARGE_PIN    22
#define INPUT_KEYSWITCH_PIN 23
  // other pins
#define OUTPUT_LIMP_PIN     18
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
  // variables
bool relay_charge_state = 0;
bool relay_drive_state  = 0;
  // prototypes
void init_DIO(void);
void init_AIO(void);
#pragma endregion DIO

// ****************************************************************************ANALOG IO SETUP
#pragma region AIO
  // input pins
#define PACK_VOLTAGE_PIN              A0  // Schematic(VSCALED-PACK), 1.4v at 100VDC, 2.4V at 170VDC
#define SUPERVISOR_TEMP_PIN           A3  // Onboard supervisor NTC (previously NTCambient)
#define CURRENT_SENSOR_CHARGE_PIN     A11 // Schematic(ICHG), PCB(CON8, CH)
#define CURRENT_SENSOR_DISCHARGE_PIN  A5  // Schematic(IDISCHG), PCB(CON8, DIS)  
#define VBALANCE                      A10 // Schematic(TP VBAL), Currently unused
  // output pins
#define DAC_OUT_FUEL_GAUGE A14
  // PWM Pins for op amp digital DACs (note: 10 bits PWM same setup as Analog DAC output)
#define PWM1_DAC1 20
#define PWM2_DAC2 21
  // variables
const int pwm_freq = 40000;
const int FULLPWMRANGE = 1000;
#pragma endregion AIO

// ****************************************************************************BLUE MY SKY CHARGER
#pragma region BMSCAN
#ifdef MYBLUESKY_S2500
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
    DEBUG_PRINT("Tx - ")
    DEBUG_PRINT_HEX(hold.id); DEBUG_PRINT(" ");
    DEBUG_PRINT_HEX(hold.len); DEBUG_PRINT(" ");
    for(int i = 0; i<hold.len; i++){
      DEBUG_PRINT_HEX(hold.buf[i]); DEBUG_PRINT(" ");
    }
    DEBUG_PRINTLN();
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
      DEBUG_PRINT("Rx - ");
      DEBUG_PRINT_HEX(hold.id); DEBUG_PRINT(" ");
      DEBUG_PRINT_HEX(hold.len); DEBUG_PRINT(" ");
      for(int i = 0; i<hold.len; i++){
        DEBUG_PRINT_HEX(hold.buf[i]); DEBUG_PRINT(" ");
      }
      DEBUG_PRINTLN();
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
          DEBUG_PRINT(F("Charger Output: ")); DEBUG_PRINT_1(cVoltage); DEBUG_PRINT("V ");
          DEBUG_PRINT_1(cCurrent); DEBUG_PRINTLN("A ");
          DEBUG_PRINT(F("DK_PS Readings: ")); DEBUG_PRINT_1(Vpack); DEBUG_PRINT("V ");
          DEBUG_PRINT_1(gAmps); DEBUG_PRINT("A ");
          switch (Charge_status_flag){
            case DO_NOT_CHARGE:
              DEBUG_PRINTLN(F("DO NOT CHARGE"))
              break;
            case TRICKLE:
              DEBUG_PRINTLN(F("TRICKLE"))
              break;
            case BULK_FULL:
              DEBUG_PRINTLN(F("BULK FULL"))
              break;
            case BULK_HALF:
              DEBUG_PRINTLN(F("BULK HALF"))
              break;  
            case BULK_QUARTER:
              DEBUG_PRINTLN(F("BULK QUARTER"))
              break;
            case TOP_UP:
              DEBUG_PRINTLN(F("TOPPING UP"))
              break;
            case COMPLETE:
              DEBUG_PRINTLN(F("COMPLETE"))
              break;
          }
        #endif
      }
      // outside of safe operating range? DO NOT CHARGE
      #ifdef CHARGER_DEBUG
        DEBUG_PRINT(F("High Temp  ADC: ")); DEBUG_PRINTLN(Hist_Highest_Tcell);
        DEBUG_PRINT(F("Low  Temp  ADC: ")); DEBUG_PRINTLN(Hist_Lowest_Tcell);
        DEBUG_PRINT(F("High Volt Cell: ")); DEBUG_PRINTLN(Hist_Highest_Vcell);
        DEBUG_PRINT(F("Low  Volt Cell: ")); DEBUG_PRINTLN(Hist_Lowest_Vcell);
      #endif
      if(Hist_Lowest_Tcell >= Tcell_Charge_Low_Cutoff || Hist_Highest_Tcell <= Tcell_Charge_High_Cutoff ||
      Hist_Lowest_Vcell <= Vcell_Charge_Low_Cutoff || Hist_Highest_Vcell >= Vcell_Charge_High_Cutoff) {
        bmsCANSend(BMS_TO_CHARGER, 0, 0, OFF);
        bmsChargeStatus = BMS_DO_NOT_CHARGE;
      }
      // inside of safe operating range? CHARGE
      else{
        // check for very low voltages, if not too low trickle charge
        if(Hist_Lowest_Vcell <= Vcell_Charge_Trickle) {
          bmsChargeStatus = BMS_TRICKLE;
        }
        // check for bulk charge voltages, adjust current based on cell temperature
        if(Hist_Lowest_Vcell > Vcell_Charge_Trickle && Hist_Highest_Vcell < Vcell_Charge_Bulk) {
          if(Hist_Highest_Tcell >  Tcell_Charge_Taper_1) bmsChargeStatus = BMS_BULK_FULL;
          if(Hist_Highest_Tcell <= Tcell_Charge_Taper_1) bmsChargeStatus = BMS_BULK_HALF;
          if(Hist_Highest_Tcell <= Tcell_Charge_Taper_2) bmsChargeStatus = BMS_BULK_QUARTER;
        }
        // check for end charge voltages, top end trickle charge
        if(Hist_Highest_Vcell > Vcell_Charge_Bulk && Hist_Highest_Vcell < Vcell_Charge_Off) {
          bmsChargeStatus = BMS_TOP_UP;
        }
        // if any cell over Charge off V, pause charger
        if(Hist_Highest_Vcell > Vcell_Charge_Off) {
          bmsChargeStatus = BMS_PAUSE;
        }
        // if all cells between balance V and charge off voltage, turn off charger
        if(Hist_Lowest_Vcell > Vcell_Balance && Hist_Highest_Vcell < Vcell_Charge_Off) {
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
#pragma endregion BMSCAN

// ****************************************************************************

 // Lithium Cell temperature specifications - use "NTC thermistor muRata NCP18W104D computations from -40-60C in 5 deg steps.ods"
  #define BAT_TYPE_MJ1  (0) // 0 = LG MJ1 type -   //cell parameters for LG MH1 3200mah

  #ifdef BAT_TYPE_MJ1
    const uint16_t OVERTEMP_CELLS_CHARGING = 701; // 702 ADC counts = 45C 
    const uint16_t OVERTEMP_CELLS_DISCHARGING = 415; // 60C
    const uint16_t UNDERTEMP_CELLS_CHARGING = 2611;  // 0C
    const uint16_t UNDERTEMP_CELLS_DISCHARGING = 3475; // -20C
  #endif 

  #define VPACK_40S (146)        // LG - MJ1 - 40S pack voltage = 40*3.625=148V
  //#define VPACK_20S (74)         // use this for 20S pack voltage like FIDO

  #ifdef VPACK_40S
    #define VPACKNOMINAL (VPACK_40S) 
    //const byte OUTPUT_LIMP_PIN = 18;            // on off signal to motor control at 20% SOC
    const byte CHARGER_CONTROL = PWM2_DAC2;      // 0-2-5V output for charger control near balance (FIDO is 2-5V = 0-100% linear charge)
    const byte CHARGER_RELAY = RELAYDR1_CHARGE_PIN;
    const byte MTRCONTROL_RELAY = RELAYDR2_DRIVE_PIN;
    uint8_t No_Of_Cells = 40 ;  // 146V system for Kents Bug
    const  uint8_t NUMBER_OF_BLOCKS = (No_Of_Cells / 2);
    #endif // only other choice is 72V for now

   #ifdef VPACK_20S              // // define application vars here (this is FIDO)
    #define VPACKNOMINAL (VPACK_20S) 
    //const byte OUTPUT_LIMP_PIN = 18;            // on off signal to motor control at 20% SOC
    const byte CHARGER_CONTROL = PWM2_DAC2;      // 0-2-5V output for charger control near balance (FIDO is 2-5V = 0-100% linear charge)
    const byte CHARGER_RELAY = RELAYDR1_CHARGE_PIN;
    const byte MTRCONTROL_RELAY = RELAYDR2_DRIVE_PIN;
    uint8_t No_Of_Cells = 20 ;  // 72V system for Fido etc
    const  uint8_t NUMBER_OF_BLOCKS = (No_Of_Cells / 2);
   #endif



// LEARN BLOCKS

const byte LEARN_TIMEOUT = 2;       // learn mode timeout after 2 minutes after power up
//uint8_t blockNum[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // the array of 10 blocks for FIDO, this will have to be user configurable
uint8_t blockNum[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Change for Kents Bug
bool LearnBlockSwitch = 1;         // switch used to determine if block values should be zeroed

// block array for block 'awareness'
const uint8_t COMM_TIMEOUT = 255;     // max 4+ minute comm timeout - then go to sleep
//uint8_t Block_Comm_Timer[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Block_Comm_Timer[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };    // Kents bug change


const float CELL_RECONNECT_V = 4.000; // reconnect charger at this (lowest) cell voltage

uint16_t Overtemp_Cells_Charging ;    // vars to store over and under cell temperature specifications
uint16_t Overtemp_Cells_Discharging ;
uint16_t Undertemp_Cells_Charging ;
uint16_t Undertemp_Cells_Discharging ;

//const float VPACK_HI_CHG_LIMIT = 60.0;    // do not allow charger to raise pack > 4.20V cell x 20 cells = 84VDC
// const float Vpack_HV_Run_Limit = 64.0; // Confirm with Jeb
//   const float VPACK_LO_RUN_LIMIT = 52.0;  // do not allow to run pack below 2.80V per cell x 20 cells = 58V
// ship with the following values ...but check with JEb first...
//const float VPACK_HI_CHG_LIMIT = 85.0;    // do not allow charger to raise pack > 4.25V cell x 20 cells = 85VDC
const float VPACK_HI_CHG_LIMIT = No_Of_Cells * Vcell_HVD_Spec;    // do not allow charger to raise pack > 4.25V cell x # of cells in series
//const float Vpack_HV_Run_Limit = 90.0; // Confirm with Jeb
//const float VPACK_LO_RUN_LIMIT = 58.0;  // do not allow to run pack below 2.80V per cell x 20 cells = 58V

// current control
uint16_t gTempPot;       // current potentiomenter
float gAmps;              // amps plus and minus through LEM sensor

// const int led = 13; //temp use of led on teensy    (ALSO used as NTC4 input)

// SPI port
/*
  const byte SCLCK = 13;
  const byte MISO = 12;
  const byte MOSI = 11;
  const byte CE = 9;
*/
    
// declare global vars
//const float EEPROM_CHG_SENSOR_OFFSET = 3094.8;  //3103;       // save offset from 2.50V at this address (0-4096 counts) - start with 2.500V exactly
const float EEPROM_CHG_SENSOR_OFFSET = 3094.77;  //3103;       // save offset from 2.50V at this address (0-4096 counts) - start with 2.500V exactly
uint16_t EEPROM_DISCH_SENSOR_OFFSET = EEPROM_CHG_SENSOR_OFFSET;       // save offset from 2.50V at this address (0-4096 counts) - start with 2.500V exactly
float LEM_Offset = EEPROM_CHG_SENSOR_OFFSET; // starting value


const float HYSTERESIS = 0.4;    // provide hysteresis to prevent relay chatter and oscillation
int sensorValue = 0;        // value read from the ADC input
//float Vnominal = 72.0 ;    // Nominal battery pack voltage at startup
float Vnominal = VPACKNOMINAL ;    // Nominal pack voltage with cells at Vnom

float Vpack = Vnominal;  // start with nominal battery voltage
float gSOCv = Vnominal;   // init with a nominal value
bool LEARNBLOCKS = OFF;   // Learn blocks is turned on by "CONNECT" switch
byte tempx = 0;

unsigned long gdebugRXtime = 0;       // get to get current time from millis()


// software real time clock vars
unsigned long currentmicros = 0;
unsigned long nextmicros = 0;
unsigned long interval = 1000278UL; // about 1000000 uS (increase constant to slow clock)
int seconds = 0;    // at reset, set clock to 0:00:00
int minutes = 0;
int hours = 0;    // at reset, set clock to 0:00:00
uint16_t gCharge_Timer = 0; // default to zero for delay timer
int ModeTimer = 0;    // use for off timer = 10 mins get set later
//byte T_MODECHECK = 10;    // update historical vars every minute for a 10 min running average
byte T_MODECHECK = 10;    // update historical vars every minute for a 10 min running average
const int T_HISTORYCHECK = 1;    // update historical vars every 1 secs
int HistoryTimer = T_HISTORYCHECK;
//  int Temp1 = 0;


/***********************************************************
     Set one second (it's adjustable with WDOG_T register) watchdog enable  by redefining the startup_early_hook which by default
     disables the COP (TURN OFF WHEN DEBUGGING). Must be before void setup();
 *************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
void startup_early_hook() {
  WDOG_TOVALL = 1000;   // The next 2 lines sets the time-out value. 
  WDOG_TOVALH = 0;      // VALH = 1 and VALL = 1000 you should get a WDT of about 1<<16+1000 = 66536 ms
  WDOG_PRESC = 0;       // prescaler
  WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN); // Enable WDG
}
#ifdef __cplusplus
}
#endif

/**************************************************************************/
/*!
	@brief  initialization of the digital IO of the microcontroller.
*/
/**************************************************************************/
void init_DIO(){
  // leds
  pinMode(LED1_GREEN_PIN, OUTPUT);
  pinMode(LED1_RED_PIN, OUTPUT);
  pinMode(LED2_GREEN_PIN, OUTPUT);
  pinMode(LED2_RED_PIN, OUTPUT);

  // relays
  pinMode(RELAYDR1_CHARGE_PIN, OUTPUT);
  pinMode(RELAYDR2_DRIVE_PIN, OUTPUT);

  // User inputs
  pinMode(INPUT_CHARGE_PIN, INPUT);
  pinMode(INPUT_KEYSWITCH_PIN, INPUT);
  pinMode(SW1_LEARN_BLKS_PIN, INPUT);

  // other
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
}

/**************************************************************************/
/*!
	@brief  initialization of the analog IO of the microcontroller.
*/
/**************************************************************************/
void init_AIO(){
  // inputs
  pinMode(SUPERVISOR_TEMP_PIN, INPUT);
  // outputs
  pinMode(PWM1_DAC1, OUTPUT);
  pinMode(PWM2_DAC2, OUTPUT);

  // configure the ADC - Teensy PWM runs at 23kHz, DAC value is 0 to 1023
  analogWriteFrequency(PWM1_DAC1, pwm_freq); 
  analogWriteResolution(DAC_RESOLUTION);

  //analogReference(INTERNAL);  // set analog reference to internal ref (was Jun 2016)
  analogReference(EXTERNAL);  // set analog reference to ext ref
  analogReadRes(ADC_RESOLUTION);          // Teensy 3.0: set ADC resolution to this many bits

}


// **************************************************************************** APPLICATION
void setup() {
  // Setup Serial Comms
  Serial.begin(115200);
  // delay(2000);

  // Setup the Digital IO
  init_DIO();
  // Setup the Analog IO
  init_AIO();

  // nRF24 2.4Mhz packet comms
  if (!manager.init())
  {
    Serial.println("Comms init failed");
    manager.init();     // and try again if not the first time
  }
  else   Serial.println("Comms init success");

 // Load Lithium Cell temperature specifications
  Overtemp_Cells_Charging = OVERTEMP_CELLS_CHARGING ;
  Overtemp_Cells_Discharging = OVERTEMP_CELLS_DISCHARGING;
  Undertemp_Cells_Charging = UNDERTEMP_CELLS_CHARGING;
  Undertemp_Cells_Discharging = UNDERTEMP_CELLS_DISCHARGING;

 
 

  analogWrite(CHARGER_CONTROL, 0);     // prog CHG current to 0

  Serial.println("1: Green LED1 ");
    WatchdogReset();  // reset the watchdog timer

  digitalWrite(LED1_GREEN_PIN, HIGH);   delay(500);  // LED on for 1 second
  Serial.println("1: then go RED ");
  digitalWrite(LED1_GREEN_PIN, LOW);
   WatchdogReset();  // reset the watchdog timer
 digitalWrite(LED1_RED_PIN, HIGH);   delay(500);  // LED on for 1 second
  Serial.println("2: Green LED2");
  WatchdogReset();  // reset the watchdog timer
  digitalWrite(LED2_GREEN_PIN, HIGH);   delay(500);  // LED on for 1 second
  Serial.println("3: Now go RED ");
  digitalWrite(LED2_GREEN_PIN, LOW);
  WatchdogReset();  // reset the watchdog timer
  digitalWrite(LED2_RED_PIN, HIGH);   delay(500);  // LED on for 1 second
  // wait for slow human to get serial capture running
  digitalWrite(LED1_GREEN_PIN, LOW); //leds all off
  digitalWrite(LED1_RED_PIN, LOW);
  digitalWrite(LED2_GREEN_PIN, LOW);
  digitalWrite(LED2_RED_PIN, LOW);

  // save all vars until blocks are awake and comms are established
  Highest_Vcell = Vcell_Nominal_Spec;        // load  vars with nominal values
  Lowest_Vcell = Vcell_Nominal_Spec;
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
  for (EE_address = 0; EE_address < NUMBER_OF_BLOCKS; EE_address++)   // change from 10 to 20 blocks for Kents bug
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

void WatchdogReset (void) {       // reset COP watchdog timer to 1.1 sec 
  noInterrupts(); 
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
  delay(1); // the smallest delay needed between each refresh is 1ms. anything faster and it will also reboot.
}

void GetBlockData(void)     // read RF24 radio FIFO's and get block data
{
//  uint8_t PAYLOAD = 16;     // bytes, use for comparing payload, comms will ignore data if wrong PL
//  float floatmatch = 0.000600; // +/- this amount to allow for matching float math on receive
//  float tempa;    //  float tempb;   //  float tempc;
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (manager.recvfromAck(buf, &len, &from))   // Wait here for a message addressed to us from the client
    {
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
      Goodcomms = YES;
    }
}

// =========================================================================================================== MAIN
void loop() {

  if (VerbosePrintSUPERDATA){
    Serial.print("Firmware Ver: ");  Serial.print(VERSION);
    Serial.print("  Server Address: ");  Serial.print(SERVER_ADDRESS);
    Serial.print("  PS Mode: ");  Serial.println(dkMode);
  }
  
  WatchdogReset();  // reset the watchdog timer (times out in 1 sec so make sure loop is under about 500-600msec)

 

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
        LEARNBLOCKS = ON;    // turn on first minute, allow to be turned off after that for time or full EE
      }
      if (LearnBlockSwitch == ON){
        blockNum[0] = blockNum[1] = blockNum[2] = blockNum[3] = blockNum[4] = blockNum[5] = blockNum[6] = blockNum[7] = blockNum[8] = blockNum[9] = 0;
        if (NUMofDKBLOCKS > 10){
          blockNum[10] = blockNum[11] = blockNum[12] = blockNum[13] = blockNum[14] = blockNum[15] = blockNum[16] = blockNum[17] = blockNum[18] = blockNum[19] = 0;
        }
      
      //is 4/18/2019
        for (tempx = 0; tempx < NUMofDKBLOCKS; tempx++){
        Serial.print(" ZERO Block Addr: "); Serial.print(tempx+1); Serial.print(" with the value: ");  Serial.println(blockNum[tempx]);
  //        blockNum[tempx] = 0;  // first time through, zero all block array locations, so they can be populated by the learn blocks feature
        }
        LearnBlockSwitch = OFF ;
        
      }

    }
    else if (seconds > 55)  tempx = 0;     // if switch is not pressed, 0 out var after 5 seconds

  }
  else
  {
    LEARNBLOCKS = OFF;
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
      while ( tempbl < NUMBER_OF_BLOCKS )
         {
           Serial.print("Block: "); Serial.print(blockNum[tempbl]); Serial.print(" = "); Serial.println(Block_Comm_Timer[tempbl]);
           //tempbl++;
           // run comm disconnect timer on all blocks here - count to "TWO_MINUTES" and stop
           if (Comm_Flag) 
           {
              if (Block_Comm_Timer[tempbl] < COMM_TIMEOUT) Block_Comm_Timer[tempbl] ++;     // run timer here, gets zero'd in comm routine
              if (Block_Comm_Timer[tempbl] >= TWO_MINUTES) {
                Disconnected_Block = YES;                       //  yes at least one block is disconnected
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
  bool Tempdisc = NO;
  tempbl = 0  ;
  // Now check all blocks for comms in "TWO_MINUTES", when timers are reset, clear Disconnect switches
    while ( tempbl < NUMBER_OF_BLOCKS )
     {
          if (Block_Comm_Timer[tempbl] > TWO_MINUTES) {
            Tempdisc = YES;                       //  yes at least one block is disconnected
              //if (VerbosePrintSUPERDATA){ 
              Serial.print("Block DISCONNECT.... ");Serial.println("Block DISCONNECT");}
          //}
      tempbl++;
    }
    
    if (Tempdisc == NO) {
       Disconnected_Block = NO;                       //  no block is disconnected
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
  if ((LEARNBLOCKS == ON) && (seconds == 0))   // Write EE once/sec when we are in learn mode
  {
    for (EE_address = 0; EE_address < 10; EE_address++)
    {
      EEPROM.write(EE_address, blockNum[EE_address]);
      EE_value = EEPROM.read(EE_address);
      Serial.print("Once/min save Block#: "); Serial.print(EE_value); Serial.print("to this EEprom address:  "); Serial.println(EE_address);
      Serial.print("Once/min save Block#: "); Serial.print(EE_value); Serial.print("to this EEprom address:  "); Serial.println(EE_address);

    }
    if (blockNum[0]) LEARNBLOCKS = OFF;   // if last block is saved to EE, turn off Learn blocks
  }

  if (LEARNBLOCKS == ON)     // turn on RED leds
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

  if (Vnominal != VPACK_40S)    // right now this is only 20S pack
  {
     // Volts = datAvg * 0.0575;    // low voltage calibrate
     // Volts = datAvg * 0.0565;    // 84.1 low voltage calibrate
    //  Volts = datAvg * 0.056679;    // 84.26 low voltage calibrate
      Volts = datAvg * 0.056680;    //  calibrate at 85V for HVD accuracy
  }
  else if (Vnominal == VPACK_40S)
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

 const float Vpack_HVD = ((Vcell_HVD_Spec + 0.02) * No_Of_Cells) ;       // eg 4.21+.02=4.23 x 20 = 84.6 (.02 is headroom to balance (remember high cellV will open relay too)
  int Vpack_LVD = Vcell_LVD_Spec * No_Of_Cells;         // eg 2.9 x 20 = 58
  int Vpack_HV_Run_Limit = Vpack_HVD * 1.2;  // Make high voltage run limit 10% higher than HVD (be careful at high voltage like Kent's bug)
  int Vpack_Lo_Run_Limit = Vpack_LVD;

  relay_charge_state = OFF;      // default to relay off
  //  pack check for charger relay and cell check for charger relay
  //      if ((Vpack > Vpack_HVD) || (Hist_Highest_Vcell > Vcell_HVD_Spec)) relay_charge_state = OFF;                  // if Vbat is too high, turn off charger
  //      else if ((Hist_Highest_Vcell < (Vcell_HVD_Spec - HYSTERESIS)) && (Vpack < (Vpack_HVD - HYSTERESIS))) relay_charge_state = ON;  // else now is ok so turn back on
   
      if ((Hist_Highest_Vcell < (Vcell_HVD_Spec )) && (Vpack < (Vpack_HVD)))    //4.21 if LG
      {
        if ((digitalRead(INPUT_CHARGE_PIN) == 0))
        {
          Serial.print(" Charge input IS ON ");
          Serial.print(" Charge Timer =  ");  Serial.print(gCharge_Timer); Serial.println(" hrs");
    
          if (gCharge_Timer == 0) relay_charge_state = ON;      // if Charge input is activated turn on Charge relay
          // open charge relays for one day if all Cells are in Vbalance
          if (Hist_Lowest_Vcell > Vcell_Balance)                                // 4.11 if LG
          {
            // shut down relay, start 1 day timer
            gCharge_Timer = 24;   //24 hours
            relay_charge_state = OFF;
            if (Print_Flag){
            Serial.println(" Charge relay now turns OFF for 24 hours = cells are ALL in balance ");
            }
          }
          else //reset charge timer if Vcell < 4V
          {
            if (Hist_Lowest_Vcell < CELL_RECONNECT_V) {
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
        if (Hist_Highest_Vcell >= Vcell_HVD_Spec ) {
          Serial.print(Hist_Highest_Vcell); Serial.print("V is > or = "); Serial.print(Vcell_HVD_Spec);
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
   
  relay_drive_state = OFF;   // default to relay off
  // Pack check for motor relay and cell check for motor relay
  if ((Vpack > Vpack_HV_Run_Limit) || (Vpack < Vpack_Lo_Run_Limit)){
    if (Print_Flag) Serial.println(" Drive relay IS OFF because Vpack is too LOW or too HIGH");
  }
  else{
    if((Vpack < (Vpack_HV_Run_Limit - HYSTERESIS)) && (Hist_Lowest_Vcell > Vcell_LVD_Spec)){ // check pack and cell specs
      if ((digitalRead(INPUT_CHARGE_PIN) != 0) && (digitalRead(INPUT_KEYSWITCH_PIN) == 0)){
        relay_drive_state = ON;   // if pack ok, and charger is off (or disconnected) turn on motor control
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

  if (Hist_Highest_Tcell < Overtemp_Cells_Discharging) {  // lower counts are hotter
      relay_drive_state = OFF;               
      Serial.println(" ***** FAULT - Drive relay is OFF because cell OVERTEMP ");
      Serial.print(" ***** FAULT - Overtemp Cell temp counts = "); Serial.println(Hist_Highest_Tcell);
      Serial.print(" *** High temp limit = "); Serial.println(Overtemp_Cells_Discharging);
  }
  
  if (Hist_Highest_Tcell < Overtemp_Cells_Charging) {     // lower is hotter with NTC thermistor
      relay_charge_state = OFF;
      Serial.println(" ***** FAULT - Charge relay is OFF because cell OVERTEMP ");
      Serial.print(" ***** FAULT - Overtemp Cell temp counts = "); Serial.println(Hist_Highest_Tcell);
      Serial.print(" *** High temp limit = "); Serial.println(Overtemp_Cells_Charging);
      } 
    
  // undertemp second
  if (Hist_Lowest_Tcell > Undertemp_Cells_Discharging) {
     relay_drive_state = OFF;               
     Serial.println(" ***** FAULT - Drive relay is OFF because cell UNDERTEMP ");
     Serial.print(" ***** FAULT - Undertemp Cell temp counts = "); Serial.print(Hist_Lowest_Tcell);
     Serial.print(" Low temp limit = "); Serial.println(Undertemp_Cells_Discharging);
    }
    
   if (Hist_Lowest_Tcell > Undertemp_Cells_Charging) {  // higher is colder with NTC
     relay_charge_state = OFF;
     Serial.println(" ***** FAULT - Charge relay is OFF because cell UNDERTEMP ");
     Serial.print(" ***** FAULT - Cell temp counts = "); Serial.print(Hist_Lowest_Tcell);
     Serial.print(" Low temp limit = "); Serial.println(Undertemp_Cells_Charging);
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
  if ((Disconnected_Block == YES) && (dkError == ERROR_UNDERVOLT_CELL)) {
     if (dkError == ERROR_UNDERVOLT_CELL) relay_drive_state = OFF; 
       if (VerbosePrintSUPERDATA) Serial.println(" Drive relay is OFF because cell UNDERVOLT/Cell Disconnect ");
     //if (dkError == OVERVOLT_CELL) relay_charge_state = OFF; 
  }


  //   electrically switch relays on/off

  //if ((relay_charge_state == ON) && (digitalRead(INPUT_CHARGE_PIN == LOW)))    // Charger input goes low when charger is conn
  if ((relay_charge_state == ON) )    // Charger input goes low when charger is conn
  {
    digitalWrite(CHARGER_RELAY, HIGH);
      if (VerbosePrintSUPERDATA) Serial.println("Charger relay K1 is closed");
    dkMode = DK_CHARGE;
  }
  else {
    dkMode = DK_PAUSE;
    digitalWrite(CHARGER_RELAY, LOW);
      if (VerbosePrintSUPERDATA) Serial.println("Charger relay K1 is open");
  }



  if ((relay_drive_state == ON))   // keyswitch input goes lo when turned on
  {
    digitalWrite(MTRCONTROL_RELAY, HIGH);
      if (VerbosePrintSUPERDATA) Serial.println(" Motor Relay K2 is closed");
    dkMode = DK_DRIVE;
  }
  else {
    digitalWrite(MTRCONTROL_RELAY, LOW);
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
  if ((Hist_Highest_Vcell < Vcell_Balance) && (Vpack < (Vcell_Balance * No_Of_Cells))) TargetI = FullChargeCurrent; // Full charge rate to 90% or 4.000V
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

  
    
  // output SOC signal from op amp PWM - digital DAC outputs
  // 60-80V Vpack ==> 0-100% for now, VOLTAGE IS NOT A GOOD FUEL GAUAGE, but ...
  //  120-160 Vpack ==> 0-100% for now...OK till we have something better.
  float TempV = Vnominal;    // temp pack Volts for SOC computations
  int SOCv;     // SOC based on voltage
  const int MAX_PWM = 1023;
  //const int MIN_PWM = 1;


  // Average SOCv over minutes (improvement) - Jul 12, 2017
  if ((relay_drive_state == OFF) && (relay_charge_state == OFF)) gSOCv = Vpack/2 ;           // both motor and charger relays are off == init SOC == Vpack

 
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
 
    if ( ACKSyncpacket.BlockNumber > NUMofDKBLOCKS)  
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
  
  Goodcomms = NO;    // default is there is no new data
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

  

  //bool GoodComms = NO;
  uint8_t PAYLOAD = 16;     // bytes, use for comparing payload, comms will ignore data if wrong PL
  float floatmatch = 0.000600; // +/- this amount to allow for matching float math on receive
  uint8_t len = sizeof(buf);
 // uint8_t from;
  float tempa;    // 4 bytes
  float tempb;
  float tempc;



 if (Goodcomms == NO) goto badcomm;    // get out right now don't do checksum or payload checks




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

       // GoodComms = YES;
       //  Serial.println(" Use and save this good data..."); 

       // new 7/10/2019
       gValidPacket = gBlockNumber;
       gPacketTimer = 0;
       // new 7/10/2019
 

badcomm:   

      // learn blocks to connect to here...if in LEARN BLOCKS mode
       //  Save this block if not already saved
      bool This_Block_Saved = NO;
      if ((LEARNBLOCKS == ON))
      {
        uint16_t x = NUMBER_OF_BLOCKS;   // number of blocks (+ 1 for zero)
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
           This_Block_Saved = YES;
           // zero comm timer for this block
           Block_Comm_Timer[x] = 0;
        }
          x--;
        }
        x = NUMBER_OF_BLOCKS;
        while ( x > 0 )
        {
          if (This_Block_Saved != YES)      // if not saved, save BLOCK to zero'd location
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
        int x = NUMBER_OF_BLOCKS;
        while ( x > 0 )
        {
          if (from == blockNum[x - 1])    // check if BLOCK already been saved?
          {
            if (VerbosePrintSUPERDATA){
            Serial.print("This block is ALREADY SAVED in EE location: "); Serial.println(x - 1);
            }
            This_Block_Saved = YES;
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







 

// Now write to KOSO meter

const uint16_t NO_BARS = 100;         // 1% PWM = (5 / 3.3) * 0.01 * 1024 = 15  ( 0 bars > 4.32)
const uint16_t ONE_BARS = 153;        // 14% PWM = (5 / 3.3) * 0.14 * 1024 = 217 (1 bars < 4.07V)
const uint16_t TWO_BARS = 225;        // 18% PWM = (5 / 3.3) * 0.18 * 1024 = 279 (2 bars < 3.7V)
const uint16_t THREE_BARS = 400;        // 22% PWM = (5 / 3.3) * 0.22 * 1024 = 341 (3 bars < 3.2V)
const uint16_t FOUR_BARS = 707 ;        // 26% PWM = (5 / 3.3) * 0.26 * 1024 = 404 (4 bars < 2.3V)

  // setup DAC output for KOSO Speedo Fuel gauge  (5/3.3 = compensates for PWM testing done with Funct gen)
  if (SOCv >= 20) analogWrite(DAC_OUT_FUEL_GAUGE, ONE_BARS);  // Need equiv of 14% PWM for 1 bars
  else    analogWrite(DAC_OUT_FUEL_GAUGE, (NO_BARS)); // No bars give *some* signal
  if (SOCv >= 40) analogWrite(DAC_OUT_FUEL_GAUGE, TWO_BARS);  // Need equiv of 18% PWM for 2 bars
  if (SOCv >= 60) analogWrite(DAC_OUT_FUEL_GAUGE, THREE_BARS);  // Need equiv of 22% PWM for 3 bars
  if (SOCv >= 80) analogWrite(DAC_OUT_FUEL_GAUGE, FOUR_BARS);  // Need equiv of 26% PWM for all 4 bars

  // Block awareness: check to see which blocks are talking, over a 100 sec window


WatchdogReset();  // wd resets in 1 sec

//debug
 // delay(30);  // ALLOW about 50 msecs between block transmits/receptions (main takes ~20ms)
  delay(20);  // ALLOW about 50 msecs between block transmits/receptions (main takes ~20ms)

}

