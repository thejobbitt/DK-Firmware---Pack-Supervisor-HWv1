//---------------------------------------------------------------------------
// Header to handle cell types, and cleanup main
// Date       	Who		WAS/IS changes
//---------------------------------------------------------------------------
// 10/05/22		NHJ 	Initial creation
// 01/06/22     NHJ     Formatting and varibale naming

#include <Arduino.h>
#include <NCP18.h>

#ifndef CELLTYPES_H
#define CELLTYPES_H

extern struct celltypes cell;

//-LITHIUM CELL SPECIFICATIONS-------------------------------------------------------------------
#ifdef LG_MH1                                       // cell parameters for LG MH1 3200mah
    struct celltypes
    {
        // voltage specifications
        const float volt_Nominal                    = 3.67f;   // nominal voltage
        const float volt_Balance                    = 4.11f;   // balancing voltage
        const float volt_HVD                        = 4.21f;   // high voltage disconnect
        const float volt_LVD                        = 2.91f;   // low voltage disconnect
        // charge voltages
        const float volt_high_chargeCut             = 4.20f;   // charger upper cutoff voltage
        const float volt_low_chargeCut              = 2.00f;   // charger lower cutoff voltage
        const float volt_chargeTrickle              = 2.80f;   // voltage under where trickle charge should happen
        const float volt_chargeBulk                 = 4.05f;   // voltage under where the bulk of the charge should happen
        const float volt_chargeOff                  = 4.15f;   // voltage above where charger should turn off
        // discharge temps
        const u_int16_t temp_over_driveRelayCut     = NTC_60C;      // Temp where relay will cut off power
        const u_int16_t temp_under_driveRelayCut    = NTC_NEG_20C;  // Temp where relay will cut off power
        // charge temps
        const u_int16_t temp_over_chargeSoftCut     = NTC_43C; // Software request charger to stop or reduce (if supported)
        const u_int16_t temp_under_chargeSoftCut    = NTC_3C;  // Software request charger to stop or reduce (if supported)
        const u_int16_t temp_over_chargeRelayCut    = NTC_45C; // Temp where relay will turn off
        const u_int16_t temp_under_chargeRelayCut   = NTC_0C;  // Temp where relay will turn off
        const u_int16_t temp_chargeTaper_1          = NTC_35C; // Temp where charger will reduce current
        const u_int16_t temp_chargeTaper_2          = NTC_40C; // Temp where charger will reduce current more
    };
#endif

#ifdef LG_MJ1                                       // cell parameters for LG MH1 3500mah
    struct celltypes
    {
        // voltage specifications
        const float volt_Nominal                    = 3.64f;   // nominal voltage
        const float volt_Balance                    = 4.11f;   // balancing voltage
        const float volt_HVD                        = 4.21f;   // high voltage disconnect
        const float volt_LVD                        = 2.91f;   // low voltage disconnect
        // charge voltages
        const float volt_high_chargeCut             = 4.20f;   // charger upper cutoff voltage
        const float volt_low_chargeCut              = 2.00f;   // charger lower cutoff voltage
        const float volt_chargeTrickle              = 2.80f;   // voltage under where trickle charge should happen
        const float volt_chargeBulk                 = 4.05f;   // voltage under where the bulk of the charge should happen
        const float volt_chargeOff                  = 4.15f;   // voltage above where charger should turn off
        // discharge temps
        const u_int16_t temp_over_driveRelayCut     = NTC_60C;      // Temp where relay will cut off power
        const u_int16_t temp_under_driveRelayCut    = NTC_NEG_20C;  // Temp where relay will cut off power
        // charge temps
        const u_int16_t temp_over_chargeSoftCut     = NTC_43C; // Software request charger to stop or reduce (if supported)
        const u_int16_t temp_under_chargeSoftCut    = NTC_3C;  // Software request charger to stop or reduce (if supported)
        const u_int16_t temp_over_chargeRelayCut    = NTC_45C; // Temp where relay will turn off
        const u_int16_t temp_under_chargeRelayCut   = NTC_0C;  // Temp where relay will turn off
        const u_int16_t temp_chargeTaper_1          = NTC_35C; // Temp where charger will reduce current
        const u_int16_t temp_chargeTaper_2          = NTC_40C; // Temp where charger will reduce current more
    };
#endif

  #ifdef SANYO_NCR18650B                            // cell parameters for Panasonic/Sanyo NCR18650B
    struct celltypes
    {
        // voltage specifications
        const float volt_Nominal                    = 3.70f;   // nominal voltage
        const float volt_Balance                    = 3.90f;   // balancing voltage
        const float volt_HVD                        = 4.30f;   // high voltage disconnect
        const float volt_LVD                        = 2.90f;   // low voltage disconnect
        // charge voltages
        const float volt_high_chargeCut             = 4.20f;   // charger upper cutoff voltage
        const float volt_low_chargeCut              = 2.00f;   // charger lower cutoff voltage
        const float volt_chargeTrickle              = 2.80f;   // voltage under where trickle charge should happen
        const float volt_chargeBulk                 = 4.05f;   // voltage under where the bulk of the charge should happen
        const float volt_chargeOff                  = 4.15f;   // voltage above where charger should turn off
        // discharge temps
        const u_int16_t temp_over_driveRelayCut     = NTC_60C;      // Temp where relay will cut off power
        const u_int16_t temp_under_driveRelayCut    = NTC_NEG_20C;  // Temp where relay will cut off power
        // charge temps
        const u_int16_t temp_over_chargeSoftCut     = NTC_43C; // Software request charger to stop or reduce (if supported)
        const u_int16_t temp_under_chargeSoftCut    = NTC_3C;  // Software request charger to stop or reduce (if supported)
        const u_int16_t temp_over_chargeRelayCut    = NTC_45C; // Temp where relay will turn off
        const u_int16_t temp_under_chargeRelayCut   = NTC_0C;  // Temp where relay will turn off
        const u_int16_t temp_chargeTaper_1          = NTC_35C; // Temp where charger will reduce current
        const u_int16_t temp_chargeTaper_2          = NTC_40C; // Temp where charger will reduce current more
    };
#endif

#endif