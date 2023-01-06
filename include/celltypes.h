//---------------------------------------------------------------------------
// Header to handle cell types, and cleanup main
// Date       	Who		WAS/IS changes
//---------------------------------------------------------------------------
// 10/05/22		NHJ 	Initial creation

#include <Arduino.h>
#include <NCP18.h>

#ifndef CELLTYPES_H
#define CELLTYPES_H

//-LITHIUM CELL SPECIFICATIONS-------------------------------------------------------------------
#ifdef LG_MH1                                       // cell parameters for LG MH1 3200mah
    #define Vcell_HVD_Spec                  4.21f   // high voltage disconnect, 4.25 max leave room for 0.1% acc.
    #define Vcell_Nominal_Spec              3.67f
    #define Vcell_Low_Spec                  2.91f   // end voltage cutoff 2.5V is spec
    #define Vcell_LVD_Spec                  2.91f   // low voltage disconnect, all off
    #define Vcell_Balance                   4.11f   // voltage above where Cells will balance
    #define Vcell_Charge_Low_Cutoff         2.00f
    #define Vcell_Charge_High_Cutoff        4.20f
    #define Vcell_Charge_Trickle            2.80f   // voltage under where trickle charge should happen  
    #define Vcell_Charge_Bulk               4.05f   // voltage under where the bulk of the charge should happen
    #define Vcell_Charge_Off                4.15f   // voltage above where charger should turn off
    #define Tcell_Charge_Low_Max_Cutoff     NTC_0C  // Temp where relay will turn off
    #define Tcell_Charge_High_Max_Cutoff    NTC_45C // Temp where relay will turn off
    #define Tcell_Charge_Low_Cutoff         NTC_3C  // Temp where charger will not attempt to charge
    #define Tcell_Charge_High_Cutoff        NTC_43C // Temp where charger will not attempt to charge
    #define Tcell_Charge_Taper_1            NTC_35C // Temp where charger will reduce current
    #define Tcell_Charge_Taper_2            NTC_40C // Temp where charger will reduce current more
#endif

#ifdef LG_MJ1                                       // cell parameters for LG MH1 3500mah
    #define Vcell_HVD_Spec                  4.21f   // high voltage disconnect, charger disconnected
    #define Vcell_Nominal_Spec              3.635f  // nominal voltage
    #define Vcell_Low_Spec                  2.91f   // end voltage cutoff 2.5V is spec
    #define Vcell_LVD_Spec                  2.91f   // low voltage disconnect, all off
    #define Vcell_Balance                   4.11f   // balancing voltage
    #define Vcell_Charge_Low_Cutoff         2.00f   // lower cutoff voltage
    #define Vcell_Charge_High_Cutoff        4.20f   // upper cutoff voltage
    #define Vcell_Charge_Trickle            2.80f   // voltage under where trickle charge should happen
    #define Vcell_Charge_Bulk               4.05f   // voltage under where the bulk of the charge should happen
    #define Vcell_Charge_Off                4.15f   // voltage above where charger should turn off
    #define Tcell_Charge_Low_Max_Cutoff     NTC_0C  // Temp where relay will turn off
    #define Tcell_Charge_High_Max_Cutoff    NTC_45C // Temp where relay will turn off
    #define Tcell_Charge_Low_Cutoff         NTC_3C  // Temp where charger will not attempt to charge
    #define Tcell_Charge_High_Cutoff        NTC_43C // Temp where charger will not attempt to charge
    #define Tcell_Charge_Taper_1            NTC_35C // Temp where charger will reduce current
    #define Tcell_Charge_Taper_2            NTC_40C // Temp where charger will reduce current more
#endif

  #ifdef SANYO_NCR18650B                            // cell parameters for Panasonic/Sanyo NCR18650B
    #define Vcell_HVD_Spec                  4.30f
    #define Vcell_Nominal_Spec              3.70f
    #define Vcell_Low_Spec                  2.90f
    #define Vcell_LVD_Spec                  2.90f
    #define Vcell_Balance                   3.90f
    #define Vcell_Charge_Low_Cutoff         2.00f
    #define Vcell_Charge_High_Cutoff        4.20f
    #define Vcell_Charge_Trickle            2.80f   // voltage under where trickle charge should happen  
    #define Vcell_Charge_Bulk               4.05f   // voltage under where the bulk of the charge should happen
    #define Vcell_Charge_Off                4.15f   // voltage above where charger should turn off
    #define Tcell_Charge_Low_Max_Cutoff     NTC_0C; // Temp where relay will turn off
    #define Tcell_Charge_High_Max_Cutoff    NTC_45C // Temp where relay will turn off
    #define Tcell_Charge_Low_Cutoff         NTC_3C  // Temp where charger will not attempt to charge
    #define Tcell_Charge_High_Cutoff        NTC_43C // Temp where charger will not attempt to charge
    #define Tcell_Charge_Taper_1            NTC_35C // Temp where charger will reduce current
    #define Tcell_Charge_Taper_2            NTC_40C // Temp where charger will reduce current more
#endif

#endif