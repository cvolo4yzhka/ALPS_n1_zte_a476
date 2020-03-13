/*****************************************************************************
*
* Filename:
* ---------
*   bq24158.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   bq24158 header file
*
* Author:
* -------
*
****************************************************************************/

#ifndef _bq24158_SW_H_
#define _bq24158_SW_H_

#define HIGH_BATTERY_VOLTAGE_SUPPORT

#define bq24158_CON0      0x00
#define bq24158_CON1      0x01
#define bq24158_CON2      0x02

/**********************************************************
  *
  *   [MASK/SHIFT] 
  *
  *********************************************************/
    //CON0
#define CON0_BOOST_MASK     0x01
#define CON0_BOOST_SHIFT    3
    
    //CON1
#define CON1_OPA_MODE_MASK  0x01
#define CON1_OPA_MODE_SHIFT 0
    
    //CON2
#define CON2_OTG_EN_MASK    0x01
#define CON2_OTG_EN_SHIFT   0

/**********************************************************
  *
  *   [Extern Function] 
  *
  *********************************************************/
    //CON0----------------------------------------------------
extern kal_uint32 bq24158_get_boost_status(void);
//CON1----------------------------------------------------
extern void bq24158_set_opa_mode(kal_uint32 val);
//CON2----------------------------------------------------
extern void bq24158_set_otg_en(kal_uint32 val);

#endif // _bq24158_SW_H_

