/* 
 * File:   main.h
 * Author: Allan
 *
 * Created on September 7, 2019, 10:56 PM
 */

#ifndef MAIN_H
#define	MAIN_H

/****************************************************************************/
/************************ Project Settings **********************************/

typedef enum
{
    PORT_1_GRN  = 0,
    PORT_1_RED,
    PORT_2_GRN,
    PORT_2_RED,
    PORT_3_GRN,
    PORT_3_RED,
    PORT_4_GRN,
    PORT_5_RED,
    ALL_LEDS
} E_LED;

typedef enum
{
    BOTTOM_ROW  = 0,
    TOP_ROW,
    NUM_ROWS
} E_ROW;

typedef enum
{
    OPTION_1 = 0,
    OPTION_2,            
    OPTION_3,
    OPTION_4,
    OPTION_MAX
} E_PARITY;

typedef struct
{
    BOOL        b_Unlocked;
    E_PARITY    e_Parity;
} ST_ROOM_STATUS;

#define LED_ON                              (TRUE)
#define LED_OFF                             (FALSE)
#define BottomRowCallBack           EX_INT0_CallBack            
#define TopRowCallBack              EX_INT1_CallBack            

extern void BottomRowCallBack       (void);
extern void TopRowCallBack          (void);
extern void SetLedState             (E_LED e_LedNumber, BOOL b_LedOn, E_ROW e_Row);
extern void main_SendUnlockRequest  (UINT8 u8_Room, UINT8* au8_CardNumber, UINT8 u8_Length);

#endif	/* MAIN_H */

