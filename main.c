/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.125
        Device            :  PIC32MM0064GPM028
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.36B
        MPLAB 	          :  MPLAB X v5.20
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include <stdio.h>
#include <string.h>
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/watchdog.h"
#include "mcc_generated_files/i2c1.h"
#include "mcc_generated_files/i2c2.h"
#include "mcc_generated_files/i2c3.h"
#include "mcc_generated_files/interrupt_manager.h"
#include "inc/CanascTypes.h"
#include "RS485/rs485.h"
#include "main.h"

/********************************* Definitions ************************************/

#define NUM_IF_PORTS                (4u)                // Number of interface ports on a row
#define NUM_IF_ROOMS                (NUM_IF_PORTS * 8u) // Number of rooms on a row of ports
#define NUM_ROOMS                   (64u)               // Total Number of rooms 
#define I2C_IF_START_ADDR           0x20u               // I2C Address of first Interface board
#define I2C_LED_CTRL_ADDR           0x27u               // I2C Address of LED control port expander (8 bit))
#define I2C_IODIR_REG               0x00u               // I/O Direction register 
#define I2C_IOPOL_REG               0x01u               // Input polarity register 
#define I2C_INTEN_REG               0x02u               // Interrupt enable register 
#define I2C_IOCON_REG               0x05u               // I/O Config register
#define I2C_GPPU_REG                0x06u               // I/O Pull Up config register
#define I2C_GPIO_REG                0x09u               // I/O Value R/W register
#define SLAVE_I2C_DEVICE_TIMEOUT    50u                 // define slave timeout 
#define KEYPAD_HASH                 0xB0u               // 4 bit value to represent the '#' button on the keypad    

/******************************  Local Variables **********************************/
static BOOL     b_BottomRowEvent = FALSE;
static BOOL     b_TopRowEvent = FALSE;

static BOOL     u8_TopRowPresent[NUM_IF_PORTS]          = {FALSE};
static BOOL     u8_BottomRowPresent[NUM_IF_PORTS]       = {FALSE};
static UINT8    u8_PortData[NUM_ROWS][NUM_IF_PORTS]     = {0};
static UINT8    u8_PortDataPrev[NUM_ROWS][NUM_IF_PORTS] = {0};

static ST_ROOM_STATUS ast_RoomStatus[NUM_ROOMS]         = {0};

/******************************  Local function prototypes ************************/
static BOOL     SendI2c1Cmd     (UINT8 u8_Addr, UINT8* au8_TxBuff, UINT8 u8_NumBytes);
static BOOL     SendI2c2Cmd     (UINT8 u8_Addr, UINT8* au8_TxBuff, UINT8 u8_NumBytes);
static void     I2c1Read        (UINT8 u8_Addr, UINT8* au8_RxBuff, UINT8 u8_NumBytes);
static void     I2c2Read        (UINT8 u8_Addr, UINT8* au8_RxBuff, UINT8 u8_NumBytes);
static void     InitLeds        (void);
static void     InitTopRow      (void);
static void     InitBottomRow   (void);
static void     ReadTopRow      (void);
static void     ReadBottomRow   (void);
static void     CheckPortData   (void);
static void     SendKeyOption   (E_PARITY e_Parity, UINT8* u8_RoomBuff, UINT8* u8_Buffer);
static void     WeigandTx       (UINT8* u8_Data, UINT8 u8_NumBits);

/************************************************************************************/
/* Sets a flag to indicate that External Interrupt 0 was detected (Bottom Row)      */
/************************************************************************************/
void BottomRowCallBack (void)
{
    b_BottomRowEvent = TRUE;
}

/************************************************************************************/
/* Sets a flag to indicate that External Interrupt 1 was detected (Top Row)         */
/************************************************************************************/
void TopRowCallBack (void)
{
    b_TopRowEvent = TRUE;
}

/************************************************************************************/
/* Send I2C command to port expander on I2C1                                        */
/* Returns TRUE on success                                                          */
/************************************************************************************/
static BOOL SendI2c1Cmd (UINT8 u8_Addr, UINT8* au8_TxBuff, UINT8 u8_NumBytes)
{
    I2C1_MESSAGE_STATUS e_status    = I2C1_MESSAGE_PENDING;
    UINT16              u16_TimeOut = 0u;


    // Set register pointer to read the input ports
    I2C1_MasterWrite(au8_TxBuff, u8_NumBytes, u8_Addr, &e_status);

    // wait for the message to be sent or status has changed.
    while(e_status == I2C1_MESSAGE_PENDING)
    {
        DELAY_microseconds(100);

        // check for max retry
        if (u16_TimeOut == SLAVE_I2C_DEVICE_TIMEOUT)
        {
            break;
        }

        u16_TimeOut++;
        if ((u16_TimeOut   == SLAVE_I2C_DEVICE_TIMEOUT) || 
            (e_status      == I2C1_MESSAGE_COMPLETE))
        {
            break;
        }
    }

    return(e_status == I2C2_MESSAGE_COMPLETE);    
}


/************************************************************************************/
/* Send I2C command to port expander on I2C2                                        */
/* Returns TRUE on success                                                          */
/************************************************************************************/
static BOOL SendI2c2Cmd (UINT8 u8_Addr, UINT8* au8_TxBuff, UINT8 u8_NumBytes)
{
    I2C2_MESSAGE_STATUS e_status    = I2C2_MESSAGE_PENDING;
    UINT16              u16_TimeOut = 0u;


    // Set register pointer to read the input ports
    I2C2_MasterWrite(au8_TxBuff, u8_NumBytes, u8_Addr, &e_status);

    // wait for the message to be sent or status has changed.
    while(e_status == I2C2_MESSAGE_PENDING)
    {
        DELAY_microseconds(100);

        // check for max retry
        if (u16_TimeOut == SLAVE_I2C_DEVICE_TIMEOUT)
        {
            break;
        }

        u16_TimeOut++;
        if ((u16_TimeOut   == SLAVE_I2C_DEVICE_TIMEOUT) || 
            (e_status      == I2C2_MESSAGE_COMPLETE))
        {
            break;
        }
    }
    
    return(e_status == I2C2_MESSAGE_COMPLETE);
}

/************************************************************************************/
/* I2C read from port expander                                                      */
/************************************************************************************/
static void I2c1Read (UINT8 u8_Addr, UINT8* au8_RxBuff, UINT8 u8_NumBytes)
{
    I2C1_MESSAGE_STATUS e_status    = I2C1_MESSAGE_PENDING;
    UINT16              u16_TimeOut = 0u;

    // Read from the input ports
    I2C1_MasterRead(au8_RxBuff, u8_NumBytes, u8_Addr, &e_status);;

    // wait for the message to be sent or status has changed.
    while(e_status == I2C1_MESSAGE_PENDING)
    {
        DELAY_microseconds(100);

        // check for max retry
        if (u16_TimeOut == SLAVE_I2C_DEVICE_TIMEOUT)
        {
            break;
        }

        u16_TimeOut++;
        if ((u16_TimeOut   == SLAVE_I2C_DEVICE_TIMEOUT) || 
            (e_status      == I2C1_MESSAGE_COMPLETE))
        {
            break;
        }
    }
}

/************************************************************************************/
/* I2C read from port expander                                                      */
/************************************************************************************/
static void I2c2Read (UINT8 u8_Addr, UINT8* au8_RxBuff, UINT8 u8_NumBytes)
{
    I2C2_MESSAGE_STATUS e_status    = I2C2_MESSAGE_PENDING;
    UINT16              u16_TimeOut = 0u;

    // Read from the input ports
    I2C2_MasterRead(au8_RxBuff, u8_NumBytes, u8_Addr, &e_status);

    // wait for the message to be sent or status has changed.
    while(e_status == I2C2_MESSAGE_PENDING)
    {
        DELAY_microseconds(100);

        // check for max retry
        if (u16_TimeOut == SLAVE_I2C_DEVICE_TIMEOUT)
        {
            break;
        }

        u16_TimeOut++;
        if ((u16_TimeOut   == SLAVE_I2C_DEVICE_TIMEOUT) || 
            (e_status      == I2C2_MESSAGE_COMPLETE))
        {
            break;
        }
    }
}

/************************************************************************************/
/* Inits the Port expanders for the top row of ports                                */
/************************************************************************************/
static void InitTopRow (void)
{
    UINT8 u8_Cntr;    
    UINT8 u8_PortIoDirValue[2]      = {I2C_IODIR_REG, 0xFF};
    UINT8 u8_EnablePullUp[2]        = {I2C_GPPU_REG, 0xFF};
    UINT8 u8_InputPolarity[2]       = {I2C_IOPOL_REG, 0xFF};
    UINT8 u8_InterruptEnable[2]     = {I2C_INTEN_REG, 0xFF};
    UINT8 u8_PortConfigValue[2]     = {I2C_IOCON_REG, 0x04};
    UINT8 u8_ReadPortValue          = I2C_GPIO_REG;

    for (u8_Cntr=0; u8_Cntr < NUM_IF_PORTS; u8_Cntr++)
    {
        // Set direction for input port
        if (SendI2c2Cmd(I2C_IF_START_ADDR + u8_Cntr, u8_PortIoDirValue, 2u))
        {
            // Turn on green LED
            u8_TopRowPresent[u8_Cntr] = TRUE;
            SetLedState(u8_Cntr * 2, LED_ON, TOP_ROW);

            // Enable weak pull ups on the input port
            SendI2c2Cmd(I2C_IF_START_ADDR + u8_Cntr, u8_EnablePullUp, 2u);

            // Invert polarity on the input port (high reads as '0')
            SendI2c2Cmd(I2C_IF_START_ADDR + u8_Cntr, u8_InputPolarity, 2u);

            // Interrupt enable on change
            SendI2c2Cmd(I2C_IF_START_ADDR + u8_Cntr, u8_InterruptEnable, 2u);

            // Configure interrupt pin as Open Drain so it can share an INT line
            SendI2c2Cmd(I2C_IF_START_ADDR + u8_Cntr, u8_PortConfigValue, 1u);
        }
        else
        {
            /* Mark port as missing and turn off both LEDs for that port */
            u8_TopRowPresent[u8_Cntr] = FALSE;
            SetLedState(u8_Cntr * 2, LED_OFF, TOP_ROW);
            SetLedState((u8_Cntr * 2) +1, LED_OFF, TOP_ROW);
        }
    }
    
    printf("[%d] [%d] [%d] [%d] -",u8_TopRowPresent[0], u8_TopRowPresent[1], u8_TopRowPresent[2], u8_TopRowPresent[3]);    
}  

/************************************************************************************/
/* Reads the value from the top row of ports                                        */
/************************************************************************************/
static void ReadTopRow (void)
{
    UINT8 u8_Cntr;    
    UINT8 u8_ReadPortValue          = I2C_GPIO_REG;

    for (u8_Cntr=0; u8_Cntr < NUM_IF_PORTS; u8_Cntr++)
    {        
        /* Only attempt to read if port has been configured */
        if (u8_TopRowPresent[u8_Cntr])
        {        
            // Set register pointer to read the input ports
            if (SendI2c2Cmd(I2C_IF_START_ADDR + u8_Cntr, &u8_ReadPortValue, 1u))
            {        
                // Read port values from input ports
                I2c2Read(I2C_IF_START_ADDR + u8_Cntr, &u8_PortData[TOP_ROW][u8_Cntr], 1u);
            }
            else
            {
                /* Mark port as now missing */
                u8_TopRowPresent[u8_Cntr] = FALSE;
            }
        }
    }    
}   

/************************************************************************************/
/* Inits the Port expanders for the top row of ports                                */
/************************************************************************************/
static void InitBottomRow (void)
{
    UINT8 u8_Cntr;    
    UINT8 u8_PortIoDirValue[2]      = {I2C_IODIR_REG, 0xFF};
    UINT8 u8_EnablePullUp[2]        = {I2C_GPPU_REG, 0xFF};
    UINT8 u8_InputPolarity[2]       = {I2C_IOPOL_REG, 0xFF};
    UINT8 u8_InterruptEnable[2]     = {I2C_INTEN_REG, 0xFF};
    UINT8 u8_PortConfigValue[2]     = {I2C_IOCON_REG, 0x04};    

    for (u8_Cntr=0; u8_Cntr < NUM_IF_PORTS; u8_Cntr++)
    {
        // Set direction for input port
        if(SendI2c1Cmd(I2C_IF_START_ADDR + u8_Cntr, u8_PortIoDirValue, 2u))
        {
            // Turn on green LED
            u8_BottomRowPresent[u8_Cntr] = TRUE;
            SetLedState(u8_Cntr * 2, LED_ON, BOTTOM_ROW);
            
            // Enable weak pull ups on the input port
            SendI2c1Cmd(I2C_IF_START_ADDR + u8_Cntr, u8_EnablePullUp, 2u);

            // Invert polarity on the input port (high reads as '0')
            SendI2c1Cmd(I2C_IF_START_ADDR + u8_Cntr, u8_InputPolarity, 2u);

            // Interrupt enable on change
            SendI2c1Cmd(I2C_IF_START_ADDR + u8_Cntr, u8_InterruptEnable, 2u);

            // Configure interrupt pin as Open Drain so it can share an INT line
            SendI2c1Cmd(I2C_IF_START_ADDR + u8_Cntr, u8_PortConfigValue, 1u);     
        }
        else
        {
            /* Mark port as missing and turn off both LEDs for that port */
            u8_BottomRowPresent[u8_Cntr] = FALSE;
            SetLedState(u8_Cntr * 2, LED_OFF, BOTTOM_ROW);
            SetLedState((u8_Cntr * 2) +1, LED_OFF, BOTTOM_ROW);
        }
    }
    
    printf(" [%d] [%d] [%d] [%d]\r\n",u8_BottomRowPresent[0], u8_BottomRowPresent[1], u8_BottomRowPresent[2], u8_BottomRowPresent[3]);   
}  

/************************************************************************************/
/* Reads the value from the bottom row of ports                                        */
/************************************************************************************/
static void ReadBottomRow (void)
{
    UINT8 u8_Cntr;    
    UINT8 u8_ReadPortValue          = I2C_GPIO_REG;

    for (u8_Cntr=0; u8_Cntr < NUM_IF_PORTS; u8_Cntr++)
    {
        /* Only attempt to read if port has been configured */
        if (u8_BottomRowPresent[u8_Cntr])
        {        
            // Set register pointer to read the input ports
            if (SendI2c1Cmd(I2C_IF_START_ADDR + u8_Cntr, &u8_ReadPortValue, 1u))
            {        
                // Read port values from input ports
                I2c1Read(I2C_IF_START_ADDR + u8_Cntr, &u8_PortData[BOTTOM_ROW][u8_Cntr], 1u);
            }
            else
            {
                /* Mark port as now missing */
                u8_BottomRowPresent[u8_Cntr] = FALSE;
            }
        }
    }
}   

/************************************************************************************/
/* Check the I2C comms - if no response, reset I2C routines                         */
/************************************************************************************/
static void CheckI2cComms (void)
{
    BOOL    b_CommsOK = FALSE;
    UINT8   u8_Cntr;
        
    /* See if we have had a response from any of our port expanders */
    for (u8_Cntr=0; u8_Cntr < NUM_IF_PORTS; u8_Cntr++)
    {
       if (u8_BottomRowPresent[u8_Cntr] || u8_TopRowPresent[u8_Cntr])
       {
           b_CommsOK = TRUE;
           break;
       }
    }
    
    if (b_CommsOK == FALSE)
    {
        /* Reset I2C interface */
        printf("No response seen on I2C - resetting interfaces\r\n");
        I2C1CON = 0x0000;
        I2C2CON = 0x0000;
        I2C3CON = 0x0000;
        
        DELAY_milliseconds(100u);

        I2C1_Initialize();
        I2C2_Initialize();
        I2C3_Initialize();        
    }
    
}

/************************************************************************************/
/* Init the LED control port expanders to be output pins - all off                  */
/************************************************************************************/
static void InitLeds (void)
{
    UINT8   u8_PortIoDirValue[2]    = {I2C_IODIR_REG, 0x00};
    UINT8   u8_PortConfigValue[2]   = {I2C_IOCON_REG, 0x20};
    UINT8   u8_PortOutputValue[2]   = {I2C_GPIO_REG,  0xFF};

    // Set register pointer to write the IODIR register to set all port pins as outputs
    SendI2c1Cmd(I2C_LED_CTRL_ADDR, u8_PortIoDirValue, 2u);
    SendI2c2Cmd(I2C_LED_CTRL_ADDR, u8_PortIoDirValue, 2u);

    // Set register pointer to write the output ports - all LEDs off
    SendI2c1Cmd(I2C_LED_CTRL_ADDR, u8_PortOutputValue, 2u);
    SendI2c2Cmd(I2C_LED_CTRL_ADDR, u8_PortOutputValue, 2u);
    
    // Set configuration register to not automatically increment register pointer
    SendI2c1Cmd(I2C_LED_CTRL_ADDR, u8_PortConfigValue, 2u);
    SendI2c2Cmd(I2C_LED_CTRL_ADDR, u8_PortConfigValue, 2u);
}     

/************************************************************************************/
/* Set LED state                                                                   */
/************************************************************************************/
void SetLedState (E_LED e_LedNumber, BOOL b_LedOn, E_ROW e_Row)
{
            UINT8   u8_NewPortValue;
            UINT8   u8_PortOutputValue[2]   = {I2C_GPIO_REG,  0xAA};
    static  UINT8   u8_LedStates[NUM_ROWS]  = {0xFFu, 0xFFu};

    /* Get the current value to modify */
    u8_NewPortValue = u8_LedStates[e_Row];
    
    if (e_LedNumber <= ALL_LEDS)
    {
        // Build command from required LED states
        if (b_LedOn)
        {
            if (e_LedNumber == ALL_LEDS)
            {
                u8_NewPortValue = 0;
            }
            else
            {
                u8_NewPortValue &= (UINT8)~(1u << e_LedNumber);
            }
        }
        else
        {
            if (e_LedNumber == ALL_LEDS)
            {
                u8_NewPortValue = 0xFFu;
            }
            else
            {
                u8_NewPortValue |= (UINT8)(1u << e_LedNumber);
            }
        }

        /* Store the new value for next time */
        u8_LedStates[e_Row] = u8_NewPortValue;

        /* Write new value to port expander */
        u8_PortOutputValue[1] = u8_NewPortValue;
        if (e_Row == TOP_ROW)
        {
            /* Write new value to top row */
            SendI2c2Cmd(I2C_LED_CTRL_ADDR, u8_PortOutputValue, 2u);
        }
        else
        {
            /* Write new value to bottom row */
            SendI2c1Cmd(I2C_LED_CTRL_ADDR, u8_PortOutputValue, 2u);
        }
    }
}  

/************************************************************************************/
/* Looks for changes in the port data and takes appropriate action                  */
/************************************************************************************/
static void CheckPortData (void)
{
    E_ROW   e_Row;
    UINT8   u8_Port;
    UINT8   u8_New;
    UINT8   u8_Old;
    UINT8   u8_Bit;
    UINT8   u8_Room;
    UINT8   u8_Mask;
    
    /* Look for an IO event from the top row of interfaces */
    ReadTopRow();

    /* Look for an IO event from the bottom row of interfaces */
    ReadBottomRow();


    /* Loop for each row of ports */
    for (e_Row = 0; e_Row < NUM_ROWS; e_Row++)
    {
        /* Loop for each port in the row */
        for (u8_Port = 0; u8_Port < NUM_IF_PORTS; u8_Port++)
        {
            u8_New = u8_PortData[e_Row][u8_Port];
            u8_Old = u8_PortDataPrev[e_Row][u8_Port];
            
            /* Compare old values with new to detect changes */
            if (u8_Old != u8_New)
            {
                /* Set orange LED state */
                SetLedState((u8_Port * 2) +1, (u8_New ? LED_ON : LED_OFF), e_Row);
                    
                /* Look for any new rooms that have been unlocked */
                for (u8_Bit = 0; u8_Bit < 8u; u8_Bit++)
                {
                    u8_Mask = 1 << u8_Bit;
                    if ((u8_New & u8_Mask) > (u8_Old & u8_Mask))
                    {
                        /* Calculate room number */
                        u8_Room = (NUM_IF_ROOMS * e_Row) + (u8_Port * 8u) + u8_Bit + 1u;
                        printf("Room %d unlocked\r\n", u8_Room);

                        /* Send unlock command to the Gateway to pass on to the room */
                        rs485_SendCommand(u8_Room, RS485_UNLOCK_CMD);
                        
                        ast_RoomStatus[u8_Room - 1u].b_Unlocked = TRUE; 
                    }
                    else if ((u8_New & u8_Mask) < (u8_Old & u8_Mask))
                    {
                        /* Calculate room number */
                        u8_Room = (NUM_IF_ROOMS * e_Row) + (u8_Port * 8u) + u8_Bit + 1u;
                        printf("Room %d locked\r\n", u8_Room);

                        /* Send lock command to the Gateway to pass on to the room */
                        rs485_SendCommand(u8_Room, RS485_LOCK_CMD);                            

                        ast_RoomStatus[u8_Room - 1u].b_Unlocked = FALSE; 
                    }
                    else
                    {
                        /* Do nothing - no change */
                    }
                }
                
                /* Update our old values */
                u8_PortDataPrev[e_Row][u8_Port] = u8_PortData[e_Row][u8_Port];
            }
        }
    }    
}

/************************************************************************************/
/* Sends the required number of bits using Wiegand Protocol                         */
/************************************************************************************/
static void WeigandTx (UINT8* u8_Data, UINT8 u8_NumBits)
{
    UINT8 u8_Mask = 1u << 7u;
    
    while (u8_NumBits--)
    {
        if (*u8_Data & u8_Mask)
        {
            /* Send a '1' */
            DATA_1_SetLow();
            DELAY_microseconds(90);
            DATA_1_SetHigh();                        
        }
        else
        {
            /* Send a '0' */
            DATA_0_SetLow();
            DELAY_microseconds(90);
            DATA_0_SetHigh();
        }
        
        DELAY_milliseconds(1);
        u8_Mask >>= 1u;
        
        /* Completed 8 bits - move on to next 8 bits */
        if (u8_Mask == 0)
        {
            u8_Mask = 1u << 7u;
            u8_Data++;
        }
    }
    
    DELAY_milliseconds(50u);
}

/************************************************************************************/
/* Convert a string of ASCII numbers into a 16 bit values and store in the buffer   */
/************************************************************************************/
static void Get16BitValue (UINT8* u8_Dest, UINT8* u8_Src, UINT8 u8_Length)
{
    UINT8   u8_Cntr;
    UINT16  u16_Total = 0u;
    
    for (u8_Cntr=0; u8_Cntr < u8_Length; u8_Cntr++)
    {
        u16_Total *= 10;
        u16_Total += u8_Src[u8_Cntr] - '0';
    }
    
    u8_Dest[0] = (UINT8)(u16_Total >> 8);
    u8_Dest[1] = (UINT8)(u16_Total & 0xFFu);    
}

/************************************************************************************/
/* Sends specified room and card number using specified parity option               */
/* We don't know how to calculate the parity bit, so there are 4 possible options   */
/* we can try. 2 parity bits = 4 possible combinations.                             */
/************************************************************************************/
static void SendKeyOption(E_PARITY e_Parity, UINT8* u8_RoomBuff, UINT8* u8_Buffer)
{
    printf("Option [%d]\r\n", e_Parity);

    switch (e_Parity)
    {
        case OPTION_1 : /* both bits = 0        */
            u8_Buffer[0] &= ~(1 << 7);
            u8_Buffer[4] &= ~(1 << 3);
            break;
            
        case OPTION_2 : /* Bit 0 = 0, Bit 36 = 1 */
            u8_Buffer[0] &= ~(1 << 7);
            u8_Buffer[4] |= 1 << 3;
            break;

        case OPTION_3 : /* Bit 0 = 1, Bit 36 = 0 */
            u8_Buffer[0] |= 1 << 7;
            u8_Buffer[4] &= ~(1 << 3);
            break;

        case OPTION_4 : /* Both bits = 1        */
            u8_Buffer[0] |= 1 << 7;
            u8_Buffer[4] |= 1 << 3;
            break;

        default:
            printf("Parity Option not supported\r\n");
            break;
    }

    WeigandTx(u8_Buffer, 37u);
    
    DELAY_milliseconds(300u);

    /* Now send the room number and Hash key */
    WeigandTx(&u8_RoomBuff[0], 4u);
    WeigandTx(&u8_RoomBuff[1], 4u);
    WeigandTx(&u8_RoomBuff[2], 4u);
    WeigandTx(&u8_RoomBuff[3], 4u);            
}

/************************************************************************************/
/* Sends an unlock request to the controller using Wiegand                         */
/************************************************************************************/
void main_SendUnlockRequest (UINT8 u8_Room, UINT8* au8_CardNumber, UINT8 u8_Length)
{
    UINT8       u8_RoomBuff[4];
    UINT8       u8_Buffer[20];
    UINT8       u8_Cntr;
    UINT8       u8_RoomOffset = u8_Room - 1u;
    E_PARITY    e_Parity;
    
    printf("Sending Wiegand unlock for room %u using card %.*s\r\n", 
            u8_Room, u8_Length, au8_CardNumber);      
    
    /* Send the room number, one digit at a time, followed by the '#' to emulate the keypad */
    u8_RoomBuff[0] = 1u << 4u;
    u8_RoomBuff[1] = (u8_Room / 10u) << 4u;
    u8_RoomBuff[2] = (u8_Room % 10u) << 4u;
    u8_RoomBuff[3] = KEYPAD_HASH;
   
    /* Prepare card number */
    u8_Buffer[0] = 40u;
    Get16BitValue(&u8_Buffer[1], &au8_CardNumber[0], 5u);
    Get16BitValue(&u8_Buffer[3], &au8_CardNumber[5], 5u);
    
    /* Shift card number up 4 bits as the facility number is only 12 bits */
    for (u8_Cntr = 1; u8_Cntr < 5u; u8_Cntr++)
    {
        u8_Buffer[u8_Cntr] <<= 4u;
        u8_Buffer[u8_Cntr] |= (UINT8)(u8_Buffer[u8_Cntr + 1] >> 4u);
    }

    /* Check if we have a stored parity option */
    if (ast_RoomStatus[u8_RoomOffset].e_Parity != OPTION_MAX)
    {
        /* Try stored option first */
        SendKeyOption(ast_RoomStatus[u8_RoomOffset].e_Parity, u8_RoomBuff, u8_Buffer);

        for (u8_Cntr=0; u8_Cntr < 30u; u8_Cntr++)
        {
            DELAY_milliseconds(10u);

            /* Check for any changes in the port data */
            CheckPortData();

            /* Check if the room unlocked */
            if (ast_RoomStatus[u8_RoomOffset].b_Unlocked)
            {
                break;
            }
        }
    }
    
    /* If the room did not unlock, try all combinations */
    if (ast_RoomStatus[u8_RoomOffset].b_Unlocked == FALSE)
    {
        /* Send all four parity options for card number */
        for (e_Parity=0; e_Parity < OPTION_MAX; e_Parity++)
        {
            SendKeyOption(e_Parity, u8_RoomBuff, u8_Buffer);
            DELAY_milliseconds(250u);

            /* Check for any changes in the port data */
            CheckPortData();
            
            /* Check if the room unlocked */
            if (ast_RoomStatus[u8_RoomOffset].b_Unlocked)
            {
                /* Store the parity option that worked */
                ast_RoomStatus[u8_RoomOffset].e_Parity = e_Parity;
                printf("Storing parity option %u for room %u\r\n", e_Parity, u8_Room);
                break;
            }
        }
    }
}

/************************************************************************************/
/* Main application                                                                 */
/************************************************************************************/
int main(void)
{   
    UINT8   u8_InitCounter  = 0;
    UINT8   u8_UpdateCntr   = 0;
    
    // initialize the device
    SYSTEM_Initialize();
        
    printf("\r\n\r\nMaster - Version 1.4\r\n");

    rs485_Init();
    InitLeds();
    InitTopRow();
    InitBottomRow();

    /* Init room status array */
    for (u8_UpdateCntr=0; u8_UpdateCntr < NUM_ROOMS; u8_UpdateCntr++)
    {
        ast_RoomStatus[u8_UpdateCntr].b_Unlocked = FALSE;
        ast_RoomStatus[u8_UpdateCntr].e_Parity   = OPTION_MAX;
    }
    
    while (1)
    {
        u8_InitCounter++; 
        if (u8_InitCounter == 0)
        {
            /* Every 2.5 seconds, look for new interface cards */
            InitTopRow();
            InitBottomRow();
            
            /* Check I2C Comms */
            CheckI2cComms();            
                    
            WATCHDOG_TimerClear();
        }
        DELAY_milliseconds(10);

        /* Check for any changes in the port data */
        CheckPortData();
        
        /* Look for data from a gateway over RS485 */
        if (UART2_IsRxReady())
        {
            rs485_ReceiveFrame();
        }
    }
    return 1; 
}
/**
 End of File
*/

