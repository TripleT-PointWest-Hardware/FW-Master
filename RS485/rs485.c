/* 
 * File:   rs485.c
 * Author: Allan
 *
 * Created on October 28, 2019, 11:12 PM
 */

#include "stdio.h"
#include "string.h"
#include "../mcc_generated_files/pin_manager.h"
#include "../inc/CanascTypes.h"
#include "../main.h"
#include "rs485.h"

/********************************* Definitions ************************************/

/* General definitions	*/
#define SET_DIR_TX              RS485_DIR_SetHigh
#define SET_DIR_RX              RS485_DIR_SetLow

/******************************  Local Variables **********************************/

static UINT8    gu8_GatewayId   = 0xFF;

/******************************  Local function prototypes ************************/
static void FlushRxBuffer   (void);
static void ProcessRxFrame  (RS485_API* st_Frame);
static void CalcChecksum    (UINT8* pu8_Buffer, UINT8 u8_Len);
static void TransmitBuffer  (UINT8* au8_Buffer, UINT8 u8_Length);

/************************************************************************************/
/* Flushes any received chars out of the receive buffer                             */
/************************************************************************************/
static void FlushRxBuffer (void)
{
    while(UART2_IsRxReady())
    {
        UART2_Read();
    }
}

/************************************************************************************/
/* Transmits the buffer out of the serial port and waits for Tx to complete         */
/************************************************************************************/
static void TransmitBuffer (UINT8* au8_Buffer, UINT8 u8_Length)
{

    SET_DIR_TX();
    DELAY_milliseconds(1);
            
    UART2_WriteBuffer(au8_Buffer, u8_Length);

    /* Wait for Tx to complete before switching direction to Rx again */
    while (!UART2_IsTxDone())
    {
        DELAY_milliseconds(1);    
    }
    
    SET_DIR_RX();

    /* Switching between Tx and Rx generates a null character in Rx queue */
    DELAY_milliseconds(1);
    if (UART2_Peek(0) == 0x00)
    {
        /* Throw away the extra char */
        UART2_Read();
    }
}

/************************************************************************************/
/* Init the RS-485 interface                                                        */
/************************************************************************************/
void rs485_Init (void)
{
    FlushRxBuffer();

    TransmitBuffer("RS485: Master Init\r\n", 20);
}

/************************************************************************************/
/* Receives a frame from the Gateway over RS-485                                    */
/************************************************************************************/
void rs485_ReceiveFrame (void)
{

    RS485_FRAME     st_Frame        = {0};
    UINT8           u8_RxLen        = 0;
    UINT8           u8_FrameSize    = RS485_MAX_FRAME_SIZE;
    UINT8           u8_Checksum     = 0;
    BOOL            b_FrameOK       = FALSE;
    UINT8           u8_RxByte;
    
    printf("\r\nRS485 RX: ");
    
    /* Give buffer time to fill */
    DELAY_milliseconds(20u);
     
    while (UART2_IsRxReady() && (u8_RxLen < u8_FrameSize))
    {
        /* Read one character from the serial port */
        u8_RxByte = UART2_Read();
        st_Frame.Bytes[u8_RxLen] = u8_RxByte;
        printf("%2.2X ", u8_RxByte);
        
        /* Only start storing data once we find the Start of Frame marker (0x7E) */
        switch (u8_RxLen)
        {
            case 0: /* Look for start of frame */
                if (u8_RxByte == RS485_START_OF_FRAME)
                {
                    u8_RxLen++;
                }
                break;
            
            case 1: /* Determine frame size */
                /* add bytes for header and checksum */
                u8_FrameSize = st_Frame.Frame.Header.u8_FrameLen + 3u; 
                u8_RxLen++;
                break;
                
            default: /* For all other bytes, keep receiving and adding to checksum */
                if (u8_RxLen < (u8_FrameSize -1))
                {
                    u8_Checksum += u8_RxByte;
                }
                u8_RxLen++;
                break;
        } /* case */
    } /* while */
    
    u8_Checksum = (UINT8)(0xFFu - u8_Checksum);
    
    if ((u8_RxLen == u8_FrameSize) && (st_Frame.Bytes[u8_RxLen -1u] == u8_Checksum))
    {
        printf("\r\nFrame Detected: \r\n");
        ProcessRxFrame(&st_Frame.Frame);
    }
/*    else
    {
        printf("\r\nNo frame found.\r\n");
        printf("\tu8_RxLen: %u\r\n", u8_RxLen);
        printf("\tChecksum. Calc: %2.2X  Actual: %2.2X\r\n", u8_Checksum, st_Frame.Bytes[u8_RxLen -1u]);
    } */   
}

/************************************************************************************/
/* Process a received frame from a Gateway on the RS485 bus                         */
/************************************************************************************/
static void ProcessRxFrame (RS485_API* st_Frame)
{
    UINT8   u8_FullRoomNumber;
    
    switch (st_Frame->Data.u8_Command)
    {
        case RS485_UNLOCK_REQ:      /* Unlock request from a Gateway */
            u8_FullRoomNumber = (st_Frame->Header.u8_Source -1u) * 10u;
            u8_FullRoomNumber += st_Frame->Data.u8_Room;
            main_SendUnlockRequest(u8_FullRoomNumber, &st_Frame->Data.u8_Payload, st_Frame->Data.u8_Length);
            break;
            
        default:
            printf("Unknown RS485 command: %c [%2.2X]\r\n");
            break;
    }
}

/************************************************************************************/
/* Calculates the checksum for the frame and adds it to the end of the buffer       */
/************************************************************************************/
static void CalcChecksum (UINT8* pu8_Buffer, UINT8 u8_Len)
{
    UINT8   u8_Cntr;
    UINT8   u8_Checksum = 0;
    
    for (u8_Cntr = 0; u8_Cntr < u8_Len; u8_Cntr++)
    {
        u8_Checksum += pu8_Buffer[u8_Cntr];
    }
    
    pu8_Buffer[u8_Len] = (UINT8)(0xFF - u8_Checksum);
}

/************************************************************************************/
/* Forward an XBEE frame to the Master over RS-485                                  */
/************************************************************************************/
void rs485_SendCommand(UINT8 u8_FullRoom, UINT8 u8_Command)
{
    RS485_FRAME st_Frame    = {0};
    UINT8       u8_FrameLen;
    UINT8       u8_GatewayId;
    UINT8       u8_RoomNumber;
    
    /* Split full room number into gateway and room, based on 10 rooms per Gateway  */
    /* E.g: Full Room 63 = Gateway 7, Room 3  (Gateway 1 covers rooms 1 to 10)      */
    u8_GatewayId    = (u8_FullRoom - 1u) / 10u;
    u8_RoomNumber   = u8_FullRoom - (u8_GatewayId * 10u);
    u8_GatewayId++;     /* Start from 1, not 0  */

    /* Calculate data frame size - exclude SOF, Size and checksum fields */
    u8_FrameLen = RS485_HEADER_SIZE + RS485_CMD_SIZE - 2u;
    
    /* Header */
    st_Frame.Frame.Header.u8_StartDelim   = RS485_START_OF_FRAME;
    st_Frame.Frame.Header.u8_FrameLen     = u8_FrameLen;
    st_Frame.Frame.Header.u8_Destination  = u8_GatewayId;
    st_Frame.Frame.Header.u8_Source       = RS485_DEST_MASTER;
            
    /* Data */
    st_Frame.Frame.Data.u8_Room         = u8_RoomNumber;
    st_Frame.Frame.Data.u8_Command      = u8_Command;
    st_Frame.Frame.Data.u8_Length       = 0;
    
    /* Checksum - start after u8_FrameLen field */
    CalcChecksum(&st_Frame.Bytes[2], u8_FrameLen);     
    
    /* Send the data. u8_FrameLen + SOF + Length + Checksum*/
    TransmitBuffer(st_Frame.Bytes, u8_FrameLen + 3u);
}
