/* 
 * File:   rs485.h
 * Author: Allan
 *
 * Created on October 28, 2019, 11:12 PM
 */

#ifndef RS485_H
#define	RS485_H

/* Defined commands */
#define RS485_UNLOCK_REQ                        'R'     /* Command to request an unlock. Followed by card ID        */
#define RS485_UNLOCK_CMD                        'U'     /* Command to perform an unlock operation on the lock       */

#define RS485_START_OF_FRAME                    (0x7Eu) /* Start of Frame delimiter for frames                      */
#define RS485_DEST_MASTER                       'M'     /* Indicate source or destination for Master                */
#define RS485_MAX_FRAME_SIZE                    (50u)   /* Max size of an RS485 frame                               */
#define RS485_CMD_SIZE                          (sizeof(RS485_CMD_DATA) - 1)    /* Don't count payload placeholder  */
#define RS485_HEADER_SIZE                       sizeof(RS485_HEADER)

/* Structure used to send data between the devices over RS485 */
typedef struct
{
    UINT8           u8_StartDelim;
    UINT8           u8_FrameLen;
    UINT8           u8_Destination;        
    UINT8           u8_Source;        
} RS485_HEADER;

typedef struct
{
    UINT8           u8_Room;        
    UINT8           u8_Command;        
    UINT8           u8_Length;          /* Payload length   */
    UINT8           u8_Payload;         /* Variable length */       
} RS485_CMD_DATA;

typedef struct
{
    RS485_HEADER    Header;
    RS485_CMD_DATA  Data;
} RS485_API;
        
/* Combined structure for RS-485 frames */
typedef union
{
    RS485_API       Frame;
    UINT8           Bytes[RS485_MAX_FRAME_SIZE];
} RS485_FRAME;

        
extern void rs485_Init          (void);
extern void rs485_ReceiveFrame  (void);
extern void rs485_SendCommand   (UINT8 u8_FullRoom, UINT8 u8_Command);

#endif	/* RS485_H */

