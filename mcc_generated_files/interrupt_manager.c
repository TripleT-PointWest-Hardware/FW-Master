/**
  System Interrupts Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    interrupt_manager.h

  @Summary:
    This is the generated driver implementation file for setting up the
    interrupts using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description:
    This source file provides implementations for PIC24 / dsPIC33 / PIC32MM MCUs interrupts.
    Generation Information : 
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.125
        Device            :  PIC32MM0064GPM028
    The generated drivers are tested against the following:
        Compiler          :  XC32 v2.20
        MPLAB             :  MPLAB X v5.20
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
    Section: Includes
*/
#include <xc.h>

/**
    void INTERRUPT_Initialize (void)
*/
void INTERRUPT_Initialize (void)
{
    // Enable Multi Vector Configuration
    INTCONbits.MVEC = 1;
    
    //    CTI: Core Timer
    //    Priority: 1
        IPC0bits.CTIP = 1;
    //    Sub Priority: 0
        IPC0bits.CTIS = 0;
    //    UERI: UART 2 Error
    //    Priority: 1
        IPC14bits.U2EIP = 1;
    //    Sub Priority: 0
        IPC14bits.U2EIS = 0;
    //    UTXI: UART 2 Transmission
    //    Priority: 1
        IPC14bits.U2TXIP = 1;
    //    Sub Priority: 0
        IPC14bits.U2TXIS = 0;
    //    URXI: UART 2 Reception
    //    Priority: 1
        IPC14bits.U2RXIP = 1;
    //    Sub Priority: 0
        IPC14bits.U2RXIS = 0;
    //    CNAI: PORT A Change Notification
    //    Priority: 2
        IPC2bits.CNAIP = 2;
    //    Sub Priority: 0
        IPC2bits.CNAIS = 0;
    //    CNBI: PORT B Change Notification
    //    Priority: 1
        IPC2bits.CNBIP = 1;
    //    Sub Priority: 0
        IPC2bits.CNBIS = 0;
    //    UERI: UART 3 Error
    //    Priority: 1
        IPC15bits.U3EIP = 1;
    //    Sub Priority: 0
        IPC15bits.U3EIS = 0;
    //    UTXI: UART 3 Transmission
    //    Priority: 1
        IPC15bits.U3TXIP = 1;
    //    Sub Priority: 0
        IPC15bits.U3TXIS = 0;
    //    URXI: UART 3 Reception
    //    Priority: 1
        IPC14bits.U3RXIP = 1;
    //    Sub Priority: 0
        IPC14bits.U3RXIS = 0;
    //    INT0I: External 0
    //    Priority: 1
        IPC0bits.INT0IP = 1;
    //    Sub Priority: 0
        IPC0bits.INT0IS = 0;
    //    INT1I: External 1
    //    Priority: 1
        IPC1bits.INT1IP = 1;
    //    Sub Priority: 0
        IPC1bits.INT1IS = 0;
    //    MICI: I2C 1 Master
    //    Priority: 1
        IPC16bits.I2C1MIP = 1;
    //    Sub Priority: 0
        IPC16bits.I2C1MIS = 0;
    //    SICI: I2C 1 Slave
    //    Priority: 1
        IPC16bits.I2C1SIP = 1;
    //    Sub Priority: 0
        IPC16bits.I2C1SIS = 0;
    //    MICI: I2C 3 Master
    //    Priority: 1
        IPC18bits.I2C3MIP = 1;
    //    Sub Priority: 0
        IPC18bits.I2C3MIS = 0;
    //    SICI: I2C 3 Slave
    //    Priority: 1
        IPC17bits.I2C3SIP = 1;
    //    Sub Priority: 0
        IPC17bits.I2C3SIS = 0;
    //    MICI: I2C 2 Master
    //    Priority: 2
        IPC17bits.I2C2MIP = 2;
    //    Sub Priority: 1
        IPC17bits.I2C2MIS = 1;
    //    SICI: I2C 2 Slave
    //    Priority: 2
        IPC17bits.I2C2SIP = 2;
    //    Sub Priority: 1
        IPC17bits.I2C2SIS = 1;
}
