/**
  PIN MANAGER Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.c

  @Summary:
    This is the generated manager file for the PIC24 / dsPIC33 / PIC32MM MCUs device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for PIN MANAGER.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.125
        Device            :  PIC32MM0064GPM028
    The generated drivers are tested against the following:
        Compiler          :  XC32 v2.20
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
    Section: Includes
*/

#include <xc.h>
#include "pin_manager.h"
#include "system.h"

/**
 Section: Driver Interface Function Definitions
*/
void PIN_MANAGER_Initialize (void)
{
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0004;
    LATB = 0x0D74;
    LATC = 0x0000;

    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISA = 0x001B;
    TRISB = 0xE3AF;
    TRISC = 0x0000;

    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPDA = 0x0000;
    CNPDB = 0x0000;
    CNPDC = 0x0000;
    CNPUA = 0x0001;
    CNPUB = 0xC200;
    CNPUC = 0x0000;

    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0C40;
    ODCC = 0x0000;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    ANSELA = 0x0000;
    ANSELB = 0x0000;

    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    SYSTEM_RegUnlock(); // unlock PPS
    RPCONbits.IOLOCK = 0;

    RPINR9bits.U2RXR = 0x0004;    //RA3->UART2:U2RX
    RPOR0bits.RP3R = 0x0004;    //RA2->UART2:U2TX
    RPINR8bits.U3RXR = 0x0005;    //RA4->UART3:U3RX
    RPOR2bits.RP10R = 0x0006;    //RB4->UART3:U3TX

    RPCONbits.IOLOCK = 1; // lock   PPS
    SYSTEM_RegLock(); 

    
    /****************************************************************************
     * Interrupt On Change: negative
     ***************************************************************************/
    CNEN1Bbits.CNIE1B14 = 1;    //Pin : RB14
    CNEN1Bbits.CNIE1B15 = 1;    //Pin : RB15
    CNEN1Bbits.CNIE1B9 = 1;    //Pin : RB9
    
    /****************************************************************************
     * Interrupt On Change: flag
     ***************************************************************************/
    CNFBbits.CNFB14 = 0;    //Pin : RB14
    CNFBbits.CNFB15 = 0;    //Pin : RB15
    CNFBbits.CNFB9 = 0;    //Pin : RB9
    
    /****************************************************************************
     * Interrupt On Change: config
     ***************************************************************************/
    CNCONBbits.CNSTYLE = 1;    //Config for PORTB
    CNCONBbits.ON = 1;    //Config for PORTB

    /****************************************************************************
     * Interrupt On Change: Interrupt Enable
     ***************************************************************************/
    IFS0CLR= 1 << _IFS0_CNAIF_POSITION; //Clear CNAI interrupt flag
    IEC0bits.CNAIE = 1; //Enable CNAI interrupt
    IFS0CLR= 1 << _IFS0_CNBIF_POSITION; //Clear CNBI interrupt flag
    IEC0bits.CNBIE = 1; //Enable CNBI interrupt
}

/* Interrupt service routine for the CNAI interrupt. */
void __attribute__ ((vector(_CHANGE_NOTICE_A_VECTOR), interrupt(IPL2SOFT))) _CHANGE_NOTICE_A( void )
{
    if(IFS0bits.CNAIF == 1)
    {
        // Clear the flag
        IFS0CLR= 1 << _IFS0_CNAIF_POSITION; // Clear IFS0bits.CNAIF
    }
}
/* Interrupt service routine for the CNBI interrupt. */
void __attribute__ ((vector(_CHANGE_NOTICE_B_VECTOR), interrupt(IPL1SOFT))) _CHANGE_NOTICE_B( void )
{
    if(IFS0bits.CNBIF == 1)
    {
        // Clear the flag
        IFS0CLR= 1 << _IFS0_CNBIF_POSITION; // Clear IFS0bits.CNBIF
        if(CNFBbits.CNFB9 == 1)
        {
            CNFBCLR = 0x200;  //Clear CNFBbits.CNFB9
            // Add handler code here for Pin - RB9
        }
        if(CNFBbits.CNFB14 == 1)
        {
            CNFBCLR = 0x4000;  //Clear CNFBbits.CNFB14
            // Add handler code here for Pin - RB14
        }
        if(CNFBbits.CNFB15 == 1)
        {
            CNFBCLR = 0x8000;  //Clear CNFBbits.CNFB15
            // Add handler code here for Pin - RB15
        }
    }
}
