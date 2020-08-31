/* 
 * File:   CanascTypes.h
 * Author: Allan
 *
 * Created on September 7, 2019, 10:43 PM
 */

#ifndef CANASCTYPES_H
#define	CANASCTYPES_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef NULL    
    #define NULL ((void *)0)
#endif
    
typedef unsigned    char        UINT8;
typedef signed      char        SINT8;
typedef unsigned    short       UINT16;
typedef signed      short       SINT16;
typedef unsigned    long        UINT32;
typedef signed      long        SINT32;
typedef unsigned    long long   UINT64;
typedef signed      long long   SINT64;

typedef enum
{
    FALSE = 0,
    TRUE
} BOOL;

#ifdef	__cplusplus
}
#endif

#endif	/* CANASCTYPES_H */

