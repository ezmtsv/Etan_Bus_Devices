/* This is a part of the ETAN BUS project
 * Copyright (c) 2018-2021 ETAN Research & Development Center.
 */

#ifndef _ERRORS_H_
#define _ERRORS_H_
#ifdef __cplusplus
extern "C"
{
#endif

#define ERROR_OK                0
#define ERROR_LINE_STUCK        -1
#define ERROR_LINE_BUSY         -2
#define ERROR_LINE_COLLISION    -3
#define ERROR_LINE_PLOST        -4
#define ERROR_BAD_DATA          -5
#define ERROR_TIMEOUT           -6
#define ERROR_LINE_DOWN         -7
#define ERROR_FIFO_FULL         -8
#define ERROR_FIFO_EMPTY        -9
#define ERROR_FIFO_INIT         -10
#define ERROR_BAD_COMMAND       -11
#define ERROR_UNKNOWN           -127

#ifdef __cplusplus
}
#endif
#endif //_ERRORS_H_
