/*
 * rk29_board.h
 *
 * Overview:  
 *
 * Copyright (c) 2011, YiFang Digital
 *
 * Version:  1.0
 * Created:  09/13/2011 06:22:50 PM
 * Author:  zqqu <zqqu@yifangdigital.com>
 * Company:  YiFang Digital
 * History:
 *
 * 
 */


#ifndef __RK29_BOARD_H 
#define __RK29_BOARD_H 

#if defined(CONFIG_MACH_M911)
#include "./board/rk29_m911.h"
#elif defined(CONFIG_MACH_M912)
#include "./board/rk29_m912.h"
#elif defined(CONFIG_MACH_M908)
#include "./board/rk29_m908.h"
#elif defined(CONFIG_MACH_M803)
#include "./board/rk29_m803.h"
#elif defined(CONFIG_MACH_M732)
#include "./board/rk29_m732.h"
#elif defined(CONFIG_MACH_M722)
#include "./board/rk29_m722.h"
#elif defined(CONFIG_MACH_M722HZ)
#include "./board/rk29_m722.h"
#endif
#endif
