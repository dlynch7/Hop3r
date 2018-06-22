#ifndef __RLS_ORBIS__H__
#define __RLS_ORBIS__H__
// Header file for RLS_orbis.c
// API for SPI interface with RLS Orbis encoder

#include <stdint.h>

void initRLS(void);
uint32_t readRLS(void);

#endif
