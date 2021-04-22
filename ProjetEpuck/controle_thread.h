#ifndef CONTROLE_THD_H
#define CONTROLE_THD_H

#include "ch.h"
#include "hal.h"

//Modes disponibles
typedef enum {
	MODE0 = 0,
	MODE1,
	MODE2,
	MODE3,
} mode_robot;


void stop_thread_camera(void);
void run_thread_mode_3(void);
void run_thread_mode_1(void);
void run_thread_mode_0(void);

#endif
