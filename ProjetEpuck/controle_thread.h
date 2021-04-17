#ifndef CONTROLE_THD_H
#define CONTROLE_THD_H

#include "ch.h"
#include "hal.h"

//Modes disponibles
typedef enum {
	Mode0 =0,
	Mode1,
	Mode2,
	Mode3,
};

void stop_thread(int mode_to_stop);
void run_thread_manette(void);

#endif
