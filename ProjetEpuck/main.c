#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <process_image.h>
#include <leds.h>
#include <selector.h>
#include <controle_thread.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

static THD_WORKING_AREA(selector_thd_wa, 1024);
static THD_FUNCTION(selector_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);


    uint8_t position_select = 0;
    uint8_t new_position_select = 0;

    while(1){
    	//detection de changement de position du selecteur
    	new_position_select = get_selector();
    	if (position_select != new_position_select){
    		position_select = new_position_select;
			switch(position_select) {
						case MODE0: //mode attente
//							clear_leds();
							set_led(1, 1);
//							set_rgb_led(0, 10, 0, 0);
//							set_rgb_led(1, 10, 0, 0);
//							set_rgb_led(2, 10, 0, 0);
//							set_rgb_led(3, 10, 0, 0);
							run_thread_mode_0();
							break;
						case MODE1: //mode 1 : conduite respectueuse
							clear_leds();
							set_led(LED3, 1);
							run_thread_mode_1();
							break;
						case MODE2: //mode 2 : mode Tom Cruise
							clear_leds();
							set_led(LED5, 1);
							stop_thread(MODE3);
							break;
						case MODE3: //mode 3 : mode manette
							clear_leds();
							set_led(LED7, 1);
							run_thread_mode_3();
							break;
						default :
							clear_leds();
							break;
			}
        }
    	chThdSleepMilliseconds(130);
    }
}


static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//init leds
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	proximity_start();
//	playMelodyStart();
    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);


	//stars the threads for the pi regulator and the processing of the image
//	pi_regulator_start();
//	process_image_start();

	chThdCreateStatic(selector_thd_wa, sizeof(selector_thd_wa), NORMALPRIO + 1, selector_thd, NULL);

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
