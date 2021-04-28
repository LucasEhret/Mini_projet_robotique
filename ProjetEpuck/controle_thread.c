#include <controle_thread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <leds.h>
#include <motors.h>
#include <communications.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <process_image.h>

#define NB_ELEMENTS_CONTROLLER 4
#define DIST_THRESHOLD 300
#define DIST_THRESHOLD_JEU 100
#define SPEED_M1 400
#define SPEED_M2 600
#define ROTATION_COEFF_M1 2
#define ROTATION_COEFF_M2 3
#define WIDTH_THRESHOLD 100
#define SPEED_M3 800

//threads du mode 0 :
static thread_t* thd_mode_0_IR = NULL;

//threads du mode 1 :
static thread_t* thd_m1_position = NULL;

//threads du mode 2 :
static thread_t* thd_mode_2 = NULL;

//threads du mode 3 :
static thread_t* thd_mode_3_manette = NULL;

//msg_t *buf;
//static MAILBOX_DECL(collision, buf, 1);

//-------------------------------MODE 0-----------------------------------------------------------
/*Thread propre au mode 0 : attente
 * 	-> petit jeu avec LEDs et capteurs de distance. (priorité normale)
 */


//thread qui utilise les capteurs de distance, et allume les leds les plus proches de l'objet détecté
static THD_WORKING_AREA(thd_m0_capteur_distance_wa, 1024);
static THD_FUNCTION(thd_m0_capteur_distance, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    calibrate_ir();

    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
        //Détection obstacles frontaux
    	volatile uint8_t indice_capteur = 10;
    	uint8_t temp_val_max = 0;
    	for (uint8_t i = 0; i < 8; i++){
    		if (get_prox(i) > temp_val_max && get_prox(i) > DIST_THRESHOLD_JEU){
    			temp_val_max = get_prox(i);
    			indice_capteur = i;
    		}
    	}
		switch(indice_capteur){
			case 0 :
				//bloqué par le miroir
//				clear_leds();
//				set_led(LED1, 1);
				break;
			case 1 :
				clear_leds();
				set_rgb_led(LED2, 100, 0, 0);
				break;
			case 2 :
				clear_leds();
				set_led(LED3, 1);
				break;
			case 3 :
				clear_leds();
				set_rgb_led(LED4, 100, 0, 0); // marche pas
				set_led(LED5, 1);
				break;
			case 4 :
				clear_leds();
				set_rgb_led(LED6, 100, 0, 0);
				set_led(LED5, 1);
				break;
			case 5 :
				clear_leds();
				set_led(LED7, 1);
				break;
			case 6 :
				clear_leds();
				set_rgb_led(LED8, 100, 0, 0);
				break;
			case 7 :
				clear_leds();
				set_led(LED1, 1);
				break;
			default :
				clear_leds();
				break;
    	}
      	chThdSleepMilliseconds(100);
    }
}



//-------------------------------MODE 1-----------------------------------------------------------
/* Threads propre au mode 1 : conduite respectueuse
 *  -> Détection des obstacles  (priorité +)
 *  -> Suivie de la ligne noire (priorité normale)
 *  ->
 *  ->
 */


//thread qui récupère les images de la caméra
static THD_WORKING_AREA(thd_m1_camera_wa, 2048);
static THD_FUNCTION(thd_m1_camera, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    int16_t speed_correction = 0;
    volatile int valeur_capteur = 0;
    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
    	valeur_capteur = get_prox(7);
        if((get_prox(0) > DIST_THRESHOLD) || (get_prox(7) > DIST_THRESHOLD)) {
      		//Arret
      		left_motor_set_speed(0);
      		right_motor_set_speed(0);
      		set_body_led(1);
      	}
      	else{
      		set_body_led(0);
            //computes a correction factor to let the robot rotate to be in front of the line
            speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

            //if the line is nearly in front of the camera, don't rotate
            if(abs(speed_correction) < ROTATION_THRESHOLD){
            	speed_correction = 0;
            }
            if(get_line_width() > WIDTH_THRESHOLD){
                //applies the speed from the PI regulator and the correction for the rotation
        		right_motor_set_speed(SPEED_M1 - ROTATION_COEFF_M1 * speed_correction);
        		left_motor_set_speed(SPEED_M1 + ROTATION_COEFF_M1 * speed_correction);
            }
            else{
          		left_motor_set_speed(0);
          		right_motor_set_speed(0);
            }
      	}
		chThdSleepMilliseconds(50);
    }
}



//-------------------------------MODE 2-----------------------------------------------------------
/* Threads propre au mode 2 : conduite Tom Cruise
 * -> Suivi de la ligne noire (priorité normale)
 *               (réutilisation du thread du mode 1 mais avec une vitesse du robot plus élevée)
 */


static THD_WORKING_AREA(thd_m2_camera_wa, 1024);
static THD_FUNCTION(thd_m2_camera, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    int16_t speed_correction = 0;
	uint16_t speed_right = SPEED_M2;
	uint16_t speed_left = SPEED_M2;

    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
    	speed_right = SPEED_M2;
    	speed_left = SPEED_M2;
    	if((get_prox(0) > DIST_THRESHOLD) || (get_prox(7) > DIST_THRESHOLD)) {
      		//Acceleration
        	speed_left = 800;
        	speed_right = 800;
        	set_body_led(1);
      	}
        else{
        	set_body_led(0);
        }
		//computes a correction factor to let the robot rotate to be in front of the line
		speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

		//if the line is nearly in front of the camera, don't rotate
		if(abs(speed_correction) < ROTATION_THRESHOLD){
			speed_correction = 0;
		}
		if(get_line_width() > WIDTH_THRESHOLD){
			//applies the speed from the PI regulator and the correction for the rotation
			right_motor_set_speed(speed_right - ROTATION_COEFF_M2 * speed_correction);
			left_motor_set_speed(speed_left + ROTATION_COEFF_M2 * speed_correction);
		}
		else {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
		chThdSleepMilliseconds(50);
    }
}




//-------------------------------MODE 3-----------------------------------------------------------
/* Threads propre au mode 3 : contrôle à la manette
 *  -> Détection des obstacle (priorité +)
 *  -> Réception manette      (priorité normale)
 *
 */



//thread qui utilise les capteurs de distance pour faire un tour sur lui meme quand il rencontre un obstacle en mode manette
static THD_WORKING_AREA(thd_m3_wa, 2048);
static THD_FUNCTION(thd_m3, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    bool collision = false;
    uint16_t compteur = 0;
	int right_speed = 0;
	int left_speed = 0;
    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
    	float data_from_computer[4] = {0, 0, 0, 0};
    	uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, data_from_computer, NB_ELEMENTS_CONTROLLER);
    	//detection collision
    	if((get_prox(7) > DIST_THRESHOLD) && !collision && !data_from_computer[2]) {
    		collision = true;
    		//Instruction en cas de collision
    		set_body_led(1);
//    		playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL); !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    		left_motor_set_speed(700);
    		right_motor_set_speed(-700);
    		compteur = 0;
    	}
    	if(!collision) {
    		set_body_led(0);
        	if(size == NB_ELEMENTS_CONTROLLER){
        		data_from_computer[1] -= 180;
        		if (data_from_computer[1] > -90 && data_from_computer[1] < 90){
        			right_speed = (90 - data_from_computer[1]) / 90 * SPEED_M3 * data_from_computer[0] / 100;
        			left_speed = (-90 - data_from_computer[1]) / (-90) * SPEED_M3 * data_from_computer[0] / 100;
        		}
        		else if (data_from_computer[1] > 90){
        			right_speed = (90 - data_from_computer[1]) / 90 * SPEED_M3 * data_from_computer[0] / 100;
        			left_speed = (150 - data_from_computer[1]) / 30 * SPEED_M3 * data_from_computer[0] / 100 ;
        		}
        		else if (data_from_computer[1] < -90){
        			right_speed = (150 + data_from_computer[1]) / 30 * SPEED_M3 * data_from_computer[0] / 100;
        			left_speed = (-90 - data_from_computer[1]) / (-90) * SPEED_M3 * data_from_computer[0] / 100;
        		}
        		right_motor_set_speed(right_speed);
        		left_motor_set_speed(left_speed);
        	}
    	}
    	else {
    		if(compteur < 45){
    			compteur++;
    		}
    		else {
    			collision = false;
    			left_motor_set_speed(0);
    			right_motor_set_speed(0);
    			stopCurrentMelody();
    		}
    	}
    	chThdSleepMilliseconds(60);
    }
}






//-----------------------------GESTION THREADS----------------------------------------

void stop_thread(void){

	//arrete tous les threads actifs
	stop_thread_camera();
	if (thd_mode_0_IR != NULL){
		chThdTerminate(thd_mode_0_IR);
		chThdWait(thd_mode_0_IR);
		thd_mode_0_IR = NULL;
		stopCurrentMelody();
	}
	if (thd_m1_position != NULL){
		chThdTerminate(thd_m1_position);
		chThdWait(thd_m1_position);
		thd_m1_position = NULL;
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		stopCurrentMelody();
	}
	if (thd_mode_2 != NULL){
		chThdTerminate(thd_mode_2);
		chThdWait(thd_mode_2);
		thd_mode_2 = NULL;
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		stopCurrentMelody();
	}
	if (thd_mode_3_manette != NULL){
		chThdTerminate(thd_mode_3_manette);
		chThdWait(thd_mode_3_manette);
		thd_mode_3_manette = NULL;
		stopCurrentMelody();
	}
}

void run_thread_mode_0(void){
	thd_mode_0_IR = chThdCreateStatic(thd_m0_capteur_distance_wa, sizeof(thd_m0_capteur_distance_wa), NORMALPRIO, thd_m0_capteur_distance, NULL);
//	playMelody(MARIO, ML_SIMPLE_PLAY, NULL);
}


void run_thread_mode_1(void){
	process_image_start();
	thd_m1_position = chThdCreateStatic(thd_m1_camera_wa, sizeof(thd_m1_camera_wa), NORMALPRIO, thd_m1_camera, NULL);
//	playMelody(PIRATES_OF_THE_CARIBBEAN, 0, NULL);

}


void run_thread_mode_2(void){
	process_image_start();
	thd_mode_2 = chThdCreateStatic(thd_m2_camera_wa, sizeof(thd_m2_camera_wa), NORMALPRIO, thd_m2_camera, NULL);
//	playMelody(IMPOSSIBLE_MISSION, ML_FORCE_CHANGE, NULL); ////////////////////////////////////////////////////////////////////////////////
}


void run_thread_mode_3(void){
	thd_mode_3_manette = chThdCreateStatic(thd_m3_wa, sizeof(thd_m3_wa), NORMALPRIO, thd_m3, NULL);
}
