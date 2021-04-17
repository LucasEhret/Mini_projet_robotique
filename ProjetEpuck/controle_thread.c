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

#define NB_ELEMENTS_CONTROLLER 4
#define DIST_THRESHOLD 15
#define DIST_THRESHOLD_JEU 6

//threads du mode 0 :
static thread_t* thd_mode_0 = NULL;
static thread_t* thd_mode_0_IR = NULL;

//threads du mode 1 :
static thread_t* thd_mode_1 = NULL;
static thread_t* thd_mode_1_IR = NULL;

//threads du mode 2 :
static thread_t* thd_mode_2 = NULL;

//threads du mode 3 :
static thread_t* thd_mode_3_manette = NULL;
static thread_t* thd_mode_3_IR = NULL;
static thread_t* thd_mode_3_musique = NULL;




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
    	uint8_t indice_capteur = 10;
    	uint8_t temp_val_max = 0;
    	for (uint8_t i = 0; i < 8; i += 2){
    		if (get_prox(i) > temp_val_max && get_prox(i) > DIST_THRESHOLD_JEU){
    			temp_val_max = get_prox(i);
    			indice_capteur = i;
    		}
    	}
    	if(indice_capteur < 7){
    		set_led(indice_capteur, 1);
    	}
      	for (uint8_t j = 0; j < 4; j++){
      		if (j*2 != indice_capteur){
      			set_led(j, 0);
      		}
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
static THD_WORKING_AREA(thd_m1_camera__wa, 1024);
static THD_FUNCTION(thd_m1_camera, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
    	/*
    	 * fonction a remplir
    	 */
    }
}


//thread qui utilise les capteurs de distance pour eviter les obstacles sur le parcours
static THD_WORKING_AREA(thd_m1_capteur_distance_wa, 1024);
static THD_FUNCTION(thd_m1_capteur_distance, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    calibrate_ir();
    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
        //Détection obstacles frontaux
      	if((get_prox(0) > DIST_THRESHOLD) || (get_prox(7) > DIST_THRESHOLD)) {
      		//Arret
      		left_motor_set_speed(0);
      		right_motor_set_speed(0);
      		set_front_led(1);
      	}
      	else{
          	set_front_led(0);
      	}


      	chThdSleepMilliseconds(100);
    }
}



//-------------------------------MODE 2-----------------------------------------------------------
/* Threads propre au mode 2 : conduite Tom Cruise
 * -> Suivi de la ligne noire (priorité normale)
 *               (réutilisation du thread du mode 1 mais avec une vitesse du robot plus élevée)
 */


static THD_WORKING_AREA(thd_m2_camera__wa, 1024);
static THD_FUNCTION(thd_m2_camera, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
    	/*
    	 * fonction a remplir
    	 */
    }
}




//-------------------------------MODE 3-----------------------------------------------------------
/* Threads propre au mode 3 : contrôle à la manette
 *  -> Détection des obstacle (priorité +)
 *  -> Réception manette      (priorité normale)
 *
 */



//thread qui utilise les capteurs de distance pour faire un tour sur lui meme quand il rencontre un obstacle en mode manette
static THD_WORKING_AREA(thd_m3_capteur_distance_wa, 1024);
static THD_FUNCTION(thd_m3_capteur_distance, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);


    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
//    	set_led(LED3, 1);
    	float data_from_computer[4] = {0, 0, 0, 0};
    	uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, data_from_computer, NB_ELEMENTS_CONTROLLER);
    	if(size == NB_ELEMENTS_CONTROLLER){
    		data_from_computer[1] -= 180;
    		if (data_from_computer[1] > -90 && data_from_computer[1] < 90){
    			int right_speed = (90-data_from_computer[1]) / 90 * 330 * data_from_computer[0] / 100;
    			int left_speed = (-90-data_from_computer[1]) / (-90) * 330 * data_from_computer[0] / 100;
    			right_motor_set_speed(right_speed);
    			left_motor_set_speed(left_speed);
    		}
    		if (data_from_computer[1] < -90 || data_from_computer[1] > 90){
    			int left_speed = -330 * data_from_computer[0] / 100;
    			int right_speed = left_speed;
    			right_motor_set_speed(right_speed);
    			left_motor_set_speed(left_speed);
    		}
    	}
    	chThdSleepMilliseconds(60);
    }
}


static THD_WORKING_AREA(thd_m3_recep_manette__wa, 1024);
static THD_FUNCTION(thd_m3_recep_manette, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
    	/*
    	 * fonction a remplir
    	 */
    }
}





//-----------------------------GESTION THREADS----------------------------------------

void stop_thread(mode_robot mode_to_stop){
	switch(mode_to_stop) {
		case 0:
			if (thd_mode_0 != NULL){
				chThdTerminate(thd_mode_0);
				chThdWait(thd_mode_0);
				thd_mode_0 = NULL;
			}
			break;
		case 1:
			if (thd_mode_1 != NULL){
				chThdTerminate(thd_mode_1);
				chThdWait(thd_mode_1);
				thd_mode_1 = NULL;
			}
			break;
		case 2:
			if (thd_mode_2 != NULL){
				chThdTerminate(thd_mode_2);
				chThdWait(thd_mode_2);
				thd_mode_2 = NULL;
			}
			break;
		case 3:
			if (thd_mode_3_manette != NULL){
				chThdTerminate(thd_mode_3_manette);
//				chThdWait(thd_mode_3_manette);
				thd_mode_3_manette = NULL;
			}
			if (thd_mode_3_IR != NULL){
				chThdTerminate(thd_mode_3_IR);
				chThdWait(thd_mode_3_IR);
				thd_mode_3_IR = NULL;
			}
			break;
	}
}

void run_thread_mode_0(void){
	thd_mode_0_IR = chThdCreateStatic(thd_m0_capteur_distance_wa, sizeof(thd_m0_capteur_distance_wa), NORMALPRIO + 1, thd_m0_capteur_distance, NULL);
}


void run_thread_mode_1(void){
	thd_mode_1_IR = chThdCreateStatic(thd_m1_capteur_distance_wa, sizeof(thd_m1_capteur_distance_wa), NORMALPRIO + 1, thd_m1_capteur_distance, NULL);
}

void run_thread_mode_3(void){
	thd_mode_3_manette = chThdCreateStatic(thd_m3_capteur_distance_wa, sizeof(thd_m3_capteur_distance_wa), NORMALPRIO, thd_m3_capteur_distance, NULL);
}
