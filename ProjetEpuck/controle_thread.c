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

#define NB_ELEMENTS_CONTROLLER 4



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

    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
    	/*
    	 * fonction a remplir
    	 */
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
static THD_WORKING_AREA(thd_camera__wa, 1024);
static THD_FUNCTION(thrd_camera, arg)
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

    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
    	/*
    	 * fonction a remplir
    	 */
    }
}



//-------------------------------MODE 2-----------------------------------------------------------
/* Threads propre au mode 2 : conduite Tom Cruise
 * -> Suivi de la ligne noire (priorité normale)
 *               (réutilisation du thread du mode 1 mais avec une vitesse du robot plus élevée)
 */


//thread qui utilise les capteurs de distance pour faire un tour sur lui meme quand il rencontre un obstacle en mode manette
static THD_WORKING_AREA(thd_m3_capteur_distance_wa, 1024);
static THD_FUNCTION(thd_m3_capteur_distance, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

//    set_led(LED1, 1);

    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
//    	set_led(LED3, 1);
    	short data_from_computer[] = {0, 0, 0, 0};
    	uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, data_from_computer, NB_ELEMENTS_CONTROLLER);
    	if(size == NB_ELEMENTS_CONTROLLER){
    		data_from_computer[1] -= 180;
    		int right_speed = (90-data_from_computer[1]) / 90 * 330 * data_from_computer[0] / 100;
    		int left_speed = (-90-data_from_computer[1]) / (-90) * 330 * data_from_computer[0] / 100;
    		right_motor_set_speed(right_speed);
    		left_motor_set_speed(left_speed);
    	}
//    	chThdSleepMilliseconds(40);
    }
}



//-------------------------------MODE 3-----------------------------------------------------------
/* Threads propre au mode 3 : contrôle à la manette
 *  -> Détection des obstacle (priorité +)
 *  -> Réception manette      (priorité normale)
 *
 */






//-----------------------------GESTION THREADS----------------------------------------

void stop_thread(thread_t* thd_to_stop){
	chThdTerminate(thd_to_stop);
	chThdWait(thd_to_stop);
	thd_to_stop = NULL;
}


void run_thread_manette(void){

//    set_body_led(2);
	chThdCreateStatic(thd_m3_capteur_distance_wa, sizeof(thd_m3_capteur_distance_wa), NORMALPRIO, thd_m3_capteur_distance, NULL);
}
