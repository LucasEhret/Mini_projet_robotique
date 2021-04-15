#include <controle_thread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"
#include <main.h>

//thread qui utilise les capteurs de distance, et allume les leds les plus proches -> mode 0, rien d'autre se passe
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


//thread qui utilise les capteurs de distance pour faire un tour sur lui meme quand il rencontre un obstacle en mode manette
static THD_WORKING_AREA(thd_m3_capteur_distance_wa, 1024);
static THD_FUNCTION(thd_m3_capteur_distance, arg)
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



void stop_thread(thread_t* thd_to_stop){
	chThdTerminate(thd_to_stop);
	chThdWait(thd_to_stop);
	thd_to_stop = NULL;
}
