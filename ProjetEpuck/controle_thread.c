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

//define général
#define LED_RGB_RED 			100, 0, 0
#define LED_RGB_BLUE			0, 0, 100
#define LED_RGB_OFF				0, 0, 0
#define ON 						1
#define OFF 					0
#define ARRET_MOTOR				0
#define WORKSPACE_AREA			1024
#define WORKSPACE_AREA_EXTENDED	2048
#define NB_CAPTEUR				8

//define mode 0
#define DIST_THRESHOLD_JEU_M0 	100

//define mode 1
#define DIST_THRESHOLD_M1 		220
#define SPEED_M1 				400
#define ROTATION_COEFF_M1 		2.2
#define WIDTH_THRESHOLD_M1 		100

//define mode 2
#define SPEED_M2 				600
#define SPEED_ACC_M2 			800
#define ROTATION_COEFF_M2 		2.7
#define DIST_THRESHOLD_M2		220

//define mode 3
#define NB_ELEMENTS_CONTROLLER 	4
#define SPEED_M3 				700
#define SPEED_BOOST_M3			1100
#define SPEED_DEATH_M3 			700
#define TEMPS_ROTA_DEATH_M3 	21
#define DIST_THRESHOLD_M3		300
#define PAS_LED_BOOST_ON		2
#define TMPS_BOOST_ON_M3 		9*PAS_LED_BOOST_ON
#define PAS_LED_BOOST_LOAD		5
#define TMPS_LOAD_BOOST_M3		9*PAS_LED_BOOST_LOAD
typedef enum {
	MODULE_JOYSTICK = 0,
	ANGLE_JOYSTICK,
	BOUTON_A,
	GACHETTE
} indices_param_controller;



void pause_thread_manette(void);

static bool run_manette = false;


//thread du mode 0 :
static thread_t* thd_mode_0_IR = NULL;

//thread du mode 1 :
static thread_t* thd_m1_position = NULL;

//thread du mode 2 :
static thread_t* thd_mode_2 = NULL;

//-------------------------------MODE 0-----------------------------------------------------------
/*Thread propre au mode 0 : attente
 * 	-> petit jeu avec LEDs et capteurs de distance.
 * 	-> (priorité normale)
 */


//thread qui utilise les capteurs de distance, et allume les leds les plus proches de l'objet détecté
static THD_WORKING_AREA(thd_m0_capteur_distance_wa, WORKSPACE_AREA);
static THD_FUNCTION(thd_m0_capteur_distance, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    calibrate_ir();

    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
        //Détection obstacles frontaux
    	uint8_t indice_capteur = NB_CAPTEUR;
    	uint16_t temp_val_max = 0;
    	//boucle ne prend pas en compte les deux capteurs frontaux car ils sont bloqués par le miroir
    	for (uint8_t i = 1; i < NB_CAPTEUR - 1; i++){
    		if (get_prox(i) > temp_val_max && get_prox(i) > DIST_THRESHOLD_JEU_M0){
    			temp_val_max = get_prox(i);
    			indice_capteur = i;
    		}
    	}
		switch(indice_capteur){
			case 1 :
				clear_leds();
				set_rgb_led(LED2, LED_RGB_RED);
				break;
			case 2 :
				clear_leds();
				set_led(LED3, ON);
				break;
			case 3 :
				clear_leds();
				set_rgb_led(LED4, LED_RGB_BLUE); //led rouge ne fonctionne pas tout le temps
				set_led(LED5, ON);
				break;
			case 4 :
				clear_leds();
				set_rgb_led(LED6, LED_RGB_RED);
				set_led(LED5, ON);
				break;
			case 5 :
				clear_leds();
				set_led(LED7, ON);
				break;
			case 6 :
				clear_leds();
				set_rgb_led(LED8, LED_RGB_RED);
				break;
			default :
				clear_leds();
				break;
    	}
      	chThdSleepMilliseconds(100);
    }
}



//-------------------------------MODE 1-----------------------------------------------------------
/* Thread propre au mode 1 : conduite respectueuse
 *  -> Détection des obstacles
 *  -> Suivie de la ligne noire
 *  -> (priorité normale)
 */

static THD_WORKING_AREA(thd_m1_camera_wa, WORKSPACE_AREA_EXTENDED);
static THD_FUNCTION(thd_m1_camera, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    int16_t speed_correction = 0;
    //boucle infinie du thread
    while(chThdShouldTerminateX() == false){
        if((get_prox(0) > DIST_THRESHOLD_M1) || (get_prox(7) > DIST_THRESHOLD_M1)) {
      		//Arret
      		left_motor_set_speed(ARRET_MOTOR);
      		right_motor_set_speed(ARRET_MOTOR);
      		set_body_led(ON);
      	}
      	else{
      		set_body_led(OFF);
            //computes a correction factor to let the robot rotate to be in front of the line
            speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

            //if the line is nearly in front of the camera, don't rotate
            if(abs(speed_correction) < ROTATION_THRESHOLD){
            	speed_correction = 0;
            }
            if(get_line_width() > WIDTH_THRESHOLD_M1){
                //applies the speed and the correction for the rotation
        		right_motor_set_speed(SPEED_M1 - ROTATION_COEFF_M1 * speed_correction);
        		left_motor_set_speed(SPEED_M1 + ROTATION_COEFF_M1 * speed_correction);
            }
            else{
          		left_motor_set_speed(ARRET_MOTOR);
          		right_motor_set_speed(ARRET_MOTOR);
            }
      	}
		chThdSleepMilliseconds(50);
    }
}



//-------------------------------MODE 2-----------------------------------------------------------
/* Thread propre au mode 2 : conduite Tom Cruise
 * -> Suivi de la ligne noire
 * (réutilisation du thread du mode 1 mais avec une vitesse du robot plus élevée)
 *  -> (priorité normale)
 */


static THD_WORKING_AREA(thd_m2_camera_wa, WORKSPACE_AREA);
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
    	if((get_prox(0) > DIST_THRESHOLD_M2) || (get_prox(7) > DIST_THRESHOLD_M2)) {
      		//Acceleration
        	speed_left = SPEED_ACC_M2;
        	speed_right = SPEED_ACC_M2;
        	set_body_led(ON);
      	}
        else{
        	set_body_led(OFF);
        }
		//computes a correction factor to let the robot rotate to be in front of the line
		speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

		//if the line is nearly in front of the camera, don't rotate
		if(abs(speed_correction) < ROTATION_THRESHOLD){
			speed_correction = 0;
		}
		if(get_line_width() > WIDTH_THRESHOLD_M1){
			//applies the speed and the correction for the rotation
			right_motor_set_speed(speed_right - ROTATION_COEFF_M2 * speed_correction);
			left_motor_set_speed(speed_left + ROTATION_COEFF_M2 * speed_correction);
		}
		else {
			left_motor_set_speed(ARRET_MOTOR);
			right_motor_set_speed(ARRET_MOTOR);
		}
		chThdSleepMilliseconds(50);
    }
}




//-------------------------------MODE 3-----------------------------------------------------------
/* Threads propre au mode 3 : contrôle à la manette
 *  -> Détection des obstacle
 *  -> Réception manette
 *  (priorité normale)
 */



//thread qui utilise les capteurs de distance pour faire un tour sur lui meme quand il rencontre un obstacle en mode manette
static THD_WORKING_AREA(thd_m3_wa, WORKSPACE_AREA_EXTENDED);
static THD_FUNCTION(thd_m3, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    bool collision = false;
    uint8_t compteur = 0;
	int16_t right_speed = 0;
	int16_t left_speed = 0;
	uint16_t coeff_speed = 0;
	bool boost_ready = false;
	bool boost_on = false;
	uint8_t timer_boost = 0;

    //boucle infinie du thread
    while(1){
    	if (run_manette){
        	float data_from_computer[NB_ELEMENTS_CONTROLLER] = {0, 0, 0, 0};
        	/*
        	 * data_from_computer contient les données envoyées par la manette en bluetooth (via le script python "get_controller.py"):
        	 * [MODULE_JOYSTICK] : controle la vitesse
        	 * [ANGLE_JOYSTICK] : fait tourner le robot
        	 * [BOUTON_A] : active/désactive les capteurs de distance
        	 * [GACHETTE] : lance le boost
        	 */
        	//réception des données de la manette via script python et bluetooth
        	uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, data_from_computer, NB_ELEMENTS_CONTROLLER);
        	//detection collision
        	if((get_prox(7) > DIST_THRESHOLD_M3 || get_prox(0) > DIST_THRESHOLD_M3) && !collision && !data_from_computer[BOUTON_A]) {
        		collision = true;
        		//Instruction en cas de collision
        		set_body_led(ON);
        		playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL);
        		left_motor_set_speed(SPEED_DEATH_M3);
        		right_motor_set_speed(-SPEED_DEATH_M3);
        		compteur = 0;
        	}
        	//Affichage led pour l'activation des détecteurs de proximité
        	if(data_from_computer[BOUTON_A] == 1) set_front_led(ON);
        	if (data_from_computer[BOUTON_A] == 0) set_front_led(OFF);

        	if(!collision) {
        		set_body_led(OFF);
            	if(size == NB_ELEMENTS_CONTROLLER){
            		//prise en compte du boost ou vitesse normale
            		coeff_speed = SPEED_M3;
            		if(boost_ready && (data_from_computer[GACHETTE] == 1 || data_from_computer[GACHETTE] == 2)){
            			boost_ready = false;
            			boost_on = true;
            			timer_boost = 0;
            		}
            		if(boost_on){
            			coeff_speed = SPEED_BOOST_M3;
            		}

            		//ramène la valeur de l'angle du joystick entre -180 et 180
            		data_from_computer[ANGLE_JOYSTICK] -= 180;

            		/*
            		 * calcul des vitesses des deux roues en fonction de l'angle du joystick et de son amplitude
            		 * -> 3 zones distinctes (selon angle du joystick) : entre -90° et 90°, supérieur à 90°, et inférieur à -90°
            		 * vitesse = coeff en fonction de l'angle du joystick (valeur entre -1 et 2, calcul pour chaque roue) * constante de vitesse * amplitude du joystick nomralisée
            		 */
            		if (data_from_computer[ANGLE_JOYSTICK] > -90 && data_from_computer[ANGLE_JOYSTICK] < 90){
            			right_speed = (90 - data_from_computer[ANGLE_JOYSTICK]) / 90 * coeff_speed * data_from_computer[MODULE_JOYSTICK] / 100;
            			left_speed = (-90 - data_from_computer[ANGLE_JOYSTICK]) / (-90) * coeff_speed * data_from_computer[MODULE_JOYSTICK] / 100;
            		}
            		else if (data_from_computer[ANGLE_JOYSTICK] > 90){
            			right_speed = (90 - data_from_computer[ANGLE_JOYSTICK]) / 90 * coeff_speed * data_from_computer[MODULE_JOYSTICK] / 100;
            			left_speed = (150 - data_from_computer[ANGLE_JOYSTICK]) / 30 * coeff_speed * data_from_computer[MODULE_JOYSTICK] / 100 ;
            		}
            		else if (data_from_computer[ANGLE_JOYSTICK] < -90){
            			right_speed = (150 + data_from_computer[ANGLE_JOYSTICK]) / 30 * coeff_speed * data_from_computer[MODULE_JOYSTICK] / 100;
            			left_speed = (-90 - data_from_computer[ANGLE_JOYSTICK]) / (-90) * coeff_speed * data_from_computer[MODULE_JOYSTICK] / 100;
            		}
            		right_motor_set_speed(right_speed);
            		left_motor_set_speed(left_speed);
            	}
        	}

        	//gestion du temps en cas de collision
        	else {
        		if(compteur < TEMPS_ROTA_DEATH_M3){
        			compteur++;
        		}
        		else {
        			collision = false;
        			left_motor_set_speed(ARRET_MOTOR);
        			right_motor_set_speed(ARRET_MOTOR);
        			stopCurrentMelody();
        		}
        	}

        	//Affichage led du chargement du boost
        	if(!boost_ready && !boost_on){
        		timer_boost++;
            	switch(timer_boost){
            		case PAS_LED_BOOST_LOAD:
            			set_led(LED1, ON);
            			break;
            		case 2*PAS_LED_BOOST_LOAD:
            			set_rgb_led(LED2, LED_RGB_RED);
            			break;
            		case 3*PAS_LED_BOOST_LOAD:
            			set_led(LED3, ON);
            			break;
            		case 4*PAS_LED_BOOST_LOAD:
            			set_rgb_led(LED4, LED_RGB_RED);
            			break;
            		case 5*PAS_LED_BOOST_LOAD:
            			set_led(LED5, ON);
            			break;
            		case 6*PAS_LED_BOOST_LOAD:
            			set_rgb_led(LED6, LED_RGB_RED);
            			break;
            		case 7*PAS_LED_BOOST_LOAD:
            			set_led(LED7, ON);
            			break;
            		case 8*PAS_LED_BOOST_LOAD:
            			set_rgb_led(LED8, LED_RGB_RED);
            			break;
            		case TMPS_LOAD_BOOST_M3:
                		timer_boost = 0;
                		boost_ready = true;
                		set_rgb_led(LED2, LED_RGB_BLUE);
                		set_rgb_led(LED4, LED_RGB_BLUE);
                		set_rgb_led(LED6, LED_RGB_BLUE);
                		set_rgb_led(LED8, LED_RGB_BLUE);
                		break;
            		default:
            			break;
            	}
        	}
        	//Affichage led lors du l'utilisation du boost
        	if(boost_on){
        		timer_boost++;
            	switch(timer_boost){
            		case PAS_LED_BOOST_ON:
            			set_rgb_led(LED8, LED_RGB_OFF);
            			break;
            		case 2*PAS_LED_BOOST_ON:
            			set_led(LED7, OFF);
            			break;
            		case 3*PAS_LED_BOOST_ON:
            			set_rgb_led(LED6, LED_RGB_OFF);
            			break;
            		case 4*PAS_LED_BOOST_ON:
            			set_led(LED5, OFF);
            			break;
            		case 5*PAS_LED_BOOST_ON:
            			set_rgb_led(LED4, LED_RGB_OFF);
            			break;
            		case 6*PAS_LED_BOOST_ON:
            			set_led(LED3, OFF);
            			break;
            		case 7*PAS_LED_BOOST_ON:
            			set_rgb_led(LED2, LED_RGB_OFF);
            			break;
            		case 8*PAS_LED_BOOST_ON:
            			set_led(LED1, OFF);
            			break;
            		case TMPS_BOOST_ON_M3:
            			boost_on = false;
            			timer_boost = 0;
            			break;
            		default:
            			break;
            	}
        	}
        	chThdSleepMilliseconds(60);
        }
    	else {
    		//remise a 0 de toutes les valeurs quand le thread est en pause
    	    collision = false;
    	    compteur = 0;
    		coeff_speed = 0;
    		boost_ready = false;
    		boost_on = false;
    		timer_boost = 0;
    		chThdSleepMilliseconds(200);
    	}
    }
}






//-----------------------------GESTION THREADS----------------------------------------

void stop_thread(void){

	//arrete tous les threads actifs
	pause_thread_camera();
	pause_thread_manette();
	if (thd_mode_0_IR != NULL){
		chThdTerminate(thd_mode_0_IR);
		chThdWait(thd_mode_0_IR);
		thd_mode_0_IR = NULL;
	}
	if (thd_m1_position != NULL){
		chThdTerminate(thd_m1_position);
		chThdWait(thd_m1_position);
		thd_m1_position = NULL;
	}
	if (thd_mode_2 != NULL){
		chThdTerminate(thd_mode_2);
		chThdWait(thd_mode_2);
		thd_mode_2 = NULL;
	}

	//remise à 0 des leds/moteurs
	clear_leds();
	stopCurrentMelody();
	set_body_led(OFF);
	set_front_led(OFF);
	right_motor_set_speed(ARRET_MOTOR);
	left_motor_set_speed(ARRET_MOTOR);
}

void run_thread_mode_0(void){
	thd_mode_0_IR = chThdCreateStatic(thd_m0_capteur_distance_wa, sizeof(thd_m0_capteur_distance_wa), NORMALPRIO, thd_m0_capteur_distance, NULL);
}


void run_thread_mode_1(void){
	start_thread_camera();
	thd_m1_position = chThdCreateStatic(thd_m1_camera_wa, sizeof(thd_m1_camera_wa), NORMALPRIO, thd_m1_camera, NULL);
}


void run_thread_mode_2(void){
	start_thread_camera();
	thd_mode_2 = chThdCreateStatic(thd_m2_camera_wa, sizeof(thd_m2_camera_wa), NORMALPRIO, thd_m2_camera, NULL);
//	playMelody(IMPOSSIBLE_MISSION, ML_FORCE_CHANGE, NULL);
}


void start_thread_mode_3(void){
	chThdCreateStatic(thd_m3_wa, sizeof(thd_m3_wa), NORMALPRIO, thd_m3, NULL);
}

void run_thread_manette(void){
	run_manette = true;
}

void pause_thread_manette(void){
	run_manette = false;
}
