/** 
 * \file   lucciOA.h
 * Author: erupter
 *
 * Created on October 29, 2012, 12:01 PM
 */

#ifndef LUCCIOA_H
#define	LUCCIOA_H

#ifdef	__cplusplus
extern "C" {
#endif

    
#include <malloc.h>
#include <math.h>
#include <sys/param.h>
#include "rover_service.h"

/** \defgroup ObstacleAvoidance */
    
/**
 * @brief Custom PI definition
 * 
 * \ingroup ObstacleAvoidance  
 */
#define PI 3.14159265

#define OA_SENSOR_CLOCKWISE             0
#define OA_SENSOR_ANTI_CLOCKWISE        1
    
//#define OA_GRID_RESOLUTION              0.1 //in meters        

/** \brief Active distance
 *
 * \ingroup ObstacleAvoidance        
 *  Meters. Distance at which the obstacle avoidance becomes active.
 * 
 */     
#define OA_ACTIVE_DISTANCE              7000 //in mm
    
/** \brief Grid radial sampling
 *
 * \ingroup ObstacleAvoidance        
 *  Adimensional. Maximum intensity of the detected occupation of the sector to allow attempting to consider the direction viable.
 * 
 */     
#define OA_GRID_RADIAL_SAMPLING         5 //in degrees
    
/** \brief Sensor resolution
 *
 * \ingroup ObstacleAvoidance        
 *  Radians. Incremental radial distance between consecutive laser samples.
 * 
 */     
#define OA_SENSOR_RESOLUTION            0.008722222 //in radians   
    
/** \brief Sensor center
 *
 * \ingroup ObstacleAvoidance        
 *  In radians. The center of the laser readings relative to the rover.
 * If a 180° FOV laser is used, than most probably this is 0.
 * 
 */     
#define OA_SENSOR_CENTER                0.0   //in radians
    
/** \brief Sensor count
 *
 * \ingroup ObstacleAvoidance        
 *  Number of readings the laser outputs.
 * 
 */     
//#define OA_SENSOR_COUNT                 140 // number of laser readings

/** \brief Window Threshold
 *
 * \ingroup ObstacleAvoidance        
 *  Adimensional. Maximum intensity of the detected occupation of the sector to allow attempting to consider the direction viable.
 * 
 */     
#define OA_WINDOW_THRESHOLD             5 // value at which to consider a space viable
    
/** \brief Window Size
 *
 * \ingroup ObstacleAvoidance        
 *  Adimensional. Number of sequencial radial sectors that must be free to consider that a viable direction.
 * 
 */     
#define OA_WINDOW_SIZE                  5 // minimum number of consecutive under threshold values to consider a window
    
/** \brief Laser sensor data sequence (CW/CCW)
 *
 * \ingroup ObstacleAvoidance        
 *  Set to reflect how the laser scanner reports its data: CW (human) or CCW (mathematical)
 * 
 */    
#define OA_SENSOR_ORIENTATION          OA_SENSOR_ANTI_CLOCKWISE// Or OA_SENSOR_CLOCKWISE
    
/** \brief Maximum Speed
 *
 * \ingroup ObstacleAvoidance        
 *  Maximum speed the OA algorithm will attempt to attain. In m/s.
 * 
 */    
#define OA_SPEED_MAX                    2.0 // meters per second
/** \brief Minimum Speed
 *
 * \ingroup ObstacleAvoidance        
 *  Minimum speed the OA algorithm will attempt to attain. In m/s.
 * 
 */     
#define OA_SPEED_MIN                    0.1 // meters per second
/** \brief Speed moderation active distance
 *
 * \ingroup ObstacleAvoidance        
 *  The distance from any detected obstacle at which the algorithm will start moderating speed.
 * 
 */     
#define OA_SPEED_MODERATION_DIST        3.0 // meters
/** \brief Safety distance
 *
 * \ingroup ObstacleAvoidance        
 *  Minimum DO-NOT-CROSS distance from any obstacle: should this
 * value be approached forward speed will diminish until it get nullified.
 * 
 */     
#define OA_SAFETY_DISTANCE              1000 // mm
/** \brief Avoiding Intensity
 *
 * \ingroup ObstacleAvoidance        
 * Adimensional multiplier used to tweak how the algorithm performs.
 * The base calculation remains the same, this value just allows to augment the strength
 * of the algorithm's response allowing for a stronger repulsion.
 * 
 */     
#define OA_AVOID_INTENSITY              2.0 // adimensional, multiplier of the avoiding vector



/** \brief OA Init
 *
 * \ingroup ObstacleAvoidance        
 *  Initializes the algorithm. Mandatory execution before calling its functions.
 * @param range_deg scan range for the laser in degree
 */     
void OA_init(float range_deg);
void OA_cleanup(void);

/** \brief Perform avoidance
 *
 * \ingroup ObstacleAvoidance        
 *  This function must be periodically called to guarantee correct recording of positions.
 * It is designed to be integrated in a superloop.
 * If the module is disabled, calling this function won't affect performance, as no operations
 * will be performed in such a case (the module will check its state and immediately return).
 * @param readings vector of RTB_FLOAT_TYPE values carrying the laser readings
 * @param count long variable containing the number of laser readings in a vector
 * @param yaw RTBvector containing the current yaw (true north) of the vehicle
 * @param desired_heading RTBvector containing the desired heading
 * @return an RTBvector with the actual commands (heading, speed)
 * @see RTBvector
 */   
RTBvector OA_perform_avoidance(unsigned int readings[], long count, RTBvector current_direction, RTBvector desired_heading);

/**
 * @brief Simulator define. Set to 1 to enable.
 * \ingroup ObstacleAvoidance    
 * This requires Player 3 and Stage 4 to be installed on the system.
 */
#define SIMUL   0

#ifdef	__cplusplus
}
#endif

#endif	/* LUCCIOA_H */

