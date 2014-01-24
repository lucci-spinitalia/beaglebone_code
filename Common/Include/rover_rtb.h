/**
 * \file   lucciRTB.h
 * Author: erupter
 *
 * Created on October 25, 2012, 12:19 PM
 */


#ifndef LUCCIRTB_H
#define	LUCCIRTB_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/param.h>
#include "rover_service.h"

/** \defgroup ReturnToBase*/

/** \brief RTB operating mode EUCLIDEAN constant
*
* \ingroup ReturnToBase
*  Constant used in defining RTB's behavior, it is used
* in conjunction with the RTB_GEO_MODE variable
* @see RTB_GEO_MODE
* @see RTB_GEO_MODE_LATLON
*/
#define RTB_GEO_MODE_EUCLIDEAN      0
    
/** \brief RTB operating mode Geographical Coordinates constant
 *
 * \ingroup ReturnToBase      
 *  Constant used in defining RTB's behavior, it is used
 * in conjunction with the RTB_GEO_MODE variable
 * @see RTB_GEO_MODE
 * @see RTB_GEO_MODE_EUCLIDEAN
 */
#define RTB_GEO_MODE_LATLON         1

/** \brief RTB Record-mode waypoint saving distance threshold
 *
 * \ingroup ReturnToBase      
 *  This value determines
 * the minimum distance between two consecutive points to use to
 * perform the automatic return-to-base. Points with relative
 * distance inferior to this value are discarded.
 * PLEASE NOTE: the software is unable to determine circular paths,
 * unless this value is such as to fully enclose the path.
 */
#define RTB_SAVE_DIST_TRSH          10.0

/** \brief RTB Record-mode adaptive saving distance, speed rate
 *
 * \ingroup ReturnToBase      
 *  If this option is enabled the above value of maximum distance between two
 * consecutive waypoints, is modulated by the vehicle actual longitudinal speed.
 * The speed is calculated with a simple difference quotient method.
 * Using this capability requires adapting the maximum and minimum vehicle speed
 * values accordingly.
 * @see RTB_SAVE_DIST_TRSH
 * @see RTB_SAVE_DIST_ADAPTIVE_ANGULAR
 * @see RTB_SAVE_DIST_ADAPTIVE_MAX_SPEED
 * @see RTB_SAVE_DIST_ADAPTIVE_MIN_SPEED
 */
#define RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL 1 //bool, either 1 or 0

/** \brief RTB Record-mode adaptive saving distance, angular rate
 *
 * \ingroup ReturnToBase      
 *  When this option is enabled, the distance between two consecutive waypoints is
 * modulated according to the angular speed of the vehicle. Meaning the faster the
 * turns, the more points will be used to store the trajectory.
 * @see RTB_SAVE_DIST_TRSH
 * @see RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL
 */
#define RTB_SAVE_DIST_ADAPTIVE_ANGULAR      1 //bool, either 1 or 0

/** \brief RTB Record-mode adaptive saving distance, maximum speed
 *
 * \ingroup ReturnToBase      
 *  The maximum speed the vehicle is capable of. Used internally to normalize the
 * distance between consecutive waypoints.
 * @see RTB_SAVE_DIST_TRSH
 * @see RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL
 */
#define RTB_SAVE_DIST_ADAPTIVE_MAX_SPEED    1 //in meters per second: was 2.0

/** \brief RTB Record-mode adaptive saving distance, minimum speed
 *
 * \ingroup ReturnToBase      
 *  The minimum speed the vehicle is capable of. Used internally to normalize the
 * distance between consecutive waypoints.
 * @see RTB_SAVE_DIST_TRSH
 * @see RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL
 */
#define RTB_SAVE_DIST_ADAPTIVE_MIN_SPEED    0.0 //in meters per second: was 0.2


/** \brief RTB Record-mode adaptive saving distance, angular sensitivity
 *
 * \ingroup ReturnToBase      
 *  This value controls how intense the sensitivity to the angular rate is.
 * The higher the value the higher the number of stored waypoints during turns.
 * @see RTB_SAVE_DIST_TRSH
 * @see RTB_SAVE_DIST_ADAPTIVE_ANGULAR
 */
#define RTB_SAVE_DIST_ADAPTIVE_ANGULAR_SENSITIVITY 4 //absolute


/** \brief RTB Record-mode adaptive saving distance, maximum angular speed
 *
 * \ingroup ReturnToBase      
 *  The maximum speed the vehicle is capable of turning at. Used internally to normalize the
 * distance between consecutive waypoints.
 * @see RTB_SAVE_DIST_TRSH
 * @see RTB_SAVE_DIST_ADAPTIVE_ANGULAR
 */
#define RTB_SAVE_DIST_ADAPTIVE_MAX_ANGULAR_SPEED    2 //in radians per second

/** \brief RTB Record-mode waypoint flushing distance threshold
 *
 * \ingroup ReturnToBase      
 *  During recording the algorithm attempts to optimize the cache of recorded waypoints.
 * This distance is the threshold to which every old point is confronted: if an already known
 * waypoint is found that passes this threshold, the chain of waypoints saved between the actual
 * position and the found one will be deleted. (circular path detection attempt)
 */
#define RTB_FLUSH_DIST_TRSH         1.0


/** \brief RTB Track-mode waypoint change threshold
 *
 * \ingroup ReturnToBase      
 *  This value determines the distance from the current waypoint at which the
 * auto-guidance switches to the next waypoint.
 */
#define RTB_GUIDE_CHANGE_DIST_TRSH  0.5

        
/** \brief RTB Track-mode global update frequency
 *
 * \ingroup ReturnToBase      
 *  This represents the decimator used to to perform a global update of all distances
 * of all cached waypoints. The base frequency is represented by the super-loop, thus
 * it is not know to the algorithm. It just chops down the number of times ti gets
 * called by this number, thus performing the global update at a frequency equal to
 * SUPERLOOP-CALLS / RTB_GUIDE_UPDATE_TIME_TRSH
 */        
#define RTB_GUIDE_UPDATE_TIME_TRSH  10
    
    
/** \brief RTB operating mode selection
 *
 * \ingroup ReturnToBase      
 *  This value takes either of the RTB_GEO_MODE constants to shape
 * the software's behavior.
 * @see RTB_GEO_MODE_EUCLIDEAN
 * @see RTB_GEO_MODE_LATLON
 */    
#define RTB_GEO_MODE                RTB_GEO_MODE_LATLON
    
    
/** \brief RTB generalized Earth radius in Km
 *
 * \ingroup ReturnToBase      
 *  Generalized Earth radius used in mathematical operations
 */
#define EARTH                       6371
    
    
/** \brief RTB minimum distance between waypoints
 *
 * \ingroup ReturnToBase      
 *  Minimum distance between waypoints
 */
#define RTB_MIN_WAYPOINT_DIST       1.0 // meters
    
        
 /** @struct RTB_point
 *  @brief Waypoint struct (Cartesian)
 * \ingroup ReturnToBase
 * 
 * Used in a list, each element holds essential information for the waypoints acquired during operation.
 *  @param x Member 'x' contains the X value of the point.
 *  @param y Member 'y' contains the Y value of the point.
 *  @param distance_from_start Member 'distance_from_start' contains the distance of the point from the origin.
 *  @param next Member 'next' contains a pointer to the next waypoint (farther from the origin).
 *  @param previous Member 'previous' contains a pointer to the previous waypoint (nearer to the origin).
 */
typedef struct RTB_point{
    RTB_FLOAT_TYPE x;                   /**< RTB_FLOAT_TYPE contains the X value of the point. */ 
    RTB_FLOAT_TYPE y;                   /**< RTB_FLOAT_TYPE contains the Y value of the point. */ 
    RTB_FLOAT_TYPE distance_from_start; /**< RTB_FLOAT_TYPE contains the distance of the point from the origin. */ 
    struct RTB_point* next;             /**< RTB_point* contains a pointer to the next waypoint (farther from the origin). */ 
    struct RTB_point* previous;         /**< RTB_point* contains a pointer to the previous waypoint (nearer to the origin). */ 
    
} RTB_point;
    
/** \brief RTB initialization call
 *
 * \ingroup ReturnToBase      
 *  This function must be called when the module is loaded, it sets the internal status to a known state.
 * It is not necessary to call this function any further after initialization.
 */
void RTB_init(void);

/** \brief RTB mode selection
 *
 * \ingroup ReturnToBase      
 *  Used in conjunction with RTB_mode
 * @see RTB_mode
 * 
 * Parent program must actively call this function to change operating mode of the module.
 */
void RTB_set_mode(RTB_mode);

/** \brief RTB latitude and longitude traslation
 *
 * \ingroup ReturnToBase      
 *  This function traslate all point into the list. This is useful when the first point 
 * has been catched by odomery with position unknown, so lon and lat start by zero. 
 * As soon as get actual position we have to tralate the previous points.
 * @param new_lat actual lon
 * @param new_lon actual lat
 * @return void
 * @see RTB_update
 */
void RTB_traslate_point(RTB_FLOAT_TYPE new_lat, RTB_FLOAT_TYPE new_lon, RTB_point *new_point);

/** \brief RTB main cycle handler
 *
 * \ingroup ReturnToBase      
 *  This function must be periodically called to guarantee correct recording of positions.
 * It is designed to be integrated in a superloop.
 * If the module is disabled, calling this function won't affect performance, as no operations
 * will be performed in such a case (the module will check its state and immediately return).
 * @param localx value either euclidean or lon
 * @param localy value either euclidean or lat
 * @param xspeed
 * @param aspeed
 * @return a pointer to the current status
 * @see RTB_status, RTB_GEO_MODE
 */
int RTB_update(RTB_FLOAT_TYPE localx, RTB_FLOAT_TYPE localy, RTB_FLOAT_TYPE xspeed, RTB_FLOAT_TYPE aspeed, unsigned char *point_catch);

//#if (RTB_GEO_MODE == RTB_GEO_MODE_EUCLIDEAN)
/** \brief Clean cache
 *
 * \ingroup ReturnToBase      
 *  This function must be called on program exit to clean
 *  cache
 */
void RTB_internal_clean_cache(void);

//#elif (RTB_GEO_MODE == RTB_GEO_MODE_LATLON)

/** @struct RTB_point
 *  @brief Waypoint struct (Lat/Lon)
 *  @var RTB_point::lat 
 *  Member 'lat' contains the Latitude value of the point.
 *  @var RTB_point::lon
 *  Member 'lon' contains the Longitude value of the point.
 *  @var RTB_point::distance_from_start
 *  Member 'distance_from_start' contains the distance of the point from the origin.
 *  @var RTB_point::next
 *  Member 'next' contains a pointer to the next waypoint (farther from the origin).
 *  @var RTB_point::previous
 *  Member 'previous' contains a pointer to the previous waypoint (nearer to the origin).
 */
/*typedef struct RTB_point{
    RTB_FLOAT_TYPE lat;
    RTB_FLOAT_TYPE lon;
    RTB_FLOAT_TYPE distance_from_start;
    struct RTB_point* next;
    struct RTB_point* previous;
    
} RTB_point;

#else
#error ("No geo mode specified in lucciRTB.h")
#endif*/

/**
 * @brief Debug define. Set to 1 to enable.
 * \ingroup ReturnToBase  
 */
#define DEBUG   0

/**
 * @brief Simulator define. Set to 1 to enable.
 * \ingroup ReturnToBase  
 * 
 */
#define SIMUL   0

#ifdef	__cplusplus
}
#endif

#endif	/* LUCCIRTB_H */

