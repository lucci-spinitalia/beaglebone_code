/** 
 * \file   lucciSERVICE.h
 * Author: erupter
 *
 * Created on November 12, 2012, 2:08 PM
 */

#ifndef LUCCISERVICE_H
#define	LUCCISERVICE_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <math.h>
/** \defgroup Service */    
    
/** \brief RTB float type mask definition
 *
 * \ingroup Service      
 *  To make the software as much platform agnostic as possible, this
 * definition allows for custom remapping of the float type to
 * whatever deemed fit. Even integer types could be used, but this
 * would surely cause problems with the mathematical operations
 * involved.
 */ 
#define RTB_FLOAT_TYPE              double
    
/** \brief RTBvehicle_control
 *
 * \ingroup Service      
 *  Holds computed control values.
 * @param heading       a RTB_FLOAT_TYPE containing the commanded heading (true north)
 * @param speed         a RTB_FLOAT_TYPE containing the commanded speed (m/s)
 */    
typedef struct {
    RTB_FLOAT_TYPE heading;         /**< RTB_FLOAT_TYPE holding the heading value. */ 
    RTB_FLOAT_TYPE speed;           /**< RTB_FLOAT_TYPE holding the speed value. */ 
} RTBvehicle_control;

/** \brief Vector type
 *
 * \ingroup Service      
 *  Holds a Vector's various properties.
 * @param x                 RTB_FLOAT_TYPE holding the X value
 * @param y                 RTB_FLOAT_TYPE holding the Y value
 * @param angle_rad         RTB_FLOAT_TYPE holding the angle in radians (math orientation
 * @param angle_deg_north   RTB_FLOAT_TYPE holding the angle in degrees true north
 * @param norm              RTB_FLOAT_TYPE holding the norm of the vector
 */
typedef struct {
    RTB_FLOAT_TYPE x;               /**< RTB_FLOAT_TYPE holding the X value. */ 
    RTB_FLOAT_TYPE y;               /**< RTB_FLOAT_TYPE holding the Y value. */ 
    RTB_FLOAT_TYPE angle_rad;       /**< RTB_FLOAT_TYPE holding the angle in radians (math orientation. */ 
    RTB_FLOAT_TYPE angle_deg_north; /**< RTB_FLOAT_TYPE holding the angle in degrees true north. */ 
    RTB_FLOAT_TYPE norm;            /**< RTB_FLOAT_TYPE holding the norm of the vector. */ 
} RTBvector;

/** \brief RTB operating main mode enumeration
*
* \ingroup Service
*  Values used to control the behavior of the module.
* @see RTB_set_mode
*/    
typedef enum {
    RTB_idle = 0,                   /**< Default state upon initialization, no activity. */ 
    RTB_recording,                  /**< Active state, positions are recorded. */ 
    RTB_tracking,                   /**< Active state, cached positions are used to guide the vehicle. */ 
    RTB_pause                       /**< Paused state, cache is retained, no new positions are addes. */
}RTB_mode;


/** \brief RTB main status structure
 *
 * \ingroup Service      
 *  Contains status informations concerning the module, as well as the last
 * instructions issued to the system to reach the origin.
 * @param mode          an RTB_mode object depicting the state of the module
 * @param complete      a Boolean notifying of the completition of the assignment
 * @param control_values
 * @param distance      a RTB_FLOAT_TYPE containing the distance to the origin, a value of -1 means N/A
 * @param control_vector
 */
typedef struct RTB_status {
    RTB_mode mode;                  /**< RTB_mode depicting the state of the module. */ 
    int complete;                   /**< Boolean showing if the module has reached the origin. */ 
    RTBvehicle_control control_values; /**< struct holding command values */ 
    RTB_FLOAT_TYPE distance;        /**< Float value telling the system the distance to the nearest point. */ 
	RTB_FLOAT_TYPE distance_from_start; /**< Float value telling the system the distance to the start. */
    RTBvector control_vector;          /**< Vector holding the last control vector */
} RTB_status;
    

/** \brief Radians to Degrees North
 *
 * \ingroup Service*      
 *  Converts a value in radians to degrees true north.
 * @param angle          RTB_FLOAT_TYPE carrying the value to be modified
 */
RTB_FLOAT_TYPE lucciSERVICE_rad2degnorth(RTB_FLOAT_TYPE angle);

/** \brief Degrees North to Radians
 *
 * \ingroup Service      
 *  Converts a value in degrees true north to radians.
 * @param angle          RTB_FLOAT_TYPE carrying the value to be modified
 */
RTB_FLOAT_TYPE lucciSERVICE_degnorth2rad(RTB_FLOAT_TYPE angle);

/** \brief Radians adjust
 *
 * \ingroup Service      
 *  Performs a sanity check on an angle in radians.
 * Meaning it can't be less then -PI/2 or higher than +PI/2
 * @param angle          RTB_FLOAT_TYPE carrying the value to be modified
 */
RTB_FLOAT_TYPE lucciSERVICE_rad_adjust(RTB_FLOAT_TYPE angle);

/** \brief Degrees adjust
 *
 * \ingroup Service      
 *  Performs a sanity check on an angle in degrees.
 * Meaning it can't be less then 0° or higher than 359.9° (precision is up to the underlying value type)
 * @param angle          RTB_FLOAT_TYPE carrying the value to be modified
 */
RTB_FLOAT_TYPE lucciSERVICE_deg_adjust(RTB_FLOAT_TYPE angle);

/** \brief Vector normalize
 *
 * \ingroup Service      
 *  Useful after mathematical operations concerning X/Y values: recalculates the norm and makes the angle
 * values coherent with the vector values.
 * @param data          RTBvector carrying the value to be modified
 */
RTBvector lucciSERVICE_vect_normalize (RTBvector data);

/** \brief Vector sum
 * \ingroup Service
 * 
 *  Performs the sum between two vectors.   
 * Emphasis is on the X/Y values, norm and angles are calculated afterwards.
 * Thus summing vectors with missing X/Y values will result in undefined behaviour.
 * 
 * @param a      RTBvector carrying operand A
 * @param b      RTBvector carrying operand B
 */
RTBvector lucciSERVICE_vect_sum (RTBvector a, RTBvector b);

/** \brief Vector set norm
 *
 * \ingroup Service      
 *  Arbitrarily set the norm of a vector. Useful to modify the speed should
 * it be connected with the length of the vector. 
 * Vector value must be fully qualified: both X and Y together with current NORM must be coherent values.
 * Angles get recalculated on the fly.
 * @param norm          RTB_FLOAT_TYPE carrying the new norm
 * @param vect        RTBvector carrying the vector to be modified
 */
RTBvector lucciSERVICE_vect_set_norm (RTB_FLOAT_TYPE norm, RTBvector vect);

#ifdef	__cplusplus
}
#endif

#endif	/* VEHICLECONTROL_H */

