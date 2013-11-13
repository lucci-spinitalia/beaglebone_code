/** 
 * \file   lucciPLAN.h
 * Author: erupter
 *
 * Created on November 2, 2012, 2:41 PM
 */

#ifndef LUCCIPLAN_H
#define	LUCCIPLAN_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "rover_rtb.h"
#include "rover_service.h"
#include <math.h>

    /** \defgroup Plan */
    
/** \brief Plan calculate new direction
 *
 * \ingroup Plan
 *  Calculates the direction towards the next waypoint given the actual position.
 * @param dest          RTBvector with the destination vector
 * @param actual        RTBvector with the actual position vector
 * @return direction    RTBvector containing the computed new direction
 */    
RTBvector lucciPLAN_givedir(RTBvector dest, RTBvector actual);

/** \brief Plan calculate new direction (4 param version
 *
 * \ingroup Plan      
 *  Calculates the direction towards the next waypoint given the actual position.
 * @param actualx       RTBvector with the actual position X component
 * @param actualy       RTBvector with the actual position Y component
 * @param destx         RTBvector with the destination X component
 * @param desty         RTBvector with the destination Y component
 * @return direction    RTBvector containing the computed new direction
 */    
RTBvector lucciPLAN_givedir_multiparam(RTB_FLOAT_TYPE actualx, RTB_FLOAT_TYPE actualy, RTB_FLOAT_TYPE destx, RTB_FLOAT_TYPE desty);


#ifdef	__cplusplus
}
#endif

#endif	/* LUCCIPLAN_H */

