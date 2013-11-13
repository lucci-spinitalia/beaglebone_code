#include "rover_plan.h"


RTBvector lucciPLAN_givedir(RTBvector actual, RTBvector dest) //RTB_FLOAT_TYPE actualx, RTB_FLOAT_TYPE actualy, RTB_FLOAT_TYPE desiredx, RTB_FLOAT_TYPE desiredy)
{
    RTBvector localvec;
    RTB_FLOAT_TYPE localx, localy;
    localx = dest.x - actual.x;
    localy = dest.y - actual.y;
    localvec.angle_rad = atan2(localy, localx);
    localvec.angle_deg_north = lucciSERVICE_rad2degnorth(localvec.angle_rad);
    localvec.x = cos(localvec.angle_rad);
    localvec.y = sin(localvec.angle_rad);
    localvec = lucciSERVICE_vect_normalize(localvec);
    localvec = lucciSERVICE_vect_set_norm(2.0, localvec);
    return localvec;
}

#if (RTB_GEO_MODE == RTB_GEO_MODE_EUCLIDEAN)
RTBvector lucciPLAN_givedir_multiparam(RTB_FLOAT_TYPE actualx, RTB_FLOAT_TYPE actualy, RTB_FLOAT_TYPE destx, RTB_FLOAT_TYPE desty)
{
    RTBvector actual, dest;
    actual.x=actualx;
    actual.y=actualy;
    dest.x=destx;
    dest.y=desty;
    return lucciPLAN_givedir(dest,actual);
}
#elif (RTB_GEO_MODE == RTB_GEO_MODE_LATLON)
RTBvector lucciPLAN_givedir_multiparam(RTB_FLOAT_TYPE actual_lon, RTB_FLOAT_TYPE actual_lat,
                                       RTB_FLOAT_TYPE dest_lon, RTB_FLOAT_TYPE dest_lat)
{
    RTBvector localvec;
    RTB_FLOAT_TYPE localx, localy;
    
    localx = (dest_lon - actual_lon) * cos(actual_lat * 3.14 / 180) * 3.14 / 180;  // lon2-lon1 must be in radians 
    localy = (dest_lat - actual_lat) * 3.14 / 180; // lat2-lat1 must be in radians
    
    localvec.angle_rad = atan2(localy, localx);
    localvec.angle_deg_north = lucciSERVICE_rad2degnorth(localvec.angle_rad);
    localvec.x = cos(localvec.angle_rad);
    localvec.y = sin(localvec.angle_rad);
    localvec = lucciSERVICE_vect_normalize(localvec);
    localvec = lucciSERVICE_vect_set_norm(2.0, localvec);
    
    return localvec;
}
#endif