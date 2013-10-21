#include "lucciPLAN.h"


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

RTBvector lucciPLAN_givedir_multiparam(RTB_FLOAT_TYPE actualx, RTB_FLOAT_TYPE actualy, RTB_FLOAT_TYPE destx, RTB_FLOAT_TYPE desty)
{
    RTBvector actual, dest;
    actual.x=actualx;
    actual.y=actualy;
    dest.x=destx;
    dest.y=desty;
    return lucciPLAN_givedir(dest,actual);
}