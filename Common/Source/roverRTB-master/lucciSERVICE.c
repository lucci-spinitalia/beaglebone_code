#include "lucciSERVICE.h"

RTB_FLOAT_TYPE lucciSERVICE_rad2degnorth(RTB_FLOAT_TYPE angle)
{
    return lucciSERVICE_deg_adjust(90-(angle/M_PI * 180));
}

RTB_FLOAT_TYPE lucciSERVICE_degnorth2rad(RTB_FLOAT_TYPE angle)
{
    return lucciSERVICE_rad_adjust((90-angle)/180*M_PI);
}

RTB_FLOAT_TYPE lucciSERVICE_rad_adjust (RTB_FLOAT_TYPE angle)
{
    if(angle<M_PI)
        angle+=2*M_PI;
    if (angle > M_PI)
        angle-=2*M_PI;
    return angle;
}

RTB_FLOAT_TYPE lucciSERVICE_deg_adjust (RTB_FLOAT_TYPE angle)
{
    if (angle < 0)
        angle += 360;
    if (angle > 360)
        angle -= 360;
    return angle;
}

RTBvector lucciSERVICE_vect_normalize (RTBvector data)
{
    RTB_FLOAT_TYPE accum;
    RTBvector local = data;
    accum = sqrt(pow(data.x,2)+pow(data.y,2));
    local.x = data.x/accum;
    local.y = data.y/accum;
    local.norm = sqrt(pow(local.x,2)+pow(local.y,2));
    local.angle_rad = atan2(local.y, local.x);
    local.angle_deg_north = lucciSERVICE_rad2degnorth(local.angle_rad);
    return local;    
}

RTBvector lucciSERVICE_vect_sum (RTBvector a, RTBvector b)
{
    RTBvector local;
    local.x = a.x + b.x;
    local.y = a.y + b.y;
    local.angle_rad = atan2(local.y,local.x);
    local.angle_deg_north = lucciSERVICE_rad2degnorth(local.angle_rad);
    return lucciSERVICE_vect_normalize(local);
}

RTBvector lucciSERVICE_vect_set_norm (RTB_FLOAT_TYPE norm, RTBvector vect)
{
    RTBvector local = vect;
    RTB_FLOAT_TYPE accum;
    if (vect.norm != 0)
    {
        accum = norm / vect.norm;
        local.x = vect.x * accum;
        local.y = vect.y * accum;
        local.norm = sqrt(pow(local.x,2)+pow(local.y,2));
    }
    
    return local;
}