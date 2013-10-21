#define local_offset 2.0
#define local_pi_2   1.570796327 //    3.14159265358979323846

#if (SIMUL == 1)
#include <player-3.1/libplayerc/playerc.h>
#include "player_manager.h"
extern playerc_position2d_t *PlayerLocalPosition2dRelative;
#endif

#include "lucciRTB.h"
#include "lucciPLAN.h"


#if (RTB_GEO_MODE == RTB_GEO_MODE_EUCLIDEAN)


long RTB_internal_counter=0, RTB_internal_decimator=1;
extern RTB_status RTBstatus;
RTB_point *RTB_internal_point_actual, *RTB_internal_point_previous, *RTB_internal_point_next, *RTB_internal_point_start, *RTB_internal_point_last;
int RTB_internal_initialized=0, RTB_active=0,RTB_complete=0,RTB_reset=0;
RTB_FLOAT_TYPE RTB_internal_distance=0, RTB_internal_angular_multiplier=1, RTB_internal_distance_threshold=0,RTB_internal_longitudinal_multiplier=1, RTB_internal_previous_speed=0;

#elif (RTB_GEO_MODE == RTB_GEO_MODE_LATLON)


#else
#error ("No geo mode specified in lucciRTB.h")
#endif



#if (RTB_GEO_MODE == RTB_GEO_MODE_EUCLIDEAN)
RTB_FLOAT_TYPE RTB_internal_getdistance(RTB_FLOAT_TYPE x1, RTB_FLOAT_TYPE y1, RTB_FLOAT_TYPE x2, RTB_FLOAT_TYPE y2)
{
    double temp1,temp2,temp3;
    temp1=pow(x2-x1,2);
    temp2=pow(y2-y1,2);
    return (sqrt(pow(x2-x1,2)+ pow(y2-y1,2)));
}
#elif (RTB_GEO_MODE == RTB_GEO_MODE_LATLON)
RTB_FLOAT_TYPE RTB_internal_getdistance(RTB_FLOAT_TYPE lon1, RTB_FLOAT_TYPE lat1, RTB_FLOAT_TYPE lon2, RTB_FLOAT_TYPE lat2)
{
    RTB_FLOAT_TYPE x,y,d;
    x = (lon2-lon1) * Math.cos((lat1+lat2)/2);
    y = (lat2-lat1);
    return(Math.sqrt(x*x + y*y) * EARTH);
}
#endif

void RTB_internal_clean_cache(void)
{
    RTB_point * local_point;
    local_point=RTB_internal_point_last;
    while (local_point->previous!=NULL)
    {
        local_point=local_point->previous;
        free(local_point->next);
    }
    free(local_point);
    
    RTB_internal_point_actual=NULL;
    RTB_internal_point_previous=NULL;
    RTB_internal_point_next=NULL;
    RTB_internal_point_last=NULL;
    RTB_internal_point_start=NULL;
}

void RTB_init(void)
{
    RTBstatus.complete = 0;
    RTBstatus.control_values.heading = -1;
    RTBstatus.distance = -1;
    RTBstatus.mode = RTB_idle;    
    RTB_internal_initialized = 1;
}

void RTB_internal_disable()
{
    RTBstatus.mode = RTB_idle;
    RTB_internal_clean_cache();
    RTB_internal_counter=0;
    RTBstatus.control_values.heading = -1;
    RTBstatus.distance = -1;
    RTBstatus.complete = 0;
    
}

void RTB_set_mode(RTB_mode mode)
{
    if (RTBstatus.mode > 0 && mode == RTB_idle)
        RTB_internal_disable();
    if (RTBstatus.mode == RTB_idle)
        assert (RTB_internal_initialized == 1);
    RTBstatus.mode = mode;
}

void RTB_internal_flush(RTB_FLOAT_TYPE localx, RTB_FLOAT_TYPE localy)
{
    if(RTB_internal_counter > 5)
    {
        if ((RTB_internal_decimator % RTB_GUIDE_UPDATE_TIME_TRSH) == 0)
        {
            RTB_internal_decimator=1;
            RTB_point* local_point;
            RTB_FLOAT_TYPE local_distance;
            local_point = RTB_internal_point_last->previous;
            while((local_distance= RTB_internal_getdistance(localx,localy,local_point->x,local_point->y) > RTB_FLUSH_DIST_TRSH) && (local_point->previous != NULL))
            {
                local_point=local_point->previous;
            }
            
            if (local_point == RTB_internal_point_start)
            
                return;
            
            
            RTB_internal_point_last = local_point;
            local_point=local_point->next;
            while(local_point->next != NULL)
            {
                local_point=local_point->next;
                free(local_point);
            }
            free(RTB_internal_point_last->next);
            RTB_internal_point_last->next=NULL;

            //REFRESH PLAYER/STAGE POINTS
#if (SIMUL == 1)
            {
                playerc_graphics2d_clear(PlayerLocalGraphics2D);
                player_point_2d_t points[4];
                player_color_t color;
                /*color.blue=20;
                color.green=20;
                color.red=255;
                color.alpha = 160;
                points[0].px=1+localx;
                points[0].py=0+localy+local_offset;
                points[1].px=0+localx;
                points[1].py=1.5+localy+local_offset;
                points[2].px=-1+localx;
                points[2].py=0+localy+local_offset;
                points[3].px=0+localx;
                points[3].py=-1.5+localy+local_offset;              
                playerc_graphics2d_draw_polygon(PlayerLocalGraphics2D,points,4,1,color);*/

                color.blue=20;
                color.green=255;
                color.red=20;
                color.alpha = 160;                
                points[0].px=1+RTB_internal_point_start->x;
                points[0].py=0+RTB_internal_point_start->y+local_offset;
                points[1].px=0+RTB_internal_point_start->x;
                points[1].py=1.5+RTB_internal_point_start->y+local_offset;
                points[2].px=-1+RTB_internal_point_start->x;
                points[2].py=0+RTB_internal_point_start->y+local_offset;
                points[3].px=0+RTB_internal_point_start->x;
                points[3].py=-1.5+RTB_internal_point_start->y+local_offset;                
                playerc_graphics2d_draw_polygon(PlayerLocalGraphics2D,points,4,1,color);
                
                color.blue=255;
                color.green=30;
                color.red=30;
                color.alpha = 160;
                local_point=RTB_internal_point_start->next;
                while (local_point->next != NULL)
                {
                    points[0].px=0.5+local_point->x;
                    points[0].py=0.5+local_point->y+local_offset;
                    points[1].px=-0.5+local_point->x;
                    points[1].py=0.5+local_point->y+local_offset;
                    points[2].px=-0.5+local_point->x;
                    points[2].py=-0.5+local_point->y+local_offset;
                    points[3].px=0.5+local_point->x;
                    points[3].py=-0.5+local_point->y+local_offset;                
                    playerc_graphics2d_draw_polygon(PlayerLocalGraphics2D,points,4,1,color);
                    local_point=local_point->next;
                }
                points[0].px=0.5+local_point->x;
                points[0].py=0.5+local_point->y+local_offset;
                points[1].px=-0.5+local_point->x;
                points[1].py=0.5+local_point->y+local_offset;
                points[2].px=-0.5+local_point->x;
                points[2].py=-0.5+local_point->y+local_offset;
                points[3].px=0.5+local_point->x;
                points[3].py=-0.5+local_point->y+local_offset;                
                playerc_graphics2d_draw_polygon(PlayerLocalGraphics2D,points,4,1,color);
            }
#endif
        }
        else
        {
            RTB_internal_decimator++;
        }
    }
}


void RTB_update(RTB_FLOAT_TYPE localx, RTB_FLOAT_TYPE localy, RTB_FLOAT_TYPE xspeed, RTB_FLOAT_TYPE aspeed)
{
    RTB_FLOAT_TYPE distance;
    switch(RTBstatus.mode)
    {
        case RTB_recording:
            if (RTB_internal_point_start==NULL)
            {
#if (DEBUG == 1)
                printf("Actual x and y: %3.2f  -  %3.2f\n",localx,localy);
#endif
                RTB_internal_point_start = malloc(sizeof(RTB_point));
                RTB_internal_point_start->x=localx;
                RTB_internal_point_start->y=localy;
                RTB_internal_point_start->distance_from_start=0;
                RTB_internal_point_start->next=NULL;
                RTB_internal_point_start->previous=NULL;
                RTB_internal_point_last = RTB_internal_point_start;
                RTB_internal_counter++;
#if (SIMUL == 1)                
                /* graphic helpers drawing*/
                player_point_2d_t points[4];
                player_color_t color;
                /*color.blue=20;
                color.green=20;
                color.red=255;
                color.alpha = 160;
                points[0].px=1+localx;
                points[0].py=0+localy+local_offset;
                points[1].px=0+localx;
                points[1].py=1.5+localy+local_offset;
                points[2].px=-1+localx;
                points[2].py=0+localy+local_offset;
                points[3].px=0+localx;
                points[3].py=-1.5+localy+local_offset;                
                playerc_graphics2d_draw_polygon(PlayerLocalGraphics2D,points,4,1,color);*/

                color.blue=20;
                color.green=255;
                color.red=20;
                color.alpha = 160;                
                points[0].px=1+RTB_internal_point_start->x;
                points[0].py=0+RTB_internal_point_start->y+local_offset;
                points[1].px=0+RTB_internal_point_start->x;
                points[1].py=1.5+RTB_internal_point_start->y+local_offset;
                points[2].px=-1+RTB_internal_point_start->x;
                points[2].py=0+RTB_internal_point_start->y+local_offset;
                points[3].px=0+RTB_internal_point_start->x;
                points[3].py=-1.5+RTB_internal_point_start->y+local_offset;                
                playerc_graphics2d_draw_polygon(PlayerLocalGraphics2D,points,4,1,color);
#endif             
                //return &RTBstatus;
            }

#if (RTB_SAVE_DIST_ADAPTIVE_ANGULAR==0 && RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL==0)
            
            RTB_internal_distance = RTB_internal_getdistance(localx,localy,RTB_internal_point_last->x,RTB_internal_point_last->y);
            if (RTB_internal_distance >= RTB_SAVE_DIST_TRSH)
            {
                RTB_point *local_point;
                local_point = RTB_internal_point_last;
                RTB_internal_point_last->next = malloc(sizeof(RTB_point));
                RTB_internal_point_last = RTB_internal_point_last->next;
                RTB_internal_point_last->previous = local_point;
                RTB_internal_point_last->next=NULL;
                RTB_internal_point_last->x=localx;
                RTB_internal_point_last->y=localy;
                RTB_internal_point_last->distance_from_start=RTB_internal_distance;
                RTB_internal_counter++;
#if (SIMUL == 1)
                player_color_t color;
                color.blue=255;
                color.green=30;
                color.red=30;
                color.alpha = 160;
                player_point_2d_t points[4];
                points[0].px=0.5+localx;
                points[0].py=0.5+localy+local_offset;
                points[1].px=-0.5+localx;
                points[1].py=0.5+localy+local_offset;
                points[2].px=-0.5+localx;
                points[2].py=-0.5+localy+local_offset;
                points[3].px=0.5+localx;
                points[3].py=-0.5+localy+local_offset;                
                playerc_graphics2d_draw_polygon(PlayerLocalGraphics2D,points,4,1,color);
#endif (SIMUL == 1)

            }
            RTB_internal_flush(localx,localy);
            RTBstatus.distance = RTB_internal_getdistance(localx,localy,RTB_internal_point_start->x,RTB_internal_point_start->y);

#else
#if (RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL)
            if (xspeed == 0)
                xspeed = RTB_SAVE_DIST_ADAPTIVE_MAX_SPEED;
            else if (xspeed < RTB_SAVE_DIST_ADAPTIVE_MIN_SPEED)
                xspeed = RTB_SAVE_DIST_ADAPTIVE_MIN_SPEED;
            else if (xspeed > RTB_SAVE_DIST_ADAPTIVE_MAX_SPEED)
                xspeed = RTB_SAVE_DIST_ADAPTIVE_MAX_SPEED;
#else
            RTB_internal_longitudinal_multiplier = 1;
#endif
            
            RTB_internal_longitudinal_multiplier = xspeed / RTB_SAVE_DIST_ADAPTIVE_MAX_SPEED;
            
            
#if (RTB_SAVE_DIST_ADAPTIVE_ANGULAR)
            if (aspeed == 0)
                RTB_internal_angular_multiplier = 1;
            else
            {
                
                
                RTB_internal_angular_multiplier = 1 - RTB_SAVE_DIST_ADAPTIVE_ANGULAR_SENSITIVITY*(sqrt(aspeed*aspeed)/RTB_SAVE_DIST_ADAPTIVE_MAX_ANGULAR_SPEED);
                if (RTB_internal_angular_multiplier < 0.05)
                    RTB_internal_angular_multiplier = 0.05;
                if (RTB_internal_angular_multiplier > 1)
                    RTB_internal_angular_multiplier = 1;
            }
#else
            RTB_internal_angular_multiplier = 1;
            
#endif
            
            RTB_internal_distance_threshold = MIN(RTB_SAVE_DIST_TRSH * RTB_internal_longitudinal_multiplier, RTB_SAVE_DIST_TRSH*RTB_internal_angular_multiplier);
            if (RTB_internal_distance_threshold < RTB_MIN_WAYPOINT_DIST)
                RTB_internal_distance_threshold = RTB_MIN_WAYPOINT_DIST;
            
            //dbg_print("RTB","Long_mult %3.2f, Ang_mult %3.2f, dst_trsh %3.2f\n",RTB_internal_longitudinal_multiplier, RTB_internal_angular_multiplier, RTB_internal_distance_threshold);
                        
            RTB_internal_distance = RTB_internal_getdistance(localx,localy,RTB_internal_point_last->x,RTB_internal_point_last->y);
            if (RTB_internal_distance >= RTB_internal_distance_threshold)
            {
                RTB_point *local_point;
                local_point = RTB_internal_point_last;
                RTB_internal_point_last->next = malloc(sizeof(RTB_point));
                RTB_internal_point_last = RTB_internal_point_last->next;
                RTB_internal_point_last->previous = local_point;
                RTB_internal_point_last->next=NULL;
                RTB_internal_point_last->x=localx;
                RTB_internal_point_last->y=localy;
                RTB_internal_point_last->distance_from_start=RTB_internal_distance;
                RTB_internal_counter++;
#if (SIMUL == 1)                
                player_color_t color;
                color.blue=255;
                color.green=30;
                color.red=30;
                color.alpha = 160;
                player_point_2d_t points[4];
                points[0].px=0.5+localx;
                points[0].py=0.5+localy+local_offset;
                points[1].px=-0.5+localx;
                points[1].py=0.5+localy+local_offset;
                points[2].px=-0.5+localx;
                points[2].py=-0.5+localy+local_offset;
                points[3].px=0.5+localx;
                points[3].py=-0.5+localy+local_offset;                
                playerc_graphics2d_draw_polygon(PlayerLocalGraphics2D,points,4,1,color);
#endif

            }
            RTB_internal_flush(localx,localy);
            RTBstatus.distance = RTB_internal_getdistance(localx,localy,RTB_internal_point_start->x,RTB_internal_point_start->y);
            
/*
#elif (RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL && !RTB_SAVE_DIST_ADAPTIVE_ANGULAR)
            int a = 1;
            
#elif (!RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL && RTB_SAVE_DIST_ADAPTIVE_ANGULAR)
            int a = 1;
            
#elif (RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL && RTB_SAVE_DIST_ADAPTIVE_ANGULAR)
            int a = 1;*/
#endif
            RTB_internal_point_actual = RTB_internal_point_last;
        break;
    
        case RTB_tracking:
            
            //RTB_internal_flush(localx,localy);
            if (RTB_internal_point_actual == RTB_internal_point_start)
            {
                    RTBstatus.control_vector.x=0;
                    RTBstatus.control_vector.y=0;
                    RTBstatus.control_vector = lucciSERVICE_vect_normalize(RTBstatus.control_vector);
                    RTBstatus.control_values.heading=0;
                    RTBstatus.control_values.speed=0;
#if (DEBUG == 1)                    
                    printf("Origin reached\n");
#endif
                    return;
            }
            if (RTB_internal_getdistance(localx,localy,RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y) <= RTB_GUIDE_CHANGE_DIST_TRSH && RTB_internal_point_actual->previous != NULL)
            {
#if (SIMUL == 1)
                playerc_position2d_set_cmd_vel(PlayerLocalPosition2dRelative,0,0,0,1);
#endif
                RTB_internal_point_actual = RTB_internal_point_actual->previous;
            }

            if (RTB_internal_point_actual->next == NULL)
            {
                
                RTBstatus.control_vector = lucciPLAN_givedir_multiparam(localx, localy, RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y);
                distance = RTB_internal_getdistance(RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y,RTB_internal_point_actual->x,RTB_internal_point_actual->y);
                if (distance < 2.0)
                    RTBstatus.control_vector = lucciSERVICE_vect_set_norm(0.4,RTBstatus.control_vector);
                else
                    RTBstatus.control_vector = lucciSERVICE_vect_set_norm(1.0,RTBstatus.control_vector);
                
                
            }
            else
            {
                if (RTB_internal_getdistance(RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y,RTB_internal_point_actual->x,RTB_internal_point_actual->y) < 2.0)
                {
                    RTBstatus.control_vector = lucciSERVICE_vect_set_norm(0.4,lucciPLAN_givedir_multiparam(localx, localy, RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y));
                }
                else
                {
                    //calcola la congiungente tra i due punti
                    
                    //calcola la distanza dalla congiungente
                    /*RTB_FLOAT_TYPE M,Q,c1,c2,d, angle;
                    c1 = RTB_internal_point_actual->previous->y - RTB_internal_point_actual->y;
                    c2 = RTB_internal_point_actual->previous->x - RTB_internal_point_actual->x;
                    M = c1 / c2;
                    Q = RTB_internal_point_actual->y - RTB_internal_point_actual->x / c2;
                    d = -(localy - localx * M -Q)/sqrt(1+(pow(M,2)));
                    angle = RTBstatus.control_vector.angle_rad + (d > 0 ? atan(d) : -atan(d));
                    printf("Angle to dest %3.2f, correction angle %3.2f\n", RTBstatus.control_vector.angle_rad, angle);*/
                    //calcola l'orientamento necessario
                    
                    RTBstatus.control_vector = lucciSERVICE_vect_set_norm(1.0,lucciPLAN_givedir_multiparam(localx, localy, RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y));
                }

                
                
            }
            
                
            break;
        
        default:
            break;
    
    }
    
    RTB_internal_previous_speed=xspeed;
    //return &RTBstatus;

}