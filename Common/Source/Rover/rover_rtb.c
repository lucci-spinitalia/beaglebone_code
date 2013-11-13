#define local_offset 2.0
#define local_pi_2   1.570796327 //    3.14159265358979323846

#include <stdio.h>
#include <math.h>
#include "rover_rtb.h"
#include "rover_plan.h"

#if (SIMUL == 1)
#include "rover_video.h"
#endif

#if ((RTB_GEO_MODE == RTB_GEO_MODE_EUCLIDEAN) || (RTB_GEO_MODE == RTB_GEO_MODE_LATLON))

long RTB_internal_counter=0, RTB_internal_decimator=1;
extern RTB_status RTBstatus;
RTB_point *RTB_internal_point_actual, *RTB_internal_point_start, *RTB_internal_point_last;
int RTB_internal_initialized=0, RTB_active=0,RTB_complete=0,RTB_reset=0;
RTB_FLOAT_TYPE RTB_internal_distance=0, RTB_internal_angular_multiplier=1, RTB_internal_distance_threshold=0,RTB_internal_longitudinal_multiplier=1;

//#elif (RTB_GEO_MODE == RTB_GEO_MODE_LATLON)

#else
#error ("No geo mode specified in lucciRTB.h")
#endif



#if (RTB_GEO_MODE == RTB_GEO_MODE_EUCLIDEAN)
RTB_FLOAT_TYPE RTB_internal_getdistance(RTB_FLOAT_TYPE x1, RTB_FLOAT_TYPE y1, RTB_FLOAT_TYPE x2, RTB_FLOAT_TYPE y2)
{
    return (sqrt(pow(x2-x1,2)+ pow(y2-y1,2)));
}
#elif (RTB_GEO_MODE == RTB_GEO_MODE_LATLON)
RTB_FLOAT_TYPE RTB_internal_getdistance(RTB_FLOAT_TYPE lon1, RTB_FLOAT_TYPE lat1, RTB_FLOAT_TYPE lon2, RTB_FLOAT_TYPE lat2)
{
  /*
   For the y coordinate, 
   we can use the north-south distance between two lines of latitude:

       y = R*(b2-b1)*pi/180

    Here, I have converted the latitude difference, (b2-b1), from degrees 
    to radians, by multiplying it by (pi radians)/(180 degrees). The 
    product of the angle in radians and the radius is the arc length in 
    the same units as the radius.

    For the x coordinate, we can use the distance along a line of latitude 
    from one line of longitude to the other:

      x = R*(a2-a1)*(pi/180)*cos(b1)

    Here we have an additional factor, the cosine of the latitude along 
    which we are measuring. The line of latitude is a circle with a 
    smaller radius than that of the equator; it is reduced by the factor 
    cos(b1).
 
    where R is the radius of the earth, R = 6367 km
   */
    RTB_FLOAT_TYPE x,y;
    x = (lon2-lon1) * cos(lat1 * 3.14 / 180) * 3.14 / 180;  // lon2-lon1 must be in radians 
    y = (lat2-lat1) * 3.14 / 180; // lat2-lat1 must be in radians
    return(sqrt(x*x + y*y) * EARTH * 1000); 
}
#endif

void RTB_internal_clean_cache(void)
{
    RTB_point *local_point;
    local_point = RTB_internal_point_last;
    while(local_point->previous != NULL)
    {
      local_point=local_point->previous;
      free(local_point->next);
      local_point->next = NULL;
      
      RTB_internal_point_actual = local_point;
      RTB_internal_point_last = local_point;
    }
    
/*    free(local_point);
    
    RTB_internal_point_actual = NULL;
    RTB_internal_point_last = NULL;
    RTB_internal_point_start = NULL;*/
}

void RTB_init(void)
{
    RTBstatus.complete = 0;
    RTBstatus.control_values.heading = 0;
    RTBstatus.distance = 0;
    RTBstatus.mode = RTB_idle;    
    RTB_internal_initialized = 1;
}

void RTB_internal_disable()
{
    RTBstatus.mode = RTB_idle;
    RTB_internal_clean_cache();
    RTB_internal_counter=0;
    RTBstatus.control_values.heading = 0;
    RTBstatus.distance = 0;
    RTBstatus.complete = 0;
    
}

void RTB_set_mode(RTB_mode mode)
{
    if (RTBstatus.mode > 0 && mode == RTB_idle)
    {
      RTB_internal_disable();
    }
    if (RTBstatus.mode == RTB_idle)
      assert(RTB_internal_initialized == 1);
    
    RTBstatus.mode = mode;
}

void RTB_internal_flush(RTB_FLOAT_TYPE localx, RTB_FLOAT_TYPE localy)
{
   RTB_point* point_to_flush;
   RTB_FLOAT_TYPE local_distance;
   
  //At every point check if i'm near the start
  if((local_distance = RTB_internal_getdistance(localx, localy, RTB_internal_point_start->x, RTB_internal_point_start->y)) < RTB_FLUSH_DIST_TRSH)
  {
    point_to_flush = RTB_internal_point_last; // take the end of the list
    RTB_internal_point_last = RTB_internal_point_start; // reset the last point value
      
    //local_point = local_point->next;
    //Erase every point after the last updated
    while(point_to_flush != RTB_internal_point_last/*local_point->next != NULL*/)
    {
      point_to_flush = point_to_flush->previous;

      free(point_to_flush->next);
      point_to_flush->next = NULL;

      RTB_internal_counter--;
    }
    
//REFRESH PLAYER/STAGE POINTS
#if (SIMUL == 1)
    rtb_video_take_point(RTB_internal_point_last);
#endif
  }
    
  // If point's number > 5
  if(RTB_internal_counter > 5)
  {
    if((RTB_internal_decimator % RTB_GUIDE_UPDATE_TIME_TRSH) == 0)
    {
      RTB_internal_decimator=1;
      RTB_point* local_point;
      
      local_point = RTB_internal_point_last->previous;
      
      /* Clear loop */
      // find the first accurance of point near current position
      while((local_distance = RTB_internal_getdistance(localx, localy, local_point->x, local_point->y)) > RTB_FLUSH_DIST_TRSH)
      {
        if(local_point->previous != NULL)
          local_point = local_point->previous;
        else
          return;
      }
          
      point_to_flush = RTB_internal_point_last; // take the end of the list
      RTB_internal_point_last = local_point; // reset the last point value
      
      //local_point = local_point->next;
      //Erase every point after the last updated
      while(point_to_flush != RTB_internal_point_last/*local_point->next != NULL*/)
      {
        point_to_flush = point_to_flush->previous;

        free(point_to_flush->next);
        point_to_flush->next = NULL;

        RTB_internal_counter--;
      }

      //REFRESH PLAYER/STAGE POINTS
#if (SIMUL == 1)
      rtb_video_take_point(RTB_internal_point_last);
#endif
    }
    else
    {
      RTB_internal_decimator++;
    }
  }
}


int RTB_update(RTB_FLOAT_TYPE localx, RTB_FLOAT_TYPE localy, RTB_FLOAT_TYPE xspeed, RTB_FLOAT_TYPE aspeed)
{
  RTB_FLOAT_TYPE distance;

  printf("aspeed: %f\n", aspeed);      
  printf("xspeed: %f\n", xspeed);
      
  // I don't take the point if my speed is < 0.
  if(xspeed < 0)
    xspeed = 0;
  
  switch(RTBstatus.mode)
  {
    case RTB_recording:
      if(RTB_internal_point_start == NULL)
      {
#if (DEBUG == 1)
        printf("Actual x and y: %f  -  %f\n", localx, localy);
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
        rtb_video_init_glut(RTB_internal_point_start);
#endif             
        //return &RTBstatus;
      }

#if (RTB_SAVE_DIST_ADAPTIVE_ANGULAR==0 && RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL==0)
           
      RTB_internal_distance = RTB_internal_getdistance(localx,localy,RTB_internal_point_last->x,RTB_internal_point_last->y);
      if((RTB_internal_distance >= RTB_SAVE_DIST_TRSH) && (xspeed >= 0))
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

#if (DEBUG == 1)
        printf("Actual x and y: %f  -  %f\n", localx, localy);
#endif
#if (SIMUL == 1)
        rtb_video_take_point(RTB_internal_point_last);
#endif (SIMUL == 1)

      }
#if (SIMUL == 1)
      rtb_video_take_current_position(localx, localy);
#endif (SIMUL == 1)

      RTB_internal_flush(localx, localy);
      RTBstatus.distance = RTB_internal_getdistance(localx, localy, RTB_internal_point_start->x, RTB_internal_point_start->y);

#else
#if (RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL)
      if (xspeed == 0)
        xspeed = RTB_SAVE_DIST_ADAPTIVE_MAX_SPEED;
      else if(xspeed < RTB_SAVE_DIST_ADAPTIVE_MIN_SPEED)
        xspeed = RTB_SAVE_DIST_ADAPTIVE_MIN_SPEED;
      else if(xspeed > RTB_SAVE_DIST_ADAPTIVE_MAX_SPEED)
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
      RTB_internal_distance_threshold = MIN(RTB_SAVE_DIST_TRSH * RTB_internal_longitudinal_multiplier, RTB_SAVE_DIST_TRSH * RTB_internal_angular_multiplier);
      
      if (RTB_internal_distance_threshold < RTB_MIN_WAYPOINT_DIST)
        RTB_internal_distance_threshold = RTB_MIN_WAYPOINT_DIST;
            
      //dbg_print("RTB","Long_mult %3.2f, Ang_mult %3.2f, dst_trsh %3.2f\n",RTB_internal_longitudinal_multiplier, RTB_internal_angular_multiplier, RTB_internal_distance_threshold);
                       
      RTB_internal_distance = RTB_internal_getdistance(localx, localy, RTB_internal_point_last->x, RTB_internal_point_last->y);
      printf("Get distance: %f, Distance Threshold: %f\n", RTB_internal_distance, RTB_internal_distance_threshold);
      if((RTB_internal_distance >= RTB_internal_distance_threshold) && (xspeed >= 0))
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
#if (DEBUG == 1)
        printf("Actual x and y: %f  -  %f\n",localx,localy);
#endif
#if (SIMUL == 1)                
        rtb_video_take_point(RTB_internal_point_last);
#endif
      }
#if (SIMUL == 1)
      rtb_video_take_current_position(localx, localy);
#endif

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
   
      //At every point check if i'm near the start
/*      if(RTB_internal_getdistance(localx, localy, RTB_internal_point_start->x, RTB_internal_point_start->y) < RTB_GUIDE_CHANGE_DIST_TRSH)
      {
        RTB_internal_point_actual = RTB_internal_point_last; // take the end of the list
        RTB_internal_point_last = RTB_internal_point_start; // reset the last point value;
        
        //Erase every point after the last updated
        while(RTB_internal_point_actual != RTB_internal_point_last)
        {
          RTB_internal_point_actual = RTB_internal_point_actual->previous;

          free(RTB_internal_point_actual->next);
          RTB_internal_point_actual->next = NULL;

          RTB_internal_counter--;
        }
    
//REFRESH PLAYER/STAGE POINTS
#if (SIMUL == 1)
        rtb_video_take_point(RTB_internal_point_last);
#endif
      }*/
  
      if (RTB_internal_point_actual == RTB_internal_point_start)
      {
        /*RTBstatus.control_vector.x=0;
          RTBstatus.control_vector.y=0;
          RTBstatus.control_vector = lucciSERVICE_vect_normalize(RTBstatus.control_vector);*/
        RTBstatus.control_values.heading = 0;
        RTBstatus.control_values.speed = 0;
#if (DEBUG == 1)                    
        printf("Origin reached\n");
#endif
        RTB_set_mode(RTB_idle);
        return RTBstatus.mode;
      }
            
      if(RTB_internal_getdistance(localx,localy,RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y) <= RTB_GUIDE_CHANGE_DIST_TRSH && RTB_internal_point_actual->previous != NULL)
      {
#if (DEBUG == 1)
        printf("Point reached\n");
#endif
        RTB_internal_point_actual = RTB_internal_point_actual->previous;
        //free(RTB_internal_point_actual->next);
                              
#if (SIMUL == 1)
        rtb_video_take_point(RTB_internal_point_actual);
#endif

        if(RTB_internal_point_actual == RTB_internal_point_start)
        {
          RTBstatus.control_values.heading = 0;
          RTBstatus.control_values.speed = 0;
#if (DEBUG == 1)
          printf("Origin reached\n"); 
#endif
          RTB_set_mode(RTB_idle);
          return RTBstatus.mode;
        }
      }
#if (SIMUL == 1)
      rtb_video_take_current_position(localx, localy);
#endif
           
      if (RTB_internal_point_actual->next == NULL)
      {
        RTBstatus.control_vector = lucciPLAN_givedir_multiparam(localx, localy, RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y);
        distance = RTB_internal_getdistance(RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y,RTB_internal_point_actual->x,RTB_internal_point_actual->y);
        
        if(distance < 2.0)
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
          RTBstatus.control_vector = lucciSERVICE_vect_set_norm(1.0,lucciPLAN_givedir_multiparam(localx, localy, RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y));
        }
      } 
            
#if (DEBUG==1)
      //printf("X: %f   Y: %f    Angle: %f rad    Angle to north: %f deg   Norm: %f\n", RTBstatus.control_vector.x, RTBstatus.control_vector.y, RTBstatus.control_vector.angle_rad, RTBstatus.control_vector.angle_deg_north, RTBstatus.control_vector.norm);
#endif
      break;
        
    default:
      break;
  }
    
  //return &RTBstatus;
  return RTBstatus.mode;

}