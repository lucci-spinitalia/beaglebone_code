#define local_offset 2.0
#define local_pi_2   1.570796327 //    3.14159265358979323846

#include <stdio.h>
#include <math.h>
#include "rover_rtb.h"
#include "rover_plan.h"
#include "rover_obstacle_avoidance.h"

#if (SIMUL == 1)
#include "rover_video.h"
#endif

#if ((RTB_GEO_MODE == RTB_GEO_MODE_EUCLIDEAN) || (RTB_GEO_MODE == RTB_GEO_MODE_LATLON))

long RTB_internal_counter=0, RTB_internal_decimator=1;
extern RTB_status RTBstatus;
RTB_point *RTB_internal_point_actual, *RTB_internal_point_start, *RTB_internal_point_last;
int RTB_internal_initialized=0, RTB_active=0,RTB_complete=0,RTB_reset=0;
RTB_FLOAT_TYPE RTB_internal_angular_multiplier=1, RTB_internal_distance_threshold=0,RTB_internal_longitudinal_multiplier=1;

//#elif (RTB_GEO_MODE == RTB_GEO_MODE_LATLON)

#else
#error ("No geo mode specified in lucciRTB.h")
#endif


void gps_log(double latitude, double longitude)
{
  FILE *file = NULL;

  // Init Log File
  file = fopen("gps_log", "a");
  
  if(!file)
  {
    perror("logfile fopen:");
    return;
  }
  
  fprintf(file, "%f,%f,0 \n", longitude, latitude);

  fclose(file);
}

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
    x = (lon2-lon1) * cos(lat1 * M_PI / 180) * M_PI / 180;  // lon2-lon1 must be in radians 
    y = (lat2-lat1) * M_PI / 180; // lat2-lat1 must be in radians
    return(sqrt(x*x + y*y) * EARTH * 1000); 
}
#endif

void RTB_internal_clean_cache(void)
{
  RTB_point *local_point;

  if(RTB_internal_point_last == NULL)
    return;
	
  local_point = RTB_internal_point_last;

  while(local_point->previous != NULL)
  {
    local_point = local_point->previous;
    free(local_point->next);
    local_point->next = NULL;
      
    RTB_internal_point_actual = local_point;
    RTB_internal_point_last = local_point;
  }

  free(local_point);
    
  RTB_internal_point_actual = NULL;
  RTB_internal_point_last = NULL;
  RTB_internal_point_start = NULL;
  
  OA_cleanup();
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
   
  //At every point check if I'm near the start
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

void RTB_traslate_point(RTB_FLOAT_TYPE new_lat, RTB_FLOAT_TYPE new_lon, RTB_point *new_point)
{
  RTB_point *local_previouse_point;
  RTB_point *local_current_point;
  RTB_point *local_start_point;
  
  if(RTB_internal_point_last == NULL)
    return;
	
  local_current_point = RTB_internal_point_last;
  local_start_point = RTB_internal_point_start;
  
  while(local_current_point->previous != NULL)
  {
    //printf("Previouse point x: %f y: %f\n", local_start_point->x, local_start_point->y);
    local_current_point->x = new_lon - local_start_point->x;
    local_current_point->y = new_lat - local_start_point->y;
    //printf("Current point x: %f y: %f\n", local_current_point->x, local_current_point->y);
    
    local_current_point = local_current_point->previous;
    local_start_point = local_start_point->next;
  }
  
  local_current_point->x = new_lon - local_start_point->x;
  local_current_point->y = new_lat - local_start_point->y;
  
  /*local_start_point = RTB_internal_point_start;
  while(local_start_point->next != NULL)
  {
    printf("Traslate Point lat: %f lon: %f\n", local_start_point->y, local_start_point->x);
    //gps_log(local_start_point->y, local_start_point->x);
    local_start_point = local_start_point->next;
  }
  
  printf("Traslate Point lat: %f lon: %f\n", local_start_point->y, local_start_point->x);*/
 
  new_point = malloc(sizeof(RTB_point));
  new_point->x = local_start_point->x;
  new_point->y = local_start_point->y;
  new_point->distance_from_start = local_start_point->distance_from_start;
  new_point->next=NULL;
  new_point->previous=NULL;
  
  local_current_point = new_point;
  local_start_point = RTB_internal_point_start;
  
  while(local_start_point->next != NULL)
  {
    local_previouse_point = local_current_point;
    local_current_point->next = malloc(sizeof(RTB_point));
    local_current_point = local_current_point->next;
    local_current_point->previous = local_current_point;
    local_current_point->next = NULL;
    local_current_point->x = local_start_point->x;
    local_current_point->y = local_start_point->y;
    local_current_point->distance_from_start = local_start_point->distance_from_start;
		
    local_start_point = local_start_point->next;
  }

}

int RTB_update(RTB_FLOAT_TYPE localx, RTB_FLOAT_TYPE localy, RTB_FLOAT_TYPE xspeed, RTB_FLOAT_TYPE aspeed, 
               unsigned char obstacle_avoid_enable, unsigned int *sensor_reading, unsigned int reading_number,
			   unsigned char *point_catch)
{
  RTB_FLOAT_TYPE distance_point2point;
  RTB_status desired_direction;
  
#if (DEBUG == 1)
  //printf("aspeed: %f\n", aspeed);      
  //printf("xspeed: %f\n", xspeed);
#endif
  
  switch(RTBstatus.mode)
  {
    case RTB_recording:
	
	  // I don't take the point if my speed is < 0.
      if(xspeed < 0)
        xspeed = 0;
	  
      if(RTB_internal_point_start == NULL)
      {
#if (DEBUG == 1)
        printf("Actual x and y: %f  -  %f\n", localx, localy);
        //gps_log(localy, localx);
#endif
        RTB_internal_point_start = malloc(sizeof(RTB_point));
        RTB_internal_point_start->x=localx;
        RTB_internal_point_start->y=localy;
        RTB_internal_point_start->distance_from_start=0;
        RTB_internal_point_start->next=NULL;
        RTB_internal_point_start->previous=NULL;
        RTB_internal_point_last = RTB_internal_point_start;
        RTB_internal_counter++;
		
		if(point_catch != NULL)
		  *point_catch = 1;
		
#if (SIMUL == 1)                
        /* graphic helpers drawing*/
        rtb_video_init_glut(RTB_internal_point_start);
#endif             
        //return &RTBstatus;
      }

#if (RTB_SAVE_DIST_ADAPTIVE_ANGULAR==0 && RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL==0)
      
      RTBstatus.distance = RTB_internal_getdistance(localx, localy, RTB_internal_point_last->x, RTB_internal_point_last->y);
	  RTBstatus.distance_from_start = RTB_internal_getdistance(localx, localy, RTB_internal_point_start->x, RTB_internal_point_start->y);
	  
      if((RTBstatus.distance >= RTB_SAVE_DIST_TRSH) && (xspeed >= 0))
      {
        RTB_point *local_point;
        local_point = RTB_internal_point_last;
        RTB_internal_point_last->next = malloc(sizeof(RTB_point));
        RTB_internal_point_last = RTB_internal_point_last->next;
        RTB_internal_point_last->previous = local_point;
        RTB_internal_point_last->next = NULL;
        RTB_internal_point_last->x = localx;
        RTB_internal_point_last->y = localy;
        RTB_internal_point_last->distance_from_start = RTBstatus.distance_from_start;
        RTB_internal_counter++;

        if(point_catch != NULL)
		  *point_catch = 1;
		  
#if (DEBUG == 1)
        printf("Actual x and y: %f  -  %f\n", localx, localy);
        //gps_log(localy, localx);
#endif
#if (SIMUL == 1)
        rtb_video_take_point(RTB_internal_point_last);
#endif (SIMUL == 1)

      }
#if (SIMUL == 1)
      rtb_video_take_current_position(localx, localy);
#endif (SIMUL == 1)

      RTB_internal_flush(localx, localy);
      //RTBstatus.distance = RTB_internal_getdistance(localx, localy, RTB_internal_point_last->x, RTB_internal_point_last->y);
      //RTBstatus.distance = RTB_internal_getdistance(localx, localy, RTB_internal_point_start->x, RTB_internal_point_start->y);

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
                       
      RTBstatus.distance = RTB_internal_getdistance(localx, localy, RTB_internal_point_last->x, RTB_internal_point_last->y);
      RTBstatus.distance_from_start = RTB_internal_getdistance(localx, localy, RTB_internal_point_start->x, RTB_internal_point_start->y);
#if (DEBUG == 1)
      printf("Current x: %f Current y: %f \nLast x: %f Last y: %f\n", localx, localy, RTB_internal_point_last->x, RTB_internal_point_last->y);
      printf("Get distance: %f, Distance Threshold: %f\n", RTBstatus.distance, RTB_internal_distance_threshold);
#endif
      
      if((RTBstatus.distance >= RTB_internal_distance_threshold) && (xspeed >= 0))
      {
        RTB_point *local_point;
        local_point = RTB_internal_point_last;
        RTB_internal_point_last->next = malloc(sizeof(RTB_point));
        RTB_internal_point_last = RTB_internal_point_last->next;
        RTB_internal_point_last->previous = local_point;
        RTB_internal_point_last->next=NULL;
        RTB_internal_point_last->x=localx;
        RTB_internal_point_last->y=localy;
        RTB_internal_point_last->distance_from_start = RTBstatus.distance_from_start;
        RTB_internal_counter++;
		
        if(point_catch != NULL)
		  *point_catch = 1;
#if (DEBUG == 1)
        printf("Actual x and y: %f  -  %f\n",localx,localy);
        //gps_log(localy, localx);
#endif
#if (SIMUL == 1)                
        rtb_video_take_point(RTB_internal_point_last);
#endif
      }
#if (SIMUL == 1)
      rtb_video_take_current_position(localx, localy);
#endif

      RTB_internal_flush(localx,localy);
      //RTBstatus.distance = RTB_internal_getdistance(localx, localy, RTB_internal_point_last->x, RTB_internal_point_last->y);
      //RTBstatus.distance = RTB_internal_getdistance(localx,localy,RTB_internal_point_start->x,RTB_internal_point_start->y);
            
/*
#elif (RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL && !RTB_SAVE_DIST_ADAPTIVE_ANGULAR)
            int a = 1;
            
#elif (!RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL && RTB_SAVE_DIST_ADAPTIVE_ANGULAR)
            int a = 1;
            
#elif (RTB_SAVE_DIST_ADAPTIVE_LONGITUDINAL && RTB_SAVE_DIST_ADAPTIVE_ANGULAR)
            int a = 1;*/
#endif
#if (DEBUG == 1)
      //printf("Distance from the previous point: %f\n", RTBstatus.distance);
#endif
      RTB_internal_point_actual = RTB_internal_point_last;
      break;
    
    case RTB_tracking:
      desired_direction = RTBstatus;
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
  
      if(RTB_internal_point_actual == RTB_internal_point_start)
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
           
      desired_direction.distance = RTB_internal_getdistance(localx, localy, RTB_internal_point_actual->previous->x, RTB_internal_point_actual->previous->y);
	  desired_direction.distance_from_start = RTB_internal_getdistance(localx, localy, RTB_internal_point_start->x, RTB_internal_point_start->y);
	  
#if (DEBUG == 1)
      printf("Current x: %f Current y: %f Previouse x: %f Previouse y: %f\n", localx, localy, RTB_internal_point_actual->previous->x,RTB_internal_point_actual->previous->y);
      printf("Get distance: %f, Distance Threshold: %f\n", RTBstatus.distance, RTB_internal_distance_threshold);
#endif
      if(desired_direction.distance <= RTB_GUIDE_CHANGE_DIST_TRSH && RTB_internal_point_actual->previous != NULL)
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
		  RTBstatus.distance = desired_direction.distance;
		  RTBstatus.distance_from_start = desired_direction.distance_from_start;
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
      
      distance_point2point = RTB_internal_getdistance(RTB_internal_point_actual->previous->x, RTB_internal_point_actual->previous->y,
	                                                  RTB_internal_point_actual->x, RTB_internal_point_actual->y);

	  // Take a direction vector normalize to 1
      desired_direction.control_vector = lucciPLAN_givedir_multiparam(localx, localy, RTB_internal_point_actual->previous->x, RTB_internal_point_actual->previous->y);
        
      if(distance_point2point < 2.0)
        desired_direction.control_vector = lucciSERVICE_vect_set_norm(0.4, desired_direction.control_vector);

	  // if enabled it will change the control vector to avoid obstacles
      if(obstacle_avoid_enable == 1)
        RTBstatus.control_vector = OA_perform_avoidance(sensor_reading, reading_number, RTBstatus.control_vector, desired_direction.control_vector);
	  else  
        RTBstatus = desired_direction;
	  
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