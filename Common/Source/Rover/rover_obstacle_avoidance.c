#if (SIMUL == 1)
#include "rtb_video.h"
#endif
#include "rover_obstacle_avoidance.h"

RTB_status RTBstatus;


typedef struct OA_radial_map_t {
    unsigned int intensity;
    unsigned int validity;
} OA_radial_map_t;

OA_radial_map_t *OA_radial_map = NULL;
unsigned int **OA_map;
int OA_map_size, OA_inited = 0;
RTB_FLOAT_TYPE OA_internal_min_angle;
unsigned int OA_radial_size;
//unsigned int *OA_radial_map;


void OA_init(float range_deg)
{
  long i;

  //OA_internal_min_angle = OA_SENSOR_CENTER - OA_SENSOR_COUNT/2 * OA_SENSOR_RESOLUTION;
  
  if((range_deg / OA_GRID_RADIAL_SAMPLING) > floor(range_deg / OA_GRID_RADIAL_SAMPLING))
    range_deg++;

  OA_radial_size = floor(range_deg / OA_GRID_RADIAL_SAMPLING);
  OA_radial_map = (OA_radial_map_t *) malloc(OA_radial_size * sizeof(OA_radial_map_t));
  
  for (i = 0; i < OA_radial_size; i++)
  {
    OA_radial_map[i].intensity=0;
    OA_radial_map[i].validity=0;
  }
  
  OA_inited = 1;
}

void OA_internal_map_update(void)
{
  int i;
  /*for (i = 0; i < OA_map_size; i++)
   {
    for (j = 0; j < OA_map_size; j++)
    {
      if(OA_map[i][j] > 0)
        OA_map[i][j]--;
    }
  }*/
	
  for(i = 0; i < OA_radial_size; i++)
  {
    OA_radial_map[i].intensity=0;
    OA_radial_map[i].validity=0;
  }
}

RTBvector OA_perform_avoidance(unsigned int readings[], long count, RTBvector current_direction, RTBvector desired_heading)
{
  RTBvector localvec;
  RTB_FLOAT_TYPE angle;
  long i,j;
  long index;
  int left_reached_end = 0, left_valid = 0, right_reached_end = 0, right_valid = 0;
  
  // if I haven't perform the initialization then return a zero vector
  if(OA_inited == 0)
  {
    localvec.x = 0;
    localvec.y = 0;
    localvec.norm = 0;
    localvec.angle_deg_north = 0;
    localvec.angle_rad = M_PI_2;
    
    return localvec;        
  }
  
  // reset OA_radial_map
  OA_internal_map_update();

  //Range points discretization and mapping
  for(i = 0; i < count; i++)
  {
    if(readings[i] < OA_ACTIVE_DISTANCE)
    {
      index = (long)floor(i / (count / OA_GRID_RADIAL_SAMPLING));
      OA_radial_map[index].intensity++;
    }
  }

  // searching for a window where I can pass
  long _validity_count = 0;
  
  for (i = 0; i < OA_radial_size; i++)
  {
    if(OA_radial_map[i].intensity < OA_WINDOW_THRESHOLD)
      _validity_count++;
    else
      _validity_count = 0;
	  
    if(_validity_count == OA_WINDOW_SIZE)
    {
      OA_radial_map[i].validity = 1;

      for(j = 0; j < _validity_count - 1; j++)
        OA_radial_map[i - _validity_count + j + 1].validity = 1;
    }
        
    if(_validity_count > OA_WINDOW_SIZE)
      OA_radial_map[i].validity = 1;
  }

  // Dividing the range in OA_GRID_RADIAL_SAMPLING, I can find where desired_haeding.angle_deg_north is in
  // the OA_radial_map by divideding it per OA_GRID_RADIAL_SAMPLING and flooring
  index = (long)floor(desired_heading.angle_deg_north / OA_GRID_RADIAL_SAMPLING);

  // Check if the desired direction is in range
  if((index >= 0) || (index < OA_radial_size))
  {
    // if I go in the wrong direction
    if(OA_radial_map[index].validity != 1)
    {
      long count_left = 0, count_right = 0;
 
      while((OA_radial_map[index - count_left].validity == 0 && OA_radial_map[index + count_right].validity == 0) && 
		    (index - count_left) > 0 && ((index + count_right) < OA_radial_size))
      {
        count_left++;
        count_right++;
      }
        
      if (index - count_left == 0)
        left_reached_end = 1;
      else
        if(OA_radial_map[index - count_left].validity == 1)
          left_valid = 1;
        
      if(index + count_right >= OA_radial_size)
        right_reached_end = 1;
      else
        if(OA_radial_map[index + count_right].validity == 1)
          right_valid = 1;
        
      // If there's no solution starting by the center current direction, start to search by one of the ends
      if(!right_valid && !left_valid)
      {
        if(left_reached_end)
        {
          int temp = count_left;
        
	      // search starting by the end
          while((OA_radial_map[OA_radial_size - count_left + temp].validity == 0 && OA_radial_map[index + count_right].validity == 0) && 
	            (index + count_right) < OA_radial_size)
          {
            count_right++;
            count_left++;
          }
        
          if(OA_radial_map[OA_radial_size - count_left + temp].validity == 1)
            left_valid = 1;

          if(OA_radial_map[index + count_right].validity == 1)
            right_valid = 1;
        }
        else // search starting by the start
        {
          int temp = count_right;
                
          while(OA_radial_map[index - count_left].validity == 0 && OA_radial_map[count_right - temp].validity == 0 && (index - count_left) < OA_radial_size)
          {
            count_left++;
            count_right++;
          }    
        
          if(OA_radial_map[index - count_left].validity == 1)
            left_valid = 1;

          if(OA_radial_map[count_right - temp].validity == 1)
            right_valid = 1;
        }
      }
    
	  // if I found a way then change direction
	  // If I can go to either right or left direction then I want to choose
	  // the shortest way to avoid the obstacle
      if(left_valid)
      {
        /*if(left_reached_end)
          angle = lucciSERVICE_degnorth2rad(lucciSERVICE_deg_adjust((RTB_FLOAT_TYPE) 5 * (OA_radial_size + index - count_left) - 20));
        else*/
          angle = lucciSERVICE_degnorth2rad(lucciSERVICE_deg_adjust((RTB_FLOAT_TYPE) OA_GRID_RADIAL_SAMPLING * (index - count_left)));
		  
        //Update direction vector and the direction angle
        localvec.x = cos(angle);
        localvec.y = sin(angle);
	    localvec.angle_deg_north = lucciSERVICE_rad2degnorth(localvec.angle_rad);
        localvec = lucciSERVICE_vect_normalize(localvec);
      }
      else if(right_valid)
      {
        /*if(right_reached_end)
          angle = lucciSERVICE_degnorth2rad(lucciSERVICE_deg_adjust((RTB_FLOAT_TYPE) 5 * (index + count_right - OA_radial_size) + 20));
        else*/
          angle = lucciSERVICE_degnorth2rad(lucciSERVICE_deg_adjust((RTB_FLOAT_TYPE) OA_GRID_RADIAL_SAMPLING * (index + count_right)));
		  
        //Update direction vector and the direction angle
        localvec.x = cos(angle);
        localvec.y = sin(angle);
	    localvec.angle_deg_north = lucciSERVICE_rad2degnorth(localvec.angle_rad);
        localvec = lucciSERVICE_vect_normalize(localvec);
      }
	  else
        localvec = desired_heading;
	  /*
      //emergency avoid. In this case I have to decrease velocity
      if(min_distance < OA_SPEED_MODERATION_DIST && localvec.norm > 0.0)
      {
        // calculate min_distance and take the index with this value
        for(i = 0; i < count; i++)
        {
          min_distance = MIN(min_distance, readings[i]);
    
          if(readings[i] == min_distance)
            index = i;
        } 
	
#if (OA_SENSOR_ORIENTATION == OA_SENSOR_CLOCKWISE)
        angle = current_direction.angle_rad + OA_SENSOR_CENTER + OA_internal_min_angle + (index * OA_SENSOR_RESOLUTION);
#elif (OA_SENSOR_ORIENTATION == OA_SENSOR_ANTI_CLOCKWISE)
        //angle = current_direction.angle_rad + OA_SENSOR_CENTER - OA_internal_min_angle - (i * OA_SENSOR_RESOLUTION);  
        angle = current_direction.angle_rad + OA_SENSOR_CENTER - OA_internal_min_angle - ((i - index) * OA_SENSOR_RESOLUTION);  		
#endif

        angle = lucciSERVICE_rad_adjust(angle + M_PI);
	
        localvec2.x = cos(angle);
        localvec2.y = sin(angle);
        localvec = lucciSERVICE_vect_sum(localvec, localvec2);
        localvec = lucciSERVICE_vect_set_norm(MIN(2, min_distance / OA_AVOID_INTENSITY), localvec);
        RTBstatus.control_vector = localvec;
      }
  
      // If I'm closer then reduce velocity more and more
      if (min_distance < OA_SAFETY_DISTANCE)
        localvec = lucciSERVICE_vect_set_norm(0.01, localvec);
	  */
	}
    else // There's no problem go by this way
      localvec = desired_heading;
  }
  else
    localvec = desired_heading;
    	
  return localvec;
}


void OA_cleanup(void)
{ 
  free(OA_radial_map);   
}