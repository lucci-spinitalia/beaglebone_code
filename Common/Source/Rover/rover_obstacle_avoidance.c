#if (SIMUL == 1)
#include "rtb_video.h"
#endif
#include "rover_obstacle_avoidance.h"

FILE *output_pty;
RTB_status RTBstatus;


typedef struct OA_radial_map_t {
    unsigned int intensity;
    unsigned int validity;
} OA_radial_map_t;

OA_radial_map_t *OA_radial_map;
unsigned int **OA_map;
int OA_map_size, OA_inited = 0;
RTB_FLOAT_TYPE OA_internal_min_angle;
int avoid_soft = 0, avoid_hard=0;
unsigned int OA_radial_size;
//unsigned int *OA_radial_map;


void OA_init (void)
{
    long i;

    OA_internal_min_angle = OA_SENSOR_CENTER - OA_SENSOR_COUNT/2 * OA_SENSOR_RESOLUTION;
    OA_radial_size = (int)(360 / OA_GRID_RADIAL_SAMPLING);
    //OA_radial_map = (unsigned int *) malloc (OA_radial_size * sizeof(unsigned int));
    OA_radial_map = (OA_radial_map_t *) malloc (OA_radial_size * sizeof(OA_radial_map_t));
    for (i=0;i<OA_radial_size;i++)
    {
        OA_radial_map[i].intensity=0;
        OA_radial_map[i].validity=0;
    }
        //OA_radial_map[i]=0;
    OA_inited = 1;
}

void OA_internal_map_update (void)
{
    int i,j;
    for (i=0;i<OA_map_size; i++)
        for (j=0; j<OA_map_size; j++)
        {
            if(OA_map[i][j]>0)
                OA_map[i][j]--;
        }
    for (i=0;i<OA_radial_size;i++)
    {
        OA_radial_map[i].intensity=0;
        OA_radial_map[i].validity=0;
    }
}

RTBvector OA_perform_avoidance (RTB_FLOAT_TYPE readings[], long count, RTBvector yaw, RTBvector desired_heading)
{
    
    RTBvector localvec,localvec2;
    if (OA_inited == 0)
    {
        localvec.x=0;
        localvec.y=0;
        localvec.norm=0;
        localvec.angle_deg_north=0;
        localvec.angle_rad=M_PI_2;
        return localvec;        
    }
    RTB_FLOAT_TYPE angle, min_distance=50;
    long i,j;
    long index,angle_int;
    int left_reached_end = 0, left_valid = 0, right_reached_end = 0, right_valid = 0;
    OA_internal_map_update();
    
    //Range points discretization and mapping
    for (i=0; i<count; i++)
    {
        if (readings[i] < OA_ACTIVE_DISTANCE)
        {
            
#if (OA_SENSOR_ORIENTATION == OA_SENSOR_CLOCKWISE)
            angle = OA_SENSOR_CENTER + OA_internal_min_angle + (i*OA_SENSOR_RESOLUTION);
            //x=cos(yaw+OA_SENSOR_CENTER+(i*OA_SENSOR_RESOLUTION))*readings[i];
            //y=sin(yaw+OA_SENSOR_CENTER+(i*OA_SENSOR_RESOLUTION))*readings[i];
#elif (OA_SENSOR_ORIENTATION == OA_SENSOR_ANTI_CLOCKWISE)
            angle = yaw + OA_SENSOR_CENTER - OA_internal_min_angle - (i*OA_SENSOR_RESOLUTION);        
            //x=cos(yaw+OA_SENSOR_CENTER+((count-i)*OA_SENSOR_RESOLUTION))*readings[count-i];
            //y=sin(yaw+OA_SENSOR_CENTER+((count-i)*OA_SENSOR_RESOLUTION))*readings[count-i];
#endif
            angle = lucciSERVICE_rad_adjust(angle);
            if ((angle < M_PI_2) && (angle > -M_PI_2))
                min_distance = MIN(min_distance, readings[i]);
            angle = lucciSERVICE_rad_adjust(angle+yaw.angle_rad);
            angle_int = 900-floor(angle/M_PI*1800);
            
            if (angle_int > 3600)
            {
                angle_int=-3600+angle_int;            
            }
            if (angle_int < 0)
            {
                angle_int=3600+angle_int;
            }
            /*angle_int=-angle_int+900;
            if (angle_int < 0)
                angle_int += 3600;
            if (angle_int > 3600)
                angle_int -= 3600;*/

            index = (long)floor(angle_int/(10*OA_GRID_RADIAL_SAMPLING));
            OA_radial_map[index].intensity++;


            /*if (abs(x) < OA_ACTIVE_DISTANCE && abs(y) < OA_ACTIVE_DISTANCE)
            {
                x=floor((x+OA_ACTIVE_DISTANCE) / OA_GRID_RESOLUTION);
                y=floor((y+OA_ACTIVE_DISTANCE) / OA_GRID_RESOLUTION);
                OA_map[(long)x][(long)y]++;
            }*/
        }
    }        
    long _validity_count=0;
    for (i=0;i<OA_radial_size;i++)
    {
        if (OA_radial_map[i].intensity<OA_WINDOW_THRESHOLD)
        {
            _validity_count++;
        }
        else
            _validity_count=0;
        if (_validity_count == OA_WINDOW_SIZE)
        {
            OA_radial_map[i].validity = 1;

            for (j=0;j<_validity_count-1;j++)
                OA_radial_map[i-_validity_count+j+1].validity=1;
        }
        if (_validity_count > OA_WINDOW_SIZE)
            OA_radial_map[i].validity = 1;
    }
#if (DEBUG == 1)    
    //Radial discretization of the map
    output_pty = fopen("/home/erupter/data.csv","w+");
    //fprintf(output_pty,"START,");

    for (i=0;i<OA_radial_size-1;i++)
    {
        fprintf(output_pty,"%03d,",OA_radial_map[i].intensity);
    }
    fprintf(output_pty,"%03d\n",OA_radial_map[OA_radial_size].validity);
    for (i=0;i<OA_radial_size-1;i++)
    {
        fprintf(output_pty,"%03d,",OA_radial_map[i].validity);
    }
#endif    
    
    fclose(output_pty);
    
    index = (long)floor(desired_heading.angle_deg_north/(OA_GRID_RADIAL_SAMPLING));
    if (OA_radial_map[index].validity != 1)
    {
        long count_left=0, count_right=0;
        while((OA_radial_map[index-count_left].validity==0 && OA_radial_map[index+count_right].validity==0) && index-count_left>0 && index+count_right<OA_radial_size)
        {
            count_left++;
            count_right++;
        }
        
        if (index-count_left == 0)
            left_reached_end = 1;
        else
            if (OA_radial_map[index-count_left].validity == 1)
                left_valid = 1;
        
        if (index+count_right >= OA_radial_size)
            right_reached_end = 1;
        else
            if (OA_radial_map[index+count_right].validity == 1)
                right_valid = 1;
        
        if (!right_valid && !left_valid )
        {
            if (left_reached_end)
            {
                int temp=count_left;
                while ((OA_radial_map[OA_radial_size-count_left+temp].validity==0 && OA_radial_map[index+count_right].validity==0) && index+count_right < OA_radial_size)
                {
                    count_right++;
                    count_left++;
                }
                if (OA_radial_map[OA_radial_size-count_left+temp].validity == 1)
                    left_valid = 1;
                if (OA_radial_map[index+count_right].validity == 1)
                    right_valid = 1;
            }
            else
            {
                int temp=count_right;
                while (OA_radial_map[index-count_left].validity==0 && OA_radial_map[count_right-temp].validity == 0 && index-count_left < OA_radial_size)
                {
                    count_left++;
                    count_right++;
                }    
                if (OA_radial_map[index-count_left].validity == 1)
                    left_valid = 1;
                if (OA_radial_map[count_right-temp].validity == 1)
                    right_valid = 1;
            }
        }
        
        if (left_valid)
        {
            if (left_reached_end)
            {
                angle = lucciSERVICE_degnorth2rad(lucciSERVICE_deg_adjust((RTB_FLOAT_TYPE) 5 * (OA_radial_size+index-count_left) - 20));
            }
            else
            {
                angle = lucciSERVICE_degnorth2rad(lucciSERVICE_deg_adjust((RTB_FLOAT_TYPE) 5 * (index-count_left) - 20));
            }
        }
        else
        {
            if (right_valid)
            {
                if (right_reached_end)
                {
                    angle = lucciSERVICE_degnorth2rad(lucciSERVICE_deg_adjust((RTB_FLOAT_TYPE) 5 * (index + count_right - OA_radial_size) + 20));
                }
                else
                {
                    angle = lucciSERVICE_degnorth2rad(lucciSERVICE_deg_adjust((RTB_FLOAT_TYPE) 5 * (index + count_right) + 20));
                }
            }
        }
        
        
        localvec.x = cos(angle);
        localvec.y = sin(angle);
        localvec = lucciSERVICE_vect_normalize(localvec);
    }
    else
        localvec = desired_heading;
    //localvec = lucciSERVICE_vect_set_norm(OA_SPEED_MAX, localvec);
    
    //emergency avoid
    if (min_distance < OA_SPEED_MODERATION_DIST && localvec.norm > 0.0)
    {
        
        for (i=0; i<count; i++)
        {
            min_distance = MIN(min_distance,readings[i]);
            if (readings[i] == min_distance)
                index = i;
        }
#if (OA_SENSOR_ORIENTATION == OA_SENSOR_CLOCKWISE)
        angle = yaw.angle_rad + OA_SENSOR_CENTER + OA_internal_min_angle + (index*OA_SENSOR_RESOLUTION);
#elif (OA_SENSOR_ORIENTATION == OA_SENSOR_ANTI_CLOCKWISE)
        angle = yaw.angle_rad + OA_SENSOR_CENTER - OA_internal_min_angle - (i*OA_SENSOR_RESOLUTION);   
#endif
        angle = lucciSERVICE_rad_adjust(angle + M_PI);
        localvec2.x = cos (angle);
        localvec2.y = sin (angle);
        localvec = lucciSERVICE_vect_sum(localvec, localvec2);
        //localvec = lucciSERVICE_vect_set_norm(OA_AVOID_INTENSITY * (1 - min_distance / OA_SPEED_MODERATION_DIST),localvec);
        localvec = lucciSERVICE_vect_set_norm(MIN(2,min_distance / OA_AVOID_INTENSITY),localvec);
        RTBstatus.control_vector = localvec;
        avoid_soft = 1;
    }
    
    if (min_distance < OA_SAFETY_DISTANCE)
        localvec = lucciSERVICE_vect_set_norm(0.01, localvec);
    return localvec;
    
}


void OA_cleanup (void)
{
    long i;
    for (i=0;i<OA_map_size;i++)
    {
        free(OA_map[i]);
    }
    free(OA_map);
    
    free(OA_radial_map);
    
}