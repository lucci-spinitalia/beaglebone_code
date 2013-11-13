#ifndef _ROVER_VIDEO_H
#define _ROVER_VIDEO_H

#include "rover_rtb.h"

typedef struct
{
  float x;
  float y;
} CIRCLE;

/* Prototype */
void rtb_video_take_point(RTB_point *points);
void rtb_video_take_current_position(double x, double y);
void rtb_video_init_glut(RTB_point *start_point);
void rtb_video_init_point(RTB_point *start_point);
void *video_loop();

#endif