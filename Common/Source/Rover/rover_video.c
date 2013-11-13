#include <GL/gl.h>
#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <pthread.h>
#include <error.h>
#include "rover_video.h"

CIRCLE circle;
float max_circle_point;

RTB_point *circle_pointer;
RTB_point current_position;
RTB_point *circle_pointer_start;

pthread_t tid;

void GpsDouble2XYRelative(RTB_point *point_src, RTB_point *point_dest)
{
  double x, y;
  
  if(point_src != circle_pointer_start)
  {
    x = (point_src->x - point_src->previous->x) * cos(point_src->previous->y * 3.14 / 180) * 3.14 / 180;  // lon2-lon1 must be in radians 
    y = (point_src->y - point_src->previous->y) * 3.14 / 180; // lat2-lat1 must be in radians 
  
    point_dest->x = x * EARTH * 1000;
    point_dest->y = y * EARTH * 1000;
    
  }
  else
  {
    point_dest->x = 0;
    point_dest->y = 0;
  }
  
  //printf("X: %f Y: %f Src X: %f Src Y: %f\n", point_dest->x, point_dest->y, point_src->x, point_src->y);
}

void GpsDouble2XYAbsoluteFromStart(RTB_point *point_src, RTB_point *point_dest)
{
  double x, y;
  
  x = (point_src->x - circle_pointer_start->x) * cos(circle_pointer_start->y * 3.14 / 180) * 3.14 / 180;  // lon2-lon1 must be in radians 
  y = (point_src->y - circle_pointer_start->y) * 3.14 / 180; // lat2-lat1 must be in radians 
  
  point_dest->x = x * EARTH * 1000;
  point_dest->y = y * EARTH * 1000;

  
  //printf("X: %f Y: %f Src X: %f Src Y: %f\n", point_dest->x, point_dest->y, point_src->x, point_src->y);
}

void GpsDouble2XYAbsolute(RTB_point *point_src, RTB_point *point_dest, RTB_point *point_return)
{
  double x, y;
  
  x = (point_src->x - point_dest->x) * cos(point_dest->y * 3.14 / 180) * 3.14 / 180;  // lon2-lon1 must be in radians 
  y = (point_src->y - point_dest->y) * 3.14 / 180; // lat2-lat1 must be in radians 
  
  point_return->x = x * EARTH * 1000;
  point_return->y = y * EARTH * 1000;

  
  //printf("X: %f Y: %f Src X: %f Src Y: %f\n", point_dest->x, point_dest->y, point_src->x, point_src->y);
}

void createcircle (int k, float r, int h) 
{
  int i;
  glBegin(GL_LINES);
  for (i = 0; i < 180; i++)
  {
    circle.x = r * cos(i) - h;
    circle.y = r * sin(i) + k;
    glVertex3f(circle.x + k,circle.y - h,0);
     
    circle.x = r * cos(i + 0.1) - h;
    circle.y = r * sin(i + 0.1) + k;
    glVertex3f(circle.x + k,circle.y - h,0);
  }
  glEnd();
}

void createcross(float width, float heigth)
{
  glBegin(GL_LINES);
  glVertex3f(-width/2, 0, 0);
  glVertex3f(width/2, 0, 0);
  glVertex3f(0, -heigth/2, 0);
  glVertex3f(0, heigth/2, 0);
  glEnd();
}

void createreference(float width, float heigth)
{
  glBegin(GL_LINES);
  glVertex3f(-width/2, 0, 0);
  glVertex3f(width/2, 0, 0);
  
  glVertex3f(0, -heigth/2, 0);
  glVertex3f(0, heigth/2, 0);
  
  glVertex3f(0, heigth/2, 0);
  glVertex3f(width/2, 0, 0);
  
  glVertex3f(0, heigth/2, 0);
  glVertex3f(-width/2, 0, 0);
  
  glEnd();
}

void display(void) 
{
  int count = 0;
  RTB_point *local_point;
  RTB_point temp_point_circle;
  RTB_point temp_point_cross;
  
  local_point = circle_pointer;
  
  glClearColor(0.0,0.0,0.0,1.0);
  glClear(GL_COLOR_BUFFER_BIT);
  glLoadIdentity();
  glTranslatef(0, 0, -max_circle_point * 2);
  
  while(local_point->previous != NULL)
  {
    count++;
    GpsDouble2XYRelative(local_point, &temp_point_circle);
     //glRotatef(rot,0,1,0);
     //glRotatef(rot,1,0,0);
     //glRotatef(rot,0,0,1);
    glColor3f(1,1,0);
    createcircle(1,0.2,1); 
    glTranslatef(-temp_point_circle.x, -temp_point_circle.y, 0);
    local_point = local_point->previous;
  }
  count++;
  //draw the first point
  //gluLookAt(circle_pointer_start->x, circle_pointer_start->y, 0, circle_pointer_start->x, circle_pointer_start->y, -max_circle_point * 2, 0, 1, 0);
  glColor3f(0,0,1);
  createcircle(1,0.2,1);

  glLoadIdentity();
  glTranslatef(0, 0, -max_circle_point * 2);
  
  //local_point = current_position;
  GpsDouble2XYAbsoluteFromStart(&current_position, &temp_point_cross);
  GpsDouble2XYAbsoluteFromStart(circle_pointer, &temp_point_circle);
  glTranslatef(temp_point_cross.x - temp_point_circle.x, temp_point_cross.y - temp_point_circle.y, 0);
  glColor3f(1,0,0);
  createcross(max_circle_point/10, max_circle_point/10);
  
  glutSwapBuffers();
}

void rtb_video_take_point(RTB_point *points)
{
  RTB_point temp_point_dest;
  RTB_point *temp_point = points;
  
  //printf("rtb_video_take_point\n");
  circle_pointer = points;
  
  if(circle_pointer == circle_pointer_start)
    max_circle_point = 1;
  else
  {
    temp_point = temp_point->previous;
  
    // calculate max distance from start for every point
    while(temp_point->previous != NULL)
    {
      GpsDouble2XYAbsolute(circle_pointer, temp_point, &temp_point_dest);
    
      if(max_circle_point < sqrt(pow(temp_point_dest.x, 2) + pow(temp_point_dest.y, 2)))
        max_circle_point = sqrt(pow(temp_point_dest.x, 2) + pow(temp_point_dest.y, 2));
    
      temp_point = temp_point->previous;
    }

    GpsDouble2XYAbsolute(circle_pointer, temp_point, &temp_point_dest);
    
    if(max_circle_point < sqrt(pow(temp_point_dest.x, 2) + pow(temp_point_dest.y, 2)))
      max_circle_point = sqrt(pow(temp_point_dest.x, 2) + pow(temp_point_dest.y, 2));
  }
  
  printf("Max: %f\n", max_circle_point);
}

void rtb_video_take_current_position(double x, double y)
{
  RTB_point temp_point_dest;
  
  //current_position = points;
  current_position.x = x;
  current_position.y = y;
 
  printf("rtb_video_take_current_position  x: %f y: %f\n", current_position.x, current_position.y);
  
  GpsDouble2XYAbsoluteFromStart(&current_position, &temp_point_dest);
  
  if(max_circle_point < sqrt(pow(temp_point_dest.x, 2) + pow(temp_point_dest.y, 2)))
    max_circle_point = sqrt(pow(temp_point_dest.x, 2) + pow(temp_point_dest.y, 2)); 
}

void reshape(int w, int h) 
{
  glViewport (0, 0, (GLsizei)w, (GLsizei)h);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity();
  gluPerspective (60, (GLfloat)w / (GLfloat)h, 0.1, 100.0);
  glMatrixMode (GL_MODELVIEW);
}
 
void rtb_video_init_point(RTB_point *start_point)
{
  circle_pointer_start  = start_point;
  circle_pointer = start_point;
  //current_position = start_point;
  current_position.x = start_point->x;
  current_position.y = start_point->y;
  
  max_circle_point = 1;
}

void rtb_video_init_glut(RTB_point *start_point)
{
  int argc = 1;
  glutInit(&argc, NULL);
  glutInitDisplayMode(GLUT_DOUBLE);
  glutInitWindowSize(500, 500);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Stand-off trajectory");
  
  rtb_video_init_point(start_point);
  
  if(pthread_create(&tid, NULL, &video_loop, NULL) != 0)
    perror("pthread_create");
}

void *video_loop()
{
  glutDisplayFunc(display);
  glutIdleFunc(display);
  glutReshapeFunc(reshape);
  
  glutMainLoop();
  
  return NULL;
}

/*
int main (int argc, char **argv)
{
  video_init(12, 41);
  video_loop();

  return 0;
}*/