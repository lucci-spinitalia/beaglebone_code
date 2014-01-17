#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_linalg.h>

#include "kalman.h"

#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

#define DATA_PATH "gps_compare_log_final4.csv"

double m_q[4]; //process error 4x4
double m_r[4];  //measure error 2x2
double m_p[4]; //predicted covariance matrix 4x4

/* Position matrix
 * [ x ]
 * [ y ]
 */
double m_x[2];  // Current x and y position 
double m_u[2];  // Input matrix. 

//double error[2][2];

double longitude, latitude;
double x_old = 0;
double y_old = 0;

void kalman_reset(double qx, double qy, double rx, double ry, double pd, double ix, double iy)
{ 
  m_q[0] = qx;
  m_q[1] = 0;
  m_q[2] = 0;
  m_q[3] = qy;
  
  m_r[0] = rx;
  m_r[1] = 0;
  m_r[2] = 0;
  m_r[3] = ry;
  
  /*
   * [pd  0  0   0]
   * [0  pd  0   0]
   * [0   0 pd   0]
   * [0   0  0  pd]
   */
  m_p[0] = pd;
  m_p[1] = 0;
  m_p[2] = 0;
  m_p[3] = pd;
  
  m_x[0] = ix; // x
  m_x[1] = iy; // y
  
  m_u[0] = 0; // x
  m_u[1] = 0; // y
  
}

/* Estimate:
 * X = F*X
 * 
 * [  x     y  ] = [1 dt] * [  x     y  ]
 * [dx/dt dy/dt]   [0  1]   [dx/dt dy/dt]
 * 
 */
void kalman_estimate(double velocity_ms, double direction_deg, long time_us)
{
  int i;
  //int j;
  double delta_x, delta_y, delta_position;
  
  // Compute delta_x and delta_y from input
  delta_position = velocity_ms * time_us / 1000000; // [m]
  delta_x = delta_position * sin(direction_deg * M_PI / 180);
  delta_y = delta_position * cos(direction_deg  * M_PI / 180);
  
  gsl_vector_view U = gsl_vector_view_array(m_u, 2);
  gsl_vector_view X = gsl_vector_view_array(m_x, 2);
  gsl_matrix_view Q = gsl_matrix_view_array(m_q, 2, 2);
  gsl_matrix_view P = gsl_matrix_view_array(m_p, 2, 2);
  
 /* for(i = 0; i < F.matrix.size1; i++)
  {
    printf("[");
    for(j = 0; j < F.matrix.size2; j++)
      printf("%f ", gsl_matrix_get(&F.matrix, i, j));
    
    printf("]\n");
  }

  for(i = 0; i < X.vector.size; i++)
  {
    printf("[");
    printf("%f ", gsl_vector_get(&X.vector, i));
    printf("]\n");
  }*/
  
  m_u[0] = delta_x;
  m_u[1] = delta_y;
  
  x_old = m_x[0];
  y_old = m_x[1];
  
  gsl_blas_daxpy(1.0, &U.vector, &X.vector);

  /*printf("X estimate\n");
  for(i = 0; i < X.vector.size; i++)
  {
    printf("[");
    printf("%f ", gsl_vector_get(&X.vector, i));
    printf("]\n");
  }*/
  
  /* State prediction covariance
   * P(k+1|k) = F(k)P(k|k)F(k)'+ Q(k)
   */    
  gsl_matrix_add(&P.matrix, &Q.matrix);

  /*printf("Prediction covariance matrix\n");
  for(i = 0; i < P.matrix.size1; i++)
  {
    printf("[");
    for(j = 0; j < P.matrix.size2; j++)
      printf("%f ", gsl_matrix_get(&P.matrix, i, j));
    
    printf("]\n");
  }*/
}

void kalman_update(double *lon, double *lat)
{
  int i, j;
  static double x_meas = 0;
  static double y_meas = 0;
  static double latitude = 0;
  static double longitude = 0;
  static double lat_old = 0;
  static double lon_old = 0;
  
  if((longitude == 0) || (latitude == 0))
  {
	longitude = *lon;
	latitude = *lat;
	lat_old = *lat;
	lon_old = *lon;
  }
  else
  { 
    //dx = R*(a2-a1)*(pi/180)*cos(b1)
    x_meas += 6367000 * (*lon - lon_old) * (M_PI / 180) * cos(lat_old * M_PI / 180);

    //dy = R*(b2-b1)*pi/180
    y_meas += 6367000 * (*lat - lat_old) * M_PI / 180;

    lon_old = *lon;
    lat_old = *lat;
  }
  
  gsl_vector_view X = gsl_vector_view_array(m_x, 2);
  gsl_matrix_view P = gsl_matrix_view_array(m_p, 2, 2);
  gsl_matrix_view R = gsl_matrix_view_array(m_r, 2, 2);
  
  gsl_vector *masure_residual = gsl_vector_alloc(2);
  gsl_matrix *state_cov_temp = gsl_matrix_alloc(2, 2);
  gsl_matrix *P_error = gsl_matrix_alloc(2, 2);
  gsl_matrix *W = gsl_matrix_alloc(2, 2);
  gsl_matrix *S = gsl_matrix_alloc(2, 2);
  gsl_matrix *S_temp = gsl_matrix_alloc(2, 2);
  gsl_matrix *S_inv = gsl_matrix_alloc(2, 2);
  int s_sign;
  gsl_permutation *S_perm = gsl_permutation_alloc(2);
  
  // Measurement residual  = new_measure - H x
  gsl_vector_set(masure_residual, 0, x_meas);
  gsl_vector_set(masure_residual, 1, y_meas);
  
  gsl_vector_sub(masure_residual, &X.vector);
  
  /*printf("Measure Residual\n");
  for(i = 0; i < masure_residual->size; i++)
  {
    printf("[");
    printf("%f ", gsl_vector_get(masure_residual, i));
    printf("]\n");
  }*/
  
  /* Measurement prediction covariance
   * S = H P H' + R 
   */
   // init to m_r array to avoid one more math
  for(i = 0; i < P.matrix.size1; i++)
  {
    for(j = 0; j < P.matrix.size2; j++)
      gsl_matrix_set(S, i, j, gsl_matrix_get(&P.matrix, i, j));
  }
  
  gsl_matrix_add(S, &R.matrix);

  /* Kalman gain
   * W = P H' S^(-1)
   */
  // S^(-1)
  // make LU decomposition an S matrix
  // from now on S contain the LU decomposition matrix of itself  
  for(i = 0; i < S->size1; i++)
  {
    for(j = 0; j < S->size2; j++)
      gsl_matrix_set(S_temp, i, j, gsl_matrix_get(S, i, j));
  }
  
  gsl_linalg_LU_decomp(S_temp, S_perm, &s_sign);
  
  // Inverting the matrix S
  gsl_linalg_LU_invert(S_temp, S_perm, S_inv);
  
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &P.matrix, S_inv, 0.0, W);
  
  printf("Kalman Gain\n");
  for(i = 0; i < W->size1; i++)
  {
    printf("[");
    for(j = 0; j < W->size2; j++)
      printf("%f ", gsl_matrix_get(W, i, j));
    
    printf("]\n");
  }
  
  // Update state estimate
  gsl_blas_dgemv(CblasNoTrans, 1.0, W, masure_residual, 1.0, &X.vector);
  
  /*printf("X update\n");
  for(i = 0; i < X.vector.size; i++)
  {
    printf("[");
    printf("%f ", gsl_vector_get(&X.vector, i));
    printf("]\n");
  }*/
  
  /* (a2 - a1) = dx * 180 / (R * pi * cos(b1))*/
  longitude += (m_x[0] - x_old) * 180 / (M_PI * 6367000 * cos(latitude * M_PI / 180));
  latitude += (m_x[1] - y_old) * 180 / (M_PI * 6367000);
	
  // Return corrected position
  *lon = longitude;
  *lat = latitude;
  
  /*Update state covariance
   * P = P - W S W'
   */
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, S, W, 0.0, state_cov_temp);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, W, state_cov_temp, 0.0, P_error);
  gsl_matrix_sub(&P.matrix, P_error);
  
  gsl_vector_free(masure_residual);
  gsl_matrix_free(W);
  gsl_matrix_free(S);
  gsl_matrix_free(S_temp);
  gsl_matrix_free(S_inv);
  gsl_permutation_free(S_perm);
}


int read_gps_data(const char *file_path, double *lon, double *lat, double *velocity, double *direction, long *time)
{  
  FILE *file = NULL;
  char *line = NULL;
  size_t len = 0;
  ssize_t read;
  static int cursor_position = 0;
  
  char *token;
  int count = 0;
  
  // Init Log File
  file = fopen(file_path, "r");
  
  if(file == NULL)
    return -1;
  
  if(fseek(file, cursor_position, SEEK_SET) == -1)
  {
    fclose(file);
    return -1;
  }

  if((read = getline(&line, &len, file)) != -1)
  {
    //printf("Line read: %s Byte read: %d\n", line, read);
    
    //arm_message_log("arm_read_path", line);
    token = strtok(line,",\n");
    while(token != NULL)
    {
      switch(count)
      {
	case 0:
	  *lon = atof(token);
	  //printf("lon: %f\t", lon);
	  break;
	  
	case 1:
	  *lat = atof(token);
	  //printf("lat: %f\n", lat);
	  break;
	  
	case 2:
	  *velocity = atof(token);
	  //printf("vel: %f\t", *velocity);
	  break;
	  
	case 3:
	  *direction = atof(token);
	  //printf("dir: %f\t", *direction);
	  break;
	  
	case 4:
	  *time = atol(token);
	  //printf("time: %lu\n", *time);
	  break;
	  
	default:
	  break;
      }

      token = strtok(NULL,",\n");
      count++;
    }
    
    //printf("X: %f\tY: %f\n", *x, *y);
    
    cursor_position += read;
  }
  else
  {
    //*cursor_position = 0;
    free(line);
    fclose(file);
    return 0;
  }
  
  free(line);
  fclose(file);
  
  return 1;
}

void write_gps_data(double latitude, double longitude)
{
  FILE *file = NULL;

  // Init Log File
  file = fopen("gps_data_kalman_q0001_r2", "a");
  
  if(!file)
  {
    perror("logfile fopen:");
    return;
  }
  
  fprintf(file, "%f,%f,0\n", longitude, latitude);

  fclose(file);
}

/*void signal_handler(int signum)
{
  // Garbage collection
  printf("Terminating program...\n");
  
  exit(signum);
}


int main()
{
  double lon, lat, velocity_ms, direction_deg;
  long time_us;
  
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
   
  // Init kalman filter
  //(double qx, double qy, double rx, double ry, double pd, double ix, double iy, double iux, double iuy)
  kalman_reset(0.0001, 0.0001, 2, 2, 0.01, 0, 0);
  
  while(read_gps_data(DATA_PATH, &lon, &lat, &velocity_ms, &direction_deg, &time_us))
  {  
    kalman_estimate(velocity_ms, direction_deg, time_us);
    kalman_update(&lon, &lat);
    
    write_gps_data(lat, lon);
    printf("\n");
  }

  return 0;
}*/

