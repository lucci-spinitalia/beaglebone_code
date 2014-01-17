#ifndef _KALMAN_H_
#define _KALMAN_H_

/* Prototype  */
void kalman_reset(double qx, double qy, double rx, double ry, double pd, double ix, double iy);
void kalman_estimate(double velocity_ms, double direction_deg, long time_us);
void kalman_update(double *lon, double *lat);
#endif