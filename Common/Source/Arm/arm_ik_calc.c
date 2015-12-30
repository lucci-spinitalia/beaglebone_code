#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arm_rs485.h>

int main(int argc, char **argv)
{
  float teta1;
  float teta2;
  float teta3;

  long step_motor[3];

  if(argc < 4)
  {
    printf("Error: parameter not valid\n");
    printf("Usage: %s <x> <y> <z>\n", argv[0]);
    printf("\t x: coordinata detta \"destra/sinistra\"\n");
    printf("\t y: coordinata detta \"in avanti/indietro\"\n");
    printf("\t z: coordintata detta \"sù/giù\"\n");

    return 1;
  }

  arm_link[0].gear = 220;
  arm_link[1].gear = 880;
  arm_link[2].gear = 220;

  printf("Calculating ik from parameter. . .\n");
  arm_ik_ang(atof(argv[1]), atof(argv[2]), atof(argv[3]), &teta1, &teta2, &teta3);

  step_motor[0] = (long)(teta1*2000*220/3.14);
  step_motor[1] = (long)(teta2*2000*880/3.14);
  step_motor[2] = (long)(teta3*2000*220/3.14);

  printf("Result IK:\n");
  printf("\tteta1: %f grad <=> %ld step\n", teta1 * 180 / 3.14, (long)(teta1*2000*220/3.14));
  printf("\tteta2: %f <=> %ld step\n", teta2* 180 / 3.14, (long)(teta2*2000*880/3.14));
  printf("\tteta3: %f <=> %ld step\n", teta3* 180 / 3.14, (long)(teta3*2000*220/3.14));

  printf("Calculating dk from ik. . .\n");

  float x;
  float y;
  float z;

  arm_ee_xyz(step_motor, &x, &y, &z);

  printf("Result DK:\n");
  printf("\tx: %f m\n", x);
  printf("\ty: %f m\n", y);
  printf("\tz3: %f m\n", z);

  return 0;
}
