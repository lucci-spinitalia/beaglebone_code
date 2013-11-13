#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[])
{
  int i;
  int checksum = 0;
  
  if(argc < 2)
  {
    printf("Usage: %s <gps string without $>\n", argv[0]);
    return 1;
  }
  
  
  // the checksum is calculated without $ and * characters
  for(i = 0; i < strlen(argv[1]); i++)
    checksum ^= *(argv[1] + i);

  printf("Checksum: %x\n", checksum);
  
  return 0;
}