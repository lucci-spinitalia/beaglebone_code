#include <nmea/nmea.h>

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#ifdef NMEA_WIN
#   include <io.h>
#endif

void trace(const char *str, int str_size)
{
/*    printf("Trace: ");
    write(1, str, str_size);
    printf("\n");*/
}
void error(const char *str, int str_size)
{
    printf("Error: ");
    write(1, str, str_size);
    printf("\n");
}

int main(int argc, char *argv[])
{
    nmeaINFO info;
    nmeaPARSER parser;
    FILE *file;
    char buff[2048];
    int size, it = 0;
    nmeaPOS dpos;

	if(argc > 1)
	  file = open(device_name, O_RDWR | O_NOCTTY);
	else
      file = fopen("gpslog.txt", "rb");

    if(!file)
        return -1;

    //nmea_property()->trace_func = &trace;
    nmea_property()->error_func = &error;

    nmea_zero_INFO(&info);
    nmea_parser_init(&parser);

    /*
    while(1)
    {
    */

    while(!feof(file))
    {
        size = (int)fread(&buff[0], 1, 100, file);

        nmea_parse(&parser, &buff[0], size, &info);

        nmea_info2pos(&info, &dpos);

        /*printf(
            "%03d, Lat: %f, Lon: %f, Sig: %d, Fix: %d\n",
            it++, dpos.lat, dpos.lon, info.sig, info.fix
            );*/
		if(it > 0)
        {
          printf("\033[14A");
        }
        else
		  it++;
      
        printf("Time: %i/%i/%i %i:%i:%i.%i\n", info.utc.day, info.utc.mon + 1, info.utc.year + 1900, info.utc.hour, info.utc.min, info.utc.sec, info.utc.hsec);
        printf("Signal: %i\n", info.fix);
        printf("Position Diluition of Precision: %f\n", info.PDOP);
        printf("Horizontal Diluition of Precision: %f\n", info.HDOP);
        printf("Vertical Diluition of Precisione: %f\n", info.VDOP);
        printf("Latitude: %f\n", info.lat);
        printf("Longitude: %f\n", info.lon);
        printf("Elevation: %f m\n", info.elv);
        printf("Speed: %f km/h\n", info.speed);
        printf("Direction: %f degrees\n", info.direction);
        printf("Magnetic variation degrees: %f\n", info.declination); 
    
        printf("\nSatellite: \tin view: %i\n\t\tin use: %i\n", info.satinfo.inview, info.satinfo.inuse);
    }

    fseek(file, 0, SEEK_SET);

    /*
    }
    */

    nmea_parser_destroy(&parser);
    fclose(file);

    return 0;
}
