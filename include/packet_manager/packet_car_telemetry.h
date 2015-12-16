#ifndef __PACKETCAR_TELEMERY_H__
#define __PACKETCAR_TELEMERY_H__

#include "c2000_type.h"
#include "packet_manager.h"

// 62 bytes (data 15*4 + 2) + 3 (header) + 1 (checksum) = 66 bytes (total)
struct Car_telemetry {
	// gps information
  i_float ecef_px;
	i_float ecef_py;
	i_float ecef_pz;
	i_float ecef_vx;
	i_float ecef_vy;
	i_float ecef_vz;
	i_uint8 fix;
  i_float time_fix;
    
  //megnetometer information
	i_float mag_x;
	i_float mag_y;
	i_float mag_z;
    
  //execution mode
	i_uint8 mode;
    
  //pi commands
	i_float pi_motore;
	i_float pi_sterzo;

  //pwm commands
  i_float pwm_motore;
  i_float pwm_sterzo;
  
  //heading_error
  i_float heading_error;
};

unsigned char car_telemetry_serialize(char* done);
char car_telemetry_deserialize(unsigned char ch);
void car_telemetry_reset(void);

extern struct Car_telemetry car_telemetry_data;

#endif
