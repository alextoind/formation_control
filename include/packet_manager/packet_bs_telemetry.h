#ifndef __PACKETBS_TELEMERY_H__
#define __PACKETBS_TELEMERY_H__

#include "c2000_type.h"
#include "packet_manager.h"


// 95 bytes (data 23*4 + 3) + 3 (header) + 1 (checksum) = 99 bytes (total)
struct Bs_telemetry {
	// time at which the basestation receive a new pck from car
	i_float counter_value_at_recv;

	// bs gps information
  i_float bs_ecef_px;
	i_float bs_ecef_py;
	i_float bs_ecef_pz;
	i_float bs_ecef_vx;
	i_float bs_ecef_vy;
	i_float bs_ecef_vz;
	i_uint8 bs_fix;
  i_float bs_time_fix;

	// car gps information
  i_float car_ecef_px;
	i_float car_ecef_py;
	i_float car_ecef_pz;
	i_float car_ecef_vx;
	i_float car_ecef_vy;
	i_float car_ecef_vz;
	i_uint8 car_fix;
	i_float car_time_fix;

  // car megnetometer information
	i_float car_mag_x;
	i_float car_mag_y;
	i_float car_mag_z;

	// execution mode
	i_uint8 car_mode;

	// pi commands
	i_float car_pi_motore;
	i_float car_pi_sterzo;

	// pwm commands
	i_float car_pwm_motore;
	i_float car_pwm_sterzo;

	// heading_error
	i_float car_heading_error;
};

unsigned char bs_telemetry_serialize(char* done);
char bs_telemetry_deserialize(unsigned char ch);
void bs_telemetry_reset(void);

extern struct Bs_telemetry bs_telemetry_data;

#endif
