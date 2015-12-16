#include "packet_car_telemetry.h"

struct Car_telemetry car_telemetry_data;


unsigned char car_telemetry_serialize(char* done) {
    GENERIC_SERIALIZE(Car_telemetry, car_telemetry_data);
}

char car_telemetry_deserialize(unsigned char ch) {
    GENERIC_DESERIALIZE(Car_telemetry, car_telemetry_data);
}

void car_telemetry_reset(void) {
    return;
}
