#include "packet_bs_telemetry.h"

struct Bs_telemetry bs_telemetry_data;


unsigned char bs_telemetry_serialize(char* done) {
    GENERIC_SERIALIZE(Bs_telemetry, bs_telemetry_data);
}

char bs_telemetry_deserialize(unsigned char ch) {
    GENERIC_DESERIALIZE(Bs_telemetry, bs_telemetry_data);
}

void bs_telemetry_reset(void) {
    return;
}
