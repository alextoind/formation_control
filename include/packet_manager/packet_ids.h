#ifndef __PACKETIDS_H__
#define __PACKETIDS_H__

#define PCK_CMD 1
#define PCK_TEL 2
#define PCK_STA 3
#define PCK_TLF 4
#define PCK_GPS 5
#define PCK_GPS2 6
#define PCK_CMDTEL 7
#define PCK_DEBUG_6_FLOATS 8
#define PCK_TELEMETRY_TYPE 9
#define PCK_STATS_TELEMETRY 10
#define PCK_STATS_I2C_TELEMETRY 11
#define PCK_GPS_STATUS 12
#define PCK_ATT 13
#define PCK_FUZZYGUIDANCE 14
#define PCK_FUZZYGUIDANCE_ENABLE 15
#define PCK_GPS_TIME 16
#define PCK_CAR_TELEMETRY 17
#define PCK_BS_TELEMETRY 18
#define PCK_PC_TO_BS 19
#define PCK_BS_TO_CAR 20
#define PCK_PC_TO_CAR 21

#define PCK_TARGET 31
#define PCK_RECEIVED 32
#define PCK_AGENT 33

//waypoint management
#define PCK_WAYP 50
#define PCK_WAYP_EXT 51
#define PCK_WAYP_ACK 52
#define PCK_WAYP_CMD 53
//new waypoint define for basestation request
#define PCK_WAYP_RCV_EXT 55
#define PCK_WAYP_REQ_ACK 56

//pacchetti per macchine virtuali
#define PCK_VIRTUAL_CAR 70

#endif
