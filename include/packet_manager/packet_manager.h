#ifndef __PACKETMANAGER_H__
#define __PACKETMANAGER_H__

#define MAX_NR_PACKET_TYPES     30	// MAX NUMBER IS 255
#define PM_HEADER               255

typedef unsigned char (*SerializeFcn)(char* done);
typedef char (*DeserializeFcn)(unsigned char ch);
typedef void (*ResetFcn)(void);

typedef void (*NewPacketFcn)(unsigned char header);
typedef void (*ErrorFcn)(unsigned char header, unsigned char errno);

void pm_init(NewPacketFcn _newPacketFcn, ErrorFcn _errorDeserializeFcn, ErrorFcn _errorSerializeFcn);
void pm_register_packet(unsigned char _header, SerializeFcn _serialize, DeserializeFcn _deserialize, ResetFcn _reset);



#define GENERIC_SERIALIZE(DATA_STRUCT,DATA_INSTANCE) 	static int s_idx = 0; unsigned char b; \
														b = ((unsigned char*)&DATA_INSTANCE)[s_idx++]; \
														if (s_idx == sizeof(struct DATA_STRUCT)) \
															{ s_idx = 0; \
																	*done = 1; \
															} else { \
																	*done = 0; \
															} \
														return b  

														
#define GENERIC_DESERIALIZE(DATA_STRUCT,DATA_INSTANCE) 	static int d_idx = 0; \
																											((unsigned char*)&DATA_INSTANCE)[d_idx++] = ch; \
																											if (d_idx == sizeof(struct DATA_STRUCT)) { \
																													d_idx = 0; \
																													return 1; \
																											} else { \
																													return 0; \
																											}	
														


#endif
