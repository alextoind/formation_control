#include "packet_manager.h"

//#define USE_ERR_CALLBACK

struct PacketDescriptor 
{
    unsigned char packet_header;
    SerializeFcn serialize;
    DeserializeFcn deserialize;
	ResetFcn reset;
};

struct PacketManager
{
	// stores the number of registered packets
    unsigned char registered_packets;
    struct PacketDescriptor packets[MAX_NR_PACKET_TYPES];
};

struct PacketManager pmgr;

NewPacketFcn new_packet_callback;
ErrorFcn error_deserialize_callback;
ErrorFcn error_serialize_callback;

short find_packet_idx_from_id(unsigned char id)
{
    // pm_register_packet adds a entry for every pmanagers[] element
    // therefore it's sufficient to search only in pmanagers[0] for
    // the specified header
    unsigned char i;
    for (i = 0; i < pmgr.registered_packets; i++)
    {
        if (pmgr.packets[i].packet_header == id)
            return i;
    }
    return -1;
}

short header_seq_recognizer(unsigned char ch)
{
	static char seq_rec_state = 0;
	short packet_idx = 0;

	switch(seq_rec_state)
	{
	case 0:
		seq_rec_state = (ch == PM_HEADER) ? 1 : 0;
		break;
	case 1:
		seq_rec_state = (ch == PM_HEADER) ? 2 : 0;
		break;
	case 2:
		seq_rec_state = 0;
		packet_idx = find_packet_idx_from_id(ch);
		
		if (packet_idx != -1)
		{			
			return packet_idx;
		}

		break;
	}
	return -1;
}

char verify_checksum(unsigned long recv_cs_calculated, unsigned char cs)
{
	return ((recv_cs_calculated & 0xFF) == cs) ? 1 : 0;
}

void pm_init(NewPacketFcn _new_packet_fcn, ErrorFcn _error_deserialize_fcn, ErrorFcn _error_serialize_fcn)
{
    unsigned char i = 0;

    for (i = 0; i < MAX_NR_PACKET_TYPES; i++)
    {
        pmgr.packets[i].serialize = 0;
        pmgr.packets[i].deserialize = 0;
        pmgr.packets[i].reset = 0;
    }
	// packet manager state variables
    pmgr.registered_packets = 0;        

	new_packet_callback = _new_packet_fcn;
	error_deserialize_callback = _error_deserialize_fcn;
	error_serialize_callback = _error_serialize_fcn;
}

void pm_register_packet(unsigned char _header, SerializeFcn _serialize, DeserializeFcn _deserialize, ResetFcn _reset)
{
    struct PacketDescriptor pdesc;
    
    pdesc.packet_header = _header;
    pdesc.serialize = _serialize;
    pdesc.deserialize = _deserialize;    
	pdesc.reset = _reset;
    
    pmgr.packets[pmgr.registered_packets++] = pdesc;
}

short _pm_send_byte(unsigned char id, char* sent)
{
    unsigned char data;
	char done;
    static unsigned char send_state = 0;
    static short send_idx = -1;
    static unsigned long send_cs_calculated = 0;
    *sent = 0;
    
    if (send_state == 0) {
        // send the first PM_HEADER
        send_state++;
        return PM_HEADER;
    } 
    else if (send_state == 1) {
        // send the second PM_HEADER
        send_state++;
        return PM_HEADER;
    }
    else if (send_state == 2) {
        // if registered, send the packet header otherwise return -1
        send_idx = find_packet_idx_from_id(id);

        if (send_idx != -1) {
			send_state++;
            return id;
        } 
        else {
			// notify the user that the specified header packet was not registerd, errno=1
			if (error_serialize_callback)
            {
#ifdef USE_ERR_CALLBACK
                error_serialize_callback(id, 1);
#endif
            }
            send_state = 0;
            return -200;
        }                
    }
    else if (send_state == 3) {
        // send the packet payload
        if (pmgr.packets[send_idx].serialize)
        {
            data = pmgr.packets[send_idx].serialize(&done);
        }
        else
        {
            // notify the user that the specified header packet was not registerd, errno=1
			if (error_serialize_callback)
            {
#ifdef USE_ERR_CALLBACK                
                error_serialize_callback(id, 1);
#endif
            }
            send_state = 0;
            return -200;
        }
		// sum up the data sent for the checksum calc
		send_cs_calculated += data;

        if (done == 1) {
			// reset the index of the packet to be sent to -1 to allow a new sending operation
            send_idx = -1;
            send_state++;
        }
        return data;       
    }
    else if (send_state == 4) {
		unsigned char csbyte = (unsigned char)(send_cs_calculated & 0xFF);
		send_cs_calculated = 0;
        send_state = 0;
		*sent = 1;
		return csbyte;
	}
	return -200;
}

void _pm_process_byte(unsigned char ch)
{
    static char need_cs = 0;
	static short current_idx = -1;
    static unsigned long recv_cs_calculated = 0;
    
    if (current_idx == -1)
    {
        current_idx = header_seq_recognizer(ch);
    }
    else
    {
        if (need_cs == 0)
        {
            recv_cs_calculated += ch;
            
            if (pmgr.packets[current_idx].deserialize)
            {
                if (pmgr.packets[current_idx].deserialize(ch))
                {
                    need_cs = 1;
                }
            }
            else
            {
                // error_deserialize_callback not registered
                if (error_deserialize_callback)
                {
#ifdef USE_ERR_CALLBACK                    
                    error_deserialize_callback(pmgr.packets[current_idx].packet_header, 0);
#endif
                }
            }
        }
        else
        {
            if (verify_checksum(recv_cs_calculated, ch) == 1)
            {
                if (new_packet_callback)
                    new_packet_callback(pmgr.packets[current_idx].packet_header);
            }
            else
            {
                if (error_deserialize_callback)
                {
#ifdef USE_ERR_CALLBACK                    
                    error_deserialize_callback(pmgr.packets[current_idx].packet_header, 0);
#endif
                }
            }
            need_cs = 0;
            current_idx = -1;
            recv_cs_calculated = 0;
        }
    }
}
