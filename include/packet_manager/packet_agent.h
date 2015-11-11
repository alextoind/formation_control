#ifndef __PACKETAGENT_H__
#define __PACKETAGENT_H__

#include "c2000_type.h"
#include "packet_manager.h"


struct Statistics {
  i_float m_x;
  i_float m_y;
  i_float m_xx;
  i_float m_xy;
  i_float m_yy;
};

struct TargetStatistics {
  struct Statistics stats;
};

struct ReceivedStatistics {
  i_uint8 number_of_agents;
  struct Statistics stats_sum;
};

unsigned char target_statistics_serialize(char* done);
char target_statistics_deserialize(unsigned char ch);
void target_statistics_reset(void);

unsigned char received_statistics_serialize(char* done);
char received_statistics_deserialize(unsigned char ch);
void received_statistics_reset(void);

struct Agent {
  i_uint8 agent_id;

  // estimated statistics
  struct Statistics stats;

  // virtual agent pose
  i_float pose_x_virtual;
  i_float pose_y_virtual;
  i_float pose_theta_virtual;

  // agent pose (from GPS)
  i_float pose_x;
  i_float pose_y;
  i_float pose_theta;
};

unsigned char agent_serialize(char* done);
char agent_deserialize(unsigned char ch);
void agent_reset(void);


extern struct TargetStatistics target_statistics_data;  // output packet (from ROS's point of view)
extern struct ReceivedStatistics received_statistics_data;  // output packet (from ROS's point of view)
extern struct Agent agent_data;  // input packet (from ROS's point of view)

#endif
