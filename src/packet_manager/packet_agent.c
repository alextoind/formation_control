#include "packet_agent.h"

struct Statistics target_statistics_data;
struct Statistics received_statistics_data;
struct Agent agent_data;


unsigned char target_statistics_serialize(char* done) {
  GENERIC_SERIALIZE(Statistics, target_statistics_data);
}

char target_statistics_deserialize(unsigned char ch) {
  GENERIC_DESERIALIZE(Statistics, target_statistics_data);
}

void target_statistics_reset(void) {
  return;
}


unsigned char received_statistics_serialize(char* done) {
  GENERIC_SERIALIZE(Statistics, received_statistics_data);
}

char received_statistics_deserialize(unsigned char ch) {
  GENERIC_DESERIALIZE(Statistics, received_statistics_data);
}

void received_statistics_reset(void) {
  return;
}


unsigned char agent_serialize(char* done) {
  GENERIC_SERIALIZE(Agent, agent_data);
}

char agent_deserialize(unsigned char ch) {
  GENERIC_DESERIALIZE(Agent, agent_data);
}

void agent_reset(void) {
  return;
}
