#include <stdbool.h>

#ifndef RADIO_H
#define RADIO_H

/* Radio Packet */
typedef struct __attribute__((__packed__)) _rf_packet {
    bool door_open;
} rf_packet;

void radio_init();
bool radio_tx(rf_packet *packet);
void radio_sleep();

#endif