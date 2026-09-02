#ifndef MOWERCONTROL_H
#define MOWERCONTROL_H

#include "gps.h"
#include "mowercom.h"

void mowercontrol_input_tcp(const uint8_t *data, size_t len);
void mowercontrol_input_mowercom(uint8_t msgid, const uint8_t *msgbuf);
void mowercontrol_input_gps(gps_recordtype type, const char *line);
void mowercontrol_execute();

#endif
