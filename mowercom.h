#ifndef MOWERCOM_H
#define MOWERCOM_H

#include <stddef.h>
#include <stdint.h>

#define MAX_MSG_SIZE 255

typedef enum {
	MSG_MOWER_STATUS   = 0x00,
	MSG_REMOTE_CONTROL = 0x01
} mower_msg_id_t;

/* 0x00, mower -> us */
typedef struct {
	uint8_t state;
	uint8_t sensor_status;		/* bit0 front bump, bit1 lift */
	uint8_t wire_sensor_status;	/* bit0 right inside, bit1 left inside, bit2 near wire */
	int8_t  left_wheel_speed;	/* -100..100 */
	int8_t  right_wheel_speed;	/* -100..100 */
	uint8_t battery_soc;		/* 0-100 */
} __attribute__((packed)) msg_mower_status_t;

/* 0x01, us -> mower */
typedef struct {
	uint8_t mode;			/* 0 stop, 1 schedule, 2 force mow, 3 charger, 4 remote */
	int8_t  left_speed;
	int8_t  right_speed;
	int8_t  disc_speed;
} __attribute__((packed)) msg_remote_control_t;

int  mowercom_open(const char *device);	/* opens uart, returns fd to poll, -1 on error */
/* -1 uart error, 0 nothing complete yet, 1 message stored in msgid/data/len */
int  mowercom_read(uint8_t *msgid, void *data, size_t size, size_t *len);
void mowercom_close(void);

#endif
