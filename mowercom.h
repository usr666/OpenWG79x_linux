#ifndef MOWERCOM_H
#define MOWERCOM_H

#include <stddef.h>
#include <stdint.h>

#define MAX_MSG_SIZE 255
#define MAX_FRAME_SIZE (1 + 2 + MAX_MSG_SIZE + 2)	/* START + msg_id + len + payload + crc */

typedef enum {
	MSG_MOWER_STATUS                = 0x00,
	MSG_REMOTE_CONTROL_RUN          = 0x01,
	MSG_REMOTE_CONTROL_TURN         = 0x02,
	MSG_REMOTE_CONTROL_MOW          = 0x03,
	MSG_REMOTE_CONTROL_FIND_CHARGER = 0x04,
	MSG_REMOTE_CONTROL_POWEROFF     = 0x05
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
	int8_t  left_speed;		/* -100..100 */
	int8_t  right_speed;		/* -100..100 */
	int8_t  disc_speed;		/* -100..100 */
	uint8_t force_run;		/* do not stop when hitting obstacles */
} __attribute__((packed)) msg_remote_control_run_t;

/* 0x02, us -> mower */
typedef struct {
	int8_t  wheel_speed;		/* -100..100 */
	uint8_t turn_angle;		/* 0..255 degrees */
	uint8_t turn_dir;		/* 0 left, 1 right */
	int8_t  disc_speed;		/* -100..100 */
	uint8_t force_run;		/* do not stop when hitting obstacles */
} __attribute__((packed)) msg_remote_control_turn_t;

/* 0x03 MOW, 0x04 FIND_CHARGER and 0x05 POWEROFF carry no payload */

int  mowercom_open(const char *device);	/* opens uart, returns fd to poll, -1 on error */
/* -1 uart error, 0 nothing complete yet, 1 message stored in msgid/data/len */
int  mowercom_read(uint8_t *msgid, void *data, size_t size, size_t *len);
/* Writes unescaped frames, each START as is and everything after it escaped. -1 on error. */
int  mowercom_write(const uint8_t *data, size_t len);
void mowercom_close(void);

#endif
