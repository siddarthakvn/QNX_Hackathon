#ifndef GESTSENSE_H
#define GESTSENSE_H

#include <stdint.h>

/* QNX named services */
#define SVC_SENSOR        "gestsense/sensor"
#define SVC_IO            "gestsense/io"

/* UDP ports */
#define UDP_ALERT_PORT    5005
#define DASHBOARD_PORT    5006
#define ACK_PORT          5007

/* Thread priorities */
#define PRIO_UDP_RECV     25
#define PRIO_ACK          23
#define PRIO_IO_PATTERN   22
#define PRIO_IO_MSG       21
#define PRIO_SENSOR       15
#define PRIO_STATUS        5

/* Message type codes */
#define MSG_IO_ALERT      0x1001
#define MSG_SENSOR_GET    0x2001

/* Pulse codes */
#define PULSE_ACK         1

/* GPIO (BCM numbering) */
#define GPIO_PHYS_BASE    0xFE200000UL
#define GPIO_MAP_LEN      0x00001000UL
#define PIN_RED           17
#define PIN_GREEN         27
#define PIN_BUZZER        19

typedef struct {
    uint16_t type;
    char     gesture[32];
    char     person[16];
    char     alert_msg[64];
    double   lat, lon;
    float    temp, hum;
    int      gps_valid;
    uint64_t t_enqueue_ns;
} io_cmd_t;

typedef struct {
    int      ok;
    uint64_t t_received_ns;
} io_reply_t;

typedef struct {
    uint16_t type;
} sensor_req_t;

typedef struct {
    double lat, lon, alt;
    int    sats;
    int    gps_valid;       /* 1 = live fix, 0 = no fix yet   */
    float  temp, hum, gas;
    int    bme_valid;       /* 1 = live reading, 0 = no data  */
} sensor_reply_t;

#endif
