/*
 * io_daemon.c — drives LED + buzzer patterns on Pi 4 GPIO.
 * Receives MSG_IO_ALERT from alert_daemon, loops the matching
 * pattern until a PULSE_ACK arrives, then switches to GREEN.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

#include "gestsense.h"

static volatile uint32_t *gpio = NULL;

static int gpio_init(void)
{
    const unsigned long bases[] = { 0xFE200000UL, 0x3F200000UL, 0 };
    for (int i = 0; bases[i]; i++) {
        volatile uint32_t *p = (volatile uint32_t *)mmap_device_memory(
            NULL, GPIO_MAP_LEN, PROT_READ | PROT_WRITE | PROT_NOCACHE,
            0, bases[i]);
        if (p == MAP_FAILED) continue;
        gpio = p;
        printf("[IO] GPIO mapped at 0x%lX\n", bases[i]);
        return 0;
    }
    printf("[IO] GPIO mapping failed — outputs disabled\n");
    return -1;
}

static void gpio_set_output(int pin)
{
    int reg = pin / 10, shift = (pin % 10) * 3;
    uint32_t v = gpio[reg];
    v &= ~(7u << shift);
    v |=  (1u << shift);
    gpio[reg] = v;
}

static inline void pin_hi(int p) { if (gpio) gpio[7]  = (1u << p); }
static inline void pin_lo(int p) { if (gpio) gpio[10] = (1u << p); }

static void gpio_setup(void)
{
    gpio_set_output(PIN_RED);
    gpio_set_output(PIN_GREEN);
    gpio_set_output(PIN_BUZZER);
    pin_lo(PIN_RED); pin_lo(PIN_GREEN); pin_lo(PIN_BUZZER);
}

/*shared state */

static char g_gesture[32] = "";
static int  g_active = 0;
static int  g_acked  = 0;
static pthread_mutex_t g_mtx  = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cond = PTHREAD_COND_INITIALIZER;

static void set_prio(int prio, int policy)
{
    struct sched_param p = { .sched_priority = prio };
    pthread_setschedparam(pthread_self(), policy, &p);
}

/* Polls g_acked every half-ms so cancel latency stays sub-millisecond */
static void buzzer_pwm(int duration_us)
{
    int elapsed = 0;
    while (elapsed < duration_us) {
        pthread_mutex_lock(&g_mtx);
        int a = g_acked;
        pthread_mutex_unlock(&g_mtx);
        if (a) { pin_lo(PIN_BUZZER); return; }
        pin_hi(PIN_BUZZER); usleep(250);
        pin_lo(PIN_BUZZER); usleep(250);
        elapsed += 500;
    }
}

static inline void buzzer_off(void) { pin_lo(PIN_BUZZER); }

/*patterns */

static void pat_ambulance(void)
{
    pin_hi(PIN_RED); buzzer_pwm(150000);
    pin_lo(PIN_RED); buzzer_off();       usleep( 80000);
    pin_hi(PIN_RED); buzzer_pwm(150000);
    pin_lo(PIN_RED); buzzer_off();       usleep(350000);
}

static void pat_police(void)
{
    static int tog = 0;
    if (tog & 1) { pin_hi(PIN_RED); pin_lo(PIN_GREEN); }
    else         { pin_lo(PIN_RED); pin_hi(PIN_GREEN); }
    buzzer_pwm(100000);
    pin_lo(PIN_RED); pin_lo(PIN_GREEN); buzzer_off();
    usleep(100000);
    tog++;
}

static void pat_fire(void)
{
    for (int i = 0; i < 3; i++) {
        pin_hi(PIN_RED); buzzer_pwm(100000);
        pin_lo(PIN_RED); buzzer_off();   usleep(80000);
    }
    usleep(300000);
}

static void pat_distress(void)
{
    for (int i = 0; i < 3; i++) {               /* S */
        pin_hi(PIN_RED); buzzer_pwm(100000);
        pin_lo(PIN_RED); buzzer_off(); usleep(100000);
    }
    usleep(150000);
    for (int i = 0; i < 3; i++) {               /* O */
        pin_hi(PIN_RED); buzzer_pwm(350000);
        pin_lo(PIN_RED); buzzer_off(); usleep(100000);
    }
    usleep(150000);
    for (int i = 0; i < 3; i++) {               /* S */
        pin_hi(PIN_RED); buzzer_pwm(100000);
        pin_lo(PIN_RED); buzzer_off(); usleep(100000);
    }
    usleep(500000);
}

/* pattern executor */

static void *pattern_thread(void *arg)
{
    (void)arg;
    set_prio(PRIO_IO_PATTERN, SCHED_FIFO);

    char gesture[32];
    while (1) {
        pthread_mutex_lock(&g_mtx);
        while (!g_active || g_acked) pthread_cond_wait(&g_cond, &g_mtx);
        strncpy(gesture, g_gesture, sizeof(gesture) - 1);
        gesture[sizeof(gesture) - 1] = 0;
        pthread_mutex_unlock(&g_mtx);

        while (1) {
            pthread_mutex_lock(&g_mtx);
            int a = g_acked, on = g_active;
            pthread_mutex_unlock(&g_mtx);
            if (a || !on) break;

            if      (!strcmp(gesture, "AMBULANCE")) pat_ambulance();
            else if (!strcmp(gesture, "POLICE"))    pat_police();
            else if (!strcmp(gesture, "FIRE"))      pat_fire();
            else if (!strcmp(gesture, "DISTRESS"))  pat_distress();
            else break;
        }

        pin_lo(PIN_RED); buzzer_off();
        pthread_mutex_lock(&g_mtx);
        if (g_acked) { pin_hi(PIN_GREEN); printf("[IO] ACK — GREEN on\n"); }
        else         { pin_lo(PIN_GREEN); }
        g_active = 0;
        pthread_mutex_unlock(&g_mtx);
        fflush(stdout);
    }
    return NULL;
}

/* idle green blink */
static void *heartbeat(void *arg)
{
    (void)arg;
    set_prio(PRIO_STATUS, SCHED_OTHER);
    while (1) {
        pthread_mutex_lock(&g_mtx);
        int act = g_active, ack = g_acked;
        pthread_mutex_unlock(&g_mtx);

        if (ack) { pin_hi(PIN_GREEN); usleep(500000); }
        else if (act) { pin_lo(PIN_GREEN); usleep(100000); }
        else {
            pin_hi(PIN_GREEN); usleep(100000);
            pin_lo(PIN_GREEN); usleep(900000);
        }
    }
    return NULL;
}

/* main */

int main(void)
{
    printf("[IO] io_daemon starting\n");
    fflush(stdout);

    FILE *pf = fopen("/tmp/io_daemon.pid", "w");
    if (pf) { fprintf(pf, "%d\n", getpid()); fclose(pf); }

    if (gpio_init() == 0) gpio_setup();

    pthread_t t_pat, t_hb;
    pthread_create(&t_pat, NULL, pattern_thread, NULL);
    pthread_create(&t_hb,  NULL, heartbeat,      NULL);

    set_prio(PRIO_IO_MSG, SCHED_FIFO);

    name_attach_t *att = name_attach(NULL, SVC_IO, 0);
    if (!att) { perror("[IO] name_attach"); return 1; }
    printf("[IO] registered service '%s' chid=%d base prio=%d\n",
           SVC_IO, att->chid, PRIO_IO_MSG);
    fflush(stdout);

    io_cmd_t cmd;
    io_reply_t rep = { .ok = 1 };
    struct _pulse pulse;

    while (1) {
        int rcvid = MsgReceive(att->chid, &cmd, sizeof(cmd), NULL);
        if (rcvid == -1) { perror("[IO] MsgReceive"); break; }

        /* Pulse (async signal) */
        if (rcvid == 0) {
            memcpy(&pulse, &cmd, sizeof(pulse));
            if (pulse.code == PULSE_ACK) {
                pthread_mutex_lock(&g_mtx);
                g_acked = 1;
                pthread_cond_broadcast(&g_cond);
                pthread_mutex_unlock(&g_mtx);
                buzzer_off();
                pin_hi(PIN_GREEN);
                printf("[IO] PULSE_ACK — pattern stopped\n");
                fflush(stdout);
            }
            continue;
        }

        /* Synchronous message */
        if (cmd.type == MSG_IO_ALERT) {
            rep.t_received_ns = ClockCycles();

            /* Log priority inheritance: base is PRIO_IO_MSG but QNX
             * boosts us to the sender's priority for this message. */
            struct sched_param sp; int policy;
            pthread_getschedparam(pthread_self(), &policy, &sp);
            printf("[IO] %s person=%s (base=%d effective=%d)\n",
                   cmd.gesture, cmd.person, PRIO_IO_MSG, sp.sched_priority);
            fflush(stdout);

            MsgReply(rcvid, 0, &rep, sizeof(rep));

            pthread_mutex_lock(&g_mtx);
            strncpy(g_gesture, cmd.gesture, sizeof(g_gesture) - 1);
            g_gesture[sizeof(g_gesture) - 1] = 0;
            g_active = 1;
            g_acked  = 0;
            pthread_cond_broadcast(&g_cond);
            pthread_mutex_unlock(&g_mtx);
        } else {
            MsgError(rcvid, ENOSYS);
        }
    }

    name_detach(att, 0);
    if (gpio) munmap_device_memory((void *)gpio, GPIO_MAP_LEN);
    return 0;
}
