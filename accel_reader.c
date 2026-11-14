/*
 * accel_reader.c
 * Reads one or two Seeed XIAO nRF52840 Sense boards running xiao_imu.ino
 * (or the older accel-only xiao_accel.ino, which gives tilt only),
 * over USB serial (/dev/ttyACM*) or Bluetooth (via ble_bridge.py -> /tmp/accel*).
 *
 * For each board it fuses accelerometer + gyroscope (Madgwick filter) into a
 * live 3D orientation (roll / pitch / yaw in degrees), and shows the angle
 * between the two boards.
 *
 * Each board sends lines like:
 *     t_ms,ax,ay,az,gx,gy,gz,backlog
 *     (accel in g, gyro in deg/s, backlog = samples still waiting on the board)
 *
 * Usage:
 *     ./accel_reader [port1] [port2] [--csv file.csv [--csv-wait]] [--json]
 *
 * --csv-wait opens the CSV file but writes no rows until the program
 * receives SIGUSR1 (kill -USR1 <pid>); the UI uses this so data from the
 * start delay isn't recorded.
 *
 * --json replaces the human-readable status line with one JSON object per
 * update (10 per second), for ui/ui_server.py. Lines on stdout that don't
 * start with '{' are ordinary messages.
 * Defaults to /dev/ttyACM0 and /dev/ttyACM1. Ports that can't be opened are
 * skipped, so it also works with just one board.
 *
 * Notes:
 *   - Roll and pitch are referenced to gravity and stay accurate.
 *   - Yaw (heading) has no compass reference and slowly drifts.
 *   - Keep each board still for ~2 s after power-up (gyro calibration).
 */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define NUM_SENSORS 2
#define PRINT_INTERVAL_MS 100
#define MADGWICK_BETA 0.1    /* higher = trusts accel more, less gyro drift but noisier */
#define NOMINAL_DT (1.0 / 104.0)
#define MAX_GAP_S 0.25       /* bigger gap = lost data -> re-level from gravity */

#define DEG (180.0 / M_PI)
#define RAD (M_PI / 180.0)

typedef struct {
    const char *path;
    int fd;
    char buf[256];
    size_t len;

    /* latest raw sample */
    unsigned long t_ms;
    double a[3], g[3];
    unsigned backlog;
    unsigned bat_mv;      /* battery voltage from "BAT <mV>" lines, 0 = unknown */

    /* orientation */
    double q[4];
    int have_data;
    unsigned long samples, gaps;
} Sensor;

static volatile sig_atomic_t running = 1;
/* CSV rows are written only while csv_on is set. --csv-wait clears it at
   start; SIGUSR1 sets it (the UI sends this when the collect period begins). */
static volatile sig_atomic_t csv_on = 1;

static void on_sigusr1(int sig) {
    (void)sig;
    csv_on = 1;
}
static FILE *csv = NULL;
static int json_mode = 0;   /* --json: machine-readable status for the UI */

static void on_sigint(int sig) {
    (void)sig;
    running = 0;
}

static long now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

/* ---------------- orientation math ---------------- */

static void quat_normalize(double q[4]) {
    double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 0) for (int i = 0; i < 4; i++) q[i] /= n;
}

/* Level the orientation from gravity alone (yaw set to 0). */
static void quat_from_accel(double q[4], const double a[3]) {
    double roll = atan2(a[1], a[2]);
    double pitch = atan2(-a[0], sqrt(a[1] * a[1] + a[2] * a[2]));
    double cr = cos(roll / 2), sr = sin(roll / 2);
    double cp = cos(pitch / 2), sp = sin(pitch / 2);
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
}

static void quat_to_euler(const double q[4], double *roll, double *pitch, double *yaw) {
    double s = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (s > 1) s = 1;
    if (s < -1) s = -1;
    *roll = atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * DEG;
    *pitch = asin(s) * DEG;
    *yaw = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * DEG;
}

/* Madgwick IMU filter update. Gyro in rad/s, accel in any unit. */
static void madgwick_update(double q[4], double gx, double gy, double gz,
                            double ax, double ay, double az, double dt) {
    double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    double qd0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    double qd1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    double qd2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    double qd3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

    double an = sqrt(ax * ax + ay * ay + az * az);
    if (an > 0) {
        ax /= an; ay /= an; az /= an;
        double _2q0 = 2 * q0, _2q1 = 2 * q1, _2q2 = 2 * q2, _2q3 = 2 * q3;
        double _4q0 = 4 * q0, _4q1 = 4 * q1, _4q2 = 4 * q2;
        double _8q1 = 8 * q1, _8q2 = 8 * q2;
        double q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

        double s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        double s1 = _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1
                    + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        double s2 = 4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
                    + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        double s3 = 4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay;
        double sn = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (sn > 0) {
            qd0 -= MADGWICK_BETA * s0 / sn;
            qd1 -= MADGWICK_BETA * s1 / sn;
            qd2 -= MADGWICK_BETA * s2 / sn;
            qd3 -= MADGWICK_BETA * s3 / sn;
        }
    }
    q[0] += qd0 * dt;
    q[1] += qd1 * dt;
    q[2] += qd2 * dt;
    q[3] += qd3 * dt;
    quat_normalize(q);
}

/* Total rotation angle (degrees) between two orientations. */
static double angle_between(const double a[4], const double b[4]) {
    double d = fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
    if (d > 1) d = 1;
    return 2 * acos(d) * DEG;
}

/* ---------------- serial input ---------------- */

static int open_serial(const char *path) {
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "tcgetattr %s: %s\n", path, strerror(errno));
        close(fd);
        return -1;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= CLOCAL | CREAD;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "tcsetattr %s: %s\n", path, strerror(errno));
        close(fd);
        return -1;
    }
    tcflush(fd, TCIFLUSH);
    return fd;
}

static void handle_line(Sensor *s, int idx, const char *line) {
    /* Battery report from the firmware (USB) or the bridge (Bluetooth). */
    unsigned mv;
    if (sscanf(line, "BAT %u", &mv) == 1) {
        s->bat_mv = mv;
        return;
    }
    /* "ID <n>": the board answering the UI's port check; nothing to do. */
    if (strncmp(line, "ID ", 3) == 0) return;

    unsigned long t;
    double a[3], g[3];
    unsigned backlog = 0;
    int n = sscanf(line, "%lu,%lf,%lf,%lf,%lf,%lf,%lf,%u",
                   &t, &a[0], &a[1], &a[2], &g[0], &g[1], &g[2], &backlog);

    /* Older accel-only firmware (xiao_accel.ino) sends just "x,y,z".
       No gyro, so show tilt from gravity only (yaw stays 0). */
    if (n < 7 && strchr(line, ',') &&
        sscanf(line, "%lf,%lf,%lf", &a[0], &a[1], &a[2]) == 3 &&
        strchr(strchr(line, ',') + 1, ',') &&
        !strchr(strchr(strchr(line, ',') + 1, ',') + 1, ',')) {
        quat_from_accel(s->q, a);
        s->have_data = 1;
        s->t_ms = (unsigned long)now_ms();
        memcpy(s->a, a, sizeof a);
        memset(s->g, 0, sizeof s->g);
        s->backlog = 0;
        s->samples++;
        if (csv && csv_on) {
            double r, p, y;
            quat_to_euler(s->q, &r, &p, &y);
            fprintf(csv, "%d,%lu,%.3f,%.3f,%.3f,,,,%.2f,%.2f,\n",
                    idx + 1, s->t_ms, a[0], a[1], a[2], r, p);
        }
        return;
    }

    if (n < 7) {
        /* status messages like "OK ...", "INFO ...", "ERR ..." */
        fprintf(stderr, "[sensor %d] %s\n", idx + 1, line);
        return;
    }

    if (!s->have_data) {
        quat_from_accel(s->q, a);
        s->have_data = 1;
    } else {
        double dt = ((long)(t - s->t_ms)) / 1000.0;
        if (dt > MAX_GAP_S || dt < 0) {
            s->gaps++;               /* data was lost: re-level from gravity */
            quat_from_accel(s->q, a);
        } else {
            if (dt == 0) dt = NOMINAL_DT;
            madgwick_update(s->q, g[0] * RAD, g[1] * RAD, g[2] * RAD,
                            a[0], a[1], a[2], dt);
        }
    }

    s->t_ms = t;
    memcpy(s->a, a, sizeof a);
    memcpy(s->g, g, sizeof g);
    s->backlog = backlog;
    s->samples++;

    if (csv && csv_on) {
        double r, p, y;
        quat_to_euler(s->q, &r, &p, &y);
        fprintf(csv, "%d,%lu,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f\n",
                idx + 1, t, a[0], a[1], a[2], g[0], g[1], g[2], r, p, y);
    }
}

static void read_sensor(Sensor *s, int idx) {
    char tmp[512];
    ssize_t n = read(s->fd, tmp, sizeof tmp);
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            fprintf(stderr, "Read error on %s: %s\n", s->path, strerror(errno));
            running = 0;
        }
        return;
    }
    for (ssize_t i = 0; i < n; i++) {
        char c = tmp[i];
        if (c == '\n' || c == '\r') {
            if (s->len > 0) {
                s->buf[s->len] = '\0';
                handle_line(s, idx, s->buf);
                s->len = 0;
            }
        } else if (s->len < sizeof s->buf - 1) {
            s->buf[s->len++] = c;
        } else {
            s->len = 0; /* line too long, discard */
        }
    }
}

/* ---------------- status output ---------------- */

/* One JSON object per line, e.g.
   {"sensors":[{"id":1,"state":"ok","roll":2.1,"pitch":-0.5,"yaw":0.0,
                "buf":0,"samples":1234,"gaps":0}, ...],"between":2.3}
   state is "ok", "waiting" (port open, no data yet) or "absent" (not opened).
   "bat_mv" (battery millivolts) is added once the board has reported it. */
static void print_status_json(const Sensor *sensors) {
    printf("{\"sensors\":[");
    for (int i = 0; i < NUM_SENSORS; i++) {
        const Sensor *s = &sensors[i];
        if (i) printf(",");
        if (s->fd < 0) {
            printf("{\"id\":%d,\"state\":\"absent\"", i + 1);
        } else if (!s->have_data) {
            printf("{\"id\":%d,\"state\":\"waiting\"", i + 1);
        } else {
            double r, p, y;
            quat_to_euler(s->q, &r, &p, &y);
            printf("{\"id\":%d,\"state\":\"ok\",\"roll\":%.1f,\"pitch\":%.1f,"
                   "\"yaw\":%.1f,\"buf\":%u,\"samples\":%lu,\"gaps\":%lu",
                   i + 1, r, p, y, s->backlog, s->samples, s->gaps);
        }
        if (s->bat_mv) printf(",\"bat_mv\":%u", s->bat_mv);
        printf("}");
    }
    printf("],\"between\":");
    if (sensors[0].have_data && sensors[1].have_data)
        printf("%.1f}\n", angle_between(sensors[0].q, sensors[1].q));
    else
        printf("null}\n");
    fflush(stdout);
}

/* ---------------- main ---------------- */

int main(int argc, char *argv[]) {
    const char *ports[NUM_SENSORS] = { "/dev/ttyACM0", "/dev/ttyACM1" };
    const char *csv_path = NULL;
    int nports = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
            csv_path = argv[++i];
        } else if (strcmp(argv[i], "--json") == 0) {
            json_mode = 1;
        } else if (strcmp(argv[i], "--csv-wait") == 0) {
            csv_on = 0;
        } else if (nports < NUM_SENSORS) {
            ports[nports++] = argv[i];
        }
    }

    Sensor sensors[NUM_SENSORS];
    memset(sensors, 0, sizeof sensors);
    for (int i = 0; i < NUM_SENSORS; i++) sensors[i].path = ports[i];

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);
    signal(SIGUSR1, on_sigusr1);

    int opened = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].fd = open_serial(sensors[i].path);
        if (sensors[i].fd < 0) {
            printf("Sensor %d: %s not available, skipping\n", i + 1, sensors[i].path);
            continue;
        }
        printf("Sensor %d: %s\n", i + 1, sensors[i].path);
        opened++;
    }
    if (opened == 0) {
        fprintf(stderr, "No sensors could be opened. Are the boards plugged in, "
                        "and is the Arduino Serial Monitor closed?\n");
        return EXIT_FAILURE;
    }

    if (csv_path) {
        csv = fopen(csv_path, "w");
        if (!csv) {
            fprintf(stderr, "Cannot write %s: %s\n", csv_path, strerror(errno));
            return EXIT_FAILURE;
        }
        fprintf(csv, "sensor,board_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                     "roll_deg,pitch_deg,yaw_deg\n");
        printf("%s %s\n", csv_on ? "Logging every sample to"
                              : "CSV ready (waiting for start signal):", csv_path);
    }

    printf("Reading... press Ctrl+C (or the red Terminate button) to stop.\n");
    fflush(stdout);

    struct pollfd pfds[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
        pfds[i].fd = sensors[i].fd;  /* negative fds are ignored by poll */
        pfds[i].events = POLLIN;
    }

    long last_print = now_ms();
    int csv_was_on = csv_on;

    while (running) {
        if (csv && csv_on && !csv_was_on) {
            csv_was_on = 1;
            printf("CSV recording started\n");
            fflush(stdout);
        }
        int rc = poll(pfds, NUM_SENSORS, 20);
        if (rc < 0) {
            if (errno == EINTR) continue;
            perror("poll");
            break;
        }

        for (int i = 0; i < NUM_SENSORS; i++) {
            if (sensors[i].fd < 0) continue;
            if (pfds[i].revents & POLLIN) {
                read_sensor(&sensors[i], i);
            } else if (pfds[i].revents & (POLLHUP | POLLERR | POLLNVAL)) {
                fprintf(stderr, "Sensor %d (%s) disconnected\n", i + 1, sensors[i].path);
                running = 0;
            }
        }

        long t = now_ms();
        if (t - last_print >= PRINT_INTERVAL_MS && json_mode) {
            last_print = t;
            print_status_json(sensors);
        } else if (t - last_print >= PRINT_INTERVAL_MS) {
            last_print = t;
            for (int i = 0; i < NUM_SENSORS; i++) {
                Sensor *s = &sensors[i];
                if (s->fd < 0) {
                    printf("S%d (not connected)                              ", i + 1);
                } else if (!s->have_data) {
                    printf("S%d (waiting for data)                           ", i + 1);
                } else {
                    double r, p, y;
                    quat_to_euler(s->q, &r, &p, &y);
                    printf("S%d roll %+6.1f pitch %+6.1f yaw %+6.1f buf %4u",
                           i + 1, r, p, y, s->backlog);
                    if (s->bat_mv) printf(" bat %.2fV", s->bat_mv / 1000.0);
                }
                printf(" | ");
            }
            if (sensors[0].have_data && sensors[1].have_data)
                printf("between %5.1f deg\n", angle_between(sensors[0].q, sensors[1].q));
            else
                printf("between   --\n");
            fflush(stdout);
        }
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensors[i].fd >= 0) {
            close(sensors[i].fd);
            if (sensors[i].samples)
                printf("Sensor %d: %lu samples, %lu gaps\n",
                       i + 1, sensors[i].samples, sensors[i].gaps);
        }
    }
    if (csv) fclose(csv);
    printf("Stopped.\n");
    return EXIT_SUCCESS;
}
