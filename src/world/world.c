#define _XOPEN_SOURCE 700
#include <stdio.h>
#include <lcm/lcm.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>

#include "robot/robot.h"
#include "world/world.h"

#include "lcmtypes/vehicle_sim_orientation_t.h"
#include "lcmtypes/vehicle_sim_robot_t.h"
#include "learn_common/lcm_handle_async.h"

#define PHYSICS_UPDATE_MS 10.0
#define REPORT_MS 20.0

#define ROBOT_CIRC (ROBOT_DIM * PI)

#define PHYSICS_ALPHA 0.10

double x;
double y;
double angle;

// both from -1 to 1
double desired_motor_left;
double desired_motor_right;

// actual motor values in the "world"
double actual_left;
double actual_right;

static void republish_world_orientation(lcm_t *lcm)
{
    vehicle_sim_orientation_t result = {x, y, angle};
    vehicle_sim_orientation_t_publish(lcm, "WORLD_ORIENTATION", &result);
}

static void receive_robot(const lcm_recv_buf_t *rbuf, const char *channel,
                          const vehicle_sim_robot_t *msg, void *user)
{
    desired_motor_left = msg->motor_left;
    desired_motor_right = msg->motor_right;
}

static void update_physics(lcm_t *lcm)
{
    actual_left = PHYSICS_ALPHA * actual_left + (1 - PHYSICS_ALPHA) * desired_motor_left;
    actual_right = PHYSICS_ALPHA * actual_right + (1 - PHYSICS_ALPHA) * desired_motor_right;

    double difference = actual_right - actual_left;
    angle += difference * (MAX_SPEED / ROBOT_CIRC * 360.0 * PHYSICS_UPDATE_MS / 1000.0);
    angle = fmod(angle, 360);

    double total_angle = angle + 90; // have 0 pointing in the y = -1 direction.

    double sum = actual_right + actual_left;
    double movement = sum * (MAX_SPEED * PHYSICS_UPDATE_MS / 1000.0);
    x += cos(total_angle * (PI / 180.0)) * movement;
    y -= sin(total_angle * (PI / 180.0)) * movement;
}

int main(int argc, char **argv)
{
    lcm_t *lcm = lcm_create(NULL);
    if (!lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }

    vehicle_sim_robot_t_subscribe(lcm, "ROBOT", &receive_robot, lcm);

    int counts_since_report = 0;
    while (1) {
        lcm_handle_async(lcm);
        update_physics(lcm);

        counts_since_report++;
        if (counts_since_report >= REPORT_MS / PHYSICS_UPDATE_MS) {
            counts_since_report = 0;
            republish_world_orientation(lcm);
        }
        nanosleep(&(struct timespec){0, (PHYSICS_UPDATE_MS * 1e6)}, NULL);
    }

    lcm_destroy(lcm);
    return 0;
}
