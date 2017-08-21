#define _XOPEN_SOURCE 700
#include <stdio.h>
#include <lcm/lcm.h>
#include "world.h"
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include "vehicle_sim_robot_t.h"
#include "vehicle_sim_lidar_result_t.h"
#include "lcm_handle_async.h"
#include "common/zarray.h"

#define CONTROL_UPDATE_MS 20.0

// in units. one map square is one unit.
#define ROBOT_DIM 0.1
#define PI 3.1415926535897932384626
#define ROBOT_CIRC (ROBOT_DIM * PI)

// in units per second. one square is one unit.
#define MAX_SPEED 0.5

#define APPROACH_DISTANCE 0.75

double motor_left = 0;
double motor_right = 0;
double lidar_angle = 0;
double lidar_forward_distance = 0;

typedef struct pose_t {
    double x;
    double y;
    double angle;
} pose_t;

pose_t pose = {0.0, 0.0, 0.0};
zarray_t *past_poses;
zarray_t *lidar_poses;

#define DRIVING_CYCLES 50
#define SCANNING_CYCLES 10

enum control_stage_t {
    DRIVING,
    SCANNING,
};

enum control_stage_t current_stage;
uint32_t stage_cycles = 0;

static void republish_robot(lcm_t *lcm)
{
    vehicle_sim_robot_t result = {motor_left, motor_right, lidar_angle};
    vehicle_sim_robot_t_publish(lcm, "ROBOT", &result);
}

static void receive_lidar(const lcm_recv_buf_t *rbuf, const char *channel,
                          const vehicle_sim_lidar_result_t *msg, void *user)
{
    if (msg->lidar_angle == 0) {
        lidar_forward_distance = msg->distance;
    }

    pose_t lidar_pose = pose;
    double total_angle = 90.0 + pose.angle + lidar_angle;
    double cos_angle = cos(total_angle * (PI / 180.0));
    double sin_angle = sin(total_angle * (PI / 180.0));
    lidar_pose.x += msg->distance * cos_angle;
    lidar_pose.y += msg->distance * sin_angle;
    lidar_pose.angle += lidar_angle;

    zarray_add(lidar_poses, &lidar_pose);
}

static void update_pose(lcm_t *lcm) {
    double difference = motor_left - motor_right;
    pose.angle += difference * (MAX_SPEED / ROBOT_CIRC * 360.0 * CONTROL_UPDATE_MS / 1000.0);

    double sum = motor_left + motor_right;
    double movement = sum * (MAX_SPEED * CONTROL_UPDATE_MS / 1000.0);
    pose.x += cos(pose.angle * (PI / 180.0)) * movement;
    pose.y += sin(pose.angle * (PI / 180.0)) * movement;

    zarray_add(past_poses, &pose);
}

static void update_control(lcm_t *lcm) {
    stage_cycles++;
    switch(current_stage) {
    case DRIVING:
        if (lidar_forward_distance > APPROACH_DISTANCE) {
            motor_left = 1.0;
            motor_right = 1.0;
        } else {
            motor_left = 1.0;
            motor_right = -1.0;
        }

        if (stage_cycles >= DRIVING_CYCLES) {
            stage_cycles = 0;
            current_stage = SCANNING;
        }
        break;
    case SCANNING:
        motor_left = 0;
        motor_right = 0;
        lidar_angle = 180.0 * stage_cycles / SCANNING_CYCLES - 90.0;
        if (stage_cycles >= SCANNING_CYCLES) {
            stage_cycles = 0;
            current_stage = DRIVING;
        }
        break;
    }

}

int main(int argc, char **argv)
{
    lcm_t *lcm = lcm_create(NULL);
    if (!lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }

    past_poses = zarray_create(sizeof(pose_t));
    lidar_poses = zarray_create(sizeof(pose_t));

    vehicle_sim_lidar_result_t_subscribe(lcm, "LIDAR", &receive_lidar, lcm);

    while (1) {
        lcm_handle_async(lcm);
        nanosleep(&(struct timespec){0, (CONTROL_UPDATE_MS * 1e6)}, NULL);
        update_pose(lcm);
        update_control(lcm);
        republish_robot(lcm);
    }

    zarray_destroy(past_poses);
    zarray_destroy(lidar_poses);
    lcm_destroy(lcm);
    return 0;
}
