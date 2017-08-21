#define _XOPEN_SOURCE 700
#include <stdio.h>
#include <lcm/lcm.h>
#include <inttypes.h>
#include <sys/select.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include "lcm_handle_async.h"
#include "vehicle_sim_orientation_t.h"
#include "vehicle_sim_robot_t.h"
#include "vehicle_sim_lidar_result_t.h"
#include "world.h"

#define PI 3.1415926535897932384626
#define LIDAR_DIST_DELTA 0.1

#define UPDATE_MS 20

double x;
double y;
double angle;

double lidar_angle;

static void republish_lidar(lcm_t *lcm)
{
    double ray_x = x + MAP_ZERO_X;
    double ray_y = y + MAP_ZERO_Y;

    double total_angle = 90.0 + angle + lidar_angle;
    double cos_angle = cos(total_angle * (PI / 180.0));
    double sin_angle = sin(total_angle * (PI / 180.0));

    bool did_hit_something = false;

    vehicle_sim_lidar_result_t result = {0, 0, lidar_angle};
    while(ray_x >= 0 && ray_x < MAP_WIDTH && ray_y >= 0 && ray_y < MAP_HEIGHT) {
        char map_c = map[(int)(ray_y + 0.5) * MAP_WIDTH + (int)(ray_x + 0.5)];
        //printf("%.2f %.2f -> %d %d\n", ray_x, ray_y, (int)(ray_x + 0.5), (int)(ray_y + 0.5));
        if (map_c == 'o') {
            result.intensity = 0.5;
            did_hit_something = true;
            break;
        } else if (map_c == 'x') {
            result.intensity = 1.0;
            did_hit_something = true;
            break;
        }
        result.distance += LIDAR_DIST_DELTA;
        ray_x += cos_angle * LIDAR_DIST_DELTA;
        ray_y += sin_angle * LIDAR_DIST_DELTA;
    }

    if (!did_hit_something) {
        result.distance = 0; // everything off the screen is an obstacle!
    }

    vehicle_sim_lidar_result_t_publish(lcm, "LIDAR", &result);
}

static void receive_world_orientation(const lcm_recv_buf_t *rbuf, const char *channel,
                                      const vehicle_sim_orientation_t *msg, void *user)
{
    x = msg->x;
    y = msg->y;
    angle = msg->angle;
}

static void receive_robot(const lcm_recv_buf_t *rbuf, const char *channel,
                                const vehicle_sim_robot_t *msg, void *user)
{
    lidar_angle = msg->lidar_angle;
    republish_lidar((lcm_t*)user);
}

int main(int argc, char **argv)
{
    lcm_t *lcm = lcm_create(NULL);
    if (!lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }

    vehicle_sim_orientation_t_subscribe(lcm, "WORLD_ORIENTATION", &receive_world_orientation, lcm);
    vehicle_sim_robot_t_subscribe(lcm, "ROBOT", &receive_robot, lcm);

    while (1) {
        lcm_handle_async(lcm);
        nanosleep(&(struct timespec){0, (UPDATE_MS * 1e6)}, NULL);
        republish_lidar(lcm);
    }

    lcm_destroy(lcm);
    return 0;
}
