#define _BSD_SOURCE

#include <stdio.h>
#include <lcm/lcm.h>
#include <math.h>
#include <poll.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>

#include "vx/vx.h"
#include "vx/vxo_box.h"
#include "vx/vxo_matrix.h"
#include "vx/webvx.h"
#include "common/time_util.h"
#include "common/getopt.h"
#include "common/zarray.h"

#include "robot/robot.h"
#include "world/world.h"
#include "lcmtypes/vehicle_sim_robot_t.h"
#include "lcmtypes/vehicle_sim_orientation_t.h"
#include "lcmtypes/vehicle_sim_lidar_result_t.h"

#define WATCHDOG_PERIOD    50 // ms
#define WATCHDOG_TIMEOUT  500 // ms

uint32_t next_id = 0;

typedef vehicle_sim_orientation_t pose_t;

typedef struct client client_t;
typedef struct state state_t;

struct state {
    lcm_t *lcm;

    webvx_t *webvx;
    zhash_t *clients;
    client_t *control_client;

    // Signal to stop publishing
    uint8_t mode;

    vehicle_sim_orientation_t world_orientation;
    vehicle_sim_lidar_result_t lidar_result;
    vehicle_sim_robot_t robot;

    double lidar_forward_distance;

    zarray_t *past_poses;
    zarray_t *lidar_poses;

    pthread_mutex_t mutex;
};

struct client
{
    state_t *state;

    uint32_t id;

    vx_world_t *vw;
    vx_canvas_t *vc;
    vx_layer_t  *vl;
    pthread_t watchdog;
    uint64_t last_echo;
    uint8_t watchdog_timeout;
    pthread_t render_thread;

    // Joypad screen coordinates
    uint8_t finger_down;
    int finger_id;
    float current_x;
    float current_y;
    float initial_x;
    float initial_y;

    // Override interface
    uint8_t override;
    int second_finger_id;

    // Signal for threads
    uint8_t close;

    pthread_mutex_t mutex;
};

int event_handler(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
    client_t *client = user;
    state_t *state = client->state;

    switch (ev->type) {

        case VX_EVENT_MOUSE_DOWN: // HACK!
        case VX_EVENT_TOUCH_START: {
            break;
        }
        case VX_EVENT_MOUSE_MOVED: // HACK!
        case VX_EVENT_TOUCH_MOVE: {
            break;
        }
        case VX_EVENT_MOUSE_UP:
        case VX_EVENT_TOUCH_END: {
            break;
        }
        case VX_EVENT_ECHO: {
            // Pet watchdog
            // XXX: Mutex?
            client->last_echo = utime_now();
            client->watchdog_timeout = 0;
            //printf("ECHO: %u %" PRIu64 ", utime: %" PRIu64 "\n", client->id, ev->u.echo.nonce, client->last_echo);
            break;
        }
        case VX_EVENT_ORIENTATION_CHANGE: {
            // Orientation change wedges touch interface. Start fresh.
            pthread_mutex_lock(&client->mutex);
            client->finger_down = 0;
            client->finger_id = -1;
            pthread_mutex_unlock(&client->mutex);
        }
        default: {
            //printf("unknown event %d\n", ev->type);
            break;
        }
    }

    return 0;
}

void *watchdog_thread(void *user)
{
    client_t *client = user;
    uint64_t nonce = 0;
    uint64_t last_echo;

    printf("starting watchdog for client %u\n", client->id);

    while (1) {

        if (client->close) {
            printf("Client %u watchdog thread exiting\n", client->id);
            return NULL;
        }

        last_echo = client->last_echo;

        if ((utime_now() - last_echo) > (WATCHDOG_TIMEOUT * 1000)) {
            client->watchdog_timeout = 1;
        }

        vx_layer_echo(client->vl, nonce);
        nonce++;

        usleep(WATCHDOG_PERIOD * 1000);
    }

    return NULL;
}

void draw_text(vx_buffer_t *vb, double x, double y, const char *fmt, ...)
{
    // va_list processing borrowed from vxo_text
    va_list va;
    va_start(va, fmt);
    int len = vsnprintf(NULL, 0, fmt, va);
    char text[len + 1];
    va_end(va);

    va_start(va, fmt);
    vsnprintf(text, len + 1, fmt, va);
    va_end(va);

    vx_buffer_add_back(vb, vxo_chain(
                      vxo_matrix_translate(x + MAP_ZERO_X, y - MAP_ZERO_Y, -0.25),
                      vxo_matrix_scale(0.25),
                      vxo_text(VXO_TEXT_ANCHOR_TOP_LEFT, "<<monospaced-bold-1,left,#884444ff>>%s", text),
                      NULL),
                  NULL);
}

void *render_thread(void *user)
{

    client_t *client = user;
    state_t *state = client->state;

    for (int iter = 0; 1; iter++) {
        pthread_mutex_lock(&state->mutex);
        uint8_t nclients = zhash_size(state->clients);
        uint8_t mode = state->mode;
        pthread_mutex_unlock(&state->mutex);

        pthread_mutex_lock(&client->mutex);
        uint8_t finger_down = client->finger_down;
        float current_x = client->current_x;
        float current_y = client->current_y;
        float initial_x = client->initial_x;
        float initial_y = client->initial_y;
        uint8_t client_close = client->close;
        pthread_mutex_unlock(&client->mutex);

        if (client->close) {
            printf("Client %u render thread exiting\n", client->id);
            return NULL;
        }

        vx_buffer_t *vb = vx_world_get_buffer(client->vw, "robot");

        vx_buffer_add_back(vb, vxo_grid(vx_white, 1.0f), NULL);

        for (int y = 0; y < MAP_HEIGHT; y++) {
            for (int x = 0; x < MAP_WIDTH; x++) {
                char mapc = map[y * MAP_WIDTH + x];
                if (mapc == 'x') {
                    vx_buffer_add_back(vb, vxo_chain(
                            vxo_matrix_translate(x, -y, 0),
                            vxo_box_solid(vx_black),
                            NULL),
                        NULL);
                } else if (mapc == 'o') {
                    vx_buffer_add_back(vb, vxo_chain(
                            vxo_matrix_translate(x, -y, 0),
                            vxo_box_solid(vx_gray),
                            NULL),
                        NULL);
                }
            }
        }

        vx_buffer_add_back(vb, vxo_chain(
                            vxo_matrix_translate(state->world_orientation.x + MAP_ZERO_X, -state->world_orientation.y - MAP_ZERO_Y, 0),
                            vxo_matrix_rotatez(state->world_orientation.angle * (PI / 180)),
                            vxo_matrix_scale3(ROBOT_DIM / 2, ROBOT_DIM, ROBOT_DIM * 3),
                            vxo_box_solid(vx_red),
                            NULL),
                        NULL);

        draw_text(vb, -4, -2, "World x: %.2f y: %.2f theta: %.2f", state->world_orientation.x, state->world_orientation.y, state->world_orientation.angle);
        pose_t last_pose;
        int past_poses_size = zarray_size(state->past_poses);
        if (past_poses_size > 0) {
            zarray_get(state->past_poses, past_poses_size - 1, &last_pose);
        }
        draw_text(vb, -4, -2.25, "Robot x: %.2f y: %.2f theta: %.2f", last_pose.x, last_pose.y, last_pose.angle);
        draw_text(vb, -4, -2.5, "Robot left: %.2f right: %.2f lidar angle: %.2f", state->robot.motor_left, state->robot.motor_right, state->robot.lidar_angle);
        draw_text(vb, -4, -2.75, "Lidar intensity: %.2f distance: %.2f forward distance: %.2f", state->lidar_result.intensity, state->lidar_result.distance, state->lidar_forward_distance);
        draw_text(vb, -4, -3.0, "past poses: %d lidar poses: %d", zarray_size(state->past_poses), zarray_size(state->lidar_poses));

        for (int i = 0; i < zarray_size(state->past_poses); i++) {
            pose_t pose;
            zarray_get(state->past_poses, i, &pose);

            vx_buffer_add_back(vb, vxo_chain(
                                vxo_matrix_translate(pose.x + MAP_ZERO_X, -pose.y - MAP_ZERO_Y, 0.5),
                                vxo_matrix_rotatez(pose.angle * (PI / 180)),
                                vxo_matrix_scale3(0.05, 0.05, 0.1),
                                vxo_box_solid(vx_yellow),
                                NULL),
                            NULL);
        }

        for (int i = 0; i < zarray_size(state->lidar_poses); i++) {
            pose_t pose;
            zarray_get(state->lidar_poses, i, &pose);

            vx_buffer_add_back(vb, vxo_chain(
                                vxo_matrix_translate(pose.x + MAP_ZERO_X, -pose.y - MAP_ZERO_Y, 0.5),
                                vxo_matrix_rotatez(pose.angle * (PI / 180)),
                                vxo_matrix_scale3(0.05, 0.05, 0.1),
                                vxo_box_solid(vx_green),
                                NULL),
                            NULL);
        }

        vx_buffer_swap(vb);
        usleep(100e3);
    }
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    //if (next_id == 1) return;
    printf("canvas at %" PRIu64 "\n", (uint64_t)vc);

    state_t *state = impl;

    printf("ON CREATE CANVAS\n");
    client_t *client = calloc(1, sizeof(client_t));

    // set up client environment
    client->id = next_id;
    next_id++;

    client->finger_down = 0;
    client->finger_id = -1;

    client->state = state;
    client->vw = vx_world_create();
    client->vc = vc;
    char layer_name[32];
    sprintf(layer_name, "default_%u", client->id);
    client->vl = vx_canvas_get_layer(vc, layer_name);
    vx_layer_set_world(client->vl, client->vw);
    vx_layer_set_title(client->vl, "Learn April");
    vx_layer_add_event_handler(client->vl, event_handler, 0, client);
    vx_layer_enable_touch_camera_controls(client->vl, 0); // turn off touch camera controls
    pthread_mutex_init(&client->mutex, NULL);

    // start threads
    pthread_mutex_lock(&client->mutex);
    client->close = 0;
    pthread_create(&client->watchdog, NULL, watchdog_thread, client);
    pthread_create(&client->render_thread, NULL, render_thread, client);

    // Add client to zhash
    zhash_put(state->clients, &client->vc, &client, NULL, NULL);
    printf("clients: %u \n", zhash_size(state->clients));

    client_t *client_test = NULL;
    zhash_get(state->clients, &client->vc, &client_test);
    assert(client_test == client);

    pthread_mutex_unlock(&client->mutex);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    state_t *state = impl;

    printf("ON DESTROY CANVAS\n");
    // XXX: Clean up
    client_t *client = NULL;
    zhash_get(state->clients, &vc, &client);

    pthread_mutex_unlock(&state->mutex);
    client->close = 1;
    pthread_join(client->watchdog, NULL);
    pthread_join(client->render_thread, NULL);

    free(client);

    zhash_remove(state->clients, &vc, NULL, NULL);
}

static void receive_world(const lcm_recv_buf_t *rbuf, const char *channel,
                          const vehicle_sim_orientation_t *msg, void *user)
{
    state_t *state = user;
    memcpy(&(state->world_orientation), msg, sizeof(state->world_orientation));
}

static void receive_lidar(const lcm_recv_buf_t *rbuf, const char *channel,
                          const vehicle_sim_lidar_result_t *msg, void *user)
{
    state_t *state = user;
    memcpy(&(state->lidar_result), msg, sizeof(state->lidar_result));
    if (state->lidar_result.lidar_angle == 0) {
        state->lidar_forward_distance = state->lidar_result.distance;
    }
}

static void receive_robot(const lcm_recv_buf_t *rbuf, const char *channel,
                          const vehicle_sim_robot_t *msg, void *user)
{
    state_t *state = user;
    memcpy(&(state->robot), msg, sizeof(state->robot));
}

static void receive_past_pose(const lcm_recv_buf_t *rbuf, const char *channel,
                              const vehicle_sim_orientation_t *msg, void *user)
{
    state_t *state = user;
    int num_poses = zarray_size(state->past_poses);
    if (num_poses > 0) {
        pose_t last_pose;
        zarray_get(state->past_poses, num_poses - 1, &last_pose);
        if (last_pose.x == msg->x && last_pose.y == msg->y) {
            zarray_set(state->past_poses, num_poses - 1, msg, NULL);
            return;
        }
    }

    zarray_add(state->past_poses, msg);

    num_poses = zarray_size(state->past_poses);
    if (num_poses >= 600) {
        zarray_t *reduced_poses = zarray_create(sizeof(pose_t));
        zarray_ensure_capacity(reduced_poses, 300);
        for (int i = 0; i < 600; i += 2) {
            pose_t *pose;
            zarray_get_volatile(state->past_poses, i, &pose);
            zarray_add(reduced_poses, pose);
        }
        zarray_destroy(state->past_poses);
        state->past_poses = reduced_poses;
    }
}

static void receive_lidar_pose(const lcm_recv_buf_t *rbuf, const char *channel,
                               const vehicle_sim_orientation_t *msg, void *user)
{
    state_t *state = user;
    int num_poses = zarray_size(state->lidar_poses);
    if (num_poses > 0) {
        pose_t last_pose;
        zarray_get(state->lidar_poses, num_poses - 1, &last_pose);
        if (last_pose.x == msg->x && last_pose.y == msg->y) {
            zarray_set(state->lidar_poses, num_poses - 1, msg, NULL);
            return;
        }
    }

    zarray_add(state->lidar_poses, msg);

    num_poses = zarray_size(state->lidar_poses);
    if (num_poses >= 600) {
        zarray_t *reduced_poses = zarray_create(sizeof(pose_t));
        zarray_ensure_capacity(reduced_poses, 300);
        for (int i = 0; i < 600; i += 2) {
            pose_t *pose;
            zarray_get_volatile(state->lidar_poses, i, &pose);
            zarray_add(reduced_poses, pose);
        }
        zarray_destroy(state->lidar_poses);
        state->lidar_poses = reduced_poses;
    }
}

int main(int argc, char **argv)
{
    state_t *state = calloc(1, sizeof(state_t));
    state->lcm = lcm_create(NULL);
    pthread_mutex_init(&state->mutex, NULL);
    state->clients = zhash_create(sizeof(vx_canvas_t*), sizeof(client_t*),
                                  zhash_ptr_hash, zhash_ptr_equals);
    state->control_client = NULL;
    state->webvx = webvx_create_server(8123, NULL, "index.html");

    state->past_poses = zarray_create(sizeof(pose_t));
    state->lidar_poses = zarray_create(sizeof(pose_t));

    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);

    printf("CLICK ME: http://localhost:8123\n");

    vehicle_sim_orientation_t_subscribe(state->lcm, "WORLD_ORIENTATION", &receive_world, state);
    vehicle_sim_lidar_result_t_subscribe(state->lcm, "LIDAR", &receive_lidar, state);
    vehicle_sim_robot_t_subscribe(state->lcm, "ROBOT", &receive_robot, state);
    vehicle_sim_orientation_t_subscribe(state->lcm, "PAST_POSE", &receive_past_pose, state);
    vehicle_sim_orientation_t_subscribe(state->lcm, "LIDAR_POSE", &receive_lidar_pose, state);

    while (1) {
        lcm_handle(state->lcm);
    }

    return 0;
}
