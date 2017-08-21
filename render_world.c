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

#include "world.h"

typedef struct state
{
    lcm_t *lcm;
    vx_world_t *world;
    vx_buffer_t *buf;
    webvx_t *webvx;
    getopt_t *gopt;

} state_t;

int main(int argc, char **argv) {
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");

    int parse_success = getopt_parse(gopt, argc, argv, 1);
    if (!parse_success || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
    }

    int port = 4096;
    printf("running on port %d \n", port);
    printf("%s Click on plot to interact. WSAD to move. QE for zoom.\n", __func__);

    state_t *state = calloc(1, sizeof(state_t));
    state->world = vx_world_create();
    state->buf = vx_world_get_buffer(state->world, "main");
    state->webvx = webvx_create_server(port, NULL, "index.html");
    state->gopt = gopt;

    vx_buffer_add_back(state->buf, vxo_grid(vx_white, 1.0f), NULL);
    vx_buffer_add_back(state->buf, vxo_chain(vxo_box_solid(vx_white),
                                              vxo_matrix_translate(3, 0, 0),
                                              vxo_box_solid(vx_red),
                                              vxo_matrix_translate(3, 0, 0),
                                              vxo_box_solid(vx_cyan),
                                              vxo_matrix_translate(3, 0, 0),
                                              vxo_box_solid(vx_yellow),
                                              NULL),
                        NULL);

    webvx_define_canvas(state->webvx, "main_canvas", NULL, NULL, state);
    int64_t last_render_utime;
    while (1) {
        int64_t now = utime_now();

        if ((now - last_render_utime) > 250000) {
            last_render_utime = utime_now();
        }

        usleep(10e3);
    }

    return 0;
}