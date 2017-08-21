#define _XOPEN_SOURCE 700
#include <stdio.h>
#include <lcm/lcm.h>
#include <inttypes.h>
#include <sys/select.h>
#include <time.h>
#include "exlcm_example_t.h"
#include "lcm_handle_async.h"

static void my_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                       const exlcm_example_t *msg, void *user)
{
    printf("Received message on channel \"%s\":\n", channel);
    printf("  timestamp   = %"PRId64"\n", msg->timestamp);
    printf("  position    = (%f, %f, %f)\n", msg->position[0], msg->position[1], msg->position[2]);
    printf("  orientation = (%f, %f, %f, %f)\n", msg->orientation[0], msg->orientation[1], msg->orientation[2], msg->orientation[3]);
    printf("  ranges:");
    for(int i = 0; i < msg->num_ranges; i++) {
        printf(" %d", msg->ranges[i]);
    }
    printf("\n");
    printf("  name        = '%s'\n", msg->name);
    printf("  enabled     = %d\n", msg->enabled);
}

int main(int argc, char **argv)
{
    lcm_t *lcm = lcm_create(NULL);
    if (!lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }

    exlcm_example_t_subscribe(lcm, "EXAMPLE", &my_handler, NULL);

    while (1) {
        lcm_handle_async(lcm);
        nanosleep(&(struct timespec){0, 100e6}, NULL);
    }

    lcm_destroy(lcm);
    return 0;
}
