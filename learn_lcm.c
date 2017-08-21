#include <stdio.h>
#include <lcm/lcm.h>
#include "exlcm_example_t.h"

int main(int argc, char **argv)
{
    lcm_t *lcm = lcm_create(NULL);
    if (!lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }

    exlcm_example_t my_data = {
        .timestamp = 0,
        .position = {1, 2, 3},
        .orientation = {1, 0, 0, 0},
        .name = "Example String",
        .enabled = 1,
    };
    int16_t ranges[15];
    for (int i = 0; i < 15; i++) {
        ranges[i] = i;
    }
    my_data.num_ranges = 15;
    my_data.ranges = ranges;

    exlcm_example_t_publish(lcm, "EXAMPLE", &my_data);

    lcm_destroy(lcm);
    printf("Done!\n");
    return 0;
}
