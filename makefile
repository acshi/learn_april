all: lidar_sim world robot render_world

LDFLAGS_GTK := `pkg-config --libs gtk+-2.0`
LDFLAGS_LCM := `pkg-config --libs lcm`

APRIL_LIB = ~/Devel/magic2/lib
LDFLAGS_APRIL = $(APRIL_LIB)/*.a
APRIL_SRC = ~/Devel/magic2/april2/src

lidar_sim: lidar_sim.c lcm_handle_async.c vehicle_sim*.c
	gcc -std=c99 -Wall -o $@ $^ -lm -I $(APRIL_SRC) $(LDFLAGS_GTK) $(LDFLAGS_LCM)

world: world.c lcm_handle_async.c vehicle_sim*.c
	gcc -std=c99 -Wall -o $@ $^ -lm -I $(APRIL_SRC) $(LDFLAGS_GTK) $(LDFLAGS_LCM)

robot: robot.c lcm_handle_async.c vehicle_sim*.c $(APRIL_SRC)/common/zarray.o
	gcc -std=c99 -Wall -o $@ $^ -lm -I $(APRIL_SRC) $(LDFLAGS_GTK) $(LDFLAGS_LCM)

render_world: render_world.c vehicle_sim*.c $(LDFLAGS_APRIL)
	@echo making $@
	script -q /dev/null -c "gcc -std=c99 -Wall -o $@ $^ -lm -L $(APRIL_LIB) -I $(APRIL_SRC) $(LDFLAGS_GTK) $(LDFLAGS_LCM)" 2>&1 | ./filter_warnings.awk

.PHONY: check clean spy

spy:
	CLASSPATH=./ lcm-spy &

check:
	cppcheck  lidar_sim.c world.c robot.c --enable=all --inconclusive --std=c99 -I /home/acshi/Devel
#  -I /usr/include -I /usr/local/include -I /usr/include/x86_64-linux-gnu -I /usr/lib/gcc/x86_64-linux-gnu/5/include -I /usr/lib/gcc/x86_64-linux-gnu/5/include-fixed

clean:
	rm -f lidar_sim world robot
