.PHONY: robot robot_clean

robot: lcmtypes graph vx

robot:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/robot -f Build.mk

robot_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/robot -f Build.mk clean

all: robot

clean: robot_clean

robot.c: robot.h ../world/world.h
