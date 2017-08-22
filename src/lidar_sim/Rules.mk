.PHONY: lidar_sim lidar_sim_clean

lidar_sim: lcmtypes common

lidar_sim:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lidar_sim -f Build.mk

lidar_sim_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lidar_sim -f Build.mk clean

all: lidar_sim

clean: lidar_sim_clean
