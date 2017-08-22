.PHONY: world world_clean

world: lcmtypes

world:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/world -f Build.mk

world_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/world -f Build.mk clean

all: world

clean: world_clean
