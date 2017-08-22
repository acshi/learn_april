.PHONY: render_world render_world_clean

render_world: lcmtypes vx

render_world:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/render_world -f Build.mk

render_world_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/render_world -f Build.mk clean

all: render_world

clean: render_world_clean
