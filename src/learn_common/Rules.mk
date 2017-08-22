.PHONY: learn_common learn_common_clean

learn_common: lcmtypes

learn_common:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/learn_common -f Build.mk

learn_common_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/learn_common -f Build.mk clean

all: learn_common

clean: learn_common_clean
