SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)

CFLAGS := $(CFLAGS_STD) $(CFLAGS_VX) $(CFLAGS_LCMTYPES)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_LCMTYPES)
DEPS := $(DEPS_STD) $(DEPS_VX) $(DEPS_LCMTYPES)

include $(BUILD_COMMON)


all: $(LIB_PATH)/libcommon.a
	@/bin/true

$(LIB_PATH)/libcommon.a: $(OBJS)
	@$(AR) rc $@ $(OBJS)

clean:
	@rm -rf *.o $(OBJS) $(LIB_PATH)/libcommon.a
