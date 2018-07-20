#  (C) 2001-2015 Force Dimension
#  All Rights Reserved.
#
#  Version 3.6.0


# import settings
TOP_DIR = ../..
include $(TOP_DIR)/Makefile.common

# haptic examples
SUBDIRS += sphere \
	       torus \
	       torques

all: $(SUBDIRS)

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
