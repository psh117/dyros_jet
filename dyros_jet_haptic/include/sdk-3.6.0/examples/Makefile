#  (C) 2001-2015 Force Dimension
#  All Rights Reserved.
#
#  Version 3.6.0


TOP_DIR = ..
include $(TOP_DIR)/Makefile.common

SUBDIRS  = CLI

ifeq ($(BUILD_GLUT), 1)
SUBDIRS += GLUT
endif

ifeq ($(BUILD_QT), 1)
SUBDIRS += Qt
endif

all: $(SUBDIRS)

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
