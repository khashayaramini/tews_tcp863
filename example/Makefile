################################################################
#
# TDRV009 Example Application Makefile
#
# Copyright (c) 2007-2018 by TEWS TECHNOLOGIES GmbH
#
################################################################

VER := $(shell uname -r)
INCLUDEDIR = /lib/modules/$(VER)/build/include

MOD_PREFIX = tdrv009

CFLAGS = -I$(INCLUDEDIR) -Wall -O2
OBJ = ../api/tdrv009api.o

$(MOD_PREFIX)exa: $(MOD_PREFIX)exa.o $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(MOD_PREFIX)exa.o $(OBJ)
