# -*- mode: makefile; -*-
# Makefile - build control for the pdwiringPi external for the Raspberry Pi.
# Copyright (c) 2014, Garth Zeglin.  All rights reserved.  Provided under the terms of the BSD 3-clause license.

# ---- user-customizable settings ----------------------------------

# Configure the include path for Pure Data. Under Debian or Ubuntu, the pd
# include files can be found on the standard system include path.

PD_INCLUDE_PATH=/usr/include

# Configure to compile and link against the default WiringPi
# system, assumed to be installed in /usr/local.

WIRINGPI_CFLAGS  := -I/usr/local/include
WIRINGPI_LDFLAGS := -L/usr/local/lib -lwiringPi

# ---- The following settings do not usually need to be changed ----

# Specify the extension to use for the loadable module.
EXTERNALS_EXT = pd_linux

# Specify a folder to hold the compiled binary Pd loadable modules.
EXTERNALS_DIR = pdwiringPi

# Specify the default targets to build.
default: $(EXTERNALS_DIR) $(EXTERNALS_DIR)/wiringPi.$(EXTERNALS_EXT) $(EXTERNALS_DIR)/wiringPi-help.pd

# Create the target folder if it doesn't exist.
$(EXTERNALS_DIR):
	mkdir $(EXTERNALS_DIR)

# Define the compile and link flags for producing a loadable module.
MODULE_CFLAGS = -fPIC -shared -I$(PD_INCLUDE_PATH)

# Build the loadable module
$(EXTERNALS_DIR)/wiringPi.$(EXTERNALS_EXT): src/pdwiringPi.c
	$(CC) $(MODULE_CFLAGS) $^ -o $@ $(WIRINGPI_CFLAGS) $(WIRINGPI_LDFLAGS)

# Copy over the help patch.
$(EXTERNALS_DIR)/wiringPi-help.pd: src/wiringPi-help.pd
	cp $< $@

# Target to clean up the build folder.
clean: $(EXTERNALS_DIR)
	-rm -r $(EXTERNALS_DIR)/*
