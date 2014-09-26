pdwiringPi
=========

This repository contains an 'external' (plugin) for [Pure
Data](http://puredata.info) to communicate with hardware on a Raspberry Pi using
the [wiringPi library](http://wiringpi.com).

This code should be assumed to be alpha-quality, it has been minimally tested
but is under active development.

The main [wiringPi] object can act either as a generic wrapper for the library
or can be bound to a specific I/O pin.  In the pin-specific mode, a [bang] input
will read the pin and a numeric input will change the pin state.  In the generic
mode, messages on the input are interpreted as function calls and are mapped to
the supported library API calls.

If pd is run as root (e.g. using sudo), then the object uses the direct hardware
API which is faster and more versatile.  Without root permission, the object
falls back to the slower and less capable /sys/class/gpio interface.

The object assumes that the wiringPi installation includes the 'gpio' program
and will run it using system() if needed.

Compiling
---------

System requirements: make, a C compiler, a Pd installation, and wiringPi.

A simple Makefile is provided for compiling directly on a Raspberry Pi:

    make

Installation
------------

There are two files to be installed:

  1. the loadable module:  wiringPi.pd_linux
  2. wiringPi-help.pd

If compiled in-place, the build folder named pdwiringPi/ can simply be added to
the pd load path.  Or these files can be copied to an existing pd externals
folder such as /usr/local/lib/pd-externals.


