/// pdwiringPi.c : Pd external to communicate with hardware on a Raspberry Pi using wiringPi
/// Copyright (c) 2014, Garth Zeglin.  All rights reserved.  Provided under the
/// terms of the BSD 3-clause license.

/****************************************************************/
// Links to related reference documentation:

//   Pd externals:      http://pdstatic.iem.at/externals-HOWTO/node9.html
//   wiringPi:          http://wiringpi.com

/****************************************************************/ 
// import standard libc API
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

// Import the API for Pd externals.  This is provided by the Pd installation,
// and this may require manually configuring the Pd include path in the
// Makefile.
#include "m_pd.h"

// Import the wiringPi API.
#include <wiringPi.h>
#include <wiringPiSPI.h>

/****************************************************************/ 
/// There is only one wiringPi interface, so the underlying
/// hardware state is stored globally to be referenced by any
/// wiringPi object.
static int sys_mode = 0;        ///< flag to indicate initialization with wiringPiSetupSys

/****************************************************************/ 
/// Data structure to hold the state of a single Pd 'wiringPi' object.
typedef struct pdwiringPi
{
  t_object x_ob;           ///< standard object header
  t_outlet *x_outlet;      ///< the outlet is the port on which return values are transmitted

  int pin;                 ///< a non-negative pin number if initalized to control a single pin
  int mode;                ///< pin mode value if initialized to control a single pin

  int spi_channel;         ///< SPI channel number, or -1 if not in SPI mode
  int spi_speed;           ///< SPI speed in Hz, or -1 if not in SPI mode
  int spi_fd;              ///< SPI file descriptor, or -1 if not in SPI mode

} t_pdwiringPi;

static t_class *pdwiringPi_class;

/****************************************************************/
// Utility functions.
static int atom_matches( t_atom *atom, char *symbol )
{
  return ( (atom->a_type == A_SYMBOL) 
	   && !strcmp( atom->a_w.w_symbol->s_name, symbol ) );
}

static inline int symbol_matches( t_symbol *sym, char *symbol )
{
  return !strcmp( sym->s_name, symbol );
}

/****************************************************************/
// Map a symbol representing a pin mode to the numeric constant.
static int atom_to_pin_mode( t_atom *s_mode )
{	
  int value = INPUT;
  if      ( atom_matches( s_mode, "output" ))     value = OUTPUT;
  else if ( atom_matches( s_mode, "pwm_output" )) value = PWM_OUTPUT;
  else if ( atom_matches( s_mode, "gpio_clock" )) value = GPIO_CLOCK;
  else if ( atom_matches( s_mode, "input" ))      value = INPUT;
  else post ("wiringPi: unrecognized pin mode symbol, assuming INPUT.");
  return value;
}

/****************************************************************/
// A bang is always interpreted as a type of read for pin-specific instances.
static void pdwiringPi_bang( t_pdwiringPi *x )
{
  if ( x->pin >= 0 ) {
    int value = digitalRead( x->pin );
    outlet_float( x->x_outlet, value );
    return;

  } else {
    post("wiringPi: bang not allowed for non-pin-specific instances.");
  }
}
/****************************************************************/
// A float is always interpreted as a type of write for pin-specific instances.
static void pdwiringPi_float( t_pdwiringPi *x, float value )
{
  // if a pin-specific instance
  if ( x->pin >= 0 ) {

    // pwm output mode
    if (x->mode == PWM_OUTPUT) {
      // "The Raspberry Pi has one on-board PWM pin, pin 1 (BMC_GPIO 18, Phys 12)
      // and the range is 0-1024."
      pwmWrite( x->pin, (int) value );
      if (sys_mode) post("wiringPi: Warning, pwmWrite has no effect in Sys mode.");
    }
    // else assume it is a digital output
    else {
      digitalWrite( x->pin, (int) value );
    }
  } else {
    post("wiringPi: float not allowed for non-pin-specific instances.");
  }
}

// still need to add SPI I/O
//  int wiringPiSPIDataRW (int channel, unsigned char *data, int len) ;

/****************************************************************/
// Abstract the difference between the Sys and Gpio initialization modes.
static void set_pin_mode( int pin, int mode )
{
  if (sys_mode) {
    if (mode == INPUT || mode == OUTPUT) {
      char *command;
      asprintf( &command, "gpio export %d %s", pin, (mode==INPUT) ? "in" : "out" );
      post("wiringPi in Sys mode: running '%s'", command );
      system(command);
      free(command);
    } else {
      post("wiringPi: error, cannot set mode %d on pin %d in Sys mode.", mode, pin );
    }
  } else {
    pinMode( pin, mode );
  }
}

/****************************************************************/
/// Process a list representing a function call or more elaborate I/O command.

static void pdwiringPi_eval( t_pdwiringPi *x, t_symbol *selector, int argcount, t_atom *argvec )
{
  // post ("pdwiringPi_eval called with %d args, selector %s\n", argcount, selector->s_name );
  
  if (x->pin >= 0) {
    post("wiringPi: general input not allowed for pin-specific instances.");
    return;
  }

  // test for a variety of function call forms
  if ( symbol_matches( selector, "digitalRead" ) && argcount == 1) {
    outlet_float( x->x_outlet, (float) digitalRead( atom_getint( &argvec[0] )));
    return;

  } else if ( symbol_matches( selector, "digitalWrite" ) && argcount == 2) {
    digitalWrite( atom_getint(&argvec[0]), atom_getint(&argvec[1]) );
    return;

  } else if ( symbol_matches( selector, "pwmWrite" ) && argcount == 2) {
    // "The Raspberry Pi has one on-board PWM pin, pin 1 (BMC_GPIO 18, Phys 12)
    // and the range is 0-1024."
    pwmWrite( atom_getint(&argvec[0]), atom_getint(&argvec[1]) );
    if (sys_mode) post("wiringPi: Warning, pwmWrite has no effect in Sys mode.");
    return;

  } else if ( symbol_matches( selector, "pinMode" ) && argcount == 2) {
    int mode = atom_to_pin_mode( &argvec[1] );
    set_pin_mode( atom_getint(&argvec[0]), mode );
    return;

  } else if ( symbol_matches( selector, "wpiPinToGpio" ) && argcount == 1) {
    outlet_float( x->x_outlet, (float) wpiPinToGpio( atom_getint(&argvec[0]) ));
    return;

  } else if ( symbol_matches( selector, "physPinToGpio" ) && argcount == 1) {
    outlet_float( x->x_outlet, (float) physPinToGpio( atom_getint(&argvec[0]) ));
    return;

  } else if ( symbol_matches( selector, "piBoardRev" ) && argcount == 0) {
    outlet_float( x->x_outlet, (float) piBoardRev() );
    return;

  } else if ( symbol_matches( selector, "load_spi_driver" ) && argcount == 0) {
    char *command = "gpio load spi";
    post("wiringPi: running '%s' to make sure SPI drivers are loaded.", command );
    system(command);
    return;

  } else if ( symbol_matches( selector, "spi_init" )) {
    // specialize an object instance to represent an SPI port
    //  [ spi_init <spi-number> <spi-speed> ]
    if (argcount == 2) {
      x->spi_channel = atom_getint( &argvec[0] );
      x->spi_speed   = atom_getint( &argvec[1] );
      x->spi_fd      = wiringPiSPISetup ( x->spi_channel, x->spi_speed);
      if (x->spi_fd == -1) {
	post("wiringPiSPISetup returned error %d.", errno );
      } else {
	post("wiringPi: opened SPI channel %d at speed %d.", x->spi_channel, x->spi_speed);
      }
    } else {
      post("wiringPi error: spi_init requires channel and speed values.");
    }

  } else {
    post("wiringPi: unrecognized input for selector %s.", selector->s_name );
  }
}

/****************************************************************/
/// Create an instance of a Pd 'wiringPi' object.
///
/// The creation arguments are all optional and are interpreted as follows:
///  [ wiringPi pin <pin-number> <mode-symbol> ] make instance pin-specific

static void *pdwiringPi_new(t_symbol *selector, int argcount, t_atom *argvec)
{
  t_pdwiringPi *x = (t_pdwiringPi *) pd_new(pdwiringPi_class);

  // initialize default values for a generic instance
  x->pin  = -1;
  x->mode = 0;

  x->spi_channel = -1;
  x->spi_speed   = -1;
  x->spi_fd      = -1;

  if (argcount > 0) {
    // check the initial creation argument
    t_atom *arg0 = &argvec[0];

    // define the object instance as 'pin-specific' representing a single pin
    if ( atom_matches( arg0, "pin" )) {
      // "Note that only wiringPi pin 1 (BCM_GPIO 18) supports PWM output and
      // only wiringPi pin 7 (BCM_GPIO 4) supports CLOCK output modes."

      if (argcount == 3 ) {
	x->pin  = atom_getint( &argvec[1] );
	x->mode = atom_to_pin_mode( &argvec[2] );
	set_pin_mode( x->pin, x->mode );
	// post("Initialized wiringPi object for pin %d in mode %d.", x->pin, x->mode );

      } else {
	post("wiringPi: incorrect number of creation arguments for pin.");
      }
    } else {
      post("wiringPi: unrecognized creation arguments.");
    }
  }

  // create an outlet on which to return values
  x->x_outlet = outlet_new( &x->x_ob, NULL );
  return (void *)x;
}

/****************************************************************/
/// Release an instance of a Pd 'wiringPi' object.
static void pdwiringPi_free(t_pdwiringPi *x)
{
  if (x) {
    outlet_free( x->x_outlet );
    x->x_outlet = NULL;
  }
}

/****************************************************************/
/// Initialization entry point for the Pd 'wiringPi' external.  This is
/// automatically called by Pd after loading the dynamic module to initialize
/// the class interface.
void wiringPi_setup(void)
{
  // specify "A_GIMME" as creation argument for both the creation
  // routine and the method (callback) for the "eval" message.

  pdwiringPi_class = class_new( gensym("wiringPi"),              // t_symbol *name
				(t_newmethod) pdwiringPi_new,    // t_newmethod newmethod
				(t_method) pdwiringPi_free,      // t_method freemethod
				sizeof(t_pdwiringPi),            // size_t size
				0,                               // int flags
				A_GIMME, 0);                     // t_atomtype arg1, ...

  // instances specialized to specific pins can accept bangs and floats
  class_addbang ( pdwiringPi_class, pdwiringPi_bang );    // t_class *c, t_method fn
  class_addfloat( pdwiringPi_class, pdwiringPi_float );  // t_class *c, t_method fn

  // general inputs represent function calls and more elaborate I/O operations
  class_addanything( pdwiringPi_class, (t_method) pdwiringPi_eval );   // (t_class *c, t_method fn)

  // static initialization follows
  if (geteuid() == 0) {
    wiringPiSetupGpio();
    sys_mode = 0;
    post("Initializing wiringPi in Gpio mode to use direct hardware access.\nUsing Broadcom GPIO pin numbering scheme.");

  } else {
    wiringPiSetupSys();    
    sys_mode = 1;
    post("Warning: initializing wiringPi in Sys mode to use /sys/class/gpio interface.\nFor a faster interface, relaunch pd as root.\nUsing Broadcom GPIO pin numbering scheme.");
  }

}

/****************************************************************/
