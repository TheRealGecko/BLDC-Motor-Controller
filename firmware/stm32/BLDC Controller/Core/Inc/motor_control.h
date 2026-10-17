#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"

/* FSM states */
typedef enum state {
  ALIGNMENT, // Align rotor to known position
  OPEN_LOOP, // Run pre-determined 6-step commutation
  CLOSED_LOOP, // Run commutation with BEMF detection
  OFF, // System is intentionally not running
  SHUTDOWN // Fault occured and system was shutdown to handle fault
} state_t;

/* FSM flags */
typedef struct flags {
  uint8_t start_request; // Potentiometer certain threshold greater than 0
  uint8_t stop_request; // Potentiometer certain threshold from 0
  uint8_t comm_request; // For closed loop. Request to commutate

  uint8_t align_complete; // Rotor alignment complete

  uint8_t bemf_updated; // BEMF validity array updated

  uint8_t bemf_stable; // Lets open/closed loop know if BEMF values are stable

  uint8_t fault; // Fault occured in alignment/open loop/closed loop
} flags_t;



#endif /* __MOTOR_CONTROL_H */