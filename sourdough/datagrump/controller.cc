#include <iostream>

#include "controller.hh"
#include "timestamp.hh"
#include <math.h>

#define AIMD false 
#define AIMD_INC 2
#define AIMD_DEC 0.5

#define DELAY_TRIGGERED false 
#define DT_INC 1
#define DT_DEC 10 
#define DT_THRESHOLD 450  // in milliseconds
#define PERIOD 2 // in milliseconds, time to wait before next increase

#define COOL_ALG true
#define TICK 20 // in ms
#define EWMA_WEIGHT 0.2
#define RTT_PERCENT 1.5

#define DEBUG false 

using namespace std;

// sum of 0 through max rate * TICKS_PER_RTT
//double total_packets;


/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{
}

/* Default: fixed window size of 100 outstanding datagrams */
unsigned int the_window_size = 20;

unsigned int in_progress_window = 0;

// last ack received, used for delay triggered
uint64_t last_ack = 0;

// last time window was updated, used for delay triggered
uint64_t last_update = 0;

// last tick time
uint64_t last_tick = 0;


uint64_t prev_rtt = 50;
uint64_t prev_rtt_sum = 50;
uint64_t min_rtt = 50;
uint64_t curr_rtt_sum = 0;

/* Get current window size, in datagrams */
unsigned int Controller::window_size()
{

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }

  return the_window_size;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp,
                                    /* in milliseconds */
				    const bool after_timeout
				    /* datagram was sent because of a timeout */ )
{
  /* Default: take no action */

  if (AIMD) {
    if (after_timeout) {
      the_window_size = the_window_size*AIMD_DEC;
    }
  }

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " (timeout = " << after_timeout << ")\n";
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  /* Default: take no action */

  if (AIMD) {
    in_progress_window += AIMD_INC;
    if (in_progress_window >= the_window_size) {
      the_window_size += 1;
      in_progress_window = 0;
    }
  }

  if (DELAY_TRIGGERED) {
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    if (rtt < DT_THRESHOLD && sequence_number_acked > last_ack) {
         in_progress_window += DT_INC;
         if (in_progress_window >= the_window_size) {
           the_window_size += 1;
           in_progress_window = 0;
         }
    } else {
       uint64_t new_window_sz = the_window_size - DT_DEC;
       the_window_size = new_window_sz < the_window_size ? new_window_sz : 0; // check for overflow
    }
    if (DEBUG) cerr << "new window sz: " << the_window_size << endl;
  }
  last_ack = sequence_number_acked > last_ack ? sequence_number_acked : last_ack;
 
  if (COOL_ALG) {
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    min_rtt = min(min_rtt, rtt);
    if (rtt <= 1 * min_rtt) {
	the_window_size += 2;
    } else if (rtt <= 1.5 * min_rtt) {
	the_window_size += 1;
    } 
    if (timestamp_ack_received - last_tick >= TICK) {
      if (curr_rtt_sum >= prev_rtt_sum)
	the_window_size = 1 + .7 * the_window_size;	
      else
	the_window_size = 1 + .9 * the_window_size;
      last_tick = timestamp_ack_received;
      prev_rtt_sum = curr_rtt_sum;
      curr_rtt_sum = 0; 
    }
    prev_rtt = rtt;
    curr_rtt_sum += prev_rtt;
  }
 
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}


/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return 2.0 * min_rtt; /* timeout of one second */
}
