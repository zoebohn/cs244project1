#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

#define FIXED_WINDOW false

#define AIMD false 
#define AIMD_INC 1
#define AIMD_DEC 0.25

#define EMMA_ZOE_ALG true
#define TICK 20 // in ms

using namespace std;

unsigned int the_window_size = 20;
unsigned int in_progress_window = 0;
uint64_t last_tick = 0;

uint64_t prev_rtt_avg = 50.0;
uint64_t min_rtt = 50;
uint64_t curr_rtt_sum = 0;
uint64_t packets_in_tick = 0;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{
    if (FIXED_WINDOW) the_window_size = 15;
}

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

  if (EMMA_ZOE_ALG) {
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    min_rtt = min(min_rtt, rtt);
    packets_in_tick++;
    if (rtt <= 1 * min_rtt) {
	the_window_size += 2;
    } else if (rtt <= 1.5 * min_rtt) {
	the_window_size += 1; 
    } 
    if (timestamp_ack_received - last_tick >= TICK) {
      double curr_rtt_avg = curr_rtt_sum / ((double)packets_in_tick);
      if (curr_rtt_avg >= prev_rtt_avg)
	the_window_size = 1 + .7 * the_window_size;	
      else
	the_window_size = 1 + .9 * the_window_size;
      last_tick = timestamp_ack_received;
      prev_rtt_avg = curr_rtt_avg;
      curr_rtt_sum = 0;
      packets_in_tick = 0; 
    }
    curr_rtt_sum += rtt;
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
  return EMMA_ZOE_ALG ? 2.0 * min_rtt: 1000; /* default timeout of one second */
}
