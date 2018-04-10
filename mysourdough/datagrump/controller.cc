#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

#define AIMD false 
#define AIMD_INC 1 
#define AIMD_DEC 0.1

#define DELAY_TRIGGERED true
#define DT_INC 1
#define DT_DEC 10
#define DT_THRESHOLD 350  // in milliseconds
#define PERIOD 2 // in milliseconds, time to wait before next increase

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{}

/* Default: fixed window size of 100 outstanding datagrams */
unsigned int the_window_size = 1;

// last ack received, used for delay triggered
uint64_t last_ack = 0;

// last time window was updated, used for delay triggered
uint64_t last_update = 0;

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
    the_window_size += AIMD_INC;
  }

  if (DELAY_TRIGGERED) {
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    if (rtt < DT_THRESHOLD && sequence_number_acked > last_ack) {
       if (timestamp_ack_received - last_update >= PERIOD) {
         last_update = timestamp_ack_received;
         the_window_size += DT_INC;
       }
    } else {
       last_update = timestamp_ack_received;
       uint64_t new_window_sz = the_window_size - DT_DEC;
       the_window_size = new_window_sz < the_window_size ? new_window_sz : 0; // check for overflow
    }
    cerr << "new window sz: " << the_window_size << endl;
  }
  last_ack = sequence_number_acked > last_ack ? sequence_number_acked : last_ack;
  
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
  return 1000; /* timeout of one second */
}
