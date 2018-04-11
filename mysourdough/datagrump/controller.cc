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
#define MAX_RATE 10 
#define TICKS_PER_RTT 5 
#define PERCENTILE_LATENCY 0.05

using namespace std;

// sum of 0 through max rate * TICKS_PER_RTT
double total_packets;

// probability distributions
double rate_probability[MAX_RATE];


/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{
//  total_packets = 0;
//  for (int i = 0; i < MAX_RATE; i++) {
//    total_packets += i * TICKS_PER_RTT;
//  }
  for (int i = 0; i < MAX_RATE; i++) {
    rate_probability[i] = 1.0 / MAX_RATE;
  }
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

// acks received during tick
uint64_t packets_in_tick = 0;

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

uint64_t factorial(uint64_t n) {
  if (n == 0) return 1;
  return n*factorial(n-1);
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
    cerr << "new window sz: " << the_window_size << endl;
  }
  last_ack = sequence_number_acked > last_ack ? sequence_number_acked : last_ack;
 
  if (COOL_ALG) {
    packets_in_tick++;
    if (timestamp_ack_received - last_tick >= TICK) {
      // Tick has elapsed.
      double sum = 0;
      // Update rate probabilities. 
      for (int i = 0; i < MAX_RATE; i++) {
        double p = pow(i * (TICK  / 1000.0), packets_in_tick);
        p /= (double) factorial(packets_in_tick);
        p *= exp(-1 * i * (TICK / 1000.0));
        rate_probability[i] = /*rate_probability[i] **/ p;
        cerr << "rate_probability[" << i << "] = " << rate_probability[i] << endl;
        sum += rate_probability[i];
      }
      // Normalize rate probabilities. 
      for (int i = 0; i < MAX_RATE; i++) {
        rate_probability[i] /= sum;
      }
      // Set window size based on largest rate_probability value
      sum = 0;
      int i = 0;
      while (sum < PERCENTILE_LATENCY && i < MAX_RATE) {
        sum += rate_probability[i];
        i++;
      }
      the_window_size = i * TICKS_PER_RTT;
      cerr << "new window sz: " << the_window_size << endl;
      // 95th percentile just use lambda * 8 (TICK * 8 = RTT)     
      // Reset for next period.
      last_tick = timestamp_ack_received;
      packets_in_tick = 0;
    }
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
  return 1000; /* timeout of one second */
}
