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
#define MAX_RATE 200
#define TICKS_PER_RTT 3.5 // was 3.5 
#define PERCENTILE_LATENCY 0.01
#define PACKETS_PER_BUCKET 20.0  // rename to INTERVAL_SIZE
#define EWMA_WEIGHT 0.2 
#define BROWNIAN_MOTION 200.0
#define MIN_PROB 0.000001

#define DEBUG false

using namespace std;

// sum of 0 through max rate * TICKS_PER_RTT
//double total_packets;

// probability distributions
double rate_probability[MAX_RATE];


/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{
//  total_packets = 0;
//  for (int i = 0; i < MAX_RATE; i++) {
//    total_packets += i / PACKETS_PER_BUCKET * TICKS_PER_RTT;
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

// acks received during previous tick
uint64_t old_packets_in_tick = 0;

// track last ackno received so don't double count packets
uint64_t last_ackno = 0;

double time_elapsed = 0.0;

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

double cdf(double mean, double stddev, double x) {
  return 0.5 * erfc(-1 * (x - mean) / (stddev));
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
    if (last_ackno != sequence_number_acked) {
      packets_in_tick++;
      last_ackno = sequence_number_acked;
    }
    if (timestamp_ack_received - last_tick >= TICK) {
      // Tick has elapsed.
      double sum = 0;
      // Evolve rate probabilities.
      if (time_elapsed != 0.0) {
      double stddev = BROWNIAN_MOTION * sqrt(time_elapsed);
      if (DEBUG) cout << "STDDEV: " << stddev << endl;
//      double mean = old_packets_in_tick;
        //double val = cdf(mean, stddev, packets_in_tick + PACKETS_PER_BUCKET - old_packets_in_tick) - cdf(mean, stddev, packets_in_tick - old_packets_in_tick);
        rate_probability[0] = max(rate_probability[0], MIN_PROB);
	if (DEBUG) cout << "evolved rate probability[0] = " << rate_probability[0] << endl;
        for (int i = 1; i < MAX_RATE; i++) {
          double mean = 0.0;
          double val = cdf(mean, stddev, packets_in_tick + (1.0 / PACKETS_PER_BUCKET) - i) - cdf(mean, stddev, packets_in_tick - i);
          //double val = cdf(mean, stddev, packets_in_tick + PACKETS_PER_BUCKET - (i * PACKETS_PER_BUCKET)) - cdf(mean, stddev, packets_in_tick - (i * PACKETS_PER_BUCKET));
          // cdf(min(ceil,new + PACKETS_PER_BUCKET) - old) - cdf(max(floor,new) - old)
	  if (DEBUG) cout << "evolved rate probability[" << i << "] by " << val << " to " << rate_probability[i] << endl;
          rate_probability[i] = max(rate_probability[i] * val, MIN_PROB);  // prevent -nan with 0 
          //rate_probability[i] = min(rate_probability[i], MIN_PROB) + val;
        }
      }
      // Update rate probabilities. 
      for (int i = 0; i < MAX_RATE; i++) {
        // Calculating poisson
        double p = pow((i / PACKETS_PER_BUCKET) * (TICK  / 1000.0), packets_in_tick);
        p /= (double) factorial(packets_in_tick);
        p *= exp(-1 * (i / PACKETS_PER_BUCKET) * (TICK / 1000.0));
        rate_probability[i] = rate_probability[i] * p;
        if (DEBUG) cerr << "rate_probability[" << i << "] = " << rate_probability[i] << endl;
        sum += rate_probability[i];
      }
      if (DEBUG) cerr << "sum = " << sum << endl;
      // Normalize rate probabilities. 
      for (int i = 0; i < MAX_RATE; i++) {
        rate_probability[i] = rate_probability[i] / sum;
        if (DEBUG) cerr << "normalized rate_probability[" << i << "] = " << rate_probability[i] << endl;
      }
      // Set window size based on largest rate_probability value
      sum = 0;
      int i = 0;
      while (sum < PERCENTILE_LATENCY && i < MAX_RATE) {
        sum += rate_probability[i];
        i++;
      }
      //uint64_t new_estimate = (i / PACKETS_PER_BUCKET) * TICKS_PER_RTT;
      //if (new_estimate >= the_window_size) {
      //  the_window_size = new_estimate;
      //} else {
      //  the_window_size = (EWMA_WEIGHT * new_estimate) + ((1 - EWMA_WEIGHT) * the_window_size);
      //}
      the_window_size = EWMA_WEIGHT * ((i / PACKETS_PER_BUCKET) * TICKS_PER_RTT) + ((1 - EWMA_WEIGHT) * the_window_size);
      if (DEBUG) cerr << "new window sz: " << the_window_size << endl;
      // 95th percentile just use lambda * 8 (TICK * 8 = RTT)     
      // Reset for next period.
      last_tick = timestamp_ack_received;
      old_packets_in_tick = packets_in_tick;
      packets_in_tick = 0;
      time_elapsed += TICK / 1000.0;
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
