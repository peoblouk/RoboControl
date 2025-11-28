// ===============================
// rt_stats.h
// Simple runtime statistics helpers
// ===============================

#ifndef RT_STATS_H
#define RT_STATS_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "esp_timer.h"

typedef struct {
    uint32_t count;
    int64_t  min;
    int64_t  max;
    double   mean;
    double   m2;   // sum of squares of differences from the mean
} rt_stats_t;

// Baisic statistics functions
void   rt_stats_reset(rt_stats_t *s);
void   rt_stats_add_sample(rt_stats_t *s, int64_t x);
double rt_stats_stddev(const rt_stats_t *s);
void   rt_stats_print(const char *tag, const rt_stats_t *s);

// ===============================
// Misc. real-time helpers
// ===============================

// Vrátí čas v µs
int64_t rt_now_us(void);

//! Start measuring a time span:
//   int64_t t0;
//   rt_span_start(&t0);
//   ... code ...
//   int64_t dt = rt_span_end(t0);
//   rt_stats_add_sample(&stats, dt);

void rt_span_start(int64_t *t_start_us);
int64_t rt_span_end(int64_t t_start_us);

//! Measuring the period of a "loop" / task:
//   static rt_stats_t loop_stats;
//   static int64_t    loop_last = 0;

//   while (1) {
//       rt_loop_mark(&loop_stats, &loop_last);
//       ... body of the task ...
//   }

// On each call, adds to stats the difference between "now" and the previous call.
void rt_loop_mark(rt_stats_t *stats, int64_t *last_timestamp_us);

#endif // RT_STATS_H
