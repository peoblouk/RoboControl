// ===============================
// rt_stats.h
// ===============================

#ifndef RT_STATS_H
#define RT_STATS_H
#include "config.h"  // Configuration

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include "esp_timer.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    uint32_t count;
    int64_t  min;
    int64_t  max;
    double   mean;
    double   m2;   // sum of squares of differences from the mean
} rt_stats_t;

// ===============================
// FUNCTION PROTOTYPES
// ===============================
void   rt_stats_reset(rt_stats_t *s);
void   rt_stats_add_sample(rt_stats_t *s, int64_t x);
double rt_stats_stddev(const rt_stats_t *s);
void   rt_stats_print(const char *tag, const rt_stats_t *s);

// return time now in microseconds
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

// ===============================
// Runtime robot statistics
// ===============================
typedef enum {
    RT_MON_CONTROL_LOOP_PERIOD_US = 0,
    RT_MON_CONTROL_LOOP_WORK_US,
    RT_MON_APPLY_JOINTS_US,
    RT_MON_PLANNER_IK_US,
    RT_MON_QUEUE_LATENCY_US,
    RT_MON_METRIC_COUNT
} rt_monitor_metric_t;

typedef enum {
    RT_MON_EVENT_QUEUE_SEND_OK = 0,
    RT_MON_EVENT_QUEUE_SEND_FAIL,
    RT_MON_EVENT_PLANNER_SEG_FULL,
    RT_MON_EVENT_PLANNER_IK_FAIL,
    RT_MON_EVENT_COUNT
} rt_monitor_event_t;

void rt_monitor_reset(void);
void rt_monitor_add_sample(rt_monitor_metric_t metric, int64_t value_us);
void rt_monitor_count_event(rt_monitor_event_t event);
void rt_monitor_start_file_task(void);
void rt_monitor_write_file_now(void);
const char *rt_monitor_file_path(void);

#endif // RT_STATS_H
