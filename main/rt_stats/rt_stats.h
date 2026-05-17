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
    uint64_t sum;
    uint64_t sum_sq;
} rt_stats_t;

// ===============================
// FUNCTION PROTOTYPES
// ===============================
void   rt_stats_reset(rt_stats_t *s);
void   rt_stats_add_sample(rt_stats_t *s, int64_t x);
double rt_stats_mean(const rt_stats_t *s);
double rt_stats_stddev(const rt_stats_t *s);

// return time now in microseconds
int64_t rt_now_us(void);

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

#endif // RT_STATS_H
