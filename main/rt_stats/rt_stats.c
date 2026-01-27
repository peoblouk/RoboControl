// ===============================
// rt_stats.c
// ===============================

#include "rt_stats.h"

// ===============================
// Basic statistics functions 
// ===============================
void rt_stats_reset(rt_stats_t *s)
{
    s->count = 0;
    s->min   = 0;
    s->max   = 0;
    s->mean  = 0.0;
    s->m2    = 0.0;
}

void rt_stats_add_sample(rt_stats_t *s, int64_t x)
{
    if (s->count == 0) {
        s->count = 1;
        s->min   = x;
        s->max   = x;
        s->mean  = (double)x;
        s->m2    = 0.0;
        return;
    }

    s->count++;
    if (x < s->min) s->min = x;
    if (x > s->max) s->max = x;

    double delta  = (double)x - s->mean;
    s->mean      += delta / (double)s->count;
    double delta2 = (double)x - s->mean;
    s->m2        += delta * delta2;
}

double rt_stats_stddev(const rt_stats_t *s)
{
    if (s->count < 2) return 0.0;
    double var = s->m2 / (double)(s->count - 1);
    return (var > 0.0) ? sqrt(var) : 0.0;
}

void rt_stats_print(const char *tag, const rt_stats_t *s)
{
    double std = rt_stats_stddev(s);
    printf(
        "[%s] n=%lu, min=%lld us, max=%lld us, mean=%.1f us, stddev=%.1f us\n",
        tag,
        (unsigned long)s->count,
        (long long)s->min,
        (long long)s->max,
        s->mean,
        std
    );
}

// ===============================
// Misc. real-time helpers
// ===============================

int64_t rt_now_us(void)
{
    return esp_timer_get_time();
}

void rt_span_start(int64_t *t_start_us)
{
    *t_start_us = esp_timer_get_time();
}

// Returns time span since t_start_us
int64_t rt_span_end(int64_t t_start_us)
{
    int64_t now = esp_timer_get_time();
    return now - t_start_us;
}

// Mearks time since last call and adds to stats
void rt_loop_mark(rt_stats_t *stats, int64_t *last_timestamp_us)
{
    int64_t now = esp_timer_get_time();

    if (*last_timestamp_us != 0) {
        int64_t dt = now - *last_timestamp_us;
        rt_stats_add_sample(stats, dt);
    }

    *last_timestamp_us = now;
}