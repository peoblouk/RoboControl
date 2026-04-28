// ===============================
// rt_stats.c
// ===============================

#include "rt_stats.h"


static const char *TAG = "rt_stats";

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

// ===============================
// Runtime robot statistics
// ===============================

static const char *const s_metric_names[RT_MON_METRIC_COUNT] = {
    [RT_MON_CONTROL_LOOP_PERIOD_US] = "control_loop_period",
    [RT_MON_CONTROL_LOOP_WORK_US]   = "control_loop_work",
    [RT_MON_APPLY_JOINTS_US]        = "apply_joints",
    [RT_MON_PLANNER_IK_US]          = "planner_ik",
    [RT_MON_QUEUE_LATENCY_US]       = "queue_latency",
};

static const char *const s_event_names[RT_MON_EVENT_COUNT] = {
    [RT_MON_EVENT_QUEUE_SEND_OK]    = "queue_send_ok",
    [RT_MON_EVENT_QUEUE_SEND_FAIL]  = "queue_send_fail",
    [RT_MON_EVENT_PLANNER_SEG_FULL] = "planner_seg_full",
    [RT_MON_EVENT_PLANNER_IK_FAIL]  = "planner_ik_fail",
};

typedef struct {
    int64_t updated_us;
    rt_stats_t metrics[RT_MON_METRIC_COUNT];
    uint32_t events[RT_MON_EVENT_COUNT];
} rt_monitor_snapshot_t;

static rt_stats_t s_monitor_metrics[RT_MON_METRIC_COUNT];
static uint32_t s_monitor_events[RT_MON_EVENT_COUNT];
static portMUX_TYPE s_monitor_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_monitor_file_task = NULL;

const char *rt_monitor_file_path(void)
{
    return RT_STATS_FILE_PATH;
}

void rt_monitor_reset(void)
{
    portENTER_CRITICAL(&s_monitor_mux);
    for (int i = 0; i < RT_MON_METRIC_COUNT; i++) {
        rt_stats_reset(&s_monitor_metrics[i]);
    }
    memset(s_monitor_events, 0, sizeof(s_monitor_events));
    portEXIT_CRITICAL(&s_monitor_mux);
}

void rt_monitor_add_sample(rt_monitor_metric_t metric, int64_t value_us)
{
    if (metric < 0 || metric >= RT_MON_METRIC_COUNT || value_us < 0) {
        return;
    }

    portENTER_CRITICAL(&s_monitor_mux);
    rt_stats_add_sample(&s_monitor_metrics[metric], value_us);
    portEXIT_CRITICAL(&s_monitor_mux);
}

void rt_monitor_count_event(rt_monitor_event_t event)
{
    if (event < 0 || event >= RT_MON_EVENT_COUNT) {
        return;
    }

    portENTER_CRITICAL(&s_monitor_mux);
    s_monitor_events[event]++;
    portEXIT_CRITICAL(&s_monitor_mux);
}

static void rt_monitor_get_snapshot(rt_monitor_snapshot_t *snapshot)
{
    if (!snapshot) {
        return;
    }

    snapshot->updated_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_monitor_mux);
    memcpy(snapshot->metrics, s_monitor_metrics, sizeof(snapshot->metrics));
    memcpy(snapshot->events, s_monitor_events, sizeof(snapshot->events));
    portEXIT_CRITICAL(&s_monitor_mux);
}

static void rt_monitor_write_metric(FILE *f, const char *name, const rt_stats_t *s)
{
    if (s->count == 0) {
        fprintf(f, "%-24s n=0\n", name);
        return;
    }

    fprintf(f,
            "%-24s n=%lu min=%lld us max=%lld us mean=%.1f us stddev=%.1f us\n",
            name,
            (unsigned long)s->count,
            (long long)s->min,
            (long long)s->max,
            s->mean,
            rt_stats_stddev(s));
}

void rt_monitor_write_file_now(void)
{
    rt_monitor_snapshot_t snapshot = {0};
    rt_monitor_get_snapshot(&snapshot);

    FILE *f = fopen(RT_STATS_FILE_PATH, "w");
    if (!f) {
        ESP_LOGW(TAG, "Failed to open stats file: %s", RT_STATS_FILE_PATH);
        return;
    }

    fprintf(f, "RT STATS SNAPSHOT\n");
    fprintf(f, "file=%s\n", RT_STATS_FILE_PATH);
    fprintf(f, "updated_us=%lld\n", (long long)snapshot.updated_us);
    fprintf(f, "exec_dt_ms=%d\n", EXEC_DT_MS);
    fprintf(f, "write_period_ms=%d\n\n", RT_STATS_WRITE_PERIOD_MS);

    fprintf(f, "METRICS\n");
    for (int i = 0; i < RT_MON_METRIC_COUNT; i++) {
        rt_monitor_write_metric(f, s_metric_names[i], &snapshot.metrics[i]);
    }

    fprintf(f, "\nEVENTS\n");
    for (int i = 0; i < RT_MON_EVENT_COUNT; i++) {
        fprintf(f, "%-24s %lu\n", s_event_names[i], (unsigned long)snapshot.events[i]);
    }

    fclose(f);
}

static void rt_monitor_file_task_fn(void *arg)
{
    (void)arg;

    rt_monitor_write_file_now();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(RT_STATS_WRITE_PERIOD_MS));
        rt_monitor_write_file_now();
    }
}

void rt_monitor_start_file_task(void)
{
    if (s_monitor_file_task != NULL) {
        return;
    }

    rt_monitor_reset();

    BaseType_t res = xTaskCreatePinnedToCore(
        rt_monitor_file_task_fn,
        "rt_stats_file",
        4096,
        NULL,
        3,
        &s_monitor_file_task,
        CORE_COMM
    );

    if (res != pdPASS) {
        s_monitor_file_task = NULL;
        ESP_LOGE(TAG, "Failed to start stats file task");
    }
}