#pragma once

typedef enum {
    STATE_NONE,
    STATE_IDLE,
    STATE_STOPPED,
    STATE_STARTED,
    STATE_MEASURING,
} state_type_e;

typedef enum {
    EVENT_STOP,
    EVENT_START,
} event_type_e;

struct state_s;

struct state_ops_s {
    void (*stop_func) (struct state_s *obj);
    void (*start_func) (struct state_s *obj);
    void (*idle_func) (struct state_s *obj);
    void (*measure_func) (struct state_s *obj);
};

typedef struct state_s {
    state_type_e curr_state, prev_state;
    void *userdata;
    struct state_ops_s ops;
} state_t;

#ifdef __cplusplus
extern "C" {
#endif

void state_init(state_t *obj, void *arg);
void state_loop(state_t *obj);
void state_transit(state_t *obj, event_type_e event);

#ifdef __cplusplus
}
#endif
