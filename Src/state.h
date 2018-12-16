#pragma once

typedef enum {
    STATE_NONE,
    STATE_IDLE,
    STATE_STOPPED,
    STATE_STARTED,
    STATE_MEASURING,
    STATE_CONFIG,
} state_type_e;

typedef enum {
    EVENT_STOP,
    EVENT_START,
    EVENT_CONFIG,
} event_type_e;

typedef struct state_s {
    state_type_e curr_state, prev_state;

    void *userdata;

    struct {
        void (*stop_func) (struct state_s *obj);
        void (*start_func) (struct state_s *obj);
        void (*idle_func) (struct state_s *obj);
        void (*measure_func) (struct state_s *obj);
        void (*config_func) (struct state_s *obj);
    } op;
} state_t;

#ifdef __cplusplus
#endif

void state_init(state_t *obj, void *arg);
void state_loop(state_t *obj);
void state_transit(state_t *obj, event_type_e event);

#ifdef __cplusplus
#endif
