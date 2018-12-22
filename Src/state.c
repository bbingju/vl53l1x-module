#include "state.h"
#include "debug.h"

void state_init(state_t *obj, void *arg)
{
    if (!obj)
        return;

    obj->userdata = arg;
}

void state_loop(state_t *obj)
{
    if (!obj)
        return;

    switch (obj->curr_state) {
    case STATE_NONE:
        obj->prev_state = obj->curr_state;
        obj->curr_state = STATE_IDLE;
        break;

    case STATE_IDLE:
        if (obj->ops.idle_func)
            obj->ops.idle_func(obj);
        break;
            
    case STATE_STOPPED:
        obj->prev_state = obj->curr_state;
        obj->curr_state = STATE_IDLE;
        break;

    case STATE_STARTED:
        obj->prev_state = obj->curr_state;
        obj->curr_state = STATE_MEASURING;
        break;

    case STATE_MEASURING:
        if (obj->ops.measure_func)
            obj->ops.measure_func(obj);
        break;

    default:
        break;
    }
}

void state_transit(state_t *obj, event_type_e event)
{
    if (!obj)
        return;

    obj->prev_state = obj->curr_state;

    switch (event) {
    case EVENT_START:
        obj->curr_state = STATE_STARTED;
        if (obj->ops.start_func)
            obj->ops.start_func(obj);
        DBG_LOG("STATE_STARTED\r\n");
        break;

    case EVENT_STOP:
        obj->curr_state = STATE_STOPPED;
        if (obj->ops.stop_func)
            obj->ops.stop_func(obj);
        DBG_LOG("STATE_STOPPED\r\n");
        break;

    default:
        break;
    }
}
