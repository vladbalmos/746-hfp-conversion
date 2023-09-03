#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    BT_EV_INITIALIZE,
    BT_EV_INCOMING_CALL,
    BT_EV_OUTGOING_CALL,
    BT_EV_ANSWER_CALL,
    BT_EV_CALL_HANGUP,
    BT_EV_CALL_IN_PROGRESS,
    BT_EV_CALL_STATUS_IDLE,
    BT_EV_DIAL_NUMBER,
    BT_EV_PAIR_BTN_ACTIVITY,
    BT_EV_ENABLE_PAIRING,
    BT_EV_PAIRING_TIMEOUT,
    BT_EV_START_PAIRING,
    BT_EV_STOP_PAIRING,
    BT_EV_CONNECTED,
    BT_EV_DISCONNECTED,
} bt_event_type_t;

typedef struct {
    bt_event_type_t ev;
    uint32_t data_size;
    void *data;
} bt_msg_t;

void bt_init(QueueHandle_t outgoing_msg_queue);
bt_msg_t bt_create_msg(bt_event_type_t ev, void *data, size_t data_size);
void bt_task_send(bt_event_type_t ev, void *data, size_t data_size);
