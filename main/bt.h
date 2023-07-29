#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    BT_EV_INITIALIZE
} bt_event_type_t;

typedef struct {
    bt_event_type_t ev;
    uint32_t data_size;
    void *data;
} bt_msg_t;

void bt_init();
void bt_task_send_msg(bt_msg_t *msg);
void bt_task_send(bt_event_type_t ev, void *data, uint32_t data_size);
