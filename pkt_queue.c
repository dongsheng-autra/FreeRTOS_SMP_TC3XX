/*
 * Copyright (c) 2023 Autra.Tech. All rights reserved.
 *
 * THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
 * ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
 * IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
 * PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
 */

#include <errno.h>
#include <stddef.h>

#include "pkt_queue.h"

#define IS_POWER_OF_TWO(x) (((x) != 0U) && (((x) & ((x) - 1U)) == 0U))

int pkt_queue_init(struct pkt_queue *queue, int size)
{
    if ((size > MAX_PKT_QUEUE_SIZE) || (!IS_POWER_OF_TWO(size))) {
        return -EINVAL;
    }

    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->mask = size - 1;

    return 0;
}

int pkt_queue_put(struct pkt_queue *queue, void *buf)
{
    if (queue->count > queue->mask) {
        return -ENOMEM;
    }

    queue->buf[queue->head] = buf;
    queue->head = ((queue->head + 1) & queue->mask);
    queue->count++;

    return 0;
}

void *pkt_queue_get(struct pkt_queue *queue)
{
    void *buf;

    if (queue->count <= 0) {
        return NULL;
    }

    buf = queue->buf[queue->tail];
    queue->tail = ((queue->tail + 1) & queue->mask);
    queue->count--;

    return buf;
}

bool pkt_queue_check(struct pkt_queue *queue)
{
    if (queue->count > 0) {
        return true;
    }
    return false;
}
