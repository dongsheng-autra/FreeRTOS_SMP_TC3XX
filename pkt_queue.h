/*
 * Copyright (c) 2023 Autra.Tech. All rights reserved.
 *
 * THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
 * ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
 * IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
 * PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
 */

#ifndef _LIB_PKT_QUEUE_H_
#define _LIB_PKT_QUEUE_H_

#include <stdbool.h>

/* malloc is limited, so define max packet queue size */
#define MAX_PKT_QUEUE_SIZE      32

struct pkt_queue {
    void *buf[MAX_PKT_QUEUE_SIZE];
    int mask;
    int head;
    int tail;
    int count;
};

int pkt_queue_init(struct pkt_queue *queue, int size);

int pkt_queue_put(struct pkt_queue *queue, void *buf);

void *pkt_queue_get(struct pkt_queue *queue);

bool pkt_queue_check(struct pkt_queue *queue);

#endif /* _LIB_PKT_QUEUE_H_ */
