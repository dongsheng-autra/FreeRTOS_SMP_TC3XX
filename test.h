/*
 * Copyright (c) 2023 Autra.Tech. All rights reserved.
 *
 * THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
 * ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
 * IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
 * PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
 */

#ifndef _TEST_H_
#define _TEST_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

/* malloc is limited, so define max packet queue size */
#define TEST_SEMBUF_SIZE      255
#define TEST_SEMBUF_INITVAL   0xAA

struct semtest_type {
    uint8_t buf[TEST_SEMBUF_SIZE];
    uint8_t crc;
};

extern struct semtest_type g_semtest;
extern xSemaphoreHandle  g_mutex;

void test_init(struct semtest_type *test);
int  test_semaphore(struct semtest_type *test, uint8_t *crc_cal);

#endif /* _TEST_H_ */
