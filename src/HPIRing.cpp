/*
 * core1 -> core0 SPSC ring — implementation over pico-sdk queue_t.
 * ============================================================================
 * This is the ONLY translation unit that includes pico/util/queue.h, keeping
 * the FreeRTOS / pico `queue.h` basename clash safely contained: the rest of
 * core0 (Protocentral_HealthyPi_5.cpp) includes FreeRTOS's queue.h freely with no collision.
 *
 * The pico queue_t uses a hardware spinlock internally, so concurrent push
 * (core1) / pop (core0) is safe without any FreeRTOS object — which is exactly
 * why core1 can use it without ever calling a FreeRTOS API.
 */
#ifdef __FREERTOS   /* part of the FreeRTOS-only runtime; empty TU otherwise */

#include "hpi_internal.h"
#include <pico/util/queue.h>

static queue_t s_ring;

void hpi_ring_init(void)
{
  queue_init(&s_ring, sizeof(hpi_sample_t), HPI_RING_DEPTH);
}

bool hpi_ring_push(const hpi_sample_t *s)
{
  return queue_try_add(&s_ring, s);     /* false when full -> caller drops */
}

bool hpi_ring_pop(hpi_sample_t *s)
{
  return queue_try_remove(&s_ring, s);  /* false when empty                */
}

uint32_t hpi_ring_level(void)
{
  return (uint32_t)queue_get_level(&s_ring);
}

uint32_t hpi_ring_capacity(void)
{
  return HPI_RING_DEPTH;
}

#endif /* __FREERTOS */
