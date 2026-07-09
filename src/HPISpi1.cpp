/*
 * SPI1 bus arbitration — see HPISpi1.h.
 */
#ifdef __FREERTOS   /* part of the FreeRTOS-only runtime; empty TU otherwise */

#include "HPISpi1.h"
#include <FreeRTOS.h>
#include <semphr.h>

static SemaphoreHandle_t s_mtx;
static StaticSemaphore_t s_mtx_buf;

void hpi_spi1_bus_init(void)
{
  if (!s_mtx) s_mtx = xSemaphoreCreateMutexStatic(&s_mtx_buf);
}

void hpi_spi1_lock(void)
{
  if (s_mtx) xSemaphoreTake(s_mtx, portMAX_DELAY);
}

void hpi_spi1_unlock(void)
{
  if (s_mtx) xSemaphoreGive(s_mtx);
}

#endif /* __FREERTOS */
