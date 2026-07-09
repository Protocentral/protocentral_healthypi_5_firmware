/*
 * HealthyPi 5 NEXT (Arduino) — SPI1 bus arbitration.
 * ============================================================================
 * SPI1 is a shared bus (separate chip-selects, one peripheral). Today the SD
 * card is its only user; a future on-panel LCD will share it too. A FreeRTOS
 * mutex serialises access so two devices can't interleave a transfer; each side
 * re-applies its own baud/format inside the locked region. Mirrors the NEXT
 * spi1_bus.{c,h}. The mutex is created in HealthyPi5.begin(); lock/unlock are
 * no-ops until then (and harmless when only one device uses the bus).
 */
#ifndef HPI_SPI1_H
#define HPI_SPI1_H

void hpi_spi1_bus_init(void);   /* create the mutex; called once from begin() */
void hpi_spi1_lock(void);
void hpi_spi1_unlock(void);

#endif /* HPI_SPI1_H */
