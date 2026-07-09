/*
 * HealthyPi 5 NEXT — SD Card Datalogger (Tutorial 11)
 * ============================================================================
 * Records the four raw waveform channels — ECG, respiration (BioZ), and the
 * PPG RED + IR — to a FAT-formatted microSD card, using the dual-core NEXT
 * runtime so logging never costs a sample. core1 acquires losslessly at 128 SPS
 * into a lock-free ring; the SD card is just another drop-newest sink on core0,
 * so a slow card only drops its OWN records (counted) and can never stall
 * acquisition.
 *
 *   Board   : "Raspberry Pi Pico" (arduino-pico) — Tools > os: "FreeRTOS SMP"
 *   Card    : microSD in the on-board slot, FAT16/FAT32 formatted, on SPI1
 *   Output  : USB Serial (Serial Monitor @115200) — human-readable log status
 *             UART0 (Serial1) also carries the library's 1 Hz HPI_INSTR line
 *
 * On-disk format (byte-identical to the production firmware):
 *   /REC0.BIN, /REC1.BIN, ...  =  32-byte "$HP5" header
 *                                 + 16-byte records: int32 LE {ecg, bioz, red, ir}
 *   Decode on a host with tools/host/decode_rec.py from healthypi5_next_rp2040.
 *
 * This sketch auto-starts one recording at boot and logs continuously. To drive
 * recording from a host instead (REC_START / REC_STOP over the command plane),
 * add HealthyPi5.enableCommands() and drop the recordStart() call below.
 *
 * Because the library defines setup1()/loop1(), this sketch must NOT.
 */
#include <FreeRTOS.h>        // selects arduino-pico's FreeRTOS-SMP variant
#include <Protocentral_HealthyPi_5.h>
#include <HPISd.h>           // hpi_sd_card_present() / _record_secs() / _drops()

void setup()
{
  Serial.begin(115200);

  HealthyPi5.recordSD();     // register the SD sink (A8): /REC*.BIN on SPI1
  HealthyPi5.begin();        // bring up the dual-core runtime (ring + broker + WDT)
  HealthyPi5.recordStart();  // open a new REC file (the card is mounted lazily
                             // on the SD task, so a missing card won't stall boot)
}

void loop()
{
  // Print a friendly 1 Hz status line to the USB Serial Monitor. All the real
  // work (acquisition + SD writes) happens in the library's pinned core0 tasks;
  // this loop is just the lowest-priority observer.
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 1000) {
    last = now;

    if (!hpi_sd_card_present()) {
      Serial.println("SD: no card detected — insert a FAT-formatted microSD and reset.");
    } else {
      Serial.printf("SD: recording=%s  file=%lus  dropped=%lu records\r\n",
                    HealthyPi5.recording() ? "yes" : "no",
                    (unsigned long)hpi_sd_record_secs(),
                    (unsigned long)hpi_sd_drops());
    }
  }

  // Example: stop after 60 s of logging, then start a fresh file. Uncomment to
  // roll files periodically instead of one continuous recording.
  //   if (hpi_sd_record_secs() >= 60) { HealthyPi5.recordStop(); HealthyPi5.recordStart(); }

  delay(50);
}
