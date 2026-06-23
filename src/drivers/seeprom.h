/*
 * seeprom.h
 *
 * Basic SmartEEPROM driver for the SAMD51J20A.
 *
 * The SAMD51 has no dedicated EEPROM, but the NVM controller can carve a small,
 * byte-addressable, automatically wear-levelled region out of main flash and map
 * it at SEEPROM_ADDR (0x44000000). This driver exposes that region as a flat
 * uint8_t array of SEEPROM_SIZE bytes -- read/write any index, it persists.
 *
 * ONE-TIME SETUP: SmartEEPROM only exists if the SEESBLK/SEEPSZ fuses in the NVM
 * user page are programmed. They are zero on a blank part, which leaves the
 * feature OFF. Either run seeprom_provision_fuses() once (then power-cycle), or
 * program the user row with your flashing tool. Until then seeprom_is_enabled()
 * returns false and reads/writes are no-ops. See seeprom.c for details.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Number of bytes this driver exposes. Must be <= the usable SmartEEPROM size
// selected by the fuses (the SEESBLK=1, SEEPSZ=0 config provisioned by this
// driver gives 512 usable bytes, so 100 fits comfortably).
#define SEEPROM_SIZE 100

// Prepare the controller for SmartEEPROM access. Selects unbuffered writes (each
// write is committed to flash immediately, so data survives an unexpected reset)
// and verifies the feature is enabled. Returns true if SmartEEPROM is active and
// large enough for SEEPROM_SIZE; false if the fuses still need provisioning.
bool seeprom_init(void);

// True once the SEESBLK/SEEPSZ fuses are programmed (SmartEEPROM is live).
bool seeprom_is_enabled(void);

// Total usable SmartEEPROM bytes as configured by the fuses (0 if disabled).
uint32_t seeprom_usable_size(void);

// Read one byte. index must be < SEEPROM_SIZE. Returns 0 if disabled.
uint8_t seeprom_read_byte(uint16_t index);

// Write one byte and commit it to flash. index must be < SEEPROM_SIZE. No-op if
// disabled. Blocks until the SmartEEPROM controller is idle.
void seeprom_write_byte(uint16_t index, uint8_t value);

// Best-effort, fire-and-forget write. Kicks off the byte write and returns
// immediately WITHOUT waiting for the flash commit -- safe to call from a
// time-critical path (e.g. the 500 Hz tick) because it never blocks on a program
// or erase. NOT guaranteed to persist: it returns false (and does nothing) if the
// controller is still busy from a previous write, and even on a true return a
// reset before the background commit finishes can lose the byte. Returns true if
// the write was started, false if it was skipped. Poll seeprom_busy() if you want
// to know when the commit has actually landed.
bool seeprom_write_byte_async(uint16_t index, uint8_t value);

// True while the controller is committing a write (the async write above is a
// no-op during this window). Reads/writes issued while busy will stall the bus.
bool seeprom_busy(void);

// Bulk read len bytes starting at index into dst.
void seeprom_read(uint16_t index, uint8_t *dst, uint16_t len);

// Bulk write len bytes from src starting at index.
void seeprom_write(uint16_t index, const uint8_t *src, uint16_t len);

// Read/write a 32-bit float as four consecutive bytes starting at index. The
// write blocks until committed (like seeprom_write_byte). index + 4 must be
// <= SEEPROM_SIZE.
float seeprom_read_float(uint16_t index);
void seeprom_write_float(uint16_t index, float value);

// Board-bringup convenience. If SmartEEPROM is not yet enabled, provision the
// fuses and issue a system reset so the controller latches them -- in that case
// this function DOES NOT RETURN (the board reboots and comes back enabled). If
// already enabled, it simply runs seeprom_init() and returns its result. Leaving
// this in the final firmware lets a fresh board self-configure on first power-up.
bool seeprom_init_or_provision(void);

// One-time provisioning: program the user-page fuses to enable SmartEEPROM with
// SEESBLK=1, SEEPSZ=0 (512 usable bytes, 16 KB of flash reserved for wear
// levelling). Preserves every other byte of the user page. Returns true if the
// fuses are now set. A SYSTEM RESET / POWER CYCLE is required afterward for the
// new size to take effect (the controller latches these fuses only at startup).
// Safe to call every boot: it does nothing if the fuses are already correct.
bool seeprom_provision_fuses(void);
