/*
 * seeprom.c
 *
 * SmartEEPROM driver for the SAMD51J20A. See seeprom.h.
 *
 * How SmartEEPROM works, briefly: the NVM controller reserves two physical flash
 * "sectors" (SEESBLK 8 KB blocks each) and presents them as a small virtual
 * EEPROM mapped at SEEPROM_ADDR. Writing a byte there transparently triggers a
 * read-modify-write of a flash page, and the controller rotates which physical
 * page backs each virtual address so no single cell is worn out. Flash endures
 * ~10k erase cycles per cell; the wear levelling multiplies that for the small
 * virtual region many times over.
 *
 * We use UNBUFFERED write mode: every byte write is flushed to flash on the spot.
 * That costs more flash traffic per write but guarantees the byte is persisted
 * before the call returns -- the right trade-off for latched fault state / config
 * that is written occasionally rather than in a tight loop.
 */

#include "seeprom.h"
#include "../periphs/gpio.h"
#include "../periphs/delay.h"
#include "../board.h"
#include <samd51j20a.h>
#include <string.h>

// LED used to signal provisioning state during board bringup. PIN_DEBUG1 is the
// board's primary debug indicator; change here if you wire the LED elsewhere.
#define SEEPROM_LED_PIN PIN_DEBUG1

// Virtual SmartEEPROM region, exposed as a plain byte array.
static volatile uint8_t *const SEE = (volatile uint8_t *)SEEPROM_ADDR;

// Usable bytes of the SBLK=1, PSZ=0 configuration that seeprom_provision_fuses()
// programs. The datasheet SmartEEPROM size table defines this combination as 512
// usable bytes (it reserves 16 KB of flash for two wear-levelling sectors). Other
// fuse combinations yield other sizes; update this if you reprovision differently.
#define SEEPROM_PROVISIONED_BYTES 512u

// Spin until the controller has finished any pending NVM/SmartEEPROM operation.
static void see_wait_ready(void) {
	while (!NVMCTRL->STATUS.bit.READY) { }
	while (NVMCTRL->SEESTAT.bit.BUSY) { }
}

// Issue an NVMCTRL command with the unlock key and wait for completion.
static void nvm_command(uint16_t cmd) {
	while (!NVMCTRL->STATUS.bit.READY) { }
	NVMCTRL->CTRLB.reg = cmd | NVMCTRL_CTRLB_CMDEX_KEY;
	while (!NVMCTRL->STATUS.bit.READY) { }
}

// SmartEEPROM fuse field selection (SEESBLK=1, SEEPSZ=0 -> 512 usable bytes).
#define SEE_FUSE_MASK (NVMCTRL_FUSES_SEESBLK_Msk | NVMCTRL_FUSES_SEEPSZ_Msk)
#define SEE_FUSE_CFG  (NVMCTRL_FUSES_SEESBLK(1) | NVMCTRL_FUSES_SEEPSZ(0))

// True if the user-page fuses are already programmed for SmartEEPROM. Note this
// reflects what is written in flash, NOT whether the controller has latched it --
// the latch only happens at reset, so this can be true while is_enabled() is false
// (exactly the reset-loop condition we guard against in init_or_provision).
static bool see_fuses_programmed(void) {
	volatile uint32_t *user = (volatile uint32_t *)NVMCTRL_USER;
	return (user[1] & SEE_FUSE_MASK) == SEE_FUSE_CFG;
}

bool seeprom_is_enabled(void) {
	// SEESBLK is loaded from the user-page fuses at startup; non-zero => active.
	return NVMCTRL->SEESTAT.bit.SBLK != 0;
}

// --- Bringup LED signalling --------------------------------------------------
// delay() spins ~10 NOPs per count; these magnitudes give visibly distinct rates
// at the 120 MHz core clock (exact period is not important, the pattern is).
static void see_led_init(void) {
	gpio_init_pin(SEEPROM_LED_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
}

// "About to write the fuses" -- three slow, deliberate blinks before we reset.
static void see_led_provisioning(void) {
	see_led_init();
	for (int i = 0; i < 3; i++) {
		gpio_set_pin(SEEPROM_LED_PIN);
		delay(0x80000);
		gpio_clear_pin(SEEPROM_LED_PIN);
		delay(0x80000);
	}
}

// Unrecoverable: the fuses are written but the controller still will not enable
// SmartEEPROM (would otherwise reset-loop forever), or the fuse write failed
// verification. Halt here with a fast continuous blink so bringup hangs visibly
// instead of silently rebooting. Never returns.
static void see_led_fault_forever(void) {
	see_led_init();
	while (1) {
		gpio_toggle_pin(SEEPROM_LED_PIN);
		delay(0x10000);
	}
}

uint32_t seeprom_usable_size(void) {
	// The exact size depends on the SBLK/PSZ fuse combination (datasheet
	// SmartEEPROM size table). We only know it precisely for the configuration we
	// provision, so report that; SBLK!=0 guarantees at least this much.
	return seeprom_is_enabled() ? SEEPROM_PROVISIONED_BYTES : 0;
}

bool seeprom_init(void) {
	// Unbuffered: a flash write is issued after every byte write (immediate
	// persistence). APRDIS left at 0 so automatic page reallocation stays on.
	NVMCTRL->SEECFG.reg = NVMCTRL_SEECFG_WMODE_UNBUFFERED;
	see_wait_ready();
	return seeprom_is_enabled() && seeprom_usable_size() >= SEEPROM_SIZE;
}

bool seeprom_init_or_provision(void) {
	if (seeprom_is_enabled()) {
		return seeprom_init();
	}

	// Not enabled. If the fuses are ALREADY programmed, then a previous boot
	// provisioned and reset, yet the controller still did not enable SmartEEPROM.
	// Resetting again would just repeat forever, so stop here with the fault blink
	// rather than silently reset-looping. (Catches an unsupported part or a bad
	// fuse value.)
	if (see_fuses_programmed()) {
		see_led_fault_forever();   // never returns
	}

	// Fresh board: announce on the LED, write the fuses, then reset so they take
	// effect (the size is only latched at startup, so we cannot continue this boot).
	see_led_provisioning();
	if (seeprom_provision_fuses()) {
		NVIC_SystemReset();        // does not return; comes back enabled
	}

	// Fuse write failed verification -- halt visibly instead of returning to a
	// caller that assumes storage works.
	see_led_fault_forever();       // never returns
	return false;                  // unreachable, keeps the compiler happy
}

uint8_t seeprom_read_byte(uint16_t index) {
	if (index >= SEEPROM_SIZE || !seeprom_is_enabled()) {
		return 0;
	}
	see_wait_ready();
	return SEE[index];
}

void seeprom_write_byte(uint16_t index, uint8_t value) {
	if (index >= SEEPROM_SIZE || !seeprom_is_enabled()) {
		return;
	}
	see_wait_ready();
	SEE[index] = value;     // triggers the SmartEEPROM read-modify-write
	see_wait_ready();       // block until it has been committed to flash
}

bool seeprom_busy(void) {
	return NVMCTRL->SEESTAT.bit.BUSY;
}

bool seeprom_write_byte_async(uint16_t index, uint8_t value) {
	if (index >= SEEPROM_SIZE || !seeprom_is_enabled()) {
		return false;
	}
	// If the controller is mid-commit, accessing the SEEPROM window would stall
	// the bus -- exactly what we're trying to avoid. Drop the write instead.
	if (NVMCTRL->SEESTAT.bit.BUSY) {
		return false;
	}
	// Issue the store and return. The store posts quickly; the controller does the
	// read-modify-write in the background (SEESTAT.BUSY stays high until done). We
	// deliberately do NOT wait, so persistence is not guaranteed on this return.
	SEE[index] = value;
	return true;
}

void seeprom_read(uint16_t index, uint8_t *dst, uint16_t len) {
	for (uint16_t i = 0; i < len && (index + i) < SEEPROM_SIZE; i++) {
		dst[i] = seeprom_read_byte(index + i);
	}
}

float seeprom_read_float(uint16_t index) {
	float value = 0.0f;
	uint8_t bytes[4];
	seeprom_read(index, bytes, sizeof bytes);
	memcpy(&value, bytes, sizeof value);   // byte-copy avoids alignment/aliasing UB
	return value;
}

void seeprom_write_float(uint16_t index, float value) {
	uint8_t bytes[4];
	memcpy(bytes, &value, sizeof value);
	seeprom_write(index, bytes, sizeof bytes);
}

void seeprom_write(uint16_t index, const uint8_t *src, uint16_t len) {
	for (uint16_t i = 0; i < len && (index + i) < SEEPROM_SIZE; i++) {
		seeprom_write_byte(index + i, src[i]);
	}
}

// ---------------------------------------------------------------------------
// One-time fuse provisioning (writes the NVM user page).
//
// The user page is a single 512-byte page at NVMCTRL_USER. It holds BOD33, WDT,
// bootloader-size, NVM lock and the SmartEEPROM (SEESBLK/SEEPSZ) configuration.
// We must preserve every byte except the SmartEEPROM nibbles, so we read the
// whole page, patch word[1], erase the page, then write it back as quad words.
// ---------------------------------------------------------------------------
bool seeprom_provision_fuses(void) {
	volatile uint32_t *user = (volatile uint32_t *)NVMCTRL_USER;

	// Already provisioned? Nothing to do (avoids a needless flash erase/write).
	if (see_fuses_programmed()) {
		return true;
	}

	// Snapshot the entire user page (512 bytes = 128 words) into RAM.
	uint32_t page[128];
	for (uint32_t i = 0; i < 128; i++) {
		page[i] = user[i];
	}
	page[1] = (page[1] & ~SEE_FUSE_MASK) | SEE_FUSE_CFG;

	// Manual write mode so loading the page buffer does not auto-trigger writes.
	NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_MAN_Val;

	// Clear the page buffer, then erase the user page (EP is valid on USER/AUX).
	nvm_command(NVMCTRL_CTRLB_CMD_PBC);
	NVMCTRL->ADDR.reg = NVMCTRL_USER;
	nvm_command(NVMCTRL_CTRLB_CMD_EP);

	// Write the page back one 128-bit quad word (4 words) at a time.
	for (uint32_t qw = 0; qw < 128; qw += 4) {
		for (uint32_t w = 0; w < 4; w++) {
			user[qw + w] = page[qw + w];   // loads page buffer in manual mode
		}
		NVMCTRL->ADDR.reg = NVMCTRL_USER + qw * 4;
		nvm_command(NVMCTRL_CTRLB_CMD_WQW);
	}

	// Verify, but remember: SEESTAT will not reflect this until a reset/power
	// cycle reloads the fuses, so do not call seeprom_is_enabled() to confirm.
	return see_fuses_programmed();
}
