/*
 * faults.h
 *
 * DRV8323 nFAULT handling. Catches the active-low nFAULT edge on PA20 via the
 * EIC, parks the motor in high-Z, and (from the main loop) reads the two DRV
 * fault-status registers and deposits them into command_center. Latch-off: the
 * motor stays disabled after a fault until a fresh CMD_ENABLE re-arms it.
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>

/*
 * DRV8323 / DRV8304-family fault status register reference (read-only, 11-bit).
 * These are the two registers read by faults_poll() (DRV_REG_FAULT_STATUS_1 = 0x00,
 * DRV_REG_FAULT_STATUS_2 = 0x01 in gate_driver.h) and reported over RS485.
 * Source: DRV8304 datasheet SLVSE39B (Tables 12 & 13); layout matches DRV832x.
 *
 * --- Fault Status Register 1 (0x00) -------------------------------------------
 *   Bit  Field     Description
 *   10   FAULT     Logic OR of all fault status bits. Mirrors the nFAULT pin.
 *    9   VDS_OCP   VDS monitor overcurrent fault
 *    8   GDF       Gate drive fault
 *    7   UVLO      Undervoltage lockout fault
 *    6   OTSD      Overtemperature shutdown
 *    5   VDS_HA    VDS overcurrent on phase A high-side MOSFET
 *    4   VDS_LA    VDS overcurrent on phase A low-side MOSFET
 *    3   VDS_HB    VDS overcurrent on phase B high-side MOSFET
 *    2   VDS_LB    VDS overcurrent on phase B low-side MOSFET
 *    1   VDS_HC    VDS overcurrent on phase C high-side MOSFET
 *    0   VDS_LC    VDS overcurrent on phase C low-side MOSFET
 *
 * --- Fault Status Register 2 (0x01) -------------------------------------------
 *   Bit  Field     Description
 *   10   SA_OC     Overcurrent on phase A sense amplifier (sense variant)
 *    9   SB_OC     Overcurrent on phase B sense amplifier (sense variant)
 *    8   SC_OC     Overcurrent on phase C sense amplifier (sense variant)
 *    7   OTW       Overtemperature warning
 *    6   CPUV      Charge pump undervoltage fault
 *    5   VGS_HA    Gate drive fault on phase A high-side MOSFET
 *    4   VGS_LA    Gate drive fault on phase A low-side MOSFET
 *    3   VGS_HB    Gate drive fault on phase B high-side MOSFET
 *    2   VGS_LB    Gate drive fault on phase B low-side MOSFET
 *    1   VGS_HC    Gate drive fault on phase C high-side MOSFET
 *    0   VGS_LC    Gate drive fault on phase C low-side MOSFET
 */

// Fault Status Register 1 (0x00) bit masks
#define DRV_FS1_FAULT    (1u << 10)  // OR of all faults; mirrors nFAULT pin
#define DRV_FS1_VDS_OCP  (1u << 9)   // VDS overcurrent
#define DRV_FS1_GDF      (1u << 8)   // gate drive fault
#define DRV_FS1_UVLO     (1u << 7)   // undervoltage lockout
#define DRV_FS1_OTSD     (1u << 6)   // overtemperature shutdown
#define DRV_FS1_VDS_HA   (1u << 5)
#define DRV_FS1_VDS_LA   (1u << 4)
#define DRV_FS1_VDS_HB   (1u << 3)
#define DRV_FS1_VDS_LB   (1u << 2)
#define DRV_FS1_VDS_HC   (1u << 1)
#define DRV_FS1_VDS_LC   (1u << 0)

// Fault Status Register 2 (0x01) bit masks
#define DRV_FS2_SA_OC    (1u << 10)  // sense amp A overcurrent
#define DRV_FS2_SB_OC    (1u << 9)   // sense amp B overcurrent
#define DRV_FS2_SC_OC    (1u << 8)   // sense amp C overcurrent
#define DRV_FS2_OTW      (1u << 7)   // overtemperature warning
#define DRV_FS2_CPUV     (1u << 6)   // charge pump undervoltage
#define DRV_FS2_VGS_HA   (1u << 5)
#define DRV_FS2_VGS_LA   (1u << 4)
#define DRV_FS2_VGS_HB   (1u << 3)
#define DRV_FS2_VGS_LB   (1u << 2)
#define DRV_FS2_VGS_HC   (1u << 1)
#define DRV_FS2_VGS_LC   (1u << 0)

// Configure the EIC and the nFAULT line (PA20, falling edge). Call after
// motor_init()/gate_driver_init() so the EIC pin mux wins over the plain-input
// configuration done there.
void faults_init(void);

// Poll firmware/system-level fault conditions (currently encoder SPI health)
// and update the command_center meta-fault register. Call from the main loop.
void faults_check_health(void);

// Decode and print the meta-fault register over UART.
void faults_print_meta(void);

// Report the gate-driver SPI result from motor_enable(): sets/clears the
// META_FAULT_GATE_SPI meta-fault (and logs on failure).
void faults_report_gate_spi(bool ok);

// Mask nFAULT detection across the gate-driver power-up sequence, which briefly
// asserts nFAULT while the charge pump settles. Pair with faults_resume().
void faults_suspend(void);

// Re-arm nFAULT detection: discard any edge latched while suspended (the
// power-up transient), then check the live pin so a still-asserted fault is not
// missed (the EIC is edge-triggered).
void faults_resume(void);

// Call from the main loop. If nFAULT has tripped since the last call, performs
// the (blocking) SERCOM3-exclusive read of FAULT_STATUS_1/2 and deposits them
// into command_center. Returns true exactly once per fault, when handled.
bool faults_poll(void);

// True while a fault is latched. The enable reconcile uses this to keep the
// motor disabled.
bool faults_is_latched(void);

// Clear the latch so the motor can be re-enabled (the subsequent gate-driver
// enable clears the DRV fault via clr_flt).
void faults_clear(void);
