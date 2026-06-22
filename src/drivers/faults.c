/*
 * faults.c
 *
 * DRV8323 nFAULT handling. See faults.h.
 */

#include "faults.h"
#include "gate_driver.h"
#include "foc_loop.h"
#include "command_center.h"
#include "motor.h"
#include "../periphs/eic.h"
#include "../periphs/spi_async.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "../board.h"
#include "sam.h"

static volatile bool fault_pending = false;  // set by ISR, consumed by poll
static volatile bool fault_latched = false;  // motor stays off while true

// Decode and print the two fault-status registers over UART. Called from
// faults_poll() (main context), only on an actual fault, so the blocking UART
// output never runs in the ISR or the hot path.

// Decode and print the firmware/system-level meta-fault register over UART.
void faults_print_meta(void) {
    uint8_t m = command_center_get_meta_fault();
    uart_print("META_FAULT: 0b");
    uart_println_int_base(m, 2);
    if (m & META_FAULT_GATE_SPI)    uart_println("  GATE_SPI: gate-driver SPI failure");
    if (m & META_FAULT_ENCODER_SPI) uart_println("  ENCODER_SPI: encoder SPI failure");
    if (m & META_FAULT_DRV)         uart_println("  DRV: DRV hardware fault latched");
}

static void print_faults(uint16_t r1, uint16_t r2) {
    uart_println("!! DRV8323 FAULT !!");

    uart_print("FAULT_STATUS_1: 0b");
    uart_println_int_base(r1, 2);
    if (r1 & DRV_FS1_VDS_OCP) uart_println("  VDS_OCP: VDS overcurrent");
    if (r1 & DRV_FS1_GDF)     uart_println("  GDF: gate drive fault");
    if (r1 & DRV_FS1_UVLO)    uart_println("  UVLO: undervoltage lockout");
    if (r1 & DRV_FS1_OTSD)    uart_println("  OTSD: overtemperature shutdown");
    if (r1 & DRV_FS1_VDS_HA)  uart_println("  VDS_HA: A high-side overcurrent");
    if (r1 & DRV_FS1_VDS_LA)  uart_println("  VDS_LA: A low-side overcurrent");
    if (r1 & DRV_FS1_VDS_HB)  uart_println("  VDS_HB: B high-side overcurrent");
    if (r1 & DRV_FS1_VDS_LB)  uart_println("  VDS_LB: B low-side overcurrent");
    if (r1 & DRV_FS1_VDS_HC)  uart_println("  VDS_HC: C high-side overcurrent");
    if (r1 & DRV_FS1_VDS_LC)  uart_println("  VDS_LC: C low-side overcurrent");

    uart_print("FAULT_STATUS_2: 0b");
    uart_println_int_base(r2, 2);
    if (r2 & DRV_FS2_SA_OC)   uart_println("  SA_OC: phase A sense overcurrent");
    if (r2 & DRV_FS2_SB_OC)   uart_println("  SB_OC: phase B sense overcurrent");
    if (r2 & DRV_FS2_SC_OC)   uart_println("  SC_OC: phase C sense overcurrent");
    if (r2 & DRV_FS2_OTW)     uart_println("  OTW: overtemperature warning");
    if (r2 & DRV_FS2_CPUV)    uart_println("  CPUV: charge pump undervoltage");
    if (r2 & DRV_FS2_VGS_HA)  uart_println("  VGS_HA: A high-side gate fault");
    if (r2 & DRV_FS2_VGS_LA)  uart_println("  VGS_LA: A low-side gate fault");
    if (r2 & DRV_FS2_VGS_HB)  uart_println("  VGS_HB: B high-side gate fault");
    if (r2 & DRV_FS2_VGS_LB)  uart_println("  VGS_LB: B low-side gate fault");
    if (r2 & DRV_FS2_VGS_HC)  uart_println("  VGS_HC: C high-side gate fault");
    if (r2 & DRV_FS2_VGS_LC)  uart_println("  VGS_LC: C low-side gate fault");

    faults_print_meta();   // include the meta-fault register in the dump
}

// EIC callback (ISR context). Keep it short and SPI-free: park the motor in
// high-Z immediately, then defer the gate-driver register read to faults_poll()
// in the main loop (gate-driver SPI must not run from an ISR).
static void on_nfault(void) {
    foc_loop_stop();          // atomic: deschedule FOC timer + high-Z, no SPI
    fault_pending = true;
}

void faults_init(void) {
    command_center_clear_meta_fault(0xFF);   // start with a clean meta-fault register
    eic_init();
    NVIC_SetPriority(EIC_4_IRQn, PRIO_FAULT);
    eic_init_pin(PIN_GATE_NFAULT, PIN_GATE_NFAULT_EIC_INDEX, EIC_MODE_FALL, on_nfault);
}

void faults_suspend(void) {
    NVIC_DisableIRQ(EIC_4_IRQn);
}

void faults_resume(void) {
    // Discard the power-up edge latched while suspended, then re-arm.
    EIC->INTFLAG.reg = (1u << PIN_GATE_NFAULT_EIC_INDEX);
    NVIC_ClearPendingIRQ(EIC_4_IRQn);
    fault_pending = false;
    NVIC_EnableIRQ(EIC_4_IRQn);

    // EIC is edge-triggered: a fault still asserted now produced no new edge.
    // nFAULT is active-low, so a low level here means a real fault persists.
    if (!gpio_get_pin(PIN_GATE_NFAULT)) {
        fault_pending = true;
    }
}

bool faults_poll(void) {
    if (!fault_pending) return false;
    fault_pending = false;
    fault_latched = true;

    // SERCOM3 is shared with the encoder SPI. Mask the 1700 Hz encoder timer
    // (TC1 == TIMER_ID_SPI_ENCODER) and drain any in-flight encoder read before
    // driving the gate-driver bus, then restore it. Mirrors motor_enable().
    NVIC_DisableIRQ(TC1_IRQn);
    while (spi_async_is_busy()) { }
    uint16_t r1 = gate_driver_read_reg(DRV_REG_FAULT_STATUS_1);
    uint16_t r2 = gate_driver_read_reg(DRV_REG_FAULT_STATUS_2);
    NVIC_EnableIRQ(TC1_IRQn);

    command_center_set_fault_registers(r1, r2);
    command_center_set_meta_fault(META_FAULT_DRV);
    print_faults(r1, r2);
    return true;
}

bool faults_is_latched(void) {
    return fault_latched;
}

void faults_clear(void) {
    // Re-arm: drop the latch and all meta-faults. Any still-present condition
    // (e.g. encoder still dead) will be re-detected and re-latch immediately.
    fault_latched = false;
    command_center_clear_meta_fault(0xFF);
}

// Number of consecutive bad encoder reads before flagging the SPI as failed.
// At 1700 Hz this is ~60 ms of sustained bad data.
#define ENCODER_FAIL_THRESHOLD 100

void faults_report_gate_spi(bool ok) {
    if (ok) {
        command_center_clear_meta_fault(META_FAULT_GATE_SPI);
    } else {
        command_center_set_meta_fault(META_FAULT_GATE_SPI);
        fault_latched = true;   // can't configure the gate driver -> keep motor off
        uart_println("!! META FAULT: gate-driver SPI !!");
        faults_print_meta();
    }
}

void faults_check_health(void) {
    // The capstan runs open-loop and has no actively-used encoder, so a dead
    // encoder SPI is expected there and must not be treated as a fault.
    if (motor_get_identity() == MOTOR_IDENT_CAPSTAN) {
        return;
    }

    static bool prev_fail = false;
    bool fail = spi_async_get_fail_count() >= ENCODER_FAIL_THRESHOLD;

    if (fail) {
        command_center_set_meta_fault(META_FAULT_ENCODER_SPI);
        fault_latched = true;   // no valid position -> keep motor off until re-enable
    } else {
        command_center_clear_meta_fault(META_FAULT_ENCODER_SPI);
    }

    // Print only on transitions so the main loop doesn't spam UART.
    if (fail && !prev_fail) {
        uart_println("!! META FAULT: encoder SPI !!");
        faults_print_meta();
    } else if (!fail && prev_fail) {
        uart_println("META FAULT cleared: encoder SPI");
    }
    prev_fail = fail;
}
