
// One-shot HW setup (gate-driver idrive). Does NOT start the FOC timer —
// callers must invoke foc_loop_start() to begin closed-loop operation.
void foc_loop_init();

// Idempotent. Schedules the 1004 Hz FOC timer (or capstan open-loop variant).
void foc_loop_start(void);

// Idempotent. Atomically descheduled FOC timer and parks the gate driver in
// high-Z so no stale PWM is left driving the motor.
void foc_loop_stop(void);

float foc_loop_get_speed();
void foc_loop_set_torque(float torque);

