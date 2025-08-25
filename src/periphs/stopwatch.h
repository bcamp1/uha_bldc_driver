#include <sam.h>
#include <stdbool.h>

void stopwatch_init();
void stopwatch_start(int id);
uint32_t stopwatch_underlying_time();
uint32_t stopwatch_stop(int id, bool lap);
void stopwatch_print(int id, bool lap);
float ticks_to_time(uint32_t ticks);

