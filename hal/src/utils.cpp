#include "utils.h"
#include "pico/time.h"

namespace hal {

void panic() { asm volatile("BKPT"); }

} // namespace hal
