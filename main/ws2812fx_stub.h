#pragma once
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

void ws2812fx_init(void); // initialize effect engine
void ws2812fx_start(void); // start background task

#ifdef __cplusplus
}
#endif

