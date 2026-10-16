#ifndef GPIO_LL_H
#define GPIO_LL_H

#include <stdint.h>

/*
 * Host stub for ESP-IDF's hal/gpio_ll.h: the open-drain switch the DShot
 * port uses, as in IDF v5.5.5 (static inline there; functions here, so
 * a test can watch them). GPIO is soc/gpio_struct.h's register block.
 */
typedef struct {
	int unused;
} gpio_dev_t;

extern gpio_dev_t GPIO;

void gpio_ll_od_enable(gpio_dev_t *hw, uint32_t gpio_num);
void gpio_ll_od_disable(gpio_dev_t *hw, uint32_t gpio_num);

#endif
