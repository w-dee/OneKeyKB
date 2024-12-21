/* This file is led_buttons.h */

#ifndef LED_BUTTONS_H_
#define LED_BUTTONS_H_

extern const struct device * gpio_dev;

void set_led(int on_or_off);
uint8_t get_dipsw();
void register_pairing_button_cb(void (*cb)(void));
void init_gpio_dev();

// キー押下検知のコールバック登録関数を追加
void register_key_press_cb(void (*cb)(int));
int get_key_pressed(void);

#endif /* LED_BUTTONS_H_ */


/* End of led_buttons.h */
