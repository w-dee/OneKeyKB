/* This file is led_buttons.c, I/O handler for 1-key simple BLE keyboard */

#include "includes.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "led_buttons.h"

/* デバイスツリーからノードを取得 */
#define PAIRING_BUTTON_PIN DT_GPIO_PIN(DT_NODELABEL(pairing_button), gpios)
#define LED_PIN DT_GPIO_PIN(DT_NODELABEL(led0), gpios)

/* GPIOデバイス */
const struct device *gpio_dev;

/* ペアリングボタンのコールバック */
static struct gpio_callback pairing_cb;
/* キーのコールバック */
static struct gpio_callback key_press_cb;

/* Key Button のピン番号 */
#define KEY_BUTTON_PIN DT_GPIO_PIN(DT_NODELABEL(key_button), gpios)

/* DIPSWのノード */
#define DIPSW_NODE DT_PATH(zephyr_user)

/* 配列長 */
#define DIPSW_LEN DT_PROP_LEN(DIPSW_NODE, dipsw_gpios)

/* マクロで静的配列の各要素を展開 */
#define GPIO_PIN_INIT(node_id, prop, idx)                                              \
    { .port = DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(node_id, prop, idx)),                  \
      .pin = DT_GPIO_PIN_BY_IDX(node_id, prop, idx),                                     \
      .dt_flags = DT_GPIO_FLAGS_BY_IDX(node_id, prop, idx) },

struct gpio_dt_spec dipsw_gpios[DIPSW_LEN] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), dipsw_gpios, GPIO_PIN_INIT)
};

/* 必要な変数や定数 */
// ポーリング用のタイマーを宣言
static void key_polling_timer_callback(struct k_timer *key_timer_id);
K_TIMER_DEFINE(key_timer, key_polling_timer_callback, NULL);

static void pairing_btn_polling_timer_callback(struct k_timer *pairing_btn_timer_id);
K_TIMER_DEFINE(pairing_btn_timer, pairing_btn_polling_timer_callback, NULL);

static bool is_pairing_button_checking = false; // ペアリングボタンのチャタリングチェック中かどうか
static bool is_key_timer_running = false; // ポーリング用タイマーが動作しているかどうか
static volatile bool key_pressed = false;
static uint8_t key_history = 0x00;

/* コールバック関数ポインタ */
void (*key_event_cb)(int) = NULL;
void (*pairing_button_cb)(void) = NULL;

#define KEY_POLLING_INTERVAL_MS 10
#define PAIRING_BTN_POLLING_INTERVAL_MS 30

/* キータイマーコールバック関数 */
static void key_polling_timer_callback(struct k_timer *timer_id)
{
    // Key history をシフト
    key_history <<= 1;

    // GPIO ピンの状態を読み取る
    if (gpio_pin_get(gpio_dev, KEY_BUTTON_PIN)) {
        key_history |= 0x01;
    }

    // キー押下判定
    if (!key_pressed && (key_history & 0x07) == 0x07) {
        key_pressed = true;
        if (key_event_cb) {
            key_event_cb(1); // キーが押されたことを示す
        }
        k_timer_stop(&key_timer);
        is_key_timer_running = false;
    }
    // キー解放判定
    else if (key_pressed && (key_history & 0x07) == 0x00) {
        key_pressed = false;
        if (key_event_cb) {
            key_event_cb(0); // キーが離されたことを示す
        }
        k_timer_stop(&key_timer);
        is_key_timer_running = false;
    }

}

/* 割り込みハンドラー */
void key_interrupt_handler(const struct device *port, struct gpio_callback *cb,
                           uint32_t pins)
{
    // タイマーが動作していない場合は開始
    if (!is_key_timer_running) {
        is_key_timer_running = true;
         k_timer_start(&key_timer, K_MSEC(KEY_POLLING_INTERVAL_MS), K_MSEC(KEY_POLLING_INTERVAL_MS));
   }
}

/* コールバック登録関数 */
void register_key_press_cb(void (*cb)(int))
{
    int ret;

    key_event_cb = cb;

    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return;
    }

    gpio_pin_configure(gpio_dev, KEY_BUTTON_PIN, GPIO_ACTIVE_LOW | GPIO_INPUT | GPIO_PULL_UP);
    
    // 両方のエッジで割り込みを設定
    ret = gpio_pin_interrupt_configure(gpio_dev, KEY_BUTTON_PIN, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        printk("Error configuring interrupt on pin %d: %d\n", KEY_BUTTON_PIN, ret);
        return;
    }
    gpio_init_callback(&key_press_cb, key_interrupt_handler, BIT(KEY_BUTTON_PIN));
    ret = gpio_add_callback(gpio_dev, &key_press_cb);
    if (ret != 0) {
        printk("Failed to add callback for key button: %d\n", ret);
    } else {
        printk("Callback added successfully for key button\n");
    }
}


static int dipsw_init(void)
{
    for (int i = 0; i < DIPSW_LEN; ++i) {
        if (!device_is_ready(dipsw_gpios[i].port)) {
            return -1; // エラー: GPIOデバイスが準備できていない
        }

        // (1) プルダウン有効の入力ピンに設定
        gpio_pin_configure_dt(&dipsw_gpios[i], GPIO_INPUT | GPIO_PULL_DOWN);
    }
    return 0;
}

uint8_t get_dipsw(void)
{
    uint8_t value = 0;

    if(dipsw_init() != 0) return 0;

    for (int i = 0; i < DIPSW_LEN; ++i) {
        // 各GPIOピンからビットを読み取る
        int pin_val = gpio_pin_get_dt(&dipsw_gpios[i]);

        if (pin_val < 0) {
            // エラー処理: ピンの読み取りに失敗した場合の対応
            continue; 
        }

        // 読み取った値を結果にマージ
        value |= (pin_val << i);

        // (3) プルアップ有効の入力ピンに再設定
        gpio_pin_configure_dt(&dipsw_gpios[i], GPIO_INPUT | GPIO_PULL_UP);
    }

    return value;
}

void set_led(int on_or_off)
{
    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return;
    }
    gpio_pin_set(gpio_dev, LED_PIN, on_or_off);
}

// ペアリングボタンのタイマーハンドラー
static void pairing_btn_polling_timer_callback(struct k_timer *pairing_btn_timer_id)
{
    if(is_pairing_button_checking && gpio_pin_get(gpio_dev, PAIRING_BUTTON_PIN))
    {
        // 押下でタイマーが開始され、一定時間後の再チェックでもボタンが押されていた
        // →ボタンが確実に押されていたと判断する
        pairing_button_cb();
    }
    is_pairing_button_checking = false;
}

static void pairing_button_intr_cb()
{
    // タイマーを起動
    if(!is_pairing_button_checking)
    {
        k_timer_start(&pairing_btn_timer, K_MSEC(PAIRING_BTN_POLLING_INTERVAL_MS), K_NO_WAIT);
        is_pairing_button_checking = true;
    }
}


void register_pairing_button_cb(void (*cb)(void))
{
    int ret;

    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return;
    }

    gpio_pin_configure(gpio_dev, PAIRING_BUTTON_PIN, GPIO_ACTIVE_LOW | GPIO_INPUT | GPIO_PULL_UP);
    ret = gpio_pin_interrupt_configure(gpio_dev, PAIRING_BUTTON_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Error configuring interrupt on pin %d: %d\n", PAIRING_BUTTON_PIN, ret);
        return;
    }
    gpio_init_callback(&pairing_cb, (gpio_callback_handler_t)pairing_button_intr_cb, BIT(PAIRING_BUTTON_PIN));
    ret = gpio_add_callback(gpio_dev, &pairing_cb);
    if (ret != 0) {
        printk("Failed to add callback for pairing button: %d\n", ret);
    } else {
        printk("Callback added successfully for pairing button\n");
    }

    pairing_button_cb = cb;
}

// キー押下状態取得関数
int get_key_pressed(void)
{
    return !gpio_pin_get(gpio_dev, KEY_BUTTON_PIN);
}

// GPIOデバイスの初期化関数
void init_gpio_dev()
{
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return;
    }
    gpio_pin_configure(gpio_dev, LED_PIN, GPIO_ACTIVE_HIGH | GPIO_OUTPUT);
}


/* End of led_buttons.c */