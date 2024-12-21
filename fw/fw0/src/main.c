/* This file is main.c, main program of 1-key simple BLE keyboard */
#include "includes.h"
#include "led_buttons.h"

// 現在のスレッド情報を出力するマクロ
#ifdef DEBUG_THREAD
    #define DEBUG_PRINT_THREAD_INFO() \
        do { \
            k_tid_t current_thread_id = k_current_get(); \
            const char *thread_name = k_thread_name_get(current_thread_id); \
            if (thread_name == NULL) { \
                thread_name = "Unnamed thread"; \
            } \
            printk("[DEBUG] Function: %s | Thread ID: %p | Thread Name: %s\n", \
                __func__, current_thread_id, thread_name); \
        } while (0)
#else
    #define DEBUG_PRINT_THREAD_INFO() 
#endif

#define USE_ONE_BYTE_REPORT 0 // 1バイトのレポートを使うかどうか。これを0にすると標準の8バイトのレポートを使う

#if USE_ONE_BYTE_REPORT
    #define INPUT_REPORT_MAX_LEN 1
#else
    #define INPUT_REPORT_MAX_LEN 8
#endif

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION 0x0101

#define ADV_LED_BLINK_ON_TIME 20 // in ms // ペアリング待ちのときの LED 点滅時間
#define ADV_LED_BLINK_OFF_TIME 500 // in ms

#define CONFIRM_LED_BLINK_ON_TIME 20 // in ms // 確認待ちのときの LED 点滅時間
#define CONFIRM_LED_BLINK_OFF_TIME 200 // in ms



#define ADV_TIME_UNIT_IN_MS 0.625
#define ADV_INTERVAL_MIN (int)(200 / ADV_TIME_UNIT_IN_MS)   // 0.2秒
#define ADV_INTERVAL_MAX (int)(500 / ADV_TIME_UNIT_IN_MS)   // 0.2秒

// ペアリングタイムアウト時間（ミリ秒）
#define PAIRING_TIMEOUT_MS 30000

// BTアドレス文字列変換マクロ
#define DEF_BT_ADDR_LE_TO_STR \
  char addr[BT_ADDR_LE_STR_LEN]; \
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

#define KEY_RESEND_INTERVAL 100 // キーコード再送信用インターバル
#define USE_KEY_RESEND 0 // キーコードを繰り返しホストに送信するかどうか


/* HIDS instance. */
BT_HIDS_DEF(hids_obj, INPUT_REPORT_MAX_LEN);

// イベントキューの定義
K_MSGQ_DEFINE(event_queue, sizeof(uint8_t), 10, 4);

// LED点滅用タイマーの定義
static void led_timeout_handler(struct k_timer *dummy);
K_TIMER_DEFINE(adv_led_timer, led_timeout_handler, NULL);

// ペアリングタイムアウト用タイマーの定義
static void pairing_timeout_handler(struct k_timer *dummy);
K_TIMER_DEFINE(pairing_timeout_timer, pairing_timeout_handler, NULL);

// アイドルモード用タイマーの定義
static void fast_mode_timeout_handler(struct k_timer *dummy);
K_TIMER_DEFINE(fast_mode_timeout_timer, fast_mode_timeout_handler, NULL);

#if USE_KEY_RESEND
// キー状態再送信用タイマー
static void key_state_resend_timeout_handler(struct k_timer *dummy);
K_TIMER_DEFINE(key_state_resend_timeout_timer, key_state_resend_timeout_handler, NULL);
#endif

// 接続パラメータの定義（単位: 1.25ms）
#define MIN_CONN_INTERVAL_FAST  0x30
#define MAX_CONN_INTERVAL_FAST  0x60 
#define SLAVE_LATENCY_FAST      0x0002 // スレーブ遅延（2イベント分）
#define CONN_SUP_TIMEOUT_FAST   0x07D0 // スーパータイムアウト 2000 * 10ms = 20秒

#define MIN_CONN_INTERVAL_SLOW  (int)(120 / 1.25)
#define MAX_CONN_INTERVAL_SLOW  (int)(240 / 1.25)
#define SLAVE_LATENCY_SLOW      0x0004 // スレーブ遅延（4イベント分）
#define CONN_SUP_TIMEOUT_SLOW   0x07D0 // スーパータイムアウト 2000 * 10ms = 20秒

// 接続パラメータの構造体
static const struct bt_le_conn_param conn_params_fast = {
    .interval_min = MIN_CONN_INTERVAL_FAST,
    .interval_max = MAX_CONN_INTERVAL_FAST,
    .latency = SLAVE_LATENCY_FAST,
    .timeout = CONN_SUP_TIMEOUT_FAST,
};

static const struct bt_le_conn_param conn_params_slow = {
    .interval_min = MIN_CONN_INTERVAL_SLOW,
    .interval_max = MAX_CONN_INTERVAL_SLOW,
    .latency = SLAVE_LATENCY_SLOW,
    .timeout = CONN_SUP_TIMEOUT_SLOW,
};


// 低消費電力モードへ切り替えるまでの時間
#define FAST_MODE_TIMEOUT 1000*30 // in ms


static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff, (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL), BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct conn_mode {
    struct bt_conn *conn;
    bool in_boot_mode;
    bool is_waiting_confirm; // 確認まちかどうか
} cm[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];
static struct k_mutex cm_mutex; // 上記 cm 構造体を保護するためのミューテックス
#define CM_MUTEX_LOCK() do { k_mutex_lock(&cm_mutex, K_FOREVER); } while(0)
#define CM_MUTEX_UNLOCK() do { k_mutex_unlock(&cm_mutex); } while(0)

// コールバック関数で使用するイベントタイプ
enum event_type {
    EVENT_PAIRING_BUTTON_PRESS,
    EVENT_KEY_PRESS,
    EVENT_KEY_RELEASE,
#if USE_KEY_RESEND
    EVENT_KEY_STATUS_RESEND,
#endif
    EVENT_PAIRING_TIMEOUT,
    EVENT_CHECK_ADV_COND,
    EVENT_FAST_MODE_TIMEOUT,
};

volatile bool is_waiting_pairing = false; // ペアリングボタンが押されて、タイムアウトするまでの間、真になる
volatile bool is_any_connected = false; // どれか一つでもセントラルが接続していたら真になる

volatile bool is_adv_ongoing = false; // アドバタイズが進行中かどうか
static bool led_on = false; // LEDが点灯中かどうか
static bool led_blinking = false; // LEDが点滅中かどうか
static bool is_waiting_confirm = false; // 「確認」待ちかどうか

static void post_check_adv();

// ペアリング用にLED点滅開始
static void start_led_blinking() {
    // LED点滅を開始
    if(!led_blinking) {
        led_blinking = true;
        led_on = false;
        k_timer_start(&adv_led_timer, K_MSEC(ADV_LED_BLINK_ON_TIME), K_NO_WAIT);
    }
}

// LED点滅終了
static void stop_led_blinking() {
    // LED点滅を終了
    if(led_blinking) {
        led_blinking = false;
        led_on = false;
        set_led(0);
        k_timer_stop(&adv_led_timer);
    }
}

// LED点滅用のタイマーコールバックハンドラ
static void led_timeout_handler(struct k_timer *dummy) {
    if(led_on)
    {
        set_led(1);
        k_timer_start(&adv_led_timer,
            is_waiting_confirm ? K_MSEC(CONFIRM_LED_BLINK_ON_TIME) : K_MSEC(ADV_LED_BLINK_ON_TIME), K_NO_WAIT);
    }
    else
    {
        set_led(0);
        k_timer_start(&adv_led_timer,
            is_waiting_confirm ? K_MSEC(CONFIRM_LED_BLINK_OFF_TIME) : K_MSEC(ADV_LED_BLINK_OFF_TIME), K_NO_WAIT);
        post_check_adv(); // アドバタイズを続けるかどうかをチェック
    }   
    led_on = !led_on;
}

static bool is_adv_condition();
static bool is_led_blinking_condition();

// LEDが点滅すべき状態かどうかをチェックし、点滅の開始や停止を行う
static void check_led_blink()
{
    if(is_led_blinking_condition()) start_led_blinking(); else stop_led_blinking();
}

// LED 点滅を行うべきかかどうか
static bool is_led_blinking_condition() {
    return is_adv_condition() || is_waiting_confirm || is_waiting_pairing;
}

// アドバタイズを行うべきかどうか
static bool is_adv_condition() { 
    printk("check: %d %d %d %d \n", is_adv_ongoing, is_waiting_pairing, is_any_connected, is_waiting_confirm);
    return is_waiting_pairing || !is_any_connected;
}

// Stops advertizing
static void stop_adv() {
    if(is_adv_ongoing)
    {
        int err = bt_le_adv_stop();
        if (err) {
            printk("bt_le_adv_stop() failed (err %d)\n", err);
        } else {
            printk("Advertising stopped\n");
            is_adv_ongoing = false;
            is_waiting_pairing = false;
            check_led_blink();
        }

        post_check_adv();

    }
}
// Starts advertising process
static void advertising_start(void) {
    if(!is_adv_ongoing)
    {
        int err;

        const struct bt_le_adv_param adv_param = {
            .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME,
            .interval_min = ADV_INTERVAL_MIN,
            .interval_max = ADV_INTERVAL_MAX,
            .peer = NULL
        };

        err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
        if (err) {
            if (err == -EALREADY) {
                printk("Advertising continued\n");
            } else {
                printk("bt_le_adv_start() failed (err %d)\n", err);
            }

            return;
        }

        printk("Advertising successfully started\n");
        is_adv_ongoing = true;
        check_led_blink();
    }
}


// アドバタイズは以下のどちらかの場合に該当した場合に行わる
// ・ペアリングボタンが押されてからそれがタイムアウトするまで
// ・何も接続がないとき
// 以下の関数はこれらの状態をチェックし、状況に合わせてアドバタイズを開始し、
// 状況に合わせてアドバタイズを終了する
// 安全性の確保のため、メインスレッドからのみ呼び出すこと
static void check_adv()
{
    if(is_adv_condition()) advertising_start(); else stop_adv();
    check_led_blink();
}


// アドバタイズの状態を再チェックするようにキューにメッセージを投入する関数
static void post_check_adv()
{
    uint8_t event = EVENT_CHECK_ADV_COND;

    if (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0) {
        printk("Failed to queue EVENT_CHECK_ADV_COND event\n");
    }
}

// 通信速度の頻度を設定する
static void set_ble_speed(bool fast)
{
    DEBUG_PRINT_THREAD_INFO();

    static bool last_mode = false;
    if(last_mode != fast)
    {
        printk("Fast mode: %s\n", fast ? "on" : "off");
#if USE_KEY_RESEND
        if(fast) // fast 中はキーボードイベントを繰り返し送信する
            k_timer_start(&key_state_resend_timeout_timer, K_MSEC(KEY_RESEND_INTERVAL), K_MSEC(KEY_RESEND_INTERVAL));
        else
            k_timer_stop(&key_state_resend_timeout_timer);
#endif
        CM_MUTEX_LOCK();
        for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
            if (cm[i].conn) {
                bt_conn_le_param_update(cm[i].conn, fast ? &conn_params_fast : &conn_params_slow); // パラメータの更新
                bt_conn_le_param_update(cm[i].conn, fast ? &conn_params_fast : &conn_params_slow); // パラメータの更新
            }
        }
        CM_MUTEX_UNLOCK();
        last_mode = fast;
    }
}

// 通信速度 fast = true のタイムアウトを設定する
static void reset_fast_mode_timeout_timer()
{
    k_timer_start(&fast_mode_timeout_timer, K_MSEC(FAST_MODE_TIMEOUT), K_NO_WAIT);
}

// 通信速度 fast = true のタイムアウトハンドラー
void fast_mode_timeout_handler(struct k_timer *dummy)
{
    uint8_t event = EVENT_FAST_MODE_TIMEOUT;

    if (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0) {
        printk("Failed to queue EVENT_FAST_MODE_TIMEOUT event\n");
    }
}


// Callback function when a device is connected
static void connected(struct bt_conn *conn, uint8_t err) {
    DEBUG_PRINT_THREAD_INFO();

    DEF_BT_ADDR_LE_TO_STR

    if (err) {
        printk("Failed to connect to %s 0x%02x %s\n", addr, err, bt_hci_err_to_str(err));
        return;
    }

    printk("Connected %s\n", addr);

    err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (err) {
        printk("Failed to set security level: %d\n", err);
    }
    
    err = bt_conn_le_param_update(conn, &conn_params_fast); // パラメータの更新
    if (err) {
        printk("bt_conn_le_param_update() failed\n");
        return;
    }

    reset_fast_mode_timeout_timer();

    err = bt_hids_connected(&hids_obj, conn);

    if (err) {
        printk("bt_hids_connected() failed\n");
        return;
    }

    // Find an empty slot for the connection
    CM_MUTEX_LOCK();
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (!cm[i].conn) {
            cm[i].conn = conn;
            cm[i].in_boot_mode = false;
            cm[i].is_waiting_confirm = false;
            is_any_connected = true;
            break;
        }
    }
    CM_MUTEX_UNLOCK();

    post_check_adv();
}

// Callback function when a device is disconnected
static void disconnected(struct bt_conn *conn, uint8_t reason) {
    DEBUG_PRINT_THREAD_INFO();

    int err;
    DEF_BT_ADDR_LE_TO_STR

    printk("Disconnected from %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

    err = bt_hids_disconnected(&hids_obj, conn);

    if (err) {
        printk("bt_hids_disconnected() failed\n");
    }

    // Clear the connection slot
    CM_MUTEX_LOCK();
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (cm[i].conn == conn) {
            cm[i].conn = NULL;
            break;
        }
    }

    // check if all connection was lost
    bool any_connected = false;
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (cm[i].conn) {
            any_connected = true;
            break;
        }
    }
    CM_MUTEX_UNLOCK();
    is_any_connected = any_connected;

    post_check_adv();
}

// 指定された接続の「確認まち」を有効にする
static void set_waiting_confirm(struct bt_conn *conn)
{
    CM_MUTEX_LOCK();
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (cm[i].conn == conn) {
            cm[i].is_waiting_confirm = true;
            break;
        }
    }
    CM_MUTEX_UNLOCK();
}

// すべての接続にて「確認」を承認する
static void confirm_all()
{
    CM_MUTEX_LOCK();
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (cm[i].conn && cm[i].is_waiting_confirm) {
            int err = bt_conn_auth_passkey_confirm(cm[i].conn);
            if (err) {
                printk("bt_conn_auth_passkey_confirm() failed\n");
            }
            cm[i].is_waiting_confirm = false;
        }
    }
    CM_MUTEX_UNLOCK();
}

// すべての接続にて「確認」をキャンセルする
static void cancel_confirm_all()
{
    CM_MUTEX_LOCK();
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (cm[i].conn && cm[i].is_waiting_confirm) {
            cm[i].is_waiting_confirm = false;
        }
    }
    CM_MUTEX_UNLOCK();
}

// Callback function for security level changes
static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
    DEBUG_PRINT_THREAD_INFO();

    DEF_BT_ADDR_LE_TO_STR

    if (!err) {
        printk("Security changed: %s level %u\n", addr, level);
    } else {
        printk("Security failed: %s level %u err %d %s\n", addr, level, err, bt_security_err_to_str(err));
    }
}

struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed,
};

// HID profile event handler
static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn *conn) {
    DEBUG_PRINT_THREAD_INFO();

    DEF_BT_ADDR_LE_TO_STR
    size_t i;

    CM_MUTEX_LOCK();
    for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (cm[i].conn == conn) {
            break;
        }
    }

    if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
        printk("Cannot find connection handle when processing PM");
        CM_MUTEX_UNLOCK();
        return;
    }

    switch (evt) {
    case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
        printk("Boot mode entered %s\n", addr);
        cm[i].in_boot_mode = true;
        break;

    case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
        printk("Report mode entered %s\n", addr);
        cm[i].in_boot_mode = false;
        break;

    default:
        break;
    }
    CM_MUTEX_UNLOCK();
}

// Initialize the HID service
static void hid_init(void) {
    int err;
    struct bt_hids_init_param hids_init = {0};
    struct bt_hids_inp_rep *inp_rep;

#if USE_ONE_BYTE_REPORT
    static const uint8_t report_map[] = {
		0x05, 0x01,       /* Usage Page (Generic Desktop) */
		0x09, 0x06,       /* Usage (Keyboard) */
		0xA1, 0x01,       /* Collection (Application) */

		/* Keys */
#if INPUT_REP_KEYS_REF_ID
		0x85, INPUT_REP_KEYS_REF_ID,
#endif
		0x05, 0x07,       /* Usage Page (Key Codes) */
		0x95, 0x01,       /* Report Count (1) */
		0x75, 0x08,       /* Report Size (8) */
		0x15, 0x00,       /* Logical Minimum (0) */
		0x25, 0x65,       /* Logical Maximum (101) */
		0x05, 0x07,       /* Usage Page (Key codes) */
		0x19, 0x00,       /* Usage Minimum (0) */
		0x29, 0x65,       /* Usage Maximum (101) */
		0x81, 0x00,       /* Input  1 bytes */

		0xC0              /* End Collection (Application) */
    };
#else
    // standard 6 byte report
    static const uint8_t report_map[] = {
		0x05, 0x01,       /* Usage Page (Generic Desktop) */
		0x09, 0x06,       /* Usage (Keyboard) */
		0xA1, 0x01,       /* Collection (Application) */

		/* Keys */
#if INPUT_REP_KEYS_REF_ID
		0x85, INPUT_REP_KEYS_REF_ID,
#endif
		0x05, 0x07,       /* Usage Page (Key Codes) */
		0x19, 0xe0,       /* Usage Minimum (224) */
		0x29, 0xe7,       /* Usage Maximum (231) */
		0x15, 0x00,       /* Logical Minimum (0) */
		0x25, 0x01,       /* Logical Maximum (1) */
		0x75, 0x01,       /* Report Size (1) */
		0x95, 0x08,       /* Report Count (8) */
		0x81, 0x02,       /* Input (Data, Variable, Absolute) */

		0x95, 0x01,       /* Report Count (1) */
		0x75, 0x08,       /* Report Size (8) */
		0x81, 0x01,       /* Input (Constant) reserved byte(1) */

		0x95, 0x06,       /* Report Count (6) */
		0x75, 0x08,       /* Report Size (8) */
		0x15, 0x00,       /* Logical Minimum (0) */
		0x25, 0x65,       /* Logical Maximum (101) */
		0x05, 0x07,       /* Usage Page (Key codes) */
		0x19, 0x00,       /* Usage Minimum (0) */
		0x29, 0x65,       /* Usage Maximum (101) */
		0x81, 0x00,       /* Input (Data, Array) Key array(6 bytes) */
    

		0xC0              /* End Collection (Application) */

    };
#endif


    hids_init.rep_map.data = report_map;
    hids_init.rep_map.size = sizeof(report_map);

    hids_init.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
    hids_init.info.b_country_code = 0x00;
    hids_init.info.flags = (BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE);

    inp_rep = &hids_init.inp_rep_group_init.reports[0];
    inp_rep->size = INPUT_REPORT_MAX_LEN;
    inp_rep->id = 0;
    hids_init.inp_rep_group_init.cnt++;

    hids_init.is_kb = true;
    hids_init.pm_evt_handler = hids_pm_evt_handler;

    err = bt_hids_init(&hids_obj, &hids_init);
    __ASSERT(err == 0, "HIDS initialization failed\n");
}

// Displays passkey when pairing
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
    DEBUG_PRINT_THREAD_INFO();
    DEF_BT_ADDR_LE_TO_STR
    printk("Passkey for %s: %06u\n", addr, passkey);
}

#if 0
// Handles passkey entry
static void auth_passkey_entry(struct bt_conn *conn) {
    DEBUG_PRINT_THREAD_INFO();
    int err;
    DEF_BT_ADDR_LE_TO_STR
    printk("Passkey entry requested for %s\n", addr);
    err = bt_conn_auth_passkey_entry(conn, 0);
    if (err) {
        printk("bt_conn_auth_passkey_entry() failed (err %d)\n", err);
        return;
    }
}
#endif

// Confirms passkey during pairing
static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey) {
    DEBUG_PRINT_THREAD_INFO();
    DEF_BT_ADDR_LE_TO_STR
    printk("Confirm passkey for %s: %06u\n", addr, passkey);
    set_waiting_confirm(conn);
    is_waiting_confirm = true;
    post_check_adv();
}

// Called if pairing is cancelled
static void auth_cancel(struct bt_conn *conn) {
    DEBUG_PRINT_THREAD_INFO();
    DEF_BT_ADDR_LE_TO_STR
    printk("Pairing cancelled: %s\n", addr);
    cancel_confirm_all();
    is_waiting_confirm = false;
    is_waiting_pairing = false;
    post_check_adv();
}

// Callback when pairing is completed
static void pairing_complete(struct bt_conn *conn, bool bonded) {
    DEBUG_PRINT_THREAD_INFO();
    DEF_BT_ADDR_LE_TO_STR
    printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
    cancel_confirm_all();
    is_waiting_confirm = false;
    is_waiting_pairing = false;
    post_check_adv();
}

// Callback when pairing fails
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
    DEBUG_PRINT_THREAD_INFO();
    DEF_BT_ADDR_LE_TO_STR
    printk("Pairing failed conn: %s, reason %d %s\n", addr, reason, bt_security_err_to_str(reason));
    cancel_confirm_all();
    is_waiting_confirm = false;
    is_waiting_pairing = false;
    post_check_adv();
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display,// auth_passkey_display, このデバイスはパスキー表示機能を持たない
    .passkey_entry = NULL, // auth_passkey_entry, このデバイスはパスキー入力ができない
    .passkey_confirm = auth_passkey_confirm, // このデバイスは「確認」入力はできる
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed,
};

// current keyboard state
static bool key_pressed;
static uint8_t key_code;

// Send key report to all connected clients
static int key_report_send() {
    int err = 0;
    uint8_t report[INPUT_REPORT_MAX_LEN] = {0}; // レポートバイト列

#if USE_ONE_BYTE_REPORT
    report[0] = key_pressed ? key_code : 0;
#else
    report[2] = key_pressed ? key_code : 0; // [シフトステート, 予約, キーコード, 0,0,0,0,0]
#endif

    CM_MUTEX_LOCK();
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (cm[i].conn) {
            if (cm[i].in_boot_mode) {
                err = bt_hids_boot_kb_inp_rep_send(&hids_obj, cm[i].conn, 
                                                   report, 
                                                   sizeof(report), NULL);
            } else {
                err = bt_hids_inp_rep_send(&hids_obj, cm[i].conn, 0, 
                                           report, 
                                           sizeof(report), NULL);
            }
            if (err) {
                CM_MUTEX_UNLOCK();
                printk("key_report_send() failed: %d\n", err);
                return err;
            }
        }
    }
    CM_MUTEX_UNLOCK();
    return 0;
}

#if USE_KEY_RESEND
// キーボード情報再送信用ハンドラー
static void key_state_resend_timeout_handler(struct k_timer *dummy)
{
    uint8_t event = EVENT_KEY_STATUS_RESEND;

    if (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0) {
        printk("Failed to queue EVENT_KEY_STATUS_RESEND event\n");
    } 
}
#endif

// ペアリングボタンが押されたときのコールバック関数
static void pairing_button_callback(void) {
    uint8_t event = EVENT_PAIRING_BUTTON_PRESS;

    printk("Pairing button pressed\n");

    if (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0) {
        printk("Failed to queue pairing button event\n");
    }
}

// キーが押されたときのコールバック関数
static void key_press_callback(int press) {
    uint8_t event = press ? EVENT_KEY_PRESS : EVENT_KEY_RELEASE;

    if (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0) {
        printk("Failed to queue key press event\n");
    }
}

// ペアリングタイムアウトハンドラ
static void pairing_timeout_handler(struct k_timer *dummy) {
    uint8_t event = EVENT_PAIRING_TIMEOUT;

    if (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0) {
        printk("Failed to queue pairing timeout event\n");
    }
}


// ボンディング情報が見つかったかを示すフラグ
static bool bonding_exists = false;

// ボンディング情報をチェックするコールバック関数
static void check_bonding(const struct bt_bond_info *info, void *user_data) {
    // ボンディング情報が存在すればフラグを設定
    bonding_exists = true;
}

// ボンディング情報が存在するか確認する関数
static bool is_bonding_info_present(void) {
    bonding_exists = false; // フラグをリセット

    // ボンディング情報を繰り返し確認
    bt_foreach_bond(BT_ID_DEFAULT, check_bonding, NULL);

    return bonding_exists;
}


// ペアリングとタイマーを開始する
static void start_pairing_and_timer()
{
    // ペアリングモードを開始
    is_waiting_pairing = true;

    // タイマーを設定してタイムアウトを待つ
    k_timer_start(&pairing_timeout_timer, K_MSEC(PAIRING_TIMEOUT_MS), K_NO_WAIT);
    check_adv();
}



int main(void) {
    int err;
    uint8_t keycode;
    uint8_t event;

    printk("SmallKB booted\n");

    k_mutex_init(&cm_mutex);

    init_gpio_dev();

    set_led(0);

    hid_init();

    // 起動時に keycode を読み込む
    k_msleep(100UL); // 入力が安定するまでのダミーウェイト
    keycode = 0x04;     // get_dipsw();
    printk("Key code : 0x%02x (%d)\n", keycode, keycode);

    register_pairing_button_cb(pairing_button_callback);
    register_key_press_cb(key_press_callback);

    err = bt_enable(NULL);
    if (err) {
        printk("bt_enable() failed (err %d)\n", err);
        return 0;
    }

    printk("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&conn_auth_callbacks);         // conn_auth_callbacksの登録
    bt_conn_auth_info_cb_register(&conn_auth_info_callbacks); // conn_auth_info_callbacksの登

    // RTTが接続するまで待つ
    k_msleep(5000UL);

    // スレッドの情報を表示
    DEBUG_PRINT_THREAD_INFO();

    // 初期化後すぐにアドバタイズを開始します
    check_adv();

    while (true) {
        if (k_msgq_get(&event_queue, &event, K_FOREVER) == 0) {
            switch (event) {
            case EVENT_PAIRING_BUTTON_PRESS:
                if(is_waiting_confirm) {
                    confirm_all();
                    check_adv();
                }
                else if (!is_waiting_pairing) {
                    start_pairing_and_timer();
                }
                break;

            case EVENT_PAIRING_TIMEOUT:
                if (is_waiting_pairing) {
                    printk("Pairing timeout\n");
                    is_waiting_pairing = false;
                    check_adv();
                }
                break;

            case EVENT_KEY_PRESS:
            case EVENT_KEY_RELEASE:
                printk("%d Key %s: %02x\n", k_uptime_get_32(),
                  (event == EVENT_KEY_PRESS) ? "Pressed" : "Released", keycode);
                if(event == EVENT_KEY_PRESS)
                {
                    key_code = keycode;
                    key_pressed = true;
                }
                else
                {
                    key_code = 0;
                    key_pressed = false;
                }
                set_ble_speed(true);
                reset_fast_mode_timeout_timer();
                key_report_send();
                break;

#if USE_KEY_RESEND
            case EVENT_KEY_STATUS_RESEND:
                printk("resending keycode\n");
                key_report_send();
                break;
#endif
            case EVENT_CHECK_ADV_COND:
                check_adv();
                break;

            case EVENT_FAST_MODE_TIMEOUT:
                set_ble_speed(false);
                break;
            }
        }
    }
}

/* End of main.c */
