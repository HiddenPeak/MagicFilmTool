#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "iot_button.h"
#include "iot_knob.h"
#include "led_strip.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "../managed_components/lvgl__lvgl/demos/lv_demos.h"

/* Knob GPIO */

#define GPIO_KNOB_BUTTON (46)
#define GPIO_KNOB_A (45)
#define GPIO_KNOB_B (42)

/* Button GPIO */
#define GPIO_BUTTON_1_1 (39)
#define GPIO_BUTTON_1_2 (38)
#define GPIO_BUTTON_1_3 (0)
#define GPIO_BUTTON_2_1 (1)
#define GPIO_BUTTON_2_2 (2)
#define GPIO_BUTTON_2_3 (46)

// GPIO assignment
#define LED_STRIP_BLINK_GPIO (48)
// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS (17)

/* LCD size */
#define EXAMPLE_LCD_H_RES (170)
#define EXAMPLE_LCD_V_RES (320)
#define EXAMPLE_LCD_OFFSET_X (0)
#define EXAMPLE_LCD_OFFSET_Y (35)

/* LCD settings */
#define EXAMPLE_LCD_SPI_NUM (SPI3_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ (40 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS (8)
#define EXAMPLE_LCD_PARAM_BITS (8)
#define EXAMPLE_LCD_COLOR_SPACE (ESP_LCD_COLOR_SPACE_BGR)
#define EXAMPLE_LCD_BITS_PER_PIXEL (16)
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (50)
#define EXAMPLE_LCD_BL_ON_LEVEL (1)

/* LCD pins */
#define EXAMPLE_LCD_GPIO_SCLK (GPIO_NUM_15)
#define EXAMPLE_LCD_GPIO_MOSI (GPIO_NUM_13)
#define EXAMPLE_LCD_GPIO_RST (GPIO_NUM_17)
#define EXAMPLE_LCD_GPIO_DC (GPIO_NUM_14)
#define EXAMPLE_LCD_GPIO_CS (GPIO_NUM_16)

static const char *TAG = "EXAMPLE";

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;

/* LVGL display and Encoder Buttons */
static lv_disp_t *lvgl_disp = NULL;
static lv_indev_t *encoder_handle = NULL;
static lv_indev_t *buttons_handle = NULL;

led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: SPI
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .flags.with_dma = true,         // Using DMA can improve performance and help drive more LEDs
        .spi_bus = SPI2_HOST,           // SPI bus ID
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with SPI backend");
    return led_strip;
}

static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD initialization */
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_LCD_GPIO_SCLK,
        .mosi_io_num = EXAMPLE_LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_LCD_GPIO_DC,
        .cs_gpio_num = EXAMPLE_LCD_GPIO_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_GPIO_RST,
        .color_space = EXAMPLE_LCD_COLOR_SPACE,
        .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_set_gap(lcd_panel, EXAMPLE_LCD_OFFSET_X, EXAMPLE_LCD_OFFSET_Y);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    return ret;

err:
    if (lcd_panel)
    {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io)
    {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(EXAMPLE_LCD_SPI_NUM);
    return ret;
}

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,       /* LVGL task priority */
        .task_stack = 4096,       /* LVGL task stack size */
        .task_affinity = -1,      /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500, /* Maximum sleep in LVGL task */
        .timer_period_ms = 5      /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
        .double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
        }};
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    lv_disp_set_rotation(lvgl_disp, LV_DISP_ROT_90);

    /* Buttons configuration structure */
    const button_config_t button_config[] = {{.type = BUTTON_TYPE_GPIO,
                                              .gpio_button_config.active_level = false,
                                              .gpio_button_config.gpio_num = GPIO_BUTTON_1_1},
                                             {.type = BUTTON_TYPE_GPIO,
                                              .gpio_button_config.active_level = false,
                                              .gpio_button_config.gpio_num = GPIO_BUTTON_1_2},
                                             {.type = BUTTON_TYPE_GPIO,
                                              .gpio_button_config.active_level = false,
                                              .gpio_button_config.gpio_num = GPIO_BUTTON_1_3},
                                             {.type = BUTTON_TYPE_GPIO,
                                              .gpio_button_config.active_level = false,
                                              .gpio_button_config.gpio_num = GPIO_BUTTON_2_1},
                                             {.type = BUTTON_TYPE_GPIO,
                                              .gpio_button_config.active_level = false,
                                              .gpio_button_config.gpio_num = GPIO_BUTTON_2_2},
                                             {.type = BUTTON_TYPE_GPIO,
                                              .gpio_button_config.active_level = false,
                                              .gpio_button_config.gpio_num = GPIO_BUTTON_2_3}};

    const lvgl_port_nav_btns_cfg_t btns = {
        .disp = lvgl_disp,
        .button_prev = &button_config[3],
        .button_next = &button_config[4],
        .button_enter = &button_config[2]};
    buttons_handle = lvgl_port_add_navigation_buttons(&btns);

    const button_config_t encoder_btn_config = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config.active_level = false,
        .gpio_button_config.gpio_num = GPIO_KNOB_BUTTON,
    };

    const knob_config_t encoder_a_b_config = {
        .default_direction = 0,
        .gpio_encoder_a = GPIO_KNOB_A,
        .gpio_encoder_b = GPIO_KNOB_B,
    };

    /* Encoder configuration structure */
    const lvgl_port_encoder_cfg_t encoder = {
        .disp = lvgl_disp,
        .encoder_a_b = &encoder_a_b_config,
        .encoder_enter = &encoder_btn_config};

    /* Add encoder input (for selected screen) */
    encoder_handle = lvgl_port_add_encoder(&encoder);

    return ESP_OK;
}

static void _app_button_cb(lv_event_t *e)
{
    lv_disp_rot_t rotation = lv_disp_get_rotation(lvgl_disp);
    rotation++;
    if (rotation > LV_DISP_ROT_90)
    {
        rotation = LV_DISP_ROT_NONE;
    }

    /* LCD HW rotation */
    lv_disp_set_rotation(lvgl_disp, rotation);
}

static void app_main_display(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* Task lock */
    lvgl_port_lock(0);

    /* Your LVGL objects code here .... */

    /* Label */
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_recolor(label, true);
    lv_obj_set_width(label, EXAMPLE_LCD_H_RES);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "#FF0000 " LV_SYMBOL_BELL " Hello world Espressif and LVGL " LV_SYMBOL_BELL "#\n#FF9400 " LV_SYMBOL_WARNING " For simplier initialization, use BSP " LV_SYMBOL_WARNING " #");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    /* Button */
    lv_obj_t *btn = lv_btn_create(scr);
    label = lv_label_create(btn);
    lv_label_set_text_static(label, "Rotate screen");
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_add_event_cb(btn, _app_button_cb, LV_EVENT_CLICKED, NULL);

    /* Task unlock */
    lvgl_port_unlock();
}

void app_main(void)
{
    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    /* Show LVGL objects */
    // app_main_display();
    lv_demo_keypad_encoder();

    led_strip_handle_t led_strip = configure_led();
    bool led_on_off = false;

    ESP_LOGI(TAG, "Start blinking LED strip");

    while (1)
    {
        if (led_on_off)
        {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 0, 0));
                /* Refresh the strip to send data */
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                vTaskDelay(10);
            }
            ESP_LOGI(TAG, "LED ON!");
        }
        else
        {
            /* Set all LED off to clear all pixels */
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            ESP_LOGI(TAG, "LED OFF!");
        }

        led_on_off = !led_on_off;
    }
}
