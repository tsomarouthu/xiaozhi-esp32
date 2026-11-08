#ifndef _RGB_SINGLE_LED_H_
#define _RGB_SINGLE_LED_H_

// Board-local copy of the common SingleLed, with the WS2812 colour order set to
// RGB instead of the stock GRB. The Waveshare ESP32-S3-Zero's onboard pixel is
// wired RGB; with the common SingleLed (GRB) green shows as red. The common file
// cannot be edited (OTA-safety: shared across boards), so this is a standalone
// class used only by this board.

#include "led/led.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <led_strip.h>
#include <esp_timer.h>
#include <atomic>
#include <mutex>

class RgbSingleLed : public Led {
public:
    RgbSingleLed(gpio_num_t gpio);
    virtual ~RgbSingleLed();

    void OnStateChanged() override;

private:
    std::mutex mutex_;
    TaskHandle_t blink_task_ = nullptr;
    led_strip_handle_t led_strip_ = nullptr;
    uint8_t r_ = 0, g_ = 0, b_ = 0;
    int blink_counter_ = 0;
    int blink_interval_ms_ = 0;
    esp_timer_handle_t blink_timer_ = nullptr;

    void StartBlinkTask(int times, int interval_ms);
    void OnBlinkTimer();

    void BlinkOnce();
    void Blink(int times, int interval_ms);
    void StartContinuousBlink(int interval_ms);
    void TurnOn();
    void TurnOff();
    void SetColor(uint8_t r, uint8_t g, uint8_t b);
};

#endif // _RGB_SINGLE_LED_H_
