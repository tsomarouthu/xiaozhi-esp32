#include "softtoy_i2c.h"
#include "config.h"

#include <esp_log.h>
#include <cstdio>
#include <cstring>

#define TAG "SoftToyI2C"

namespace SoftToyI2c {

static i2c_master_bus_handle_t s_bus = nullptr;

bool Probe(uint8_t addr) {
    return s_bus != nullptr && i2c_master_probe(s_bus, addr, 50) == ESP_OK;
}

i2c_master_bus_handle_t Init() {
    if (s_bus != nullptr) {
        return s_bus;
    }
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = I2C_NUM_0;
    cfg.sda_io_num = SOFTTOY_I2C_SDA_GPIO;
    cfg.scl_io_num = SOFTTOY_I2C_SCL_GPIO;
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.flags.enable_internal_pullup = 0;
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s (haptics and motion disabled)", esp_err_to_name(err));
        s_bus = nullptr;
        return nullptr;
    }

    // Scan the 7-bit space once and report. Expected: exactly 0x19 and 0x5A.
    char found[160] = "";
    int count = 0;
    bool has_drv = false, has_lis = false;
    for (int addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(s_bus, addr, 20) == ESP_OK) {
            size_t len = strlen(found);
            snprintf(found + len, sizeof(found) - len, "%s0x%02X", count ? " " : "", addr);
            count++;
            has_drv |= (addr == SOFTTOY_I2C_ADDR_DRV2605L);
            has_lis |= (addr == SOFTTOY_I2C_ADDR_LIS3DH);
        }
    }
    ESP_LOGI(TAG, "I2C scan SDA=%d SCL=%d: %d device(s): %s", (int)SOFTTOY_I2C_SDA_GPIO,
             (int)SOFTTOY_I2C_SCL_GPIO, count, count ? found : "none");
    if (has_drv && has_lis) {
        ESP_LOGI(TAG, "I2C OK: DRV2605L (0x5A) and LIS3DH (0x19) both present");
    } else if (has_drv) {
        ESP_LOGW(TAG, "Only DRV2605L (0x5A) found: Qwiic cable to the LIS3DH not seated (or LIS3DH ADR jumper shorted = 0x18)");
    } else if (has_lis) {
        ESP_LOGW(TAG, "Only LIS3DH (0x19) found: DRV2605L missing (no 3V3 or not wired)");
    } else if (count == 0) {
        ESP_LOGW(TAG, "No I2C devices: SDA/SCL swapped, or the DRV2605L has no 3V3");
    } else {
        ESP_LOGW(TAG, "Unexpected I2C addresses only: check wiring.md (LIS3DH at 0x18 = ADR jumper shorted)");
    }
    return s_bus;
}

}  // namespace SoftToyI2c
