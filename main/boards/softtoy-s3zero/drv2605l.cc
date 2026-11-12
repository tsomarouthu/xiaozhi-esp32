#include "drv2605l.h"
#include "config.h"

#include <esp_log.h>

#define TAG "DRV2605L"

// Registers (datasheet SLOS854)
static constexpr uint8_t REG_STATUS = 0x00;     // bits 7:5 device id (7 = DRV2605L)
static constexpr uint8_t REG_MODE = 0x01;       // 0x00 = active, internal trigger; bit 6 = standby
static constexpr uint8_t REG_RTP = 0x02;
static constexpr uint8_t REG_LIBRARY = 0x03;    // 1 = TS2200 library A (ERM)
static constexpr uint8_t REG_WAVESEQ1 = 0x04;
static constexpr uint8_t REG_GO = 0x0C;
static constexpr uint8_t REG_FEEDBACK = 0x1A;   // bit 7: 0 = ERM, 1 = LRA
static constexpr uint8_t REG_CONTROL3 = 0x1D;   // bit 5: ERM open-loop

bool Drv2605l::Write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev_, buf, 2, 50) == ESP_OK;
}

bool Drv2605l::Read(uint8_t reg, uint8_t* val) {
    return i2c_master_transmit_receive(dev_, &reg, 1, val, 1, 50) == ESP_OK;
}

bool Drv2605l::Init(i2c_master_bus_handle_t bus) {
    present_ = false;
    if (bus == nullptr) {
        ESP_LOGW(TAG, "No I2C bus: haptics disabled (calls are no-ops)");
        return false;
    }
    if (i2c_master_probe(bus, SOFTTOY_I2C_ADDR_DRV2605L, 50) != ESP_OK) {
        ESP_LOGW(TAG, "DRV2605L not found at 0x%02X: haptics disabled (calls are no-ops)",
                 SOFTTOY_I2C_ADDR_DRV2605L);
        return false;
    }
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = SOFTTOY_I2C_ADDR_DRV2605L;
    dev_cfg.scl_speed_hz = SOFTTOY_I2C_FREQ_HZ;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev_) != ESP_OK) {
        ESP_LOGW(TAG, "Could not add DRV2605L to the bus: haptics disabled");
        return false;
    }
    uint8_t status = 0, fb = 0, c3 = 0;
    bool ok = Read(REG_STATUS, &status) && Write(REG_MODE, 0x00)       // leave standby, internal trigger
              && Write(REG_RTP, 0x00)
              && Write(REG_LIBRARY, 1)                                  // ERM library 1 (TS2200 A)
              && Read(REG_FEEDBACK, &fb) && Write(REG_FEEDBACK, fb & 0x7F)   // ERM mode
              && Read(REG_CONTROL3, &c3) && Write(REG_CONTROL3, c3 | 0x20);  // ERM open-loop (Adafruit default)
    if (!ok) {
        ESP_LOGW(TAG, "DRV2605L init I2C error: haptics disabled");
        return false;
    }
    present_ = true;
    ESP_LOGI(TAG, "DRV2605L ready (device id %u, ERM, library 1, internal trigger)", status >> 5);
    return true;
}

bool Drv2605l::Play(const uint8_t* seq, uint8_t count) {
    if (!present_ || seq == nullptr) return false;
    if (count > kMaxSlots) count = kMaxSlots;
    for (uint8_t i = 0; i < count; i++) {
        if (!Write(REG_WAVESEQ1 + i, seq[i])) return false;
    }
    if (count < kMaxSlots && !Write(REG_WAVESEQ1 + count, 0)) return false;  // end marker
    return Write(REG_GO, 1);
}

bool Drv2605l::Busy() {
    uint8_t go = 0;
    return present_ && Read(REG_GO, &go) && (go & 1);
}

void Drv2605l::Stop() {
    if (present_) Write(REG_GO, 0);
}
