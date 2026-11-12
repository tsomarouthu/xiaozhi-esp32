#ifndef _SOFTTOY_I2C_H_
#define _SOFTTOY_I2C_H_

#include <driver/i2c_master.h>

// The one I2C master bus on GPIO15/16, shared by the DRV2605L (0x5A) and the LIS3DH (0x19).
// Created once by the board; never create a second bus on these pins. Pull-ups live on the
// breakouts (docs/wiring.md), so the internal ones are off.
namespace SoftToyI2c {

// Create the bus and log a scan. Safe to call twice (second call is a no-op).
// Returns the bus handle, or nullptr if the bus itself could not be created.
i2c_master_bus_handle_t Init();

// The shared bus handle (nullptr before Init() or if it failed).
i2c_master_bus_handle_t Bus();

// Probe one address on the shared bus.
bool Probe(uint8_t addr);

}  // namespace SoftToyI2c

#endif  // _SOFTTOY_I2C_H_
