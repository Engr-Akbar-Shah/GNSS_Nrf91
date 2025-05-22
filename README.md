# GNSS_NRF91

## Overview

**DEV_HUB_GNSS_NRF91** is a Zephyr-based GNSS tracking application for the **Nordic nRF91 series**. It utilizes Nordic’s modem GNSS API to acquire satellite fixes and compute geolocation data. This application also demonstrates periodic and continuous GNSS tracking modes, distance measurement from a reference point, and live terminal output of satellite and fix statistics.

---

## Project Structure

```plaintext
DEV_HUB_GNSS_NRF91/
├── CMakeLists.txt                # Build configuration
├── Kconfig                       # GNSS modes & settings
├── prj.conf                      # Project configuration
├── src/
│   └── main.c                    # Application entry point
├── components/
│   ├── gnss/
│   │   ├── gnss.c                # GNSS logic implementation
│   │   └── gnss.h                # GNSS interface
│   └── nrf91_modem/
│       └── nrf91_modem.c         # Modem setup (LTE GNSS activation)
````

---

## Configuration

### `prj.conf`

Contains key configurations:

* `CONFIG_LTE_LINK_CONTROL` — Enables LTE/GNSS coexistence
* `CONFIG_NRF_MODEM_LIB` — Nordic modem API support
* `CONFIG_UART_INTERRUPT_DRIVEN` — Async terminal output
* `CONFIG_HEAP_MEM_POOL_SIZE`, `CONFIG_MAIN_STACK_SIZE` — Stack/memory tuning

### `Kconfig`

Offers selectable runtime modes:

* **Operation Modes**:

  * `GNSS_SAMPLE_MODE_CONTINUOUS`
  * `GNSS_SAMPLE_MODE_PERIODIC`
* **Power Saving Options** (for continuous mode):

  * `GNSS_SAMPLE_POWER_SAVING_DISABLED`
  * `GNSS_SAMPLE_POWER_SAVING_MODERATE`
  * `GNSS_SAMPLE_POWER_SAVING_HIGH`
* **Reference Position Support**:

  * Set `GNSS_SAMPLE_REFERENCE_LATITUDE` / `LONGITUDE` for distance calculations.

---

## Getting Started

### Requirements

* Nordic nRF9160 DK (or compatible hardware)
* `west`, `nrf SDK`, `Zephyr` installed
* Modem firmware updated and GNSS supported

---

### Build & Flash

```bash
# Initialize and build the project
west build -b nrf9160dk_nrf9160_ns

# Flash it to the board
west flash
```

---

### Run the Application

The device will:

* Activate GNSS
* Print fix data, satellite stats, and optionally distance from a reference
* Automatically refresh terminal output using ANSI escape codes

Output is printed to the UART interface (e.g., via `nRF Terminal`, `PuTTY`, or `screen`).

---

## How to Use This Component in Your Project

### Integration Steps

1. **Copy GNSS Component**:

   * Copy the `components/gnss` folder into your project.
2. **Include in Your CMakeLists**:

   ```cmake
   target_sources(app PRIVATE
       components/gnss/gnss.c)

   target_include_directories(app PRIVATE
       ${CMAKE_CURRENT_SOURCE_DIR}/components/gnss)
   ```
3. **Include Header** in your source:

   ```c
   #include "gnss.h"
   ```
4. **Call Initialization & Polling**:

   ```c
   gnss_init_and_start();
   while (1) {
       gnss_start_searching();
   }
   ```

---

### Things to Take Care Of

* Ensure the modem is in GNSS-compatible functional mode.
* GNSS and LTE can conflict — LTE must yield to GNSS for time slices.
* Stack sizes (`prj.conf`) may need tuning for other project combinations.
* Reference coordinates must be valid decimal degrees.
* If power saving is enabled, expect reduced update frequency.

---

## 🧪 Example Output

```text
Tracking:  5 Using:  4 Unhealthy: 0
Latitude:          60.169500
Longitude:         24.935400
Accuracy:          2.3 m
Speed:             0.2 m/s
Distance from reference: 103.0 m
```

---

## GNSS Modes Summary

| Mode         | Description                      | Config                                         |
| ------------ | -------------------------------- | ---------------------------------------------- |
| Continuous   | Fixes GNSS location continuously | `GNSS_SAMPLE_MODE_CONTINUOUS`                  |
| Periodic     | Fixes at set intervals           | `GNSS_SAMPLE_MODE_PERIODIC` + interval/timeout |
| Power Saving | Reduces GNSS activity            | `GNSS_SAMPLE_POWER_SAVING_*`                   |

---

## 👤 Author

**Developer**: Engr Akbar Shah
**Date**: May 16, 2025

---

## 📄 License

SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
© 2025 Nordic Semiconductor / Engr Akbar Shah
