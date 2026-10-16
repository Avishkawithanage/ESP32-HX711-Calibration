# ESP32 HX711 Load Cell Calibration Tool

A standalone 3-channel HX711 calibration and diagnostic tool for ESP32 DevKit boards using PlatformIO and the Arduino framework.

## Features

- Checks whether each HX711 module is providing data.
- Performs a tare with the load cell empty.
- Calculates a calibration factor using a known reference weight.
- Rejects calibration when readings are unstable, saturated, too small, or communication fails.
- Streams raw readings or calibrated weight through the Serial Monitor.
- Includes quick checks and detailed diagnostics.

## Hardware and software

- ESP32 DevKit (`esp32dev`)
- Up to three HX711 modules and load cells
- PlatformIO with the Arduino framework
- `bogde/HX711` library version 0.7.5

## Default pin mapping

| Channel | DT / DOUT | SCK / CLK |
|---|---:|---:|
| Plastic | GPIO 36 | GPIO 33 |
| Food | GPIO 4 | GPIO 16 |
| Metal | GPIO 17 | GPIO 21 |

The pin mapping is already defined in `src/main.cpp`. Change the `DT_PIN` and `SCK_PIN` arrays only if your own wiring is different.

## Reference weight

The code is currently configured for a **500 g** reference weight (`REF_WEIGHT_G`). Place a known weight of that mass on the selected load cell during calibration. If you use a different known weight, update `REF_WEIGHT_G` to match it before uploading.

## Build and upload

1. Open this folder in VS Code with PlatformIO installed.
2. Connect your ESP32 DevKit.
3. Select the `esp32dev` environment.
4. Build and upload the project.
5. Open the Serial Monitor at **115200 baud**.

## Serial commands

| Command | Action |
|---|---|
| `1` / `2` / `3` | Select Plastic / Food / Metal channel |
| `A` | Quick data-ready check of all channels |
| `D` | Full diagnostic for the selected channel |
| `T` | Tare with the platform empty |
| `C` | Start calibration |
| `G` | Measure after placing the reference weight |
| `R` | Stream raw readings |
| `W` | Stream weight in grams after calibration |
| `X` | Stop streaming or cancel calibration |
| `F` | Print verified calibration factors |
| `H` | Show help |

Run `T` before `C` for each channel. Follow the Serial Monitor prompts and keep the platform still while taking measurements.

## Notes

- Calibration factors are kept in RAM for the current session; copy the printed factors before resetting or powering off.
- Check the pin mapping against your wiring before powering the device.
- The project should be built and tested with your actual ESP32 and HX711 hardware before relying on measurements.

## License

Released under the MIT License. See `LICENSE`.
