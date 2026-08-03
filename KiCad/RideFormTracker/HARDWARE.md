# RideFormTracker Hardware Specification

## Overview

BLE-enabled IMU device with USB-C charging, LiPo battery, 9-axis motion sensing, and large flash storage.

---

## Major Components

| Ref | Part | Function |
|-----|------|----------|
| **U1** | MDBT50Q-1MV2 | nRF52840 BLE module (Cortex-M4F, 64MHz, 1MB Flash, 256KB RAM) |
| **U2** | LSM6DSV | 6-axis IMU (accelerometer + gyroscope), STMicroelectronics |
| **U4** | IIS2MDC | 3-axis magnetometer, STMicroelectronics |
| **U3** | W25Q256JVP | 256Mbit (32MB) Quad SPI flash, Winbond |
| **U7** | MCP73831-2-MC | LiPo charge controller (single cell, 4.2V) |
| **U8** | XC6206P332MR | 3.3V LDO regulator |
| **U11** | TPD4E02B04DQA | USB ESD protection |
| **U5** | ICSP_SWD | SWD programming header (custom footprint) |
| **D1** | APFA3010LSEEZGKQBKC | RGB LED (common anode) |
| **D8** | Yellow LED | Charge status indicator |
| **J1** | USB-C 16pin | USB-C receptacle |
| **SW1** | SW_SPDT (PCM12) | Power switch |
| **X1** | 32.768kHz crystal | RTC oscillator |

---

## Power Architecture

```
USB-C (5V) --> SW1 (SPDT switch) --> MCP73831 (U7) --> LiPo battery (4.2V)
                                          |
                                     +V_BAT --> XC6206 (U8) --> +3.3V (system)
                                          |
                                     R10+R11 (1M+1M) --> BAT_CHECK (ADC)
```

- **Charge current**: R20 = 2k on PROG pin -> ~500mA (I_charge = 1000V / R_PROG)
- **Charge status**: D8 (yellow LED) indicates charging
- **Battery monitoring**: 1:1 voltage divider to P0.09 (AIN5)
- **System voltage**: 3.3V regulated by XC6206

---

## nRF52840 (U1) GPIO Pin Map

### SPI Bus - IMU (LSM6DSV)

| nRF52840 Pin | Module Pad | Signal | Target |
|-------------|------------|--------|--------|
| **P0.06** | 22 | MOSI | U2 SDA/SDI (pin 14) |
| **P0.04** | 20 | MISO | U2 SDO/SA0 (pin 1) |
| **P0.08** | 24 | SCLK | U2 SCL/SPC (pin 13) |
| **P1.09** | 26 | CS_0 | U2 CS (pin 12) |

### QSPI Bus - Flash (W25Q256JVP)

| nRF52840 Pin | Module Pad | Signal | W25Q256 Pin |
|-------------|------------|--------|-------------|
| **P0.18** | 40 | ~CS | pin 1 (~CS) |
| **P0.19** | 42 | CLK | pin 6 (CLK) |
| **P0.21** | 43 | IO0 (DI) | pin 5 (DI/IO0) |
| **P0.20** | 44 | IO1 (DO) | pin 2 (DO/IO1) |
| **P0.23** | 45 | IO2 (~WP) | pin 3 (~WP/IO2) |
| **P0.22** | 46 | IO3 (~HOLD) | pin 7 (~HOLD/IO3) |

### Interrupts

| nRF52840 Pin | Module Pad | Signal | Source |
|-------------|------------|--------|--------|
| **P0.26** | 19 | IMU_INT1 | U2 (LSM6DSV) INT1 |
| **P0.05** | 21 | IMU_INT2 | U2 (LSM6DSV) INT2 |

### RGB LED (D1, Common Anode - Active LOW)

| nRF52840 Pin | Module Pad | Signal |
|-------------|------------|--------|
| **P0.28** | 13 | LED_R |
| **P0.31** | 12 | LED_G |
| **P0.30** | 14 | LED_B |

Current limiting: R8 (1.5k), R9 (510)

### Analog Input

| nRF52840 Pin | Module Pad | Signal | Function |
|-------------|------------|--------|----------|
| **P0.09** | 52 | BAT_CHECK | Battery voltage (AIN5, 1:1 divider) |

### USB

| nRF52840 Pin | Module Pad | Signal |
|-------------|------------|--------|
| D+ | 35 | USB_D+ |
| D- | 34 | USB_D- |
| VBUS | 32 | +5V |

USB data lines are protected by U11 (TPD4E02B04DQA ESD protection).

### 32.768kHz Crystal (X1)

| nRF52840 Pin | Module Pad |
|-------------|------------|
| P0.00 (XL1) | 17 |
| P0.01 (XL2) | 18 |

Load capacitors: C3, C4 = 12pF

### SWD Debug Header (U5)

| Header Pin | Signal | Connection |
|-----------|--------|------------|
| 1 (V) | +V_BAT | Battery power |
| 2 (GND) | GND | Ground |
| 3 | SWDIO | U1 Pad 51 |
| 4 | SWCLK | U1 Pad 53 |

### Unconnected GPIOs

P0.02, P0.03, P0.07, P0.10, P0.11, P0.12, P0.13, P0.14, P0.15, P0.16, P0.17, P0.24, P0.25, P0.27, P0.29, P1.00, P1.01, P1.02, P1.03, P1.04, P1.05, P1.06, P1.07, P1.08, P1.10, P1.11, P1.12, P1.13, P1.14, P1.15

---

## Sensor Topology

```
nRF52840 (U1)
    |
    | SPI (MOSI/MISO/SCLK/CS_0)
    v
LSM6DSV (U2) -- 6-axis IMU
    |
    | Auxiliary I2C (SCX/SDX, pulled up by R6/R7 = 4.7k)
    v
IIS2MDC (U4) -- 3-axis magnetometer (I2C mode, ~CS tied to 3.3V)
```

- **LSM6DSV**: Accessed via SPI from nRF52840
- **IIS2MDC**: Connected to LSM6DSV's auxiliary I2C bus (not directly to nRF52840)
  - ~CS tied HIGH = I2C mode
  - Default I2C address: 0x1E
  - DRDY pin is unconnected
- **Sensor Hub**: LSM6DSV's sensor hub feature reads IIS2MDC automatically. The nRF52840 only needs to communicate with LSM6DSV over SPI to get all 9 axes.

---

## Firmware Pin Definitions

```c
// SPI0 - IMU (LSM6DSV)
#define IMU_SPI_MOSI    NRF_GPIO_PIN_MAP(0, 6)   // P0.06
#define IMU_SPI_MISO    NRF_GPIO_PIN_MAP(0, 4)   // P0.04
#define IMU_SPI_SCLK    NRF_GPIO_PIN_MAP(0, 8)   // P0.08
#define IMU_SPI_CS      NRF_GPIO_PIN_MAP(1, 9)   // P1.09

// QSPI - Flash (W25Q256JVP)
#define FLASH_QSPI_CS   NRF_GPIO_PIN_MAP(0, 18)  // P0.18
#define FLASH_QSPI_CLK  NRF_GPIO_PIN_MAP(0, 19)  // P0.19
#define FLASH_QSPI_IO0  NRF_GPIO_PIN_MAP(0, 21)  // P0.21 (DI)
#define FLASH_QSPI_IO1  NRF_GPIO_PIN_MAP(0, 20)  // P0.20 (DO)
#define FLASH_QSPI_IO2  NRF_GPIO_PIN_MAP(0, 23)  // P0.23 (~WP)
#define FLASH_QSPI_IO3  NRF_GPIO_PIN_MAP(0, 22)  // P0.22 (~HOLD)

// Interrupts
#define IMU_INT1        NRF_GPIO_PIN_MAP(0, 26)   // P0.26
#define IMU_INT2        NRF_GPIO_PIN_MAP(0, 5)    // P0.05

// RGB LED (active LOW - drive low to turn on)
#define LED_R           NRF_GPIO_PIN_MAP(0, 28)   // P0.28
#define LED_G           NRF_GPIO_PIN_MAP(0, 31)   // P0.31
#define LED_B           NRF_GPIO_PIN_MAP(0, 30)   // P0.30

// Battery monitoring
#define BAT_CHECK       NRF_GPIO_PIN_MAP(0, 9)    // P0.09 (AIN5)
```

### Zephyr Devicetree Overlay (reference)

```dts
&spi0 {
    compatible = "nordic,nrf-spim";
    status = "okay";
    pinctrl-0 = <&spi0_default>;
    cs-gpios = <&gpio1 9 GPIO_ACTIVE_LOW>;

    lsm6dsv: lsm6dsv@0 {
        compatible = "st,lsm6dsv";
        reg = <0>;
        spi-max-frequency = <8000000>;
        int1-gpios = <&gpio0 26 GPIO_ACTIVE_HIGH>;
        int2-gpios = <&gpio0 5 GPIO_ACTIVE_HIGH>;
    };
};

&qspi {
    status = "okay";
    pinctrl-0 = <&qspi_default>;

    w25q256: w25q256jvp@0 {
        compatible = "nordic,qspi-nor";
        reg = <0>;
        sck-frequency = <16000000>;
        size = <DT_SIZE_M(256)>;
        quad-enable-requirements = "S2B1v1";
    };
};
```

---

## Programming

- **Interface**: SWD (4-pin header U5) or USB DFU
- **Debugger**: J-Link, CMSIS-DAP, or any SWD-compatible probe
- **SDK**: nRF Connect SDK (Zephyr) or nRF5 SDK
- **Bootloader**: nRF52840 supports USB DFU via Adafruit nRF52 Bootloader or MCUboot
