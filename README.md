# SX1261 LoRa Radio Driver Documentation

## 📋 Project Overview

This project implements a comprehensive driver for the SX1261 LoRa radio transceiver on ESP32-S3 using ESP-IDF 5.3.1. The driver provides both traditional LoRa communication and Wake-up Radio (WuR) functionality using GFSK modulation for ultra-low power applications.

### Key Features

- **Full SX1261 Radio Control**: Complete implementation of SX126x command set
- **LoRa Communication**: Traditional LoRa packet transmission and reception
- **Wake-up Radio (WuR)**: Ultra-low power wake-up signal detection using GFSK
- **ESP32-S3 Optimized**: Hardware abstraction layer for ESP32-S3 SPI interface
- **Power Management**: Support for both LDO and DC-DC regulator modes
- **TCXO Support**: Configurable temperature-compensated crystal oscillator control
- **Flexible GPIO Control**: Configurable TX/RX enable pins and interrupt handling

---

## 🔧 Hardware Requirements

### ESP32-S3 Pin Configuration

| Signal | GPIO Pin | Description | Required |
|--------|----------|-------------|----------|
| **SPI Signals** | | | |
| MISO | `PIN_SPI_MISO` | SPI Master In Slave Out | ✅ |
| MOSI | `PIN_SPI_MOSI` | SPI Master Out Slave In | ✅ |
| SCLK | `PIN_SPI_SCLK` | SPI Clock | ✅ |
| **Radio Control** | | | |
| NSS | `CONFIG_NSS_GPIO` | SPI Chip Select (Active Low) | ✅ |
| RESET | `CONFIG_RST_GPIO` | Radio Reset (Active Low) | ✅ |
| BUSY | `CONFIG_BUSY_GPIO` | Radio Busy Status | ✅ |
| DIO1 | `CONFIG_DIO1_GPIO` | Digital I/O 1 (Interrupts) | ✅ |
| **RF Switching** | | | |
| TXEN | `CONFIG_TXEN_GPIO` | TX Enable Control | ⚠️ Optional |
| RXEN | `CONFIG_RXEN_GPIO` | RX Enable Control | ⚠️ Optional |

### SX1261 Radio Module Specifications

- **Frequency Range**: 150 MHz to 960 MHz
- **LoRa Data Rate**: 0.018 to 62.5 kbps
- **GFSK Data Rate**: 0.6 to 300 kbps
- **Output Power**: -17 to +14 dBm
- **Sensitivity**: Down to -148 dBm (LoRa SF12, 125 kHz)
- **Power Consumption**: 
  - RX: 4.2 mA
  - TX 14dBm: 44 mA
  - Sleep: 160 nA

---

## 🏗️ System Architecture

### Hardware Block Diagram

```mermaid
graph TB
    subgraph ESP32["ESP32-S3 SoC"]
        SPI[SPI Controller]
        GPIO[GPIO Controller]
        CPU[Dual Core CPU]
        PWR[Power Management]
    end
    
    subgraph SIGNALS["Interface Signals"]
        MISO[MISO - Pin 37]
        MOSI[MOSI - Pin 35]
        SCLK[SCLK - Pin 36]
        NSS[NSS - Pin 10]
        RESET[RESET - Pin 11]
        BUSY[BUSY - Pin 12]
        DIO1[DIO1 - Pin 13]
        TXEN[TXEN - Pin 14]
        RXEN[RXEN - Pin 15]
    end
    
    subgraph SX1261["SX1261 Radio Module"]
        RF[RF Frontend]
        MODEM[LoRa/GFSK Modem]
        XTAL[32MHz Crystal]
        TCXO[TCXO Optional]
        REGS[Internal Regulators]
    end
    
    subgraph EXTERNAL["External Components"]
        ANT[📡 Antenna]
        SUPPLY[🔋 3.3V Supply]
        CAPS[⚡ Bypass Caps]
    end
    
    SPI --> MISO
    SPI --> MOSI  
    SPI --> SCLK
    GPIO --> NSS
    GPIO --> RESET
    GPIO --> BUSY
    GPIO --> DIO1
    GPIO --> TXEN
    GPIO --> RXEN
    
    MISO --> MODEM
    MOSI --> MODEM
    SCLK --> MODEM
    NSS --> MODEM
    RESET --> MODEM
    BUSY --> MODEM
    DIO1 --> MODEM
    TXEN --> RF
    RXEN --> RF
    
    RF --> ANT
    SUPPLY --> REGS
    REGS --> MODEM
    XTAL --> MODEM
    TCXO --> MODEM
```

### Software Architecture Block Diagram

```mermaid
graph TB
    subgraph APP["Application Layer"]
        USER[User Application]
        LORA_APP[LoRa Communication]
        WUR_APP[WuR Communication]
        TASKS[FreeRTOS Tasks]
    end
    
    subgraph API["Driver API Layer"]
        LORA_API[LoRa Driver API]
        WUR_API[WuR Driver API]
        CONFIG_API[Configuration API]
        UTIL[Utility Functions]
    end
    
    subgraph CORE["Core Driver Layer"]
        CMD[Command Layer]
        PACKET[Packet Manager]
        IRQ[IRQ Handler]
        STATE[State Manager]
    end
    
    subgraph HAL["Hardware Abstraction Layer"]
        SPI_HAL[SPI HAL]
        GPIO_HAL[GPIO HAL]
        TIMER_HAL[Timer HAL]
        ERROR[Error Handler]
    end
    
    subgraph DRIVERS["ESP-IDF Drivers"]
        SPI_DRV[SPI Driver]
        GPIO_DRV[GPIO Driver]
        TIMER_DRV[Timer Driver]
        LOG[Logging]
    end
    
    USER --> LORA_API
    USER --> WUR_API
    LORA_APP --> LORA_API
    WUR_APP --> WUR_API
    
    LORA_API --> CMD
    WUR_API --> CMD
    CONFIG_API --> PACKET
    
    CMD --> SPI_HAL
    PACKET --> SPI_HAL
    IRQ --> GPIO_HAL
    STATE --> TIMER_HAL
    
    SPI_HAL --> SPI_DRV
    GPIO_HAL --> GPIO_DRV
    TIMER_HAL --> TIMER_DRV
    ERROR --> LOG
```

### Data Flow Block Diagram

```mermaid
graph LR
    subgraph TX["Transmit Path"]
        APP_TX[Application Data] --> TX_BUF[TX Buffer]
        TX_BUF --> FORMAT[Packet Formatter]
        FORMAT --> MOD[Modulator]
        MOD --> PA[Power Amplifier]
        PA --> ANT_TX[📡 Antenna]
    end
    
    subgraph RX["Receive Path"]
        ANT_RX[📡 Antenna] --> LNA[Low Noise Amp]
        LNA --> DEMOD[Demodulator]
        DEMOD --> DECODE[Packet Decoder]
        DECODE --> RX_BUF[RX Buffer]
        RX_BUF --> APP_RX[Application Data]
    end
    
    subgraph CTRL["Control & Status"]
        CMD_IF[Command Interface]
        STATUS[Status Registers]
        IRQ_ST[IRQ Status]
        SM[State Machine]
        TIMING[Timing Control]
    end
    
    CMD_IF --> SM
    SM --> TIMING
    IRQ_ST --> STATUS
```

### SPI Communication Block Diagram

```mermaid
graph TB
    subgraph ESP32_SPI["ESP32-S3 SPI Master"]
        SPI_CTRL[SPI Controller]
        DMA[DMA Controller]
        GPIO_SPI[GPIO Matrix]
    end
    
    subgraph SPI_SIGNALS["SPI Signal Lines"]
        MOSI_S[MOSI: Commands & Data →]
        MISO_S[MISO: Status & Data ←]
        SCLK_S[SCLK: 2MHz Clock]
        NSS_S[NSS: Chip Select ↓]
    end
    
    subgraph SX1261_SPI["SX1261 SPI Slave"]
        SPI_IF[SPI Interface]
        CMD_DEC[Command Decoder]
        REG_MAP[Register Map]
        FIFO[FIFO Buffers]
    end
    
    subgraph PROTOCOL["SPI Protocol Phases"]
        CMD_PHASE[1. Command Phase]
        ADDR_PHASE[2. Address Phase]
        DATA_PHASE[3. Data Phase]
        STATUS_PHASE[4. Status Phase]
    end
    
    SPI_CTRL --> MOSI_S
    MISO_S --> SPI_CTRL
    SPI_CTRL --> SCLK_S
    GPIO_SPI --> NSS_S
    
    MOSI_S --> SPI_IF
    SPI_IF --> MISO_S
    SCLK_S --> SPI_IF
    NSS_S --> SPI_IF
    
    SPI_IF --> CMD_DEC
    CMD_DEC --> REG_MAP
    CMD_DEC --> FIFO
```

### Power Management Block Diagram

```mermaid
graph TB
    subgraph POWER_IN["Power Input"]
        VDD[VDD 3.3V Input]
        GND[Ground]
        BYPASS[Bypass Capacitors]
    end
    
    subgraph REGULATORS["Internal Regulators"]
        LDO[LDO Regulator]
        DCDC[DC-DC Regulator]
        REG_SEL[Regulator Select]
    end
    
    subgraph DOMAINS["Power Domains"]
        DIGITAL[Digital Core]
        RF_PWR[RF Frontend]
        OSC[Crystal/TCXO]
    end
    
    subgraph MODES["Power Modes"]
        SLEEP[Sleep Mode<br/>160nA]
        STANDBY[Standby Mode<br/>1.5µA]
        RX_MODE[RX Mode<br/>4.2mA]
        TX_MODE[TX Mode<br/>17-44mA]
    end
    
    VDD --> LDO
    VDD --> DCDC
    REG_SEL --> LDO
    REG_SEL --> DCDC
    
    LDO --> DIGITAL
    DCDC --> DIGITAL
    DIGITAL --> RF_PWR
    DIGITAL --> OSC
    
    DIGITAL --> SLEEP
    DIGITAL --> STANDBY
    DIGITAL --> RX_MODE
    DIGITAL --> TX_MODE
```

### Complete System Integration Block Diagram

```mermaid
graph TB
    subgraph EXTERNAL["External System"]
        USER_APP[User Application]
        FREERTOS[FreeRTOS Kernel]
        ANTENNA[📡 Antenna System]
        POWER_SYS[🔋 Power System]
    end
    
    subgraph ESP32_SYSTEM["ESP32-S3 System"]
        APP_LAYER[Application Layer]
        DRIVER_LAYER[Driver Layer]
        HAL_LAYER[HAL Layer]
        HW_LAYER[Hardware Layer]
    end
    
    subgraph SX1261_SYSTEM["SX1261 Radio System"]
        DIGITAL_CORE[Digital Core]
        RF_CORE[RF Core]
        MODEM_CORE[Modem Core]
        BUFFER_CORE[Buffer Core]
    end
    
    subgraph INTERFACES["Interface Layer"]
        SPI_BUS[SPI Bus Interface]
        GPIO_BUS[GPIO Interface]
        RF_INTERFACE[RF Interface]
        POWER_INTERFACE[Power Interface]
    end
    
    USER_APP --> APP_LAYER
    FREERTOS --> APP_LAYER
    
    APP_LAYER --> DRIVER_LAYER
    DRIVER_LAYER --> HAL_LAYER
    HAL_LAYER --> HW_LAYER
    
    HW_LAYER --> SPI_BUS
    HW_LAYER --> GPIO_BUS
    
    SPI_BUS --> DIGITAL_CORE
    GPIO_BUS --> DIGITAL_CORE
    
    DIGITAL_CORE --> MODEM_CORE
    DIGITAL_CORE --> BUFFER_CORE
    MODEM_CORE --> RF_CORE
    
    RF_CORE --> RF_INTERFACE
    RF_INTERFACE --> ANTENNA
    
    POWER_SYS --> POWER_INTERFACE
    POWER_INTERFACE --> DIGITAL_CORE
```

### Driver Layers

#### 1. **Application Interface Layer**
- `LoRaBegin()` - Initialize radio
- `LoRaSend()` - Send LoRa packets
- `LoRaReceive()` - Receive LoRa packets
- `send_WUR()` - Send wake-up signals
- `Listen_WUR()` - Listen for wake-up signals

#### 2. **Command Abstraction Layer**
- High-level radio configuration functions
- Packet handling and buffer management
- IRQ status management
- Power and frequency control

#### 3. **SX126x Protocol Layer**
- Direct SX126x command implementation
- Register access functions
- Low-level packet operations
- Hardware abstraction interface

#### 4. **Hardware Abstraction Layer (HAL)**
- SPI communication interface
- GPIO control and status reading
- Timing and delay functions
- Platform-specific implementations

---

## 📡 Communication Protocols

### LoRa Communication Flow

```mermaid
sequenceDiagram
    participant App as Application
    participant Driver as LoRa Driver
    participant Radio as SX1261 Radio
    participant HAL as HAL Layer
    
    Note over App,HAL: Initialization Sequence
    App->>Driver: LoRaBegin(freq, power, tcxo, regulator)
    Driver->>HAL: Reset()
    HAL->>Radio: Hardware Reset
    Driver->>Radio: SetStandby()
    Driver->>Radio: SetRegulatorMode()
    Driver->>Radio: Calibrate()
    Driver->>Radio: SetPaConfig()
    Driver->>Radio: SetRfFrequency()
    
    Note over App,HAL: Transmission Sequence
    App->>Driver: LoRaSend(data, length, mode)
    Driver->>Radio: SetPacketParams()
    Driver->>Radio: ClearIrqStatus()
    Driver->>Radio: WriteBuffer(data)
    Driver->>Radio: SetTx(timeout)
    
    alt Synchronous Mode
        Driver->>Radio: GetIrqStatus()
        Radio-->>Driver: IRQ Status
        Driver->>Radio: SetRx() (Auto-fallback)
    end
    
    Note over App,HAL: Reception Sequence
    App->>Driver: LoRaReceive(buffer, max_len)
    Driver->>Radio: GetIrqStatus()
    alt RX_DONE IRQ
        Driver->>Radio: ReadBuffer()
        Radio-->>Driver: Received Data
        Driver->>Radio: ClearIrqStatus()
    end
    Driver-->>App: Received Length
```

### Wake-up Radio (WuR) Communication

```mermaid
sequenceDiagram
    participant TX as WuR Transmitter
    participant RX as WuR Receiver
    participant Radio_TX as SX1261 TX
    participant Radio_RX as SX1261 RX
    
    Note over TX,Radio_RX: WuR Initialization
    TX->>Radio_TX: Sx1261_InitParam_WUR()
    TX->>Radio_TX: Sx1261_InitFonction_WUR()
    RX->>Radio_RX: Sx1261_InitParam_WUR()
    RX->>Radio_RX: Sx1261_InitFonction_WUR()
    
    Note over TX,Radio_RX: Duty Cycle Setup
    RX->>Radio_RX: Sx1261_WurMode(sleep_time, rx_time)
    
    Note over TX,Radio_RX: Wake-up Signal Transmission
    TX->>Radio_TX: Sx1261_Send_WuR_Signal(data, length)
    Radio_TX->>Radio_TX: SetTx(TX_TIME_MS)
    
    Note over TX,Radio_RX: Signal Detection
    Radio_RX->>Radio_RX: SYNC_WORD_VALID IRQ
    RX->>Radio_RX: GetIrqStatus()
    
    alt Wake-up Detected
        RX->>RX: Process Wake-up Event
        RX->>Radio_RX: Return to WuR Mode
    end
```

---

## 🔄 System State Machines

### Radio State Machine

```mermaid
stateDiagram-v2
    [*] --> SLEEP
    SLEEP --> STANDBY_RC : Wakeup Command
    STANDBY_RC --> STANDBY_XOSC : Set Standby XOSC
    STANDBY_XOSC --> FS : Set FS
    
    FS --> TX : SetTx()
    TX --> STANDBY_RC : TX_DONE/Timeout
    TX --> STANDBY_XOSC : TX_DONE (with fallback)
    
    FS --> RX : SetRx()
    RX --> STANDBY_RC : RX_DONE/Timeout
    RX --> STANDBY_XOSC : RX_DONE (with fallback)
    
    STANDBY_RC --> SLEEP : SetSleep()
    STANDBY_XOSC --> SLEEP : SetSleep()
    
    FS --> CAD : SetCad()
    CAD --> STANDBY_RC : CAD_DONE
```

### WuR Operation Flow

```mermaid
flowchart TD
    A[Initialize WuR Parameters] --> B[Configure GFSK Modulation]
    B --> C[Set Sync Word Pattern]
    C --> D[Configure IRQ for SYNC_WORD_VALID]
    D --> E[Enter RX Duty Cycle Mode]
    
    E --> F{Signal Detected?}
    F -->|No| G[Sleep Period]
    G --> H[Wake for RX Window]
    H --> F
    
    F -->|Yes| I[Process Wake-up Event]
    I --> J[Execute Wake-up Actions]
    J --> K[Clear IRQ Status]
    K --> E
    
    L[WuR Transmitter] --> M[Load Wake-up Pattern]
    M --> N[Set TX Mode]
    N --> O[Transmit Signal]
    O --> P[Return to Standby]
```

---

## 📚 API Reference

### Core Initialization Functions

#### `LoRaBegin()`
```c
int16_t LoRaBegin(uint32_t frequencyInHz, int8_t txPowerInDbm, 
                  float tcxoVoltage, bool useRegulatorLDO)
```
**Purpose**: Initialize the LoRa radio with specified parameters.

**Parameters**:
- `frequencyInHz`: Operating frequency (150MHz - 960MHz)
- `txPowerInDbm`: TX power (-17 to +14 dBm)
- `tcxoVoltage`: TCXO voltage (0.0 to disable, 1.6V - 3.3V)
- `useRegulatorLDO`: `true` for LDO, `false` for DC-DC regulator

**Returns**: `ERR_NONE` on success, error code on failure

**Example**:
```c
// Initialize at 868 MHz, 14 dBm, 3.3V TCXO, DC-DC regulator
int16_t result = LoRaBegin(868000000, 14, 3.3, false);
if (result != ERR_NONE) {
    ESP_LOGE(TAG, "LoRa initialization failed: %d", result);
}
```

#### `LoRaConfig()`
```c
void LoRaConfig(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, 
                uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq)
```
**Purpose**: Configure LoRa modulation parameters.

**Parameters**:
- `spreadingFactor`: SF7 to SF12 (SX126X_LORA_SF7 - SX126X_LORA_SF12)
- `bandwidth`: 125kHz, 250kHz, 500kHz (SX126X_LORA_BW_125, etc.)
- `codingRate`: CR 4/5 to 4/8 (SX126X_LORA_CR_4_5 - SX126X_LORA_CR_4_8)
- `preambleLength`: Preamble length in symbols (8-65535)
- `payloadLen`: Fixed payload length (0 for variable length)
- `crcOn`: Enable CRC checking
- `invertIrq`: Invert I/Q signals

### Data Transmission Functions

#### `LoRaSend()`
```c
bool LoRaSend(uint8_t *pData, uint8_t len, uint8_t mode)
```
**Purpose**: Send LoRa packet.

**Parameters**:
- `pData`: Pointer to data buffer
- `len`: Data length (1-255 bytes)
- `mode`: `SX126x_TXMODE_SYNC` for blocking, `SX126x_TXMODE_ASYNC` for non-blocking

**Returns**: `true` on success, `false` on failure

#### `LoRaReceive()`
```c
uint8_t LoRaReceive(uint8_t *pData, uint16_t len)
```
**Purpose**: Receive LoRa packet.

**Parameters**:
- `pData`: Buffer for received data
- `len`: Maximum buffer size

**Returns**: Number of bytes received (0 if no packet)

### Wake-up Radio Functions

#### `send_WUR()`
```c
void send_WUR(void)
```
**Purpose**: Initialize and start WuR transmission task.

**Usage**: Call once to start continuous wake-up signal transmission.

#### `Listen_WUR()`
```c
void Listen_WUR(void)
```
**Purpose**: Initialize and start WuR reception mode.

**Usage**: Call once to start listening for wake-up signals.

### Configuration Functions

#### `SetTxPower()`
```c
void SetTxPower(int8_t txPowerInDbm)
```
**Purpose**: Change transmission power.

**Parameters**:
- `txPowerInDbm`: Power level (-17 to +14 dBm)

#### `GetPacketStatus()`
```c
void GetPacketStatus(int8_t *rssiPacket, int8_t *snrPacket)
```
**Purpose**: Get signal quality metrics for last received packet.

**Parameters**:
- `rssiPacket`: Pointer to store RSSI value (dBm)
- `snrPacket`: Pointer to store SNR value (dB)

---

## ⚙️ Configuration Guide

### Menuconfig Settings

Configure your ESP-IDF project with the following settings:

```bash
idf.py menuconfig
```

Navigate to your custom configuration and set:

```
CONFIG_NSS_GPIO=10          # SPI Chip Select
CONFIG_RST_GPIO=11          # Reset pin
CONFIG_BUSY_GPIO=12         # Busy status pin
CONFIG_DIO1_GPIO=13         # Interrupt pin
CONFIG_TXEN_GPIO=14         # TX enable (optional)
CONFIG_RXEN_GPIO=15         # RX enable (optional)
```

### SPI Configuration

The driver uses the following SPI settings:
- **Frequency**: 2 MHz
- **Mode**: 0 (CPOL=0, CPHA=0)
- **Bit Order**: MSB first
- **CS Control**: Software controlled

### Power Management Options

#### LDO vs DC-DC Regulator
```c
// Use LDO regulator (simpler, higher power consumption)
LoRaBegin(868000000, 14, 3.3, true);

// Use DC-DC regulator (more efficient, requires external components)
LoRaBegin(868000000, 14, 3.3, false);
```

#### TCXO Configuration
```c
// No external TCXO
LoRaBegin(868000000, 14, 0.0, false);

// With external TCXO at 3.3V
LoRaBegin(868000000, 14, 3.3, false);
```

---

## 🔨 Build Instructions

### Prerequisites

- ESP-IDF 5.3.1 or later
- ESP32-S3 development board
- SX1261 radio module

### Build Steps

1. **Clone and Setup**:
```bash
git clone https://gitlab.inria.fr/fun-team/lora_wur_v2.git
cd lora_wur_v2
idf.py set-target esp32s3
```

2. **Configure Hardware Pins**:
```bash
idf.py menuconfig
# Navigate to your component configuration
# Set GPIO pins according to your hardware
```

3. **Build Project**:
```bash
idf.py build
```

4. **Flash and Monitor**:
```bash
idf.py flash monitor
```

### Build Flags

Add these to your `CMakeLists.txt` for optimization:

```cmake
# Enable radio debug output
target_compile_definitions(${COMPONENT_LIB} PRIVATE 
    CONFIG_LORA_DEBUG=1
)

# Optimize for size
set_property(TARGET ${COMPONENT_LIB} PROPERTY COMPILE_OPTIONS "-Os")
```

---

## 📖 Usage Examples

### Basic LoRa Communication

#### Transmitter Example
```c
#include "sx1261_driver.h"

void app_main(void) {
    // Initialize LoRa
    int16_t result = LoRaBegin(868000000, 14, 3.3, false);
    if (result != ERR_NONE) {
        ESP_LOGE(TAG, "LoRa init failed: %d", result);
        return;
    }
    
    // Configure LoRa parameters
    LoRaConfig(SX126X_LORA_SF7,        // Spreading Factor 7
               SX126X_LORA_BW_125,     // 125 kHz bandwidth  
               SX126X_LORA_CR_4_5,     // Coding Rate 4/5
               8,                       // Preamble length
               0,                       // Variable length
               true,                    // CRC enabled
               false);                  // Standard IQ

    // Send data
    uint8_t message[] = "Hello LoRa!";
    while (1) {
        bool success = LoRaSend(message, sizeof(message), SX126x_TXMODE_SYNC);
        if (success) {
            ESP_LOGI(TAG, "Message sent successfully");
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Send every 5 seconds
    }
}
```

#### Receiver Example
```c
#include "sx1261_driver.h"

void app_main(void) {
    // Initialize with same parameters as transmitter
    LoRaBegin(868000000, 14, 3.3, false);
    LoRaConfig(SX126X_LORA_SF7, SX126X_LORA_BW_125, SX126X_LORA_CR_4_5,
               8, 0, true, false);
    
    // Start receiving
    SetRx(0xFFFFFF); // Continuous RX mode
    
    uint8_t rxBuffer[256];
    while (1) {
        uint8_t rxLen = LoRaReceive(rxBuffer, sizeof(rxBuffer));
        if (rxLen > 0) {
            rxBuffer[rxLen] = '\0'; // Null terminate
            
            // Get signal quality
            int8_t rssi, snr;
            GetPacketStatus(&rssi, &snr);
            
            ESP_LOGI(TAG, "Received: %s (RSSI: %d dBm, SNR: %d dB)", 
                     rxBuffer, rssi, snr);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Wake-up Radio (WuR) Implementation

#### WuR Transmitter
```c
void wur_transmitter_task(void *pvParameters) {
    // Initialize WuR transmitter
    context_t sx_parameters;
    sx1261_init();
    sx126x_reset(&sx_parameters);
    
    Sx1261_InitParam_WUR(&sx_parameters);
    Sx1261_InitFonction_WUR(&sx_parameters);
    
    uint8_t wakeup_signal[10] = {0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 
                                 0x7E, 0x7E, 0x7E, 0x7E, 0x7E};
    
    while (1) {
        ESP_LOGI(TAG, "Sending WuR signal");
        Sx1261_Send_WuR_Signal(&sx_parameters, wakeup_signal, 10);
        
        // Send wake-up signal every 3 seconds
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main(void) {
    xTaskCreate(wur_transmitter_task, "wur_tx", 4096, NULL, 5, NULL);
}
```

#### WuR Receiver
```c
void wur_receiver_task(void *pvParameters) {
    context_t sx_parameters;
    sx1261_init();
    sx126x_reset(&sx_parameters);
    
    Sx1261_InitParam_WUR(&sx_parameters);
    Sx1261_InitFonction_WUR(&sx_parameters);
    
    // Configure duty cycle: 500ms sleep, 500ms receive
    Sx1261_WurMode(&sx_parameters, 500, 500);
    
    sx126x_irq_mask_t irq_status;
    int wake_count = 0;
    
    while (1) {
        sx126x_get_irq_status(&sx_parameters, &irq_status);
        
        if (irq_status & SX126X_IRQ_SYNC_WORD_VALID) {
            wake_count++;
            ESP_LOGI(TAG, "Wake-up signal detected! Count: %d", wake_count);
            
            // Process wake-up event here
            // ... your application logic ...
            
            // Return to WuR mode
            Sx1261_WurMode(&sx_parameters, 500, 500);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    xTaskCreate(wur_receiver_task, "wur_rx", 4096, NULL, 5, NULL);
}
```

---

## 🐛 Troubleshooting

### Common Issues and Solutions

#### 1. SPI Communication Errors
**Symptoms**: `ERR_SPI_TRANSACTION` errors, radio not responding

**Solutions**:
- Check SPI wiring and connections
- Verify GPIO pin assignments in menuconfig
- Ensure proper power supply (3.3V)
- Check BUSY pin connection and pull-down resistor

#### 2. Radio Initialization Failure
**Symptoms**: `ERR_INVALID_MODE` during `LoRaBegin()`

**Solutions**:
```c
// Enable debug output to see detailed logs
LoRaDebugPrint(true);

// Check if radio is detected
uint8_t wk[2];
ReadRegister(SX126X_REG_LORA_SYNC_WORD_MSB, wk, 2);
uint16_t syncWord = (wk[0] << 8) + wk[1];
if (syncWord != SX126X_SYNC_WORD_PUBLIC && syncWord != SX126X_SYNC_WORD_PRIVATE) {
    ESP_LOGE(TAG, "Radio not detected - check connections");
}
```

#### 3. TX/RX State Machine Errors
**Symptoms**: `ERR_INVALID_SETTX_STATE` or `ERR_INVALID_SETRX_STATE`

**Solutions**:
- Ensure proper reset sequence before configuration
- Check BUSY pin timing
- Verify antenna switching logic (TXEN/RXEN pins)

#### 4. Poor Reception Performance
**Symptoms**: Low RSSI, missed packets, high error rate

**Solutions**:
- Check antenna connection and impedance matching
- Verify frequency calibration
- Optimize LoRa parameters (SF, BW, CR)
- Enable RX boosted mode for better sensitivity:
```c
sx126x_cfg_rx_boosted(&context, true);
```

#### 5. WuR Mode Not Working
**Symptoms**: Wake-up signals not detected

**Solutions**:
- Verify GFSK modulation parameters match between TX and RX
- Check sync word configuration
- Ensure proper duty cycle timing
- Verify IRQ configuration for SYNC_WORD_VALID

### Debug Output

Enable detailed logging:
```c
// Enable driver debug output
LoRaDebugPrint(true);

// Check radio status
uint8_t status = GetStatus();
ESP_LOGI(TAG, "Radio status: 0x%02X", status);

// Monitor IRQ status
uint16_t irq = GetIrqStatus();
ESP_LOGI(TAG, "IRQ status: 0x%04X", irq);
```

### Performance Optimization

#### Power Consumption
```c
// Use DC-DC regulator for better efficiency
LoRaBegin(868000000, 14, 3.3, false);

// Reduce TX power when possible
SetTxPower(0); // 0 dBm instead of 14 dBm

// Use sleep mode between operations
sx126x_set_sleep(&context, SX126X_SLEEP_CFG_COLD_START);
```

#### Range Optimization
```c
// Use higher spreading factor for longer range
LoRaConfig(SX126X_LORA_SF12,       // Maximum SF
           SX126X_LORA_BW_125,     // Narrower bandwidth
           SX126X_LORA_CR_4_8,     // Higher coding rate
           8, 0, true, false);

// Enable RX boosted mode
sx126x_cfg_rx_boosted(&context, true);
```

---

## 📊 Performance Specifications

### LoRa Performance

| Parameter | SF7/125kHz | SF9/125kHz | SF12/125kHz |
|-----------|------------|------------|-------------|
| **Data Rate** | 5469 bps | 1758 bps | 293 bps |
| **Sensitivity** | -124 dBm | -135 dBm | -148 dBm |
| **Time on Air** (50 bytes) | 46 ms | 144 ms | 1154 ms |
| **Range** (typical) | 2-5 km | 5-10 km | 10-15 km |

### WuR Performance

| Parameter | Value | Unit |
|-----------|-------|------|
| **GFSK Bit Rate** | 1200 | bps |
| **Frequency Deviation** | 5000 | Hz |
| **Bandwidth** | 14600 | Hz |
| **RX Current** | 4.2 | mA |
| **Sleep Current** | 160 | nA |
| **Wake-up Time** | <500 | µs |

### Power Consumption

| Mode | Current | Conditions |
|------|---------|------------|
| **Sleep** | 160 nA | Cold start |
| **Standby RC** | 1.5 µA | 32 MHz RC |
| **Standby XOSC** | 2.1 µA | 32 MHz XTAL |
| **RX Continuous** | 4.2 mA | LoRa mode |
| **TX +14dBm** | 44 mA | Maximum power |
| **TX 0dBm** | 17 mA | Medium power |

---

## 📄 License

This project is based on the Semtech SX126x driver library and is distributed under the Clear BSD License.

```
Copyright INRIA 2025. All rights reserved.

To be completed
```

---

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Implement your changes with proper documentation
4. Add test cases for new functionality
5. Submit a pull request with detailed description

### Code Style Guidelines

- Follow ESP-IDF coding standards
- Use clear, descriptive function and variable names
- Add comprehensive documentation for public APIs
- Include error handling for all operations
- Use consistent indentation (4 spaces)

---

## 📞 Support

For technical support and questions:

1. Check this documentation first
2. Review the troubleshooting section
3. Enable debug output to diagnose issues
4. Submit issues with detailed logs and hardware configuration

### Useful Resources

- [SX1261/62 Datasheet](https://www.semtech.com/products/wireless-rf/lora-core/sx1261)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [LoRa Alliance Specification](https://lora-alliance.org/resource_hub/lorawan-104-specification-package/)

---

*Last Updated: January 2025*  
*ESP-IDF Version: 5.3.1*  
*Target: ESP32-S3*