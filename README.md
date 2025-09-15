# ESP32-S3 Wubble Protocol Implementation Documentation
## ESP-IDF 5.3.1

## Table of Contents
1. [System Overview](#1-system-overview)
2. [Architecture](#2-architecture)
3. [Block Diagrams](#3-block-diagrams)
4. [Sequence Diagrams](#4-sequence-diagrams)
5. [Flowcharts](#5-flowcharts)
6. [Component Details](#6-component-details)
7. [Configuration](#7-configuration)
8. [API Reference](#8-api-reference)
9. [State Machine](#9-state-machine)
10. [Power Management](#10-power-management)

---

## 1. System Overview

The Wubble protocol is a hybrid wake-up and discovery mechanism designed for ultra-low power IoT devices using ESP32-S3 with ESP-IDF 5.3.1.

### Key Features
- **Wake-up Radio (WUR)**: SX1261 LoRa transceiver for ultra-low power wake-up
- **BLE Neighbor Discovery**: NimBLE stack for discovering nearby devices (TDV - Table De Voisinage)
- **Multi-role State Machine**: Initiator, Listener, Announcer, Voyager states
- **Deep Sleep Optimization**: RTC memory retention and GPIO wake-up
- **Dual Communication**: 868 MHz WUR + 2.4 GHz BLE

### System Specifications
- **MCU**: ESP32-S3 (Dual-core Xtensa LX7 @ 160 MHz)
- **WUR Radio**: SX1261 (868 MHz, GFSK modulation)
- **BLE**: NimBLE stack (BLE 5.0 compatible)
- **Power States**: Active, Light Sleep, Deep Sleep
- **Wake Sources**: GPIO (EXT1), WUR signal

---

## 2. Architecture

### 2.1 Layered Architecture

```mermaid
graph TB
    subgraph "Application Layer"
        APP[Application Logic]
        WUBBLE[Wubble Protocol Manager]
        SM[State Machine Controller]
    end
    
    subgraph "Protocol Layer"
        TDV[TDV Manager<br/>Table De Voisinage]
        WUR_PROTO[WUR Protocol]
        BLE_ADV[BLE Advertisement]
        BLE_SCAN[BLE Scanner]
    end
    
    subgraph "Communication Layer"
        NIMBLE[NimBLE Stack]
        SX1261_DRV[SX1261 Driver]
    end
    
    subgraph "HAL - Hardware Abstraction Layer"
        SPI_HAL[SPI HAL]
        GPIO_HAL[GPIO HAL]
        TIMER_HAL[Timer HAL]
        BT_CTRL[BT Controller]
    end
    
    subgraph "System Services"
        PM[Power Manager]
        DS[Deep Sleep Controller]
        RTC_MEM[RTC Memory]
        ISR[Interrupt Service]
    end
    
    subgraph "Hardware Layer"
        ESP32S3[ESP32-S3 SoC]
        SX1261_HW[SX1261 Radio]
        BLE_HW[BLE Radio]
    end
    
    APP --> WUBBLE
    WUBBLE --> SM
    SM --> TDV
    SM --> WUR_PROTO
    TDV --> BLE_ADV
    TDV --> BLE_SCAN
    WUR_PROTO --> SX1261_DRV
    BLE_ADV --> NIMBLE
    BLE_SCAN --> NIMBLE
    SX1261_DRV --> SPI_HAL
    SX1261_DRV --> GPIO_HAL
    NIMBLE --> BT_CTRL
    SPI_HAL --> ESP32S3
    GPIO_HAL --> ESP32S3
    TIMER_HAL --> ESP32S3
    BT_CTRL --> BLE_HW
    ESP32S3 --> SX1261_HW
    PM --> DS
    DS --> RTC_MEM
    GPIO_HAL --> ISR
```

---

## 3. Block Diagrams

### 3.1 Hardware Block Diagram

```mermaid
flowchart TD
    esp["ESP32-S3<br/>160MHz Dual Core<br/>512KB SRAM"]
    sx["SX1261<br/>868MHz<br/>WUR Radio"]
    ble["BLE 5.0<br/>2.4GHz<br/>Radio"]
    pwr["Power<br/>Management<br/>Unit"]
    
    spi["SPI2 Bus<br/>2MHz"]
    gpio["GPIO Control"]
    
    spi_pins["MISO: GPIO11<br/>MOSI: GPIO10<br/>SCLK: GPIO9"]
    ctrl_pins["NSS: GPIO7<br/>RST: GPIO1<br/>BUSY: GPIO5<br/>DIO1: GPIO6"]
    wake["Wake GPIO33<br/>(EXT1)"]
    rtc["RTC Memory<br/>16KB"]
    
    pwr --> esp
    rtc --> esp
    esp --> spi
    esp --> gpio
    esp --> ble
    spi --> sx
    gpio --> sx
    esp --> wake
    spi_pins -.-> spi
    ctrl_pins -.-> gpio
    
    classDef espStyle fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    classDef sxStyle fill:#fff9c4,stroke:#f57f17,stroke-width:2px
    classDef bleStyle fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    classDef pwrStyle fill:#ffebee,stroke:#b71c1c,stroke-width:2px
    classDef rtcStyle fill:#e8f5e9,stroke:#1b5e20,stroke-width:2px
    classDef pinStyle fill:#f5f5f5,stroke:#424242,stroke-width:1px
    
    class esp espStyle
    class sx sxStyle
    class ble bleStyle
    class pwr pwrStyle
    class rtc rtcStyle
    class spi_pins,ctrl_pins,wake pinStyle
```

### 3.2 Software Component Diagram

```mermaid
graph LR
    subgraph "Application Components"
        MAIN[main.c<br/>Entry Point]
        WUBBLE_C[wubble.c<br/>Protocol Logic]
    end
    
    subgraph "Communication Components"
        SX1261_DRV[sx1261_driver.c<br/>WUR Driver]
        NIMBLE_ADV[Nimble_adv.c<br/>BLE Manager]
    end
    
    subgraph "Configuration"
        CONFIG[config.h<br/>System Config]
        SDK[sdkconfig<br/>ESP-IDF Config]
    end
    
    subgraph "Data Structures"
        NT[Neighbour Table<br/>Max: 128 entries]
        CTX[Context Structure<br/>Radio Parameters]
    end
    
    MAIN --> WUBBLE_C
    MAIN --> SX1261_DRV
    WUBBLE_C --> NIMBLE_ADV
    WUBBLE_C --> SX1261_DRV
    WUBBLE_C --> NT
    SX1261_DRV --> CTX
    NIMBLE_ADV --> NT
    CONFIG --> ALL[All Components]
    SDK --> ALL
    
    style MAIN fill:#bbdefb
    style WUBBLE_C fill:#c8e6c9
    style SX1261_DRV fill:#ffe0b2
    style NIMBLE_ADV fill:#f8bbd0
```

---

## 4. Sequence Diagrams

### 4.1 System Initialization Sequence

```mermaid
sequenceDiagram
    participant Main
    participant SPI
    participant SX1261
    participant BLE
    participant PM as Power Manager
    
    Main->>Main: app_main()
    Main->>SPI: spi_bus_initialize(SPI2_HOST)
    SPI-->>Main: ESP_OK
    
    Main->>SX1261: sx1261_init()
    activate SX1261
    SX1261->>SX1261: Configure GPIOs
    SX1261->>SX1261: gpio_set_direction()
    SX1261->>SX1261: Reset sequence
    SX1261->>SPI: spi_bus_add_device()
    SPI-->>SX1261: handle
    SX1261->>SX1261: sx126x_reset()
    deactivate SX1261
    SX1261-->>Main: Initialized
    
    Main->>SX1261: Sx1261_InitParam_WUR()
    activate SX1261
    SX1261->>SX1261: Set sync_word[8]
    SX1261->>SX1261: Set frequency: 868MHz
    SX1261->>SX1261: Set power: 14dBm
    SX1261->>SX1261: Set modulation params
    deactivate SX1261
    
    Main->>SX1261: Sx1261_InitFonction_WUR()
    activate SX1261
    SX1261->>SX1261: sx126x_set_pkt_type(GFSK)
    SX1261->>SX1261: sx126x_set_tx_params()
    SX1261->>SX1261: sx126x_set_rf_freq()
    SX1261->>SX1261: sx126x_set_standby()
    deactivate SX1261
    
    Main->>BLE: ble_init()
    activate BLE
    BLE->>BLE: nvs_flash_init()
    BLE->>BLE: esp_nimble_hci_init()
    BLE->>BLE: nimble_port_init()
    BLE->>BLE: nimble_port_freertos_init()
    deactivate BLE
    
    Main->>PM: Configure deep sleep
    PM->>PM: esp_sleep_enable_ext1_wakeup()
    PM->>PM: gpio_deep_sleep_hold_en()
    
    Main->>Main: Enter main loop
```

### 4.2 Wake-up Radio (WUR) Communication Sequence

```mermaid
sequenceDiagram
    participant Init as Initiator
    participant SX_I as SX1261_Init
    participant Air as 868MHz
    participant SX_L as SX1261_Listen
    participant List as Listener
    
    Init->>Init: Backoff timer expires
    Init->>Init: Role = Initiator
    
    Init->>SX_I: send_WUR()
    activate SX_I
    SX_I->>SX_I: sx126x_set_dio_irq_params()
    SX_I->>SX_I: sx126x_clear_irq_status()
    SX_I->>SX_I: sx126x_write_buffer(data)
    SX_I->>SX_I: sx126x_set_tx(2000ms)
    SX_I->>Air: WUR Signal (GFSK, 50kbps)
    deactivate SX_I
    
    Air->>SX_L: Preamble detected
    activate SX_L
    SX_L->>SX_L: IRQ_SYNC_WORD_VALID
    SX_L->>List: Wake interrupt
    List->>List: Wake from deep sleep
    List->>List: Check wake reason
    List->>List: Role = Listener
    deactivate SX_L
    
    Init->>Init: Role = Listener
    Init->>Init: Start TDV process
    List->>List: Start TDV process
```

### 4.3 BLE Neighbor Discovery (TDV) Sequence

```mermaid
sequenceDiagram
    participant Dev as Device
    participant BLE as BLE Stack
    participant Scan as Scanner
    participant Adv as Advertiser
    participant NT as Neighbour Table
    
    Dev->>Dev: make_tdv()
    Dev->>BLE: ble_init()
    BLE-->>Dev: Initialized
    
    Dev->>Scan: BLE_scan()
    activate Scan
    Scan->>Scan: ble_gap_disc(FOREVER)
    Scan->>Scan: Set scan params
    
    loop For MAX1 duration (5s)
        Adv->>Scan: BLE Advertisement
        Scan->>Scan: ble_gap_event(DISC)
        Scan->>Scan: Parse UUID
        alt UUID matches
            Scan->>Scan: receive_gf_iot_beacon()
            Scan->>NT: Add to table
            NT->>NT: Check duplicates
            NT-->>Scan: Added/Skipped
        end
    end
    
    Scan->>Scan: Timer expires
    deactivate Scan
    Scan->>Dev: scanner_stop()
    
    Dev->>NT: get_neighbour_table()
    NT-->>Dev: Table with K neighbours
    
    alt No neighbours
        Dev->>Dev: Role = Voyager
    else Has neighbours
        Dev->>Dev: Role = Announcer
        Dev->>Adv: send_tdv()
        activate Adv
        Adv->>Adv: type = IOTBEACON
        Adv->>Adv: ble_server_start()
        Adv->>Adv: Advertise for 3s
        deactivate Adv
    end
```

### 4.4 Deep Sleep and Wake-up Sequence

```mermaid
sequenceDiagram
    participant App
    participant WUR as WUR Module
    participant PM as Power Manager
    participant GPIO
    participant RTC as RTC Memory
    
    App->>RTC: Save state variables
    RTC->>RTC: wake_counter
    RTC->>RTC: wubble_state
    
    App->>WUR: Configure WUR for wake
    WUR->>WUR: Sx1261_WurMode()
    WUR->>WUR: Set duty cycle
    WUR->>WUR: sleepTime: 999ms
    WUR->>WUR: rxTime: 1ms
    
    App->>GPIO: Configure wake sources
    GPIO->>GPIO: gpio_config(GPIO33)
    GPIO->>GPIO: gpio_deep_sleep_hold_en()
    
    App->>PM: esp_sleep_enable_ext1_wakeup()
    PM->>PM: Configure EXT1 wake
    
    App->>PM: esp_deep_sleep_start()
    PM->>PM: Enter deep sleep
    Note over PM: System in deep sleep<br/>Current: ~10µA
    
    alt WUR Signal Received
        WUR->>GPIO: DIO1 interrupt
        GPIO->>PM: Wake trigger
    else GPIO33 External Wake
        GPIO->>PM: EXT1 wake trigger
    end
    
    PM->>PM: Wake sequence
    PM->>RTC: Restore variables
    RTC-->>App: Restored state
    
    App->>App: Check wake reason
    App->>App: esp_sleep_get_wakeup_cause()
    
    alt ESP_SLEEP_WAKEUP_EXT1
        App->>App: Handle GPIO wake
        App->>RTC: Increment wake_counter
    else ESP_SLEEP_WAKEUP_UNDEFINED
        App->>App: Power-on reset
        App->>RTC: Reset counters
    end
```

---

## 5. Flowcharts

### 5.1 Main Application Flow

```mermaid
flowchart TD
    START([Power On/Reset])
    START --> INIT[Initialize System]
    
    INIT --> SPI_INIT[Initialize SPI Bus]
    SPI_INIT --> WUR_INIT[Initialize SX1261 WUR]
    WUR_INIT --> CHECK_WAKE{Check Wake Reason}
    
    CHECK_WAKE -->|Power On| RESET_COUNTERS[Reset RTC Counters]
    CHECK_WAKE -->|EXT1 Wake| INC_COUNTER[Increment Wake Counter]
    CHECK_WAKE -->|WUR Wake| WUR_HANDLER[Handle WUR Event]
    
    RESET_COUNTERS --> CONFIG_WAKE[Configure Wake Sources]
    INC_COUNTER --> CONFIG_WAKE
    WUR_HANDLER --> CONFIG_WAKE
    
    CONFIG_WAKE --> LISTEN_WUR[Listen for WUR Signal]
    LISTEN_WUR --> DEEP_SLEEP[Enter Deep Sleep]
    
    DEEP_SLEEP -->|Wake Event| CHECK_WAKE
```

### 5.2 Wubble Protocol State Machine Flow

```mermaid
flowchart TD
    START([Start Wubble])
    START --> INIT_TIMERS[Initialize Timers]
    
    INIT_TIMERS --> VOYAGER[State: Voyager<br/>Wait for trigger]
    
    VOYAGER --> CHECK_TRIGGER{Trigger?}
    CHECK_TRIGGER -->|Backoff Timer| INITIATOR[State: Initiator]
    CHECK_TRIGGER -->|WUR Received| LISTENER[State: Listener]
    CHECK_TRIGGER -->|No| VOYAGER
    
    INITIATOR --> SEND_WUR[Send WUR Signal]
    SEND_WUR --> LISTENER
    
    LISTENER --> MAKE_TDV[Start TDV Process]
    MAKE_TDV --> BLE_SCAN[BLE Scan for 5s]
    BLE_SCAN --> CHECK_NEIGHBORS{Neighbours<br/>Found?}
    
    CHECK_NEIGHBORS -->|No| VOYAGER
    CHECK_NEIGHBORS -->|Yes| ANNOUNCER[State: Announcer]
    
    ANNOUNCER --> IDENTIFY[Identify Entity R]
    IDENTIFY --> SEND_TDV[Send BLE Advertisement]
    SEND_TDV --> ADV_3S[Advertise for 3s]
    ADV_3S --> VOYAGER
```

### 5.3 BLE Scanning Flow

```mermaid
flowchart TD
    START(["BLE_scan"])
    START --> INIT["Initialize BLE Stack"]
    
    INIT --> CHECK_ACTIVE{"Scan Active?"}
    CHECK_ACTIVE -->|"Yes"| LOG_ACTIVE["Log: Already Scanning"]
    CHECK_ACTIVE -->|"No"| RESET_COUNT["Reset Device Count"]
    
    LOG_ACTIVE --> RETURN(["Return"])
    
    RESET_COUNT --> START_SCAN["Start BLE Scan<br>ble_gap_disc()"]
    START_SCAN --> CHECK_START{"Started OK?"}
    
    CHECK_START -->|"Error"| LOG_ERROR["Log Error"]
    CHECK_START -->|"Success"| LOG_SUCCESS["Log: Scanning Started"]
    
    LOG_ERROR --> RETURN
    LOG_SUCCESS --> SCAN_LOOP["Scanning..."]
    
    SCAN_LOOP --> EVENT{"BLE Event"}
    
    EVENT -->|"Discovery"| PARSE["Parse Advertisement"]
    EVENT -->|"Timeout"| STOP_SCAN["Stop Scanning"]
    EVENT -->|"Other"| SCAN_LOOP
    
    PARSE --> CHECK_UUID{"UUID Match?"}
    CHECK_UUID -->|"No"| SCAN_LOOP
    CHECK_UUID -->|"Yes"| CHECK_TABLE{"In Table?"}
    
    CHECK_TABLE -->|"Yes"| SCAN_LOOP
    CHECK_TABLE -->|"No"| ADD_NEIGHBOR["Add to Neighbour Table"]
    
    ADD_NEIGHBOR --> CHECK_FULL{"Table Full?"}
    CHECK_FULL -->|"No"| SCAN_LOOP
    CHECK_FULL -->|"Yes"| TABLE_FULL["Set Table Full Flag"]
    
    TABLE_FULL --> STOP_SCAN
    STOP_SCAN --> RETURN
```

### 5.4 WUR Reception Flow

```mermaid
flowchart TD
    START([WUR RX Mode])
    START --> CONFIG[Configure RX Duty Cycle]
    
    CONFIG --> SET_PARAMS[Set Parameters:<br/>Sleep: 999ms<br/>RX: 1ms]
    SET_PARAMS --> DUTY_CYCLE[Enter Duty Cycle Mode]
    
    DUTY_CYCLE --> SLEEP_PHASE[Sleep Phase<br/>999ms]
    SLEEP_PHASE --> RX_PHASE[RX Phase<br/>1ms]
    
    RX_PHASE --> CHECK_SIGNAL{Signal<br/>Detected?}
    CHECK_SIGNAL -->|No| SLEEP_PHASE
    CHECK_SIGNAL -->|Yes| CHECK_PREAMBLE{Preamble<br/>Valid?}
    
    CHECK_PREAMBLE -->|No| SLEEP_PHASE
    CHECK_PREAMBLE -->|Yes| CHECK_SYNC{Sync Word<br/>Valid?}
    
    CHECK_SYNC -->|No| SLEEP_PHASE
    CHECK_SYNC -->|Yes| GEN_IRQ[Generate IRQ]
    
    GEN_IRQ --> SET_FLAG[Set Wake Flag]
    SET_FLAG --> WAKE_SYSTEM[Wake System]
    
    WAKE_SYSTEM --> HANDLE[Handle WUR Event]
    HANDLE --> CLEAR_FLAG[Clear WUR Flag]
    CLEAR_FLAG --> RETURN([Return to App])
```

---

## 6. Component Details

### 6.1 Main Component (main.c)

```markdown
**Purpose**: Application entry point and deep sleep management

**Key Functions**:
- `app_main()`: Initialize system and enter deep sleep loop
- Wake source configuration (GPIO33 as EXT1)
- RTC memory management for state persistence

**RTC Variables**:
- `wake_counter`: Number of wake-ups since power-on
- `wubble_state`: Current protocol state (0-3)
```

### 6.2 Wubble Component

```markdown
**File**: components/wubble/wubble.c

**Purpose**: Wubble protocol state machine implementation

**Key Functions**:
- `wubble()`: Main protocol loop
- `make_tdv()`: Create neighbor table via BLE scanning
- `send_tdv()`: Broadcast neighbor information
- `timer_callback()`: Handle protocol timers

**States**:
- 0: Listener - Waiting for commands
- 1: Initiator - Sending WUR signals
- 2: Announcer - Broadcasting neighbor info
- 3: Voyager - Idle/waiting state

**Timers**:
- Backoff Timer: Random delay (0-10s)
- Timer-L1: TDV scan duration (5s)
- Timer-L2: Maximum wait time (4s)
```

### 6.3 SX1261 Driver Component

```markdown
**File**: components/wubble/sx1261_driver.c

**Purpose**: Low-level driver for SX1261 WUR transceiver

**Key Functions**:
- `sx1261_init()`: Initialize radio hardware
- `Sx1261_InitParam_WUR()`: Configure WUR parameters
- `Sx1261_WurMode()`: Set duty-cycled receive mode
- `Sx1261_Send_WuR_Signal()`: Transmit wake-up beacon
- `Listen_WUR()`: Listen for incoming WUR signals

**Configuration**:
- Frequency: 868 MHz
- Modulation: GFSK
- Bit Rate: 50 kbps
- Bandwidth: 312 kHz
- Power: +14 dBm
- Preamble: 50000 bits
- Sync Word: 8 bytes (0x7E pattern)
```

### 6.4 NimBLE Advertisement Component

```markdown
**File**: components/Nimble_adv/Nimble_adv.c

**Purpose**: BLE advertisement and scanning management

**Key Functions**:
- `ble_init()`: Initialize NimBLE stack
- `BLE_scan()`: Start neighbor discovery scan
- `send_tdv()`: Send BLE advertisement
- `receive_gf_iot_beacon()`: Process received beacons
- `get_neighbour_table()`: Retrieve discovered neighbors

**BLE Configuration**:
- Stack: NimBLE
- Mode: Observer + Broadcaster
- UUID: 0x850c7d3c-d435-4662-a61f-25627693ddac
- Advertisement Types:
  - IBEACON: Standard iBeacon
  - IOTBEACON: Custom IoT beacon
  - APPBEACON: Application beacon

**Neighbor Table**:
- Maximum entries: 128
- Entry format: [ID(4 bytes) + States(1 byte)]
```

---

## 7. Configuration

### 7.1 Hardware Pin Configuration (config.h)

```c
// SPI Configuration
#define PIN_SPI_SCLK    9
#define PIN_SPI_MOSI    10
#define PIN_SPI_MISO    11
#define USED_SPI_HOST   SPI2_HOST

// SX1261 WUR Pins
#define CONFIG_NSS_GPIO   7   // Chip Select
#define CONFIG_RST_GPIO   1   // Reset
#define CONFIG_BUSY_GPIO  5   // Busy indicator
#define CONFIG_DIO1_GPIO  6   // Digital IO 1

// Wake-up Configuration
#define WAKEUP_GPIO      GPIO_NUM_33  // EXT1 wake source
```

### 7.2 WUR Parameters (config.h)

```c
// WUR Timing
#define DEEP_SLEEP_SLEEP_PERIOD_MS  999
#define DEEP_SLEEP_RX_PERIOD_MS      1
#define TX_TIME_MS                   2000

// WUR Radio Configuration
#define SYNCWORD_LENGTH              8
#define PREAMBLE_LENGTH_IN_BIT       50000
#define BR_IN_BPS                    50000   // Bit rate
#define FDEV_IN_HZ                   23848   // Frequency deviation
#define BW_DSB_PARAM                 SX126X_GFSK_BW_312000
#define POWER                        14      // +14 dBm
```

### 7.3 BLE Configuration (config.h)

```c
// BLE Parameters
#define ID_BLE_SIZE                     4
#define MAX_IOT_IDS_IN_NEIGHBOUR_TABLE  128
#define ID_IOT                          {0x66, 0x66, 0x66, 0x66}

// Timing
#define BLE_SCAN_ADV_MAX_DURATION_S     2
#define ADV_DURATION_DSCV_S             4

// UUID
#define GF_BEACON_UUID  {0x85, 0x0c, 0x7d, 0x3c, 0xd4, 0x35, \
                        0x46, 0x62, 0xa6, 0x1f, 0x25, 0x62, \
                        0x76, 0x93, 0xdd, 0xac}
```

### 7.4 Wubble Protocol Timers (config.h)

```c
// Wubble Timers
#define K      10        // Neighbor threshold
#define MAX    10000000  // Max backoff (10s in µs)
#define MAX1   5000000   // TDV scan duration (5s in µs)
#define MAX2   5000000   // Max wait time (5s in µs)
```

---

## 8. API Reference

### 8.1 Wubble Protocol API

```c
// Main protocol function
void wubble(void);

// TDV Management
void make_tdv(void);
void send_tdv(void);
neighbour_table_t get_neighbour_table(void);

// Timer callback
void timer_callback(int arg);
```

### 8.2 SX1261 WUR API

```c
// Initialization
void sx1261_init(void);
void Sx1261_InitParam_WUR(context_t *context);
void Sx1261_InitFonction_WUR(context_t *context);

// WUR Operations
void send_WUR(void);
void Listen_WUR(void);
void Sx1261_WurMode(context_t *context, uint32_t sleepTime, uint32_t rxTime);
void Sx1261_Send_WuR_Signal(context_t *context, uint8_t* data, uint8_t length);
void Sx1261_RstWurFlag(context_t *context);

// Low-level SX126x functions
sx126x_status_t sx126x_set_sleep(const void* context, const sx126x_sleep_cfgs_t cfg);
sx126x_status_t sx126x_set_standby(const void* context, const sx126x_standby_cfg_t cfg);
sx126x_status_t sx126x_set_rx_duty_cycle(const void* context, 
                                         const uint32_t rx_time_in_ms,
                                         const uint32_t sleep_time_in_ms);
```

### 8.3 BLE API

```c
// Initialization
void ble_init(void);

// Server Operations
void ble_server_start(void);
void ble_server_stop(void);

// Scanning
void BLE_scan(void);
void scanner_stop(void);

// Advertisement
void send_BLE_ID(void);

// Neighbor Management
void receive_gf_iot_beacon(uint8_t* gf_beacon);
bool is_iot_id_in_ble_table(size_t table_size, uint8_t* value);
void print_ble_table(void);
```

### 8.4 Data Structures

```c
// Neighbor entry
typedef struct {
    uint8_t neighbour[4];      // Device ID
    uint8_t states_neighbour;  // Device state
} neighbour_t;

// Neighbor table
typedef struct {
    size_t size;
    neighbour_t neighbours[MAX_NEIGHBOURS];
} neighbour_table_t;

// Radio context
typedef struct context_s {
    uint32_t frequency;
    uint8_t power;
    uint8_t sync_word[8];
    sx126x_mod_params_gfsk_t mode_parameters;
    sx126x_pkt_params_gfsk_t pkt_parameters;
} context_t;

// BLE beacon structures
typedef struct {
    adv_head_t adv_head;
    esp_ble_uuid_t uuid;
    uint8_t iot_id[ID_BLE_SIZE];
    uint8_t both_states;
    uint8_t relevance;
    uint8_t x;
} __attribute__((packed)) goodfloow_iot_beacon_t;
```

---

## 9. State Machine

### 9.1 State Transitions

```mermaid
stateDiagram-v2
    [*] --> Voyager: Power On
    
    Voyager --> Initiator: Backoff Timer Expires
    Voyager --> Listener: WUR Signal Received
    
    Initiator --> Listener: After Sending WUR
    
    Listener --> Voyager: No Neighbors Found
    Listener --> Announcer: Neighbors Found
    
    Announcer --> Voyager: After Broadcasting
    
    note right of Voyager
        Idle state
        Waiting for trigger
        Deep sleep enabled
    end note
    
    note right of Initiator
        Sends WUR signal
        Wakes up neighbors
        Duration: ~2s
    end note
    
    note right of Listener
        Performs TDV scan
        BLE discovery
        Duration: 5s
    end note
    
    note right of Announcer
        Broadcasts neighbor info
        BLE advertisement
        Duration: 3s
    end note
```

To be continued