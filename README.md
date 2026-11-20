
# WiFi-Based Motion Detection using ESP32-S3 and CSI

A WiFi-based motion detection system using Channel State Information (CSI) and ESP-NOW communication. The system detects movement by analysing variations in WiFi signal characteristics without requiring dedicated motion sensors.

It uses a WiFi router as the primary sensing transmitter and multiple ESP32 boards for additional board-to-board sensing.

## Features

- **WiFi CSI Motion Detection:** Detects movement by analysing variations in Channel State Information.
- **Router-Based Sensing:** Uses WiFi packets exchanged with the router to monitor environmental changes.
- **ESP-NOW Communication:** Enables wireless communication between multiple ESP32 boards.
- **Automatic Peer Discovery:** Detects nearby ESP32 boards without manually entering their MAC addresses.
- **Multi-Board Motion Detection:** Independently monitors CSI variations from multiple ESP32 boards.
- **Automatic Calibration:** Establishes a baseline and detection threshold based on the surrounding environment.
- **Adjustable Sensitivity:** Allows motion detection sensitivity to be modified through HTTP endpoints.
- **Live Web Dashboard:** Displays motion status, CSI metrics, signal ratios and peer information.
- **Motion Event Logging:** Records motion start time, duration and peak signal ratio.

## Hardware Requirements

- Seeed Studio XIAO ESP32-S3 or compatible ESP32 board
- WiFi router
- Additional ESP32 boards for ESP-NOW-based sensing
- USB cable for programming and power supply
- Computer for monitoring and configuration

## Software Requirements

- Arduino IDE 2.x
- Arduino-ESP32 Core 3.x
- ESP-IDF compatible CSI and ESP-NOW APIs
- WiFi library
- WebServer library
- ESP-NOW library
- ESP Ping library

## Working Principle

The system uses WiFi Channel State Information (CSI) to identify changes in the wireless environment.

When a person moves within the monitored area, the surrounding wireless signal propagation changes due to reflection, absorption and multipath effects.

These changes influence the CSI measurements received by the ESP32.

The system analyses these variations using statistical signal processing techniques to determine whether motion is present.

Two independent sensing paths are implemented:

### 1. Router-Based CSI Detection

The ESP32 continuously sends ping requests to the WiFi router to generate wireless traffic.

It captures CSI information from the received router packets and processes the signal to identify variations.

Processing steps:

1. CSI data extraction.
2. Subcarrier amplitude calculation.
3. Signal normalisation.
4. Sliding window storage.
5. Standard deviation calculation.
6. Exponential moving average smoothing.
7. Baseline calibration.
8. Threshold-based motion detection.

### 2. ESP-NOW-Based CSI Detection

Multiple ESP32 boards communicate using ESP-NOW.

Each board periodically broadcasts a unique beacon, allowing other boards to discover it automatically.

When a beacon is received, the sender's MAC address is stored in the peer list.

The receiving board then processes CSI measurements from the discovered ESP32 boards independently.

This enables additional board-to-board wireless sensing links.

## Signal Processing

### CSI Amplitude Extraction

The magnitude of each CSI subcarrier is calculated using its real and imaginary components.

A = sqrt(Re² + Im²)

### Signal Normalisation

Each subcarrier amplitude is divided by the average amplitude to reduce the influence of overall signal strength variations.

### Sliding Window

Recent CSI samples are stored in a sliding window.

- Router-based sensing: 50 packets
- ESP-NOW-based sensing: 30 packets

### Standard Deviation

The standard deviation of each subcarrier is calculated to estimate signal variations.

Higher variations can indicate changes in the wireless propagation environment.

### Signal Smoothing

An exponential moving average is applied to reduce sudden fluctuations.

smoothMetric = 0.6 × previousMetric + 0.4 × currentMetric

### Calibration and Threshold

During calibration, the system records the maximum observed signal variation.

The detection threshold is calculated using the calibration baseline and sensitivity multiplier.

Motion is triggered when the smoothed signal metric exceeds the calculated threshold.

## Configuration

Modify the following parameters in the main Arduino sketch:

| Parameter | Description |
|---|---|
| WIFI_SSID | WiFi network name |
| WIFI_PASS | WiFi password |
| NAME | Unique name for each ESP32 |
| PEER_IPS | IP addresses of other boards for dashboard communication |
| NSUB | Number of CSI subcarriers |
| WIN | Router CSI sliding window size |
| EN_WIN | ESP-NOW CSI sliding window size |
| STEP | Signal processing interval |
| HOLD_MS | Motion hold duration |
| CAL_MS | Calibration duration |
| MAXPEERS | Maximum number of discovered peer boards |

### Default Parameters

```cpp
#define NSUB 64
#define WIN 50
#define STEP 5
#define EVMAX 30

#define EN_WIN 30
#define MAXPEERS 8

const uint32_t HOLD_MS = 3000;
const uint32_t CAL_MS = 15000;
```

## Project Structure

```text
ESP32-WiFi-Sensing/
│
├── ESP32-WiFi-Sensing.ino
│
├── webpage.h
│
└── README.md
```

`WiFi_Motion_Detection.ino` contains the main firmware, CSI processing, ESP-NOW communication, motion detection and HTTP server implementation.

`webpage.h` contains the HTML, CSS and JavaScript for the web dashboard.

## Important Functions

| Function | Description |
|---|---|
| `csiCb()` | Main CSI callback that separates router and peer board processing paths. |
| `processPeerPacket()` | Processes CSI data received from other ESP32 boards. |
| `addPeerIfNew()` | Automatically discovers and registers new ESP32 boards. |
| `startPing()` | Generates continuous ping traffic to the WiFi router. |
| `espNowBeaconTask()` | Broadcasts ESP-NOW discovery beacons. |
| `onEspNowRecv()` | Receives ESP-NOW beacons and identifies other boards. |
| `setup()` | Initialises WiFi, CSI, ESP-NOW and the web server. |
| `loop()` | Handles HTTP requests and periodic diagnostic output. |

## Web Server Endpoints

The ESP32 hosts an HTTP server on port 80.

| Endpoint | Description |
|---|---|
| `/` | Loads the web dashboard. |
| `/status` | Returns live motion status and CSI metrics. |
| `/events` | Returns recorded motion events. |
| `/espnow` | Returns discovered peer board information. |
| `/calibrate` | Restarts the calibration process. |
| `/sens` | Adjusts motion detection sensitivity. |

## Calibration

The system performs an automatic calibration process when it starts.

During the default 15-second calibration period, the system establishes the baseline signal variation.

For better results:

- Keep the monitored environment relatively stationary during calibration.
- Avoid unnecessary movement around the sensing area.
- Ensure the ESP32 boards have stable WiFi connectivity.
- Recalibrate when the environment or board placement changes significantly.

## Applications

- Contactless indoor motion detection
- Smart home automation
- Room occupancy monitoring
- Wireless sensing and IoT applications
- Multi-node environmental monitoring
- Experimental WiFi-based human movement sensing

## Limitations

- Motion detection depends on the wireless environment.
- Furniture, moving objects and environmental changes can influence CSI measurements.
- Detection performance depends on board placement, WiFi conditions and calibration.
- The system detects signal disturbances and does not independently identify whether the disturbance was caused by a human.
- Multiple sensing links are processed independently; they are not currently combined into a unified occupancy or motion map.

## Future Improvements

- Multi-node sensor fusion.
- Direction-of-motion estimation.
- Human presence detection.
- Improved noise filtering and adaptive thresholds.
- Room-level motion mapping.
- Machine learning-based activity classification.
- Integration with smart home automation platforms.

## License

This project is intended for educational, research and experimental purposes.

Feel free to modify and extend the implementation for your own WiFi sensing applications.
