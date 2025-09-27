# Solar Logger ESP32

Real-time solar panel monitoring system with web dashboard, data logging, and innovative audio feedback for optimal panel positioning.

## Features
- **Real-time Dashboard**: Live voltage, current, power, and daily energy display with auto-updating every second
- **Power Status Gauge**: Visual progress bar showing generation status (Low/Fair/Good/Excellent) as percentage of 175W max
- **Historical Charts**:
  - Hourly chart: Last 24 hours with 12-hour AM/PM format, shows only hours up to current time
  - Daily chart: Last 7 days with date labels showing actual power generation
- **Aim Mode Audio Feedback**: Revolutionary audio tone system that rises with power output to help optimize panel angle
  - Peak power tracking
  - Mute option
  - Real-time tone adjustment based on power output
- **Data Logging**: CSV file stored on SD card with readings every 5 seconds
- **OTA Updates**: Wireless firmware updates protected by password

## Hardware
- Adafruit Feather ESP32
- INA219 current sensor (I2C address: 0x40)
- External 75mV/50A shunt resistor (0.0015Ω, calibration factor: 6.2x)
- Adalogger FeatherWing with PCF8523 RTC and SD card (CS pin: 33)
- Solar panel (175W maximum power)

## Web Interface
Access the dashboard at `http://[ESP32_IP]/` (typically `192.168.1.144`)

### API Endpoints
- `/` - Main dashboard with live metrics
- `/data` - JSON API returning current sensor readings
- `/hourly` - Hourly power chart
- `/daily` - Daily energy chart with 7-day history
- `/charts` - Combined charts view
- `/update` - OTA firmware update endpoint

## Aim Mode Usage
1. Position yourself where you can adjust the solar panel
2. Click "Enable Aim Mode" button on the dashboard
3. Listen to the audio tone - higher pitch means more power
4. Slowly adjust panel angle to find the highest pitch
5. Peak power value is displayed and tracked
6. Use "Mute" to silence audio while keeping tracking active

## Data Format
CSV files are created daily with naming pattern: `solar_log_YYYY-MM-DD.csv`

Columns:
- Timestamp (ISO 8601 format)
- Voltage (V)
- Current (mA)
- Power (mW)
- Energy (Wh)

## Configuration
Key settings in the code:
- WiFi SSID: `weenieNet4`
- OTA Password: `solar123`
- Shunt resistance: 0.0015Ω
- Max current: 50A
- Calibration factor: 6.2x
- Panel max power: 175W

## Build
Compile and upload using Arduino CLI:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 solar_logger.ino
arduino-cli upload --fqbn esp32:esp32:esp32doit-devkit-v1 --port /dev/[your_port] solar_logger.ino
```

Or use Arduino IDE with ESP32 board support installed.

## Recent Improvements
- Fixed Aim Mode audio functionality with proper DOM event handling
- Corrected power calibration calculations (175W maximum)
- Improved JavaScript polling mechanism for reliable dashboard updates
- Added 12-hour time format to hourly chart
- Fixed daily chart to display actual dates on x-axis
- Replaced complex canvas gauge with simple CSS progress bar
- Consolidated duplicate polling functions for better performance

