# Crazyflie Autonomous Flight Project

A basic autonomous flight system for the Crazyflie platform with comprehensive safety features.

## Quick Start

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Configure your Crazyflie URI in:**
   ```
   config/flight_config.yaml
   ```

3. **Run the basic flight test:**
   ```bash
   python src/basic_flight.py
   ```

## Safety Features

- Emergency landing on connection loss
- Automatic timeout protection  
- Battery monitoring
- Ctrl+C emergency shutdown
- Flight data logging

## Project Structure

The repository is organized as follows: