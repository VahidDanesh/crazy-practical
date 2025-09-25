# 🚁 Hardware Setup & Testing Guide

## **📋 Pre-Flight Checklist**

### **🔧 Hardware Setup**
- [ ] **Crazyflie 2.1** fully charged (green LED)
- [ ] **Flow deck v2** mounted underneath
- [ ] **Multiranger deck** mounted on top
- [ ] **Crazyradio PA** connected to computer
- [ ] Landing pad prepared (3-10cm elevated platform)

### **💻 Software Setup**
- [ ] Virtual environment activated: `source /home/vahid/envs/aerial/bin/activate`
- [ ] In project directory: `cd /home/vahid/repos/crazyflie/crazy-practical`
- [ ] All dependencies installed: `pip install -r requirements.txt`

### **🎯 Landing Pad Setup**
- [ ] Flat elevated surface (books, box, platform)
- [ ] Height: 3-10cm above ground
- [ ] Size: 20-50cm square/circular
- [ ] Contrasting color (different from floor)
- [ ] Stable and won't move when drone lands

### **🚧 Obstacle Avoidance Setup**
- [ ] Clear flight corridors (min 1m wide)
- [ ] No low-hanging objects (min 50cm clearance)
- [ ] Walls/obstacles at least 50cm from flight path
- [ ] Ceiling height min 2m above flight altitude
- [ ] Test obstacles placed strategically to verify avoidance

### **🛡️ Safety Setup**
- [ ] Clear flight area (3m x 3m minimum)
- [ ] No people/pets in flight area
- [ ] Soft landing surface (carpet/mat)
- [ ] Emergency stop ready (spacebar)
- [ ] Battery voltage > 3.5V

---

## **🚀 Step-by-Step Testing**

### **Step 1: Connection Test**
```bash
# Activate environment
source /home/vahid/envs/aerial/bin/activate
cd /home/vahid/repos/crazyflie/crazy-practical

# Test basic connection and flight
python src/hardware_test.py --test basic
```
**Expected result:** Drone takes off, hovers, lands safely

### **Step 2: Sensor Test**
```bash
# Test multiranger sensors
python src/hardware_test.py --test sensors
```
**Expected result:** Drone reads sensors, moves away from obstacles

### **Step 3: Landing Pad Detection**
```bash
# Test full landing pad detection
python src/hardware_test.py --test landing
```
**Expected result:** Drone searches for and lands on elevated pad

### **Step 4: Data Analysis**
```bash
# Plot flight data (replace with actual log file)
python src/flight_data_plotter.py logs/flight_log_YYYY_MM_DD_HH_MM_SS.csv --trajectory --detection --report
```

---

## **⚙️ Configuration Tuning**

### **Peak Detection Parameters** (`config/flight_config.yaml`)

For **thin landing pad (1-3cm)**:
```yaml
peak_detection:
  active_profile: "indoor_precise"
```

For **elevated platform (5-10cm)**:
```yaml
peak_detection:
  active_profile: "outdoor_robust"  # Default
```

For **fast flight/quick detection**:
```yaml
peak_detection:
  active_profile: "fast_flight"
```

### **Flight Parameters**
```yaml
flight:
  default_height: 0.5      # Takeoff height (meters)
  default_velocity: 0.3    # Flight speed (m/s)
  hover_duration: 5.0      # Initial hover time (seconds)
```

### **Safety Parameters**
```yaml
safety:
  max_flight_time: 30.0    # Auto-land timeout (seconds)
  battery_threshold: 3.2   # Emergency land voltage (volts)
```

### **Obstacle Avoidance Parameters**
```yaml
obstacle_avoidance:
  min_safe_distance: 500      # mm - clearance from obstacles
  ceiling_min_distance: 300   # mm - clearance from ceiling  
  search_velocity: 0.15       # m/s - slow speed for safety
  alternative_search_radius: 0.3  # m - backup position search
```

---

## **📊 Data Collection & Analysis**

### **Real-time Monitoring**
During flight, monitor:
- **Battery voltage** (should stay > 3.2V)
- **Position estimates** (X, Y, Z coordinates)
- **Height sensors** (Z-range readings)
- **Detection events** (logged to console)

### **Post-Flight Analysis**
```bash
# Generate comprehensive plots
python src/flight_data_plotter.py logs/latest_flight.csv --trajectory --detection --report --save plots/

# This creates:
# - plots/flight_trajectory.png     (3D flight path)
# - plots/landing_detection.png     (height analysis)
# - plots/flight_report.json        (statistics)
```

### **Key Metrics to Check**
- **Detection accuracy**: How many pad detections vs false positives
- **Center calculation**: How close to actual pad center
- **Flight stability**: Position variance during hover
- **Battery consumption**: Voltage drop during flight

---

## **🐛 Troubleshooting**

### **Connection Issues**
```bash
# Check USB permissions
ls -la /dev/radio*

# Scan for Crazyflie
python -c "import cflib.crtp; cflib.crtp.init_drivers(); from cflib.crtp import scan_interfaces; print(scan_interfaces())"
```

### **Sensor Issues**
- **All sensors read 8000mm**: Check deck connections
- **Noisy readings**: Reduce `threshold` in peak detection
- **No detections**: Lower `min_peak_height` parameter

### **Flight Issues**
- **Unstable flight**: Check battery, reduce velocity
- **Won't take off**: Check battery > 3.5V, ensure clear takeoff area
- **Emergency landing**: Normal safety behavior, check logs

### **Detection Issues**
- **No pad detected**: Check pad height, adjust `min_peak_height`
- **False positives**: Increase `threshold`, check for obstacles
- **Poor center calculation**: Need more border points, fly closer to pad

---

## **🎯 Success Criteria**

### **Basic Flight**: ✅
- Smooth takeoff and landing
- Stable hover at target height
- No emergency landings
- Battery voltage stable

### **Sensor Flight**: ✅
- Multiranger readings < 8000mm
- Movement away from obstacles
- Return to start position
- No collisions

### **Landing Pad Detection**: ✅
- Detection of elevated landing pad
- Center calculation within 15cm
- Successful precision landing
- Confidence score > 0.5

---

## **📈 Next Steps**

After successful hardware testing:
1. **Optimize parameters** based on your specific setup
2. **Test different landing pad types** (size, height, material)
3. **Implement obstacle avoidance** during search
4. **Add return-to-home** functionality
5. **Test full autonomous mission**

---

**🚁 Ready to fly? Run the test suite:**
```bash
python src/hardware_test.py --test all
```
