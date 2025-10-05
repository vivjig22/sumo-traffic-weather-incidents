# SUMO Traffic Simulation with Dynamic Weather and Incident Management

## Overview

This project implements an advanced traffic simulation system using SUMO (Simulation of Urban Mobility) with real-time weather conditions and traffic incident integration. The simulation focuses on the Wolfsburg road network and analyzes how weather-related friction changes and traffic incidents affect vehicle speeds and routing behavior.

## Features

### Core Simulation Capabilities
- **Dynamic Weather Integration**: Fetches real-time weather data from Open-Meteo API
- **Friction Modeling**: Applies weather-based friction coefficients to road surfaces
- **Traffic Incident Management**: Real-time integration with German Autobahn traffic services
- **Intelligent Routing**: Dynamic re-routing based on incidents and road conditions
- **High-Precision Timing**: Second-level weather updates and minute-level forecasting

### Weather System
- **Real-time Weather Data**: Integration with Open-Meteo weather API
- **Friction Mapping**: Comprehensive weather-to-friction coefficient mapping
- **Weather Conditions Supported**:
  - Clear sky (friction: 1.0)
  - Rain conditions (0.5 - 0.7 friction)
  - Snow conditions (0.3 - 0.55 friction)
  - Fog conditions (0.65 - 0.8 friction)
  - Extreme weather (0.25 - 0.45 friction)

### Traffic Incident Integration
- **Real-time Data**: Fetches incidents from German Autobahn API (A2, A39 corridors)
- **Incident Classification**:
  - **High Severity**: Road closures, complete blockages
  - **Medium Severity**: Construction work, lane restrictions
  - **Low Severity**: Minor incidents, warnings
- **Smart Re-routing**: Severity-based penalties for affected road segments
- **Duplicate Filtering**: Removes nearby duplicate incidents (< 100m proximity)

### Analysis and Visualization
- **Speed Analysis**: Real-time vehicle speed tracking and plotting
- **Weather Correlation**: Visual indicators for weather changes during simulation
- **Performance Metrics**: Average and maximum speed calculations
- **Export Capabilities**: PNG plot generation for analysis reports

## Technical Architecture

### Key Components

1. **Weather Module** (`get_diverse_friction_from_weather()`)
   - Fetches current weather conditions
   - Converts weather codes to friction coefficients
   - Supports both real-time and test modes

2. **Incident Management** (`get_incidents()`)
   - Retrieves traffic incidents from Autobahn API
   - Filters incidents by geographic relevance
   - Maps incidents to affected road segments

3. **Route Processing** (`preprocess_routes_with_duarouter()`)
   - Generates initial routes using SUMO's duarouter
   - Applies incident-based penalties for re-routing
   - Handles route optimization with traffic conditions

4. **Simulation Engine** (`run_simulation()`)
   - Manages SUMO simulation execution
   - Applies real-time weather and friction updates
   - Handles dynamic incident-based re-routing

### File Structure
```
├── SUMO.py                    # Main simulation script
├── wolfsburg.net.xml          # SUMO network file (Wolfsburg)
├── wolfsburg.rou.xml          # Initial route definitions
├── wolfsburg.routed.xml       # Processed routes with vehicle types
├── wolfsburg.sumocfg          # SUMO configuration file
├── main_vehicle_speed_analysis_*.png  # Analysis output plots
└── SUMO_Backup_*.py          # Backup versions of the simulation
```

## Installation & Setup

### Prerequisites
- **SUMO** (Simulation of Urban Mobility) - Install from [eclipse.org/sumo](https://eclipse.org/sumo/)
- **Python 3.7+** with required packages
- **SUMO_HOME environment variable** properly configured

### Required Dependencies
Install the required Python packages:
```bash
pip install requests matplotlib pandas pyproj
```

### Network Files (Required but Not Included)

 **Important**: This simulation requires Wolfsburg network files that are **not included in this repository** due to GitHub's file size limit.

#### Required Files:
1. **`wolfsburg.net.xml`** - SUMO network file - **Must be obtained separately**
2. **`wolfsburg.rou.xml`** - Initial route definitions - **Included in repository**

#### File Naming Requirements:

**Important**: The simulation expects specific file names:
- `wolfsburg.net.xml` - Network file (required)
- `wolfsburg.rou.xml` - Route file (optional - has automatic fallback)

**If you generate your own route file**, ensure it's named `wolfsburg.rou.xml` and placed in the project root.

**Fallback Behavior**: If `wolfsburg.rou.xml` is missing or incompatible, the simulation automatically generates basic routes and continues running.

#### Complete Setup Process:

**Step 1: Generate Network File**
```bash
# Option 1: Generate from OpenStreetMap (Recommended)
python "%SUMO_HOME%/tools/osmWebWizard.py"
# Select Wolfsburg area and export as wolfsburg.net.xml

# Option 2: Use netconvert with OSM data
netconvert --osm-files wolfsburg.osm -o wolfsburg.net.xml
```

**Step 2: Generate Initial Route File (Optional)**

**Windows Command Prompt:**
```cmd
python "%SUMO_HOME%/tools/randomTrips.py" -n wolfsburg.net.xml -o wolfsburg.rou.xml --min-distance 30000 --max-distance 80000 -e 20 -p 5 --fringe-factor 1000 -s 43
```

**Windows PowerShell:**
```powershell
python "$env:SUMO_HOME/tools/randomTrips.py" -n wolfsburg.net.xml -o wolfsburg.rou.xml --min-distance 30000 --max-distance 80000 -e 20 -p 5 --fringe-factor 1000 -s 43
```

**Linux/macOS:**
```bash
python "$SUMO_HOME/tools/randomTrips.py" -n wolfsburg.net.xml -o wolfsburg.rou.xml --min-distance 30000 --max-distance 80000 -e 20 -p 5 --fringe-factor 1000 -s 43
```

**Alternative (if SUMO_HOME not set):**
Replace with your actual SUMO installation path:
```bash
# Example for Windows (typical installation path)
python "C:\Program Files (x86)\Eclipse\Sumo\tools\randomTrips.py" -n wolfsburg.net.xml -o wolfsburg.rou.xml --min-distance 5000 --max-distance 30000 -e 20 -p 5 --fringe-factor 1000 -s 43

# Example for Linux (typical installation path)
python "/usr/share/sumo/tools/randomTrips.py" -n wolfsburg.net.xml -o wolfsburg.rou.xml --min-distance 5000 --max-distance 30000 -e 20 -p 5 --fringe-factor 1000 -s 43
```

**Parameters Explained:**
- `--min-distance 5000`: Minimum trip distance (5km)
- `--max-distance 30000`: Maximum trip distance (30km) 
- `-e 20`: Number of trips to generate (20)
- `-p 5`: Probability of departure (5%)
- `--fringe-factor 1000`: Prefer routes starting/ending at network edges
- `-s 43`: Random seed for reproducible results

**Step 3: Run the Simulation**
```bash
python SUMO.py
```

#### Route Compatibility Notice:
 **Critical**: The included `wolfsburg.rou.xml` file contains edge IDs that are specific to a particular network generation. If you generate your own `wolfsburg.net.xml` file, the edge IDs may differ, causing route compatibility issues.

**Solution**: The simulation uses **duarouter** to automatically:
- Validate route compatibility with your network file
- Repair invalid edge references
- Generate new routes if needed
- Fall back to automatically generated routes if validation fails

#### Automatic Route Processing:
The simulation automatically handles route compatibility through these steps:
1. **duarouter validation** - Checks if routes are compatible with your network
2. **Route repair** - Fixes invalid edge references where possible
3. **Fallback generation** - Creates new routes if repair fails
4. **Vehicle type integration** - Adds friction device parameters

```python
# The simulation automatically runs:
duarouter -n wolfsburg.net.xml -r wolfsburg.rou.xml -o wolfsburg.routed.xml --repair --ignore-errors
```

#### File Placement:
Place the network file in the project root directory:
```
 Project Directory/
├── SUMO.py
├── README.md
├── wolfsburg.net.xml  ← **Required - obtain separately**
├── wolfsburg.rou.xml  ← **Included - may need route repair or regenerate**
└── [generated files will appear during simulation]
```

#### Generated Files (Auto-created):
- `wolfsburg.routed.xml` - Processed and validated routes
- `wolfsburg.sumocfg` - SUMO configuration file
- `edge_weights.xml` - Dynamic incident-based weights
- `main_vehicle_speed_analysis.png` - Analysis results

### Verification
Before running the simulation, ensure you have:
- `SUMO_HOME` environment variable set
- `wolfsburg.net.xml` in project root 
- `wolfsburg.rou.xml` in project root (included)
-  All Python dependencies installed

## Dependencies

### Required Python Packages
```python
import os
import sys
import requests          # Weather and incident API calls
import matplotlib.pyplot # Plotting and visualization
import subprocess       # SUMO process management
import xml.etree.ElementTree  # XML parsing
import math            # Mathematical calculations
import logging         # Logging and debugging
import pandas          # Data handling for forecasts
from datetime import datetime, timedelta  # Time management
```

### Optional Dependencies
```python
import pyproj          # Enhanced coordinate conversion
import traci          # SUMO TraCI interface
```

### External Requirements
- **SUMO**: Simulation of Urban Mobility (with SUMO_HOME environment variable)
- **Network Access**: For weather and traffic incident APIs
- **German Traffic Data**: Access to Autobahn incident reporting system

## Configuration

### Environment Setup
1. Install SUMO and set `SUMO_HOME` environment variable
2. Install required Python packages: `pip install requests matplotlib pandas pyproj`
3. Ensure network access for API calls

### Simulation Parameters
```python
# Key configuration constants
SUMO_CONFIG_FILE = "wolfsburg.sumocfg"
ROUTED_ROUTE_FILE = "wolfsburg.routed.xml"
TRIP_FILE = "wolfsburg.rou.xml"
NET_FILE = "wolfsburg.net.xml"

# Simulation timing
SIMULATION_DURATION = 3600  # 1 hour
WEATHER_CHECK_INTERVAL = 1  # Every second
INCIDENT_CHECK_INTERVAL = 900  # Every 15 minutes
```

### Weather API Configuration
- **Location**: Wolfsburg, Germany (52.42°N, 10.78°E)
- **Provider**: Open-Meteo API
- **Update Frequency**: Real-time with minute-level precision
- **Forecast Range**: 3 hours ahead

## Usage

### Basic Execution
```bash
python SUMO.py
```

### Test Mode
For controlled testing and research purposes, the simulation includes a dedicated test mode that applies consistent extreme weather conditions:

**Features of Test Mode:**
- **Fixed Friction**: Maintains constant 0.25 friction coefficient (simulating heavy snow/thunderstorm conditions)
- **Reproducible Results**: Eliminates weather API variability for consistent testing
- **Extreme Conditions**: Simulates worst-case scenario for safety research
- **Controlled Environment**: Perfect for comparing different routing algorithms or vehicle parameters

To enable test mode:
```python
# Modify line 702 in SUMO.py (uncomment this line and comment the real weather line)
friction, weather_code, weather_name = test_low_friction_scenario()
```

### Real Weather Mode
For realistic simulations using current weather conditions:
```python
# Modify line 702 in SUMO.py (uncomment this line and comment the test mode line)
friction, weather_code, weather_name = get_diverse_friction_from_weather()
```

**Note**: The simulation automatically detects which mode is active and adjusts logging and behavior accordingly. Test mode provides consistent "TEST: Heavy Thunderstorm with Hail" conditions throughout the simulation.

## Simulation Process

1. **Initialization**
   - Validates network and route files
   - Sets up vehicle types with friction devices
   - Creates SUMO configuration if needed

2. **Route Preprocessing**
   - Processes initial routes with duarouter
   - Checks for current traffic incidents
   - Applies incident penalties to affected segments

3. **Simulation Execution**
   - Starts SUMO with GUI interface
   - Applies real-time weather-based friction
   - Monitors for incident changes every 15 minutes
   - Performs dynamic re-routing when needed

4. **Data Collection**
   - Records vehicle speeds at each simulation step
   - Logs weather condition changes
   - Tracks incident impacts on routing

5. **Analysis and Reporting**
   - Generates speed analysis plots
   - Provides simulation summary with metrics
   - Exports results for further analysis

## Output Analysis

### Generated Files
- **Speed Analysis Plots**: `main_vehicle_speed_analysis_*.png`
- **Weather Logs**: Embedded in simulation output
- **Route Files**: Updated with incident-based modifications

### Key Metrics
- Average and maximum vehicle speeds
- Weather condition impacts on traffic flow
- Incident-based re-routing effectiveness
- Real-world time correlation with simulation time

## API Integrations

### Weather Data (Open-Meteo)
- **Endpoint**: `https://api.open-meteo.com/v1/forecast`
- **Data**: Current weather codes, forecasts
- **Frequency**: Real-time updates

### Traffic Incidents (German Autobahn)
- **Endpoint**: `https://verkehr.autobahn.de/o/autobahn`
- **Coverage**: A2, A39 highway corridors
- **Data Types**: Warnings, roadworks, closures

## Performance Characteristics

- **Simulation Speed**: Real-time execution (1 sim second = 1 wall-clock second)
- **Weather Updates**: Sub-second precision
- **Incident Processing**: 15-minute interval checks
- **Memory Usage**: Optimized with edge center caching
- **Network Efficiency**: Session reuse for API calls

## Research Applications

This simulation framework is designed for research into:
- Weather impact on traffic flow and safety
- Real-time traffic management systems
- Incident response effectiveness
- Friction modeling in transportation networks
- Dynamic routing algorithm performance

## Limitations and Considerations

- **Geographic Scope**: Currently focused on Wolfsburg network
- **Weather Accuracy**: Dependent on API availability and accuracy
- **Incident Data**: Limited to German Autobahn system
- **Computational Requirements**: Requires SUMO installation and sufficient system resources

## References and Citations

### APIs and Data Sources
- Open-Meteo Weather API. (2025). *Free Weather Forecast API*. Retrieved from https://open-meteo.com/
- German Federal Highway Research Institute. (2025). *Autobahn Traffic Information Service*. Retrieved from https://verkehr.autobahn.de/

### Software and Tools
- Krajzewicz, D., Erdmann, J., Behrisch, M., & Bieker, L. (2012). Recent development and applications of SUMO - Simulation of Urban Mobility. *International Journal On Advances in Systems and Measurements*, 5(3&4), 128-138.
- Eclipse SUMO. (2025). *Simulation of Urban Mobility*. Retrieved from https://www.eclipse.org/sumo/

### How to Cite This Work
```
Hirpara, V. (2025). SUMO Traffic Simulation with Dynamic Weather and Incident Management. 
Ostfalia University of Applied Sciences, Institute for Automotive Engineering Wolfsburg.
```

**Author**: Vivek Hirpara  
**Supervisor**: Prof.Dr.-Ing. Harald Bachem  
**Mentor**: Christoph Rohmann, M.Eng.  
**Institution**: Ostfalia University of Applied Sciences  
**Date**: September 2025  
**Version**: 1.0


