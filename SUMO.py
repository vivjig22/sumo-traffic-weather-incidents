import os
import sys
import requests
import matplotlib.pyplot as plt
import subprocess
import xml.etree.ElementTree as ET
import math
import logging
import pandas as pd  # Add this import
from datetime import datetime, timedelta  # Add this import

# Try to import pyproj for coordinate conversion
try:
    import pyproj
    HAS_PYPROJ = True
except ImportError:
    HAS_PYPROJ = False
    print("Warning: pyproj not installed. Install with 'pip install pyproj' for better coordinate accuracy")

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Add a verbose flag for detailed output
VERBOSE = False  # Set to True for detailed output

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    sumo_bin_path = os.path.join(os.environ['SUMO_HOME'], 'bin')
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

def confirm_traci_connection():
    """Confirming TraCI connection"""
    try:
        # Test the connection by getting simulation time
        traci.simulation.getTime()
        print("Connection to TraCI successfully established")
        return True
    except Exception as e:
        print(f"TraCI connection failed: {e}")
        return False

SUMO_CONFIG_FILE = "wolfsburg.sumocfg"
ROUTED_ROUTE_FILE = "wolfsburg.routed.xml"
TRIP_FILE = "wolfsburg.rou.xml"
NET_FILE = "wolfsburg.net.xml"
EDGE_WEIGHT_FILE = "edge_weights.xml"
HEAVY_PENALTY = 100000

SUMO_CMD = ["sumo-gui", "-c", SUMO_CONFIG_FILE, "--step-length", "1", "--end", "3600", 
            "--no-warnings", "--no-step-log"]

# Cache for edge centers to avoid repeated calculations
EDGE_CENTERS_CACHE = None

def add_vehicle_type_with_friction_device():
    if VERBOSE:
        print("Adding vehicle type with SUMO friction device to route file...")
    tree = ET.parse(ROUTED_ROUTE_FILE)
    root = tree.getroot()
    existing_vtype = root.find('vType[@id="default"]')
    if existing_vtype is not None:
        root.remove(existing_vtype)
    vtype = ET.Element('vType')
    vtype.set('id', 'default')
    vtype.set('accel', '2.6')
    vtype.set('decel', '4.5')
    vtype.set('sigma', '0.5')
    vtype.set('length', '5')
    vtype.set('maxSpeed', '19.44')  # ~70 km/h
    vtype.set('speedFactor', '1.0')
    vtype.set('speedDev', '0.1')
    vtype.set('minGap', '2.5')
    vtype.set('tau', '1.0')
    p1 = ET.SubElement(vtype, 'param')
    p1.set('key', 'has.friction.device')
    p1.set('value', 'true')
    p2 = ET.SubElement(vtype, 'param')
    p2.set('key', 'device.friction.stdDev')
    p2.set('value', '0.1')
    p3 = ET.SubElement(vtype, 'param')
    p3.set('key', 'device.friction.offset')
    p3.set('value', '0.0')
    root.insert(0, vtype)
    for veh in root.findall('vehicle'):
        veh.set('type', 'default')
        if veh.find("param[@key='has.friction.device']") is None:
            param = ET.SubElement(veh, 'param')
            param.set('key', 'has.friction.device')
            param.set('value', 'true')
    tree.write(ROUTED_ROUTE_FILE, encoding='utf-8', xml_declaration=True)
    if VERBOSE:
        print("Done.")

def create_sumo_config_if_needed():
    if not os.path.exists(SUMO_CONFIG_FILE):
        print("Creating sumo config...")
        cfg = f"""<configuration>
    <input>
        <net-file value="{NET_FILE}"/>
        <route-files value="{ROUTED_ROUTE_FILE}"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="3600"/>
    </time>
</configuration>"""
        with open(SUMO_CONFIG_FILE, 'w') as f:
            f.write(cfg)

def setup_main_vehicles():
    if not os.path.exists(TRIP_FILE):
        return generate_fallback_routes()
    tree = ET.parse(TRIP_FILE)
    root = tree.getroot()
    trips = root.findall("trip")
    if len(trips) < 2:
        return generate_fallback_routes()
    main_vehicles = []
    for i in range(2):
        trip = trips[i]
        trip.set('id', f'vehicle_{i+1}')
        trip.set('depart', f'{i*5:.2f}')
        main_vehicles.append(f'vehicle_{i+1}')
    tree.write(TRIP_FILE, encoding='utf-8', xml_declaration=True)
    return main_vehicles

def generate_fallback_routes():
    import random
    random.seed(42)
    root = ET.Element("routes")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/routes_file.xsd")
    fallback = [
        {"from": "-25009994#6", "to": "1164243934#1"},
        {"from": "25009994#6", "to": "-1164243940#0"},
    ]
    names = []
    for i, val in enumerate(fallback):
        veh = ET.SubElement(root, "trip")
        veh.set("id", f"vehicle_{i+1}")
        veh.set("depart", f"{i*5:.2f}")
        veh.set("from", val["from"])
        veh.set("to", val["to"])
        names.append(f"vehicle_{i+1}")
    tree = ET.ElementTree(root)
    tree.write(TRIP_FILE, encoding='utf-8', xml_declaration=True)
    print("Fallback routes created.")
    return names

def preprocess_routes_with_duarouter():
    """Step 1: Basic route preprocessing without incidents"""
    print("Preprocessing routes with duarouter...")
    duarouter_exe = os.path.join(sumo_bin_path, "duarouter")
    cmd = [
        duarouter_exe,
        "-n", NET_FILE,
        "-r", TRIP_FILE,
        "-o", ROUTED_ROUTE_FILE,
        "--ignore-errors",
        "--repair",
    ]
    subprocess.run(cmd, check=True, capture_output=True, text=True)
    add_vehicle_type_with_friction_device()
    print("Initial route preprocessing completed")

def get_routes_from_file():
    """Extract the actual routes from the routed file"""
    routes = {}
    try:
        tree = ET.parse(ROUTED_ROUTE_FILE)
        root = tree.getroot()
        
        for vehicle in root.findall('vehicle'):
            vehicle_id = vehicle.get('id')
            route_elem = vehicle.find('route')
            if route_elem is not None:
                edges = route_elem.get('edges', '').split()
                routes[vehicle_id] = edges
    except Exception as e:
        if VERBOSE:
            print(f"Error reading routes: {e}")
    
    return routes

def update_sumocfg_to_use_routed_file():
    tree = ET.parse(SUMO_CONFIG_FILE)
    root = tree.getroot()
    routefiles = root.find("input/route-files")
    routefiles.set("value", ROUTED_ROUTE_FILE)
    tree.write(SUMO_CONFIG_FILE)

def get_diverse_friction_from_weather(verbose=None):
    if verbose is None:
        verbose = VERBOSE
    try:
        lat, lon = 52.42, 10.78
        url = f"https://api.open-meteo.com/v1/forecast?latitude={lat}&longitude={lon}&current_weather=true"
        r = requests.get(url, timeout=10)
        r.raise_for_status()
        data = r.json()
        code = data.get("current_weather", {}).get("weathercode", 0)
        map_ = {
            0: ("Clear sky", 1.0),
            1: ("Mainly clear", 0.95),
            2: ("Partly cloudy", 0.9),
            3: ("Overcast", 0.85),
            45: ("Fog", 0.8),
            48: ("Dense fog", 0.65),
            51: ("Light drizzle", 0.75),
            53: ("Moderate drizzle", 0.7),
            55: ("Dense drizzle", 0.65),
            61: ("Light rain", 0.7),
            63: ("Moderate rain", 0.6),
            65: ("Heavy rain", 0.5),
            71: ("Light snow", 0.55),
            73: ("Moderate snow", 0.45),
            75: ("Heavy snow", 0.35),
            77: ("Snow grains", 0.5),
            80: ("Light rain showers", 0.65),
            81: ("Moderate rain showers", 0.55),
            82: ("Violent rain showers", 0.4),
            85: ("Light snow showers", 0.5),
            86: ("Heavy snow showers", 0.3),
            95: ("Thunderstorm", 0.45),
            96: ("Thunderstorm with hail", 0.35),
            99: ("Thunderstorm with heavy hail", 0.25),
        }
        name, friction = map_.get(code, ("Unknown", 1.0))
        if verbose:
            print(f"Weather code {code}: {name}, friction {friction}")
        # Remove automatic printing when not verbose - let caller decide when to print
        return str(friction), code, name
    except requests.exceptions.RequestException as e:
        if verbose:
            print(f"Failed to fetch weather (network error): {e}")
        # Remove automatic printing - let caller handle error messaging
        return "1.0", None, "Clear (default)"
    except Exception as e:
        if verbose:
            print(f"Failed to fetch weather: {e}")
        else:
            print("Weather API error, using default conditions")
        return "1.0", None, "Clear (default)"

def fetch_minute_level_forecast(lat=52.42, lon=10.78, hours_ahead=3):
    """Fetch minute-level weather forecast for next 3 hours"""
    try:
        # Use minutely_15 for 15-minute precision
        url = (f"https://api.open-meteo.com/v1/forecast?latitude={lat}&longitude={lon}"
               f"&minutely_15=weathercode&forecast_hours={hours_ahead}&timezone=Europe/Berlin")
        
        r = requests.get(url, timeout=10)
        r.raise_for_status()
        data = r.json()
        
        # Create DataFrame with minute-level precision
        df = pd.DataFrame({
            "time": pd.to_datetime(data["minutely_15"]["time"]),
            "weathercode": data["minutely_15"]["weathercode"]
        })
        
        # Debug information to understand the forecast data
        if len(df) > 0:
            start_time = df.iloc[0]['time']
            end_time = df.iloc[-1]['time']
            duration_hours = (end_time - start_time).total_seconds() / 3600
            print(f"Loaded {len(df)} forecast points covering {duration_hours:.1f} hours ({start_time.strftime('%H:%M')} to {end_time.strftime('%H:%M')})")
        else:
            print(f"Loaded {len(df)} forecast points (empty dataset)")
        
        return df
        
    except Exception as e:
        print(f"Failed to fetch minute-level forecast: {e}, using fallback")
        # Create fallback forecast starting from current time
        now = datetime.now()
        forecast_times = [now + timedelta(minutes=15*i) for i in range(hours_ahead * 4)]  # 4 points per hour
        return pd.DataFrame({
            "time": forecast_times,
            "weathercode": [0] * len(forecast_times)  # Clear weather as fallback
        })

def get_weather_for_simulation_time(forecast_df, sim_start_dt, sim_seconds):
    """Get weather forecast for exact simulation time using existing friction mapping"""
    # Calculate real-world time corresponding to simulation time
    real_time_now = sim_start_dt + timedelta(seconds=sim_seconds)
    
    # Find the most recent forecast point that's not in the future
    valid_forecasts = forecast_df[forecast_df['time'] <= real_time_now]
    
    if valid_forecasts.empty:
        # If no past forecasts, use the first available
        weather_row = forecast_df.iloc[0]
    else:
        # Use the most recent forecast
        weather_row = valid_forecasts.iloc[-1]
    
    weather_code = weather_row['weathercode']

    #using the existing friction method
    map_ = {
        0: ("Clear sky", 1.0),
        1: ("Mainly clear", 0.95),
        2: ("Partly cloudy", 0.9),
        3: ("Overcast", 0.85),
        45: ("Fog", 0.8),
        48: ("Dense fog", 0.65),
        51: ("Light drizzle", 0.75),
        53: ("Moderate drizzle", 0.7),
        55: ("Dense drizzle", 0.65),
        61: ("Light rain", 0.7),
        63: ("Moderate rain", 0.6),
        65: ("Heavy rain", 0.5),
        71: ("Light snow", 0.55),
        73: ("Moderate snow", 0.45),
        75: ("Heavy snow", 0.35),
        77: ("Snow grains", 0.5),
        80: ("Light rain showers", 0.65),
        81: ("Moderate rain showers", 0.55),
        82: ("Violent rain showers", 0.4),
        85: ("Light snow showers", 0.5),
        86: ("Heavy snow showers", 0.3),
        95: ("Thunderstorm", 0.45),
        96: ("Thunderstorm with hail", 0.35),
        99: ("Thunderstorm with heavy hail", 0.25),
    }
    
    weather_name, friction_value = map_.get(weather_code, ("Unknown", 1.0))
    
    return str(friction_value), weather_code, weather_name

def test_low_friction_scenario():
    print("TEST MODE: Applying low friction 0.25 for heavy snow/thunderstorm test")
    return "0.25", 0, "TEST: Heavy Thunderstorm with Hail"

def get_friction_settings():
    
    friction, weather_code, weather_name = test_low_friction_scenario()
    
    USE_TEST_MODE = friction == "0.25"
    return friction, weather_code, weather_name, USE_TEST_MODE

def set_route_friction(routes, friction):
    """Apply friction only to edges that are in the actual routes"""
    f = float(friction)
    affected_edges = set()
    
    # Collect all edges from all routes
    for vehicle_id, route_edges in routes.items():
        affected_edges.update(route_edges)
    
    # Apply friction only to route edges
    count = 0
    for edge in affected_edges:
        try:
            traci.edge.setFriction(edge, f)
            count += 1
        except:
            continue
    
    if VERBOSE:
        print(f"Applied friction {f} to {count} route edges")
    else:
        print(f"Applied friction {f} to {count:,} route segments")

def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # km
    dlat = math.radians(lat2-lat1)
    dlon = math.radians(lon2-lon1)
    a = (math.sin(dlat/2))**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))* (math.sin(dlon/2))**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R*c

def create_coordinate_transformer():
    """Create transformer for GPS to local coordinate conversion"""
    if not HAS_PYPROJ:
        return None
    try:
        # Wolfsburg area uses UTM Zone 32N (EPSG:32632)
        gps_crs = pyproj.CRS("EPSG:4326")  # WGS84 (GPS coordinates)
        local_crs = pyproj.CRS("EPSG:32632")  # UTM Zone 32N (common for Germany)
        return pyproj.Transformer.from_crs(gps_crs, local_crs, always_xy=True)
    except:
        return None

def convert_gps_to_local(lat, lon, transformer):
    """Convert GPS coordinates to local network coordinates"""
    if not transformer:
        return None, None
    try:
        x, y = transformer.transform(lon, lat)  # Note: lon, lat order for pyproj
        return x, y
    except:
        return None, None

def get_network_bounds():
    """Get the bounds of the network for filtering incidents"""
    try:
        # Get network boundary
        net_boundary = traci.simulation.getNetBoundary()
        return net_boundary
    except:
        return None

def get_incident_severity(incident_data):
    """Determine incident severity for different penalty weights"""
    title = incident_data.get("title", "").lower()
    
    # High severity
    if any(word in title for word in ["gesperrt", "vollsperrung", "blocked", "closed"]):
        return "high"
    # Medium severity  
    elif any(word in title for word in ["baustelle", "construction", "roadwork", "lane"]):
        return "medium"
    # Low severity
    else:
        return "low"

def remove_duplicate_incidents(incidents):
    """Remove incidents that are very close to each other (likely duplicates)"""
    unique_incidents = []
    min_distance = 100  # 100 meters minimum distance
    
    for incident in incidents:
        is_duplicate = False
        for existing in unique_incidents:
            if "local_x" in incident and "local_x" in existing:
                # Calculate distance in local coordinates (more accurate)
                dx = incident["local_x"] - existing["local_x"]
                dy = incident["local_y"] - existing["local_y"]
                distance = math.sqrt(dx*dx + dy*dy)
            else:
                # Fall back to GPS distance calculation
                distance = haversine(incident["latitude"], incident["longitude"], 
                                   existing["latitude"], existing["longitude"]) * 1000  # Convert to meters
            
            if distance < min_distance and incident["title"] == existing["title"]:
                is_duplicate = True
                break
        
        if not is_duplicate:
            unique_incidents.append(incident)
    
    return unique_incidents

def get_edge_centers_for_routes(routes):
    """Get edge centers with enhanced data for accurate distance calculation"""
    global EDGE_CENTERS_CACHE
    
    # Collect all unique edges from routes
    route_edges = set()
    for vehicle_id, route_edges_list in routes.items():
        route_edges.update(route_edges_list)
    
    centers = {}
    for edge in route_edges:
        try:
            # Get edge shape in local coordinates (SUMO's coordinate system)
            lane_count = traci.edge.getLaneNumber(edge)
            coords = []
            
            for i in range(lane_count):
                lane_id = f"{edge}_{i}"
                try:
                    shape = traci.lane.getShape(lane_id)
                    coords.extend(shape)
                except:
                    continue
            
            if not coords:
                try:
                    shape = traci.edge.getShape(edge)
                    coords = shape
                except:
                    try:
                        from_junction = traci.edge.getFromJunction(edge)
                        to_junction = traci.edge.getToJunction(edge)
                        from_pos = traci.junction.getPosition(from_junction)
                        to_pos = traci.junction.getPosition(to_junction)
                        coords = [from_pos, to_pos]
                    except:
                        continue
            
            if coords:
                # Calculate center in local coordinates
                avg_x = sum(p[0] for p in coords) / len(coords)
                avg_y = sum(p[1] for p in coords) / len(coords)
                
                # Store with edge length for weighted detection
                try:
                    edge_length = traci.edge.getLength(edge)
                except:
                    edge_length = 100  # Default length
                
                centers[edge] = {
                    "x": avg_x,
                    "y": avg_y,
                    "length": edge_length
                }
        except Exception as e:
            continue
    
    EDGE_CENTERS_CACHE = centers
    print(f"Processed {len(centers):,} route segments for incident detection")
    return centers

def get_incidents(show_message=True):
    """Enhanced incident fetching with coordinate conversion and filtering"""
    incidents = []
    transformer = create_coordinate_transformer()
    
    try:
        base_url = "https://verkehr.autobahn.de/o/autobahn"
        areas = ["A2", "A39"]
        
        session = requests.Session()
        for area in areas:
            for service in ["warning", "roadworks"]:
                url = f"{base_url}/{area}/services/{service}"
                try:
                    r = session.get(url, timeout=5)
                    if r.status_code != 200:
                        continue
                    data = r.json()
                    items = data.get(service, [])
                    
                    for it in items:
                        coord = it.get("coordinate", {})
                        if "lat" in coord and "long" in coord:
                            lat = float(coord["lat"])
                            lon = float(coord["long"])
                            
                            # Convert to local coordinates if possible
                            local_x, local_y = convert_gps_to_local(lat, lon, transformer)
                            
                            # Create incident with enhanced data
                            incident = {
                                "id": f"{area}_{service}_{len(incidents)}",  # Unique ID
                                "title": it.get("title", "").strip(),
                                "latitude": lat,
                                "longitude": lon,
                                "type": service,
                                "area": area,
                                "severity": get_incident_severity(it),
                                "timestamp": it.get("startTimestamp", ""),
                                "location": f"{lat},{lon}"
                            }
                            
                            # Add local coordinates if conversion was successful
                            if local_x is not None and local_y is not None:
                                incident["local_x"] = local_x
                                incident["local_y"] = local_y
                            
                            incidents.append(incident)
                            
                except requests.exceptions.RequestException:
                    continue
        
        session.close()
        
        # Remove duplicate incidents based on proximity
        incidents = remove_duplicate_incidents(incidents)
        
        if show_message:
            print(f"Found {len(incidents)} traffic incidents")
            
    except Exception as e:
        print(f"Failed to get incidents: {e}")
    return incidents

def get_affected_route_edges(incidents, centers, radius_km=1.0):
    """More accurate incident-to-edge mapping with dynamic radius and severity"""
    affected = {}  # Store edge and severity
    
    for incident in incidents:
        severity = incident["severity"]
        
        # Dynamic radius based on incident type
        if severity == "high":
            base_radius = 200  # 200m for closures
        elif severity == "medium":
            base_radius = 100  # 100m for construction
        else:
            base_radius = 50   # 50m for minor incidents
        
        # Use local coordinates if available, otherwise fall back to GPS
        if "local_x" in incident and "local_y" in incident:
            incident_x = incident["local_x"]
            incident_y = incident["local_y"]
            use_local_coords = True
        else:
            incident_lat = incident["latitude"]
            incident_lon = incident["longitude"]
            use_local_coords = False
        
        for edge, edge_data in centers.items():
            if use_local_coords:
                # Calculate distance in local coordinates (meters) - more accurate
                edge_x = edge_data["x"]
                edge_y = edge_data["y"]
                distance = math.sqrt((incident_x - edge_x)**2 + (incident_y - edge_y)**2)
                
                # Adjust radius based on edge length (longer edges = larger detection radius)
                edge_length = edge_data["length"]
                dynamic_radius = base_radius + min(edge_length * 0.1, 100)
            else:
                # Fall back to GPS coordinates and haversine distance
                edge_x = edge_data["x"]
                edge_y = edge_data["y"]
                # This is a simplified approach - ideally you'd convert edge coords to GPS
                # For now, use a larger radius to compensate for coordinate system mismatch
                distance = haversine(incident_lat, incident_lon, edge_x/111000, edge_y/111000) * 1000
                dynamic_radius = base_radius * 2  # Double the radius for GPS fallback
            
            if distance <= dynamic_radius:
                # Store the most severe incident affecting this edge
                if edge not in affected or get_severity_weight(severity) > get_severity_weight(affected[edge]):
                    affected[edge] = severity
    
    return affected

def get_severity_weight(severity):
    """Convert severity to numeric weight"""
    weights = {"high": 3, "medium": 2, "low": 1}
    return weights.get(severity, 1)

def write_weights_file(affected, filename="edge_weights.xml", penalty=100000):
    """Write edge weights with different penalties based on severity"""
    if not affected:
        return
        
    penalties = {
        "high": 100000,    # Complete blockage
        "medium": 10000,   # Significant delay
        "low": 2000        # Minor delay
    }
    
    root = ET.Element("meandata")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/meandata_file.xsd")
    
    interval = ET.SubElement(root, "interval")
    interval.set("begin", "0")
    interval.set("end", "3600")
    
    for edge, severity in affected.items():
        edge_elem = ET.SubElement(interval, "edge")
        edge_elem.set("id", edge)
        edge_elem.set("traveltime", str(penalties[severity]))
    
    tree = ET.ElementTree(root)
    tree.write(filename, encoding='utf-8', xml_declaration=True)

def compare_incidents_accurately(prev_incidents, new_incidents):
    """More accurate incident comparison using IDs"""
    prev_ids = {inc["id"] for inc in prev_incidents}
    new_ids = {inc["id"] for inc in new_incidents}
    
    # Find truly new and resolved incidents
    new_incident_ids = new_ids - prev_ids
    resolved_incident_ids = prev_ids - new_ids
    
    return len(new_incident_ids), len(resolved_incident_ids)

def reroute_if_needed(affected_edges):
    """Re-route with severity-based penalties"""
    if len(affected_edges) == 0:
        print("No route segments affected by incidents")
        return
    
    # Count incidents by severity
    high_severity = sum(1 for s in affected_edges.values() if s == "high")
    medium_severity = sum(1 for s in affected_edges.values() if s == "medium")
    low_severity = sum(1 for s in affected_edges.values() if s == "low")
    
    print(f"Re-routing due to incidents: {high_severity} high, {medium_severity} medium, {low_severity} low severity")
    
    write_weights_file(affected_edges)
    
    duarouter_exe = os.path.join(sumo_bin_path, "duarouter")
    cmd = [
        duarouter_exe,
        "-n", NET_FILE,
        "-r", TRIP_FILE,
        "-o", ROUTED_ROUTE_FILE,
        "--weight-files", "edge_weights.xml",
        "--ignore-errors",
        "--repair",
    ]
    
    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True)
        add_vehicle_type_with_friction_device()
        print("Re-routing completed with severity-based penalties")
    except subprocess.CalledProcessError:
        print("Re-routing failed, keeping original routes")

def run_simulation(main_vehicles, routes):
    traci.start(SUMO_CMD)
    
    # Add connection confirmation
    if not confirm_traci_connection():
        print("Error: Could not establish TraCI connection for main simulation")
        return None

    # Initialize with current incidents to avoid false new incident counts
    prev_incidents = get_incidents(show_message=False)  # Don't show message for initialization
    
    data = {"times": [], "speeds": {}, "weather_log": []}
    for v in main_vehicles:
        data["speeds"][v] = []

    ############################### Choose real or test friction mode here: ####################################
    #friction, weather_code, weather_name = get_diverse_friction_from_weather()
    friction, weather_code, weather_name = test_low_friction_scenario()
    
    # Determine which mode we're using based on the active line above
    USE_TEST_MODE = friction == "0.25"  # Test mode returns 0.25 friction
    ##############################################################################################################

    set_route_friction(routes, friction)
    data["weather_log"].append({"time": 0, "weather": weather_name, "friction": friction})

    # Weather timing configuration for second-level precision
    weather_log_interval = 900      # Log weather every 15 minutes
    incident_check = 900           # Every 15 minutes (900 seconds) 
    prev_weather_code = weather_code
    last_weather_log_time = 0
    
    # Record simulation start time for real-world time tracking
    sim_start_datetime = datetime.now()  # Real-world time when simulation starts
    
    # Fetch minute-level weather forecast for next 3 hours
    weather_forecast_df = None
    if not USE_TEST_MODE:
        weather_forecast_df = fetch_minute_level_forecast()
        print(f"Loaded weather forecast for realistic weather changes")
    
    print(f"Starting simulation at real-world time: {sim_start_datetime.strftime('%H:%M:%S on %Y-%m-%d')}")
    print("Simulation timeline: Each simulation second = 1 real-world second")

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        t = traci.simulation.getTime()
        data["times"].append(t)
        vehicles = traci.vehicle.getIDList()
        for v in main_vehicles:
            speed = traci.vehicle.getSpeed(v) * 3.6 if v in vehicles else 0
            data["speeds"][v].append(speed)

        # CHECK WEATHER EVERY SIMULATION STEP (every second) for maximum precision
        if not USE_TEST_MODE and weather_forecast_df is not None:
            # Get current weather from forecast for this exact simulation time
            friction2, wcode2, wname2 = get_weather_for_simulation_time(
                weather_forecast_df, sim_start_datetime, t
            )
            
            # Apply changes immediately when weather changes
            if wcode2 != prev_weather_code:
                current_routes = get_routes_from_file()
                set_route_friction(current_routes, friction2)
                data["weather_log"].append({"time": t, "weather": wname2, "friction": friction2})
                prev_weather_code = wcode2
                
                # Calculate real-world time for immediate logging
                real_time = sim_start_datetime + timedelta(seconds=t)
                print(f"Weather changed at {t/60:.1f}min (real-time {real_time.strftime('%H:%M')}): {wname2} (friction: {friction2})")
                last_weather_log_time = t
            
            # Log status every 15 minutes even if no change
            elif int(t) % weather_log_interval == 0 and int(t) > 0:
                real_time = sim_start_datetime + timedelta(seconds=t)
                print(f"Weather check: {wname2} (friction: {friction2})")
        
        # Test mode - only log every 15 minutes
        elif USE_TEST_MODE and int(t) % weather_log_interval == 0 and int(t) > 0:
            new_friction, new_code, new_name = test_low_friction_scenario()
            print(f"TEST MODE: Maintaining {new_name} (friction: {new_friction})")

        # Check incidents every 15 minutes
        if int(t) % incident_check == 0 and int(t) > 0:
            new_incidents = get_incidents(show_message=False)  # Don't show message for updates
            
            # Accurate comparison using incident IDs
            new_count, resolved_count = compare_incidents_accurately(prev_incidents, new_incidents)
            
            if new_count > 0 or resolved_count > 0:
                print(f"Traffic update: +{new_count} new, -{resolved_count} resolved incidents")
                
                # Check if incidents affect current vehicles with accurate mapping
                if EDGE_CENTERS_CACHE:
                    affected_edges = get_affected_route_edges(new_incidents, EDGE_CENTERS_CACHE)
                    
                    if affected_edges:
                        rerouted_count = 0
                        for v in vehicles:
                            route = traci.vehicle.getRoute(v)
                            if any(edge in affected_edges for edge in route):
                                traci.vehicle.rerouteTraveltime(v)
                                rerouted_count += 1
                        
                        high_count = sum(1 for s in affected_edges.values() if s == "high")
                        medium_count = sum(1 for s in affected_edges.values() if s == "medium")
                        low_count = sum(1 for s in affected_edges.values() if s == "low")
                        print(f"Found {len(affected_edges)} affected route segments (high:{high_count}, medium:{medium_count}, low:{low_count})")
                        
                        if rerouted_count > 0:
                            print(f"Rerouted {rerouted_count} vehicles due to incidents")
                        else:
                            print("Affected segments don't impact current vehicle routes")
                    else:
                        print("No incidents affecting current vehicle routes")
                
                prev_incidents = new_incidents
            else:
                print("Traffic check: No new incident changes detected")

    # Calculate simulation end time and real-world time correlation
    sim_end_datetime = datetime.now()
    final_sim_time = traci.simulation.getTime()
    
    # Calculate what real-world time the simulation represents
    sim_real_world_end_time = sim_start_datetime + timedelta(seconds=final_sim_time)
    
    # Calculate actual wall-clock duration
    actual_duration = sim_end_datetime - sim_start_datetime
    
    print(f"\nSimulation ended at real-world time: {sim_end_datetime.strftime('%H:%M:%S on %Y-%m-%d')}")
    print(f"Simulation represented time period: {sim_start_datetime.strftime('%H:%M:%S')} to {sim_real_world_end_time.strftime('%H:%M:%S')} ({final_sim_time/60:.1f} minutes)")
    print(f"Actual computation time: {actual_duration.total_seconds():.1f} seconds")
    
    traci.close()
    
    # Return data with simulation start time for summary reporting
    data["sim_start_datetime"] = sim_start_datetime
    return data

def generate_plot(data, vehicles):
    plt.style.use('default')
    fig, ax = plt.subplots(figsize=(12,7))
    colors = ['#1f77b4', '#ff7f0e']
    for idx, v in enumerate(vehicles):
        if v in data["speeds"]:
            speeds = data["speeds"][v]
            times = data["times"][:len(speeds)]
            if max(speeds) > 0:
                times_smooth = times[::10]
                speeds_smooth = speeds[::10]
                ax.plot(times_smooth, speeds_smooth, label=v, color=colors[idx % len(colors)], linewidth=0.8)
    # Plot vertical lines for weather changes
    for evt in data.get("weather_log", []):
        ax.axvline(evt["time"], color='red', linestyle='--', alpha=0.5, linewidth=0.8)
    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Speed (km/h)")
    ax.set_title("Vehicle Speeds Over Time")
    ax.grid(True, linestyle='-', alpha=0.2)
    ax.legend()


        # Save and print file location
    filename = "main_vehicle_speed_analysis.png"
    plt.savefig(filename)
    full_path = os.path.abspath(filename)
    print(f"Analysis plot saved: {full_path}")

    plt.close()

def print_simulation_summary(data, main_vehicles, incidents_count, sim_start_time=None):
    """Print a clean summary of the simulation with real-world time correlation"""
    print("\n" + "="*50)
    print("SIMULATION SUMMARY")
    print("="*50)
    
    # Duration and time correlation
    total_time = max(data["times"]) if data["times"] else 0
    print(f"Simulation duration: {int(total_time)}s ({total_time/60:.1f} minutes)")
    
    # Real-world time correlation if provided
    if sim_start_time:
        sim_end_time = sim_start_time + timedelta(seconds=total_time)
        print(f"Real-world time simulated: {sim_start_time.strftime('%H:%M:%S')} to {sim_end_time.strftime('%H:%M:%S')}")
        print(f"Date: {sim_start_time.strftime('%Y-%m-%d')}")
    
    # Vehicle performance
    for vehicle in main_vehicles:
        if vehicle in data["speeds"]:
            speeds = data["speeds"][vehicle]
            avg_speed = sum(speeds) / len(speeds) if speeds else 0
            max_speed = max(speeds) if speeds else 0
            print(f"{vehicle}: Avg {avg_speed:.1f} km/h, Max {max_speed:.1f} km/h")
    
    # Weather changes
    weather_changes = len(data.get("weather_log", [])) - 1
    print(f"Weather changes: {weather_changes}")
    
    # Traffic incidents
    print(f"Traffic incidents processed: {incidents_count}")
    
    print("="*50)
    print("Simulation completed successfully!")

if __name__=="__main__":
    try:
        print("Initializing SUMO traffic simulation...")
        
        # Validate required files exist
        if not os.path.exists(NET_FILE):
            print(f"Error: Network file {NET_FILE} not found!")
            sys.exit(1)
            
        create_sumo_config_if_needed()
        main_vehicles = setup_main_vehicles()
        
        if not main_vehicles:
            print("Error: No main vehicles could be set up!")
            sys.exit(1)

        # Step 1: Preprocess routes first (without incidents)
        preprocess_routes_with_duarouter()
        update_sumocfg_to_use_routed_file()
        
        # Step 2: Get the preprocessed routes
        routes = get_routes_from_file()
        print(f"Routes established for {len(routes)} vehicles")
        
        # Step 3: Start SUMO temporarily to get route edge centers
        temp_cmd = [os.path.join(sumo_bin_path, "sumo"), "-c", SUMO_CONFIG_FILE, 
                   "--no-step-log", "--no-warnings"]
        print("Connecting to SUMO for route analysis...")
        traci.start(temp_cmd)
        
        # Add connection confirmation
        if not confirm_traci_connection():
            print("Error: Could not establish temporary TraCI connection")
            sys.exit(1)
            
        centers = get_edge_centers_for_routes(routes)
        traci.close()
        print("Temporary SUMO session completed successfully")
        
        # Step 4: Check incidents against routes
        print("Checking traffic incidents against routes...")
        incidents = get_incidents()
        affected_edges = get_affected_route_edges(incidents, centers)
        
        # Step 5: Re-route if needed
        if affected_edges:
            high_count = sum(1 for s in affected_edges.values() if s == "high")
            medium_count = sum(1 for s in affected_edges.values() if s == "medium") 
            low_count = sum(1 for s in affected_edges.values() if s == "low")
            print(f"Found {len(affected_edges)} affected route segments (high:{high_count}, medium:{medium_count}, low:{low_count})")
            reroute_if_needed(affected_edges)
            routes = get_routes_from_file()  # Get updated routes
        else:
            print("No traffic incidents affecting current vehicle routes")
        
        # Step 6: Run simulation with optimized checks
        print("Starting main simulation...")
        data = run_simulation(main_vehicles, routes)
        
        if data is None:
            print("Error: Simulation failed to start")
            sys.exit(1)
        
        print("Generating analysis plots...")
        generate_plot(data, main_vehicles)
        
        # Print clean summary with real-world time correlation
        sim_start_time = data.get("sim_start_datetime")
        print_simulation_summary(data, main_vehicles, len(incidents), sim_start_time)
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback; traceback.print_exc()