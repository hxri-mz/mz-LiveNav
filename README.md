# MZ-LiveNav — Live Navigation using OSRM

### Overview

**MZ-LiveNav** provides an API for nav commands and a real-time navigation interface powered by **OSRM (Open Source Routing Machine)** and **live GNSS updates**.  
It visualizes ego-vehicle position, heading (yaw), route geometry, and turn-by-turn maneuvers.

---

### Features and Functionality

#### 1. Interactive Map (Leaflet)
- Uses **Leaflet 1.9.4** with **leaflet-rotate** for map bearing (yaw) control.  
- Displays **OpenStreetMap tiles** as base map.  
- The map can dynamically rotate to align with the vehicle’s heading (ego-centric view).

#### 2. Ego-Vehicle Representation
- The ego vehicle is shown as a **blue circular marker** with a directional arrow indicating the forward direction.
- Position is updated live from the `/latest_position` endpoint (50 ms interval).
- When in “Follow ON” mode, the map automatically pans or rotates with the ego vehicle.

#### 3. Compass Rose and Orientation Info
- A **compass rose** dynamically rotates to show map orientation and cardinal direction (N, NE, E, etc.).
- The **Yaw angle (°)** is displayed and continuously updated at the top-right corner.

#### 4. Adding Waypoints
- Click anywhere on the map to add a waypoint marker.
- Each waypoint is labeled and shown on the map.
- Supports multiple waypoints before route creation.
- Bearing is temporarily reset during waypoint placement to ensure correct alignment.

#### 5. Route Creation
- Press **“Create Route”** to generate a route between the ego position and the selected waypoint(s).
- Sends a `POST` request to `/route` with all waypoints:
  ```json
  { "waypoints": [[lon1, lat1], [lon2, lat2], ...] }
  ```
- Displays the computed **route polyline (blue)**.
- Automatically fits or re-centers the map to show the full route.

#### 6. Turn-by-Turn Instructions
- Each turn (maneuver) is marked with an emoji arrow along the route.  
- The next turn direction and distance are displayed in the **top-right arrow box**.
- The main status bar below the control panel shows textual updates like:
  ```
  Next: left in 163 m
  ```
- The map automatically reroutes if the GNSS drift exceeds threshold distance.

#### 7. Live Rerouting
- If the server signals a reroute (`"rerouted": true`),  
  - Old route and turn markers are cleared.  
  - New route geometry and maneuver list are redrawn.  
  - The active route ID updates automatically.

#### 8. Destination and Maneuvers
- The **final destination** is highlighted with a red flag icon.
- Each intermediate turn or waypoint includes a popup with the turn instruction.

#### 9. Clear Route
- The **“Clear”** button resets the map state:
  - Removes current route, markers, and destination.
  - Sends a `POST` request to `/clear_route`.
  - Resets the status bar and turn arrow panel.

#### 10. Follow Mode (Ego-centric View)
- The **“Follow: ON/OFF”** toggle button enables automatic map centering and rotation based on vehicle yaw.
- When ON, the map view moves and rotates with ego motion.
- Smooth yaw interpolation prevents jittery map rotations.

#### 11. Debug & Testing Utilities
- **`debugMarkerPosition()`**: draw red circles at marker positions for debugging.
- **`testYawAdjustment()`**: console test to verify yaw-to-bearing transformation.
- Optional **projected point visualization** for route projection debugging.

#### 12. Smooth User Experience
- Smooth rotation and follow-mode transitions prevent disorientation when clicking or generating routes.
- Bearing automatically resets temporarily during route creation and marker placement to maintain consistency.

---

### Setup Environment

```bash
conda create -n livenav38 python=3.8
conda activate livenav38
pip install -r requirements.txt
```

Also install:
- **mz-schemas-protobuf**
- **ecal5 Python wheel**

---

### Running the Server and UI

```bash
conda activate livenav38
bash run.sh
```

This will:
- Start both backend servers (`gnss_bridge.py` and `live_nav.py`)
- Launch the **MZ-LiveNav** frontend automatically in your browser  
- Use `Ctrl + X` to stop all servers.

---

### Integration with `mz-infer`

You can query navigation status or turn-by-turn summaries from the `/nav_cmd` endpoint.  
Example response:

```json
{
  "destination_m": 291.757842024151,
  "message": "10th A Cross Road",
  "status": "success",
  "turn_m": 163.125590328045,
  "turn_type": "left"
}
```
