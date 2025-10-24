#!/usr/bin/env python3
from flask import Flask, request, jsonify, abort
import requests
import uuid
import math
from threading import Lock

app = Flask(__name__)

OSRM_BASE_URL = "http://router.project-osrm.org"
ROUTE_SERVICE = f"{OSRM_BASE_URL}/route/v1/driving"
OVERVIEW = "full"
GEOMETRY = "geojson"
STEPS = "true"

REROUTE_THRESHOLD_M = 20.0
WAYPOINT_PASSED_BUFFER_M = 5.0
STEP_M = 0.5 # Interpolation resolution in centimeters 
WINDOW_SIZE = 20
ENABLE_REROUTE = False

LATEST_GNSS = None
LATEST_GNSS_LOCK = Lock()

NAV_CMD = {"status": None, "turn_type": None, "turn_m": None, "destination_m": None, "message": None}
NAV_CMD_LOCK = Lock()

ROUTES = {}
ROUTES_LOCK = Lock()

def haversine_m(lon1, lat1, lon2, lat2):
    lon1_r, lat1_r, lon2_r, lat2_r = map(math.radians, (lon1, lat1, lon2, lat2))
    dlon = lon2_r - lon1_r
    dlat = lat2_r - lat1_r
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    R = 6371000.0
    return R * c

def decode_polyline(polyline_str):
    index, lat, lng = 0, 0, 0
    coordinates = []
    str_len = len(polyline_str)
    while index < str_len:
        shift, result = 0, 0
        while True:
            b = ord(polyline_str[index]) - 63
            index += 1
            result |= (b & 0x1f) << shift
            shift += 5
            if b < 0x20: break
        dlat = ~(result >> 1) if (result & 1) else (result >> 1)
        lat += dlat
        shift, result = 0, 0
        while True:
            b = ord(polyline_str[index]) - 63
            index += 1
            result |= (b & 0x1f) << shift
            shift += 5
            if b < 0x20: break
        dlng = ~(result >> 1) if (result & 1) else (result >> 1)
        lng += dlng
        coordinates.append((lng / 1e5, lat / 1e5))
    return coordinates

def densify_polyline(poly, step_m=1.0):
    dense = [poly[0]]
    for i in range(1, len(poly)):
        lon1, lat1 = poly[i - 1]
        lon2, lat2 = poly[i]
        dist = haversine_m(lon1, lat1, lon2, lat2)
        n_points = int(dist // step_m)
        for j in range(1, n_points):
            t = j / n_points
            lon_i = lon1 + (lon2 - lon1) * t
            lat_i = lat1 + (lat2 - lat1) * t
            dense.append((lon_i, lat_i))
        dense.append((lon2, lat2))
    return dense

def cumulative_distances_along(poly):
    if not poly: return []
    cum = [0.0]
    for i in range(1, len(poly)):
        d = haversine_m(poly[i-1][0], poly[i-1][1], poly[i][0], poly[i][1])
        cum.append(cum[-1] + d)
    return cum

def find_projection_on_polyline_window(lon, lat, poly, cum_dist, last_idx=0, window_size=WINDOW_SIZE):
    n = len(poly)
    if not poly:
        return 0.0, 0, lon, lat
    start_idx = last_idx
    end_idx = min(n, last_idx + window_size)
    best_dist = float('inf')
    best_idx = last_idx
    for i in range(start_idx, end_idx):
        p_lon, p_lat = poly[i]
        d = haversine_m(lon, lat, p_lon, p_lat)
        if d < best_dist:
            best_dist = d
            best_idx = i
    proj_along = cum_dist[best_idx] if cum_dist else 0.0
    best_lon, best_lat = poly[best_idx]
    return proj_along, best_idx, best_lon, best_lat

def request_route_from_osrm_multi(waypoints):
    if len(waypoints) < 2:
        raise ValueError("At least 2 waypoints required")
    coords = ";".join([f"{lon},{lat}" for lon, lat in waypoints])
    params = {"overview": OVERVIEW, "geometries": GEOMETRY, "steps": STEPS}
    url = f"{ROUTE_SERVICE}/{coords}"
    resp = requests.get(url, params=params, timeout=15)
    resp.raise_for_status()
    data = resp.json()
    if not data.get("routes"):
        raise ValueError("No routes returned by OSRM.")
    route = data["routes"][0]
    geom = route.get("geometry")
    if isinstance(geom, dict) and "coordinates" in geom:
        poly = [(pt[0], pt[1]) for pt in geom["coordinates"]]
    elif isinstance(geom, str):
        poly = decode_polyline(geom)
    else:
        try:
            poly = [(pt[0], pt[1]) for pt in route["geometry"]["coordinates"]]
        except Exception:
            raise ValueError("Unsupported geometry format from OSRM.")
    poly = densify_polyline(poly, step_m=STEP_M)
    cuml = cumulative_distances_along(poly)
    maneuvers = []
    for leg in route.get("legs", []):
        for step in leg.get("steps", []):
            maneuver = step.get("maneuver", {})
            man_type = maneuver.get("type")
            if man_type in ("turn", "new name", "roundabout", "fork", "merge", "ramp"):
                loc = maneuver.get("location")
                if not loc: continue
                best_along = find_projection_on_polyline_window(loc[0], loc[1], poly, cuml, 0, len(poly))[0]
                maneuvers.append({
                    "instruction": step.get("name") or step.get("ref") or "",
                    "name": step.get("name", ""),
                    "type": man_type,
                    "modifier": maneuver.get("modifier"),
                    "location": (loc[0], loc[1]),
                    "distance_along": best_along
                })
    maneuvers.sort(key=lambda m: m["distance_along"])
    return {
        "poly": poly,
        "cumulative": cuml,
        "maneuvers": maneuvers,
        "distance": route.get("distance"),
        "duration": route.get("duration"),
        "geometry": [[p[0], p[1]] for p in poly],
        "waypoints": waypoints,
        "last_proj_idx": 0,
        "no_progress_count": 0
    }

@app.route("/route", methods=["POST"])
def create_route():
    body = request.get_json()
    if not body: abort(400, "Missing JSON body")
    waypoints = [body["origin"], body["destination"]] if "origin" in body and "destination" in body else body.get("waypoints")
    if not waypoints or len(waypoints) < 2: abort(400, "Need at least 2 waypoints")
    try:
        parsed = request_route_from_osrm_multi(waypoints)
    except Exception as e:
        abort(500, f"OSRM error: {e}")
    route_id = str(uuid.uuid4())
    with ROUTES_LOCK:
        ROUTES[route_id] = parsed
    man_summary = [{
        "instruction": m["instruction"],
        "name": m["name"],
        "type": m["type"],
        "modifier": m["modifier"],
        "location": [m["location"][0], m["location"][1]],
        "distance_along_m": m["distance_along"]
    } for m in parsed["maneuvers"]]
    return jsonify({
        "route_id": route_id,
        "distance_m": parsed["distance"],
        "duration_s": parsed["duration"],
        "maneuvers": man_summary,
        "geometry": parsed["geometry"]
    })

@app.route("/position", methods=["POST"])
def update_position():
    remaining_to_end = 0.0
    global LATEST_GNSS
    body = request.get_json()
    if not body or "route_id" not in body or "position" not in body:
        abort(400, "Missing route_id or position")
    route_id = body["route_id"]
    lon, lat = body["position"]
    with LATEST_GNSS_LOCK:
        LATEST_GNSS = {"lon": lon, "lat": lat}
    with ROUTES_LOCK:
        route = ROUTES.get(route_id)
    if route is None: abort(404, "Unknown route_id")

    poly = route["poly"]
    cuml = route["cumulative"]
    maneuvers = route["maneuvers"]
    last_idx = route.get("last_proj_idx", 0)

    proj_along, seg_idx, lon_p, lat_p = find_projection_on_polyline_window(lon, lat, poly, cuml, last_idx, WINDOW_SIZE)
    route["last_proj_idx"] = seg_idx
    dist_to_route = haversine_m(lon, lat, lon_p, lat_p)
    remaining_to_end = cuml[-1] - proj_along if cuml else 0.0

    next_man = None
    buffer_m = 0.0
    for m in maneuvers:
        if m["distance_along"] > proj_along + buffer_m:
            next_man = m
            break

    rerouted = False
    new_route_info = None
    new_route_id = None
    if ENABLE_REROUTE and dist_to_route > REROUTE_THRESHOLD_M:
        original_waypoints = route.get("waypoints", []) or []
        remaining_waypoints = []
        for wp in original_waypoints:
            wp_lon, wp_lat = wp
            wp_along, _, _, _ = find_projection_on_polyline_window(wp_lon, wp_lat, poly, cuml, seg_idx, len(poly)-seg_idx)
            if wp_along > proj_along + WAYPOINT_PASSED_BUFFER_M:
                remaining_waypoints.append([wp_lon, wp_lat])
        if not remaining_waypoints:
            remaining_waypoints = [original_waypoints[-1]]
        new_waypoints = [[lon, lat]] + remaining_waypoints
        try:
            parsed_new = request_route_from_osrm_multi(new_waypoints)
            new_route_id = str(uuid.uuid4())
            with ROUTES_LOCK:
                ROUTES[new_route_id] = parsed_new
            rerouted = True
            new_route_info = parsed_new
            route["poly"] = parsed_new["poly"]
            route["cumulative"] = parsed_new["cumulative"]
            route["maneuvers"] = parsed_new["maneuvers"]
            route["geometry"] = [[p[0], p[1]] for p in parsed_new["poly"]]
            route["waypoints"] = parsed_new["waypoints"]
            route["last_proj_idx"] = 0
            poly = route["poly"]
            cuml = route["cumulative"]
            maneuvers = route["maneuvers"]
            proj_along, seg_idx, lon_p, lat_p = find_projection_on_polyline_window(lon, lat, poly, cuml, 0, len(poly))
            route["last_proj_idx"] = seg_idx
            remaining_to_end = cuml[-1] - proj_along if cuml else 0.0
            next_man = None
            for m in maneuvers:
                if m["distance_along"] > proj_along + buffer_m:
                    next_man = m
                    break
        except Exception as e:
            print("Reroute failed:", e)
            rerouted = False
            new_route_info = None

    distance_to_next = None
    next_maneuver_info = None
    if next_man:
        distance_to_next = max(0.0, haversine_m(lon_p, lat_p, next_man["location"][0], next_man["location"][1]))
        next_maneuver_info = {
            "instruction": next_man.get("instruction"),
            "name": next_man.get("name"),
            "type": next_man.get("type"),
            "modifier": next_man.get("modifier"),
            "location": [next_man["location"][0], next_man["location"][1]],
            "distance_to_maneuver_m": distance_to_next
        }

    with NAV_CMD_LOCK:
        if remaining_to_end != 0.0 and next_man and dist_to_route < REROUTE_THRESHOLD_M:
            man_type = (next_man.get("type") or "").lower()
            modifier = (next_man.get("modifier") or "").lower()
            turn = "straight"
            if "turn" in man_type:
                if modifier in ("left", "slight left", "sharp left"):
                    turn = "left"
                elif modifier in ("right", "slight right", "sharp right"):
                    turn = "right"
            NAV_CMD["status"] = "success"
            NAV_CMD["turn_type"] = turn
            NAV_CMD["turn_m"] = distance_to_next
            NAV_CMD["destination_m"] = remaining_to_end
            NAV_CMD["message"] = next_man.get("instruction")
        else:
            NAV_CMD["status"] = "error"
            NAV_CMD["turn_type"] = ""
            NAV_CMD["turn_m"] = 0.0
            NAV_CMD["destination_m"] = ""
            NAV_CMD["message"] = "Went away from planned route."

    resp = {
        "projected_point": [lon_p, lat_p],
        "distance_to_next_m": distance_to_next,
        "next_maneuver": next_maneuver_info,
        "remaining_distance_m": remaining_to_end,
        "rerouted": rerouted
    }
    if rerouted and new_route_info:
        resp.update({
            "new_route_id": new_route_id,
            "geometry": new_route_info["geometry"],
            "maneuvers": [{
                "instruction": m["instruction"],
                "name": m.get("name", ""),
                "type": m.get("type"),
                "modifier": m.get("modifier"),
                "location": [m["location"][0], m["location"][1]],
                "distance_along": m["distance_along"]
            } for m in new_route_info["maneuvers"]],
            "distance_m": new_route_info.get("distance"),
            "duration_s": new_route_info.get("duration")
        })
    return jsonify(resp)

@app.route("/latest_position", methods=["GET"])
def latest_position():
    with LATEST_GNSS_LOCK:
        if LATEST_GNSS is None:
            return jsonify({"error": "No GNSS data yet"}), 404
        return jsonify(LATEST_GNSS)

@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok", "routes_stored": len(ROUTES)})

@app.route("/update_gnss", methods=["POST"])
def update_gnss():
    global LATEST_GNSS
    data = request.get_json()
    if not data or "lat" not in data or "lon" not in data:
        return jsonify({"error": "Missing lat/lon"}), 400
    with LATEST_GNSS_LOCK:
        LATEST_GNSS = {"lat": data["lat"], "lon": data["lon"], "yaw": data.get("yaw")}
    return jsonify({"status": "ok"})

@app.route("/clear_route", methods=["POST"])
def clear_route():
    body = request.get_json(silent=True) or {}
    route_id = body.get("route_id", None)

    with ROUTES_LOCK:
        if route_id:
            ROUTES.pop(route_id, None)
        else:
            ROUTES.clear()

    with NAV_CMD_LOCK:
        NAV_CMD["status"] = None
        NAV_CMD["turn_type"] = None
        NAV_CMD["turn_m"] = None
        NAV_CMD["destination_m"] = None
        NAV_CMD["message"] = None

    return jsonify({"status": "ok", "cleared_route_id": route_id})


@app.route("/nav_cmd", methods=["GET"])
def get_next_turn():
    with NAV_CMD_LOCK:
        if NAV_CMD["turn_type"] is None:
            # return jsonify({"error": "No turn info available"}), 404
            NAV_CMD["status"] = "error"
            NAV_CMD["turn_type"] = ""
            NAV_CMD["turn_m"] = 0.0
            NAV_CMD["destination_m"] = ""
            NAV_CMD["message"] = "Route not created yet"
            return jsonify(NAV_CMD)
        return jsonify(NAV_CMD)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
