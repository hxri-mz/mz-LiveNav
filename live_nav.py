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
NO_PROGRESS_LIMIT = 4
NO_PROGRESS_EPS_M = 1.0
WAYPOINT_PASSED_BUFFER_M = 5.0

LATEST_GNSS = None
LATEST_GNSS_LOCK = Lock()

active_route_id = None
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


def point_segment_projection(lon, lat, lon1, lat1, lon2, lat2):
    lat0 = math.radians((lat1 + lat2) / 2.0)
    def xy_from(lon_, lat_):
        x = math.radians(lon_) * math.cos(lat0) * 6371000.0
        y = math.radians(lat_) * 6371000.0
        return (x, y)
    px, py = xy_from(lon, lat)
    ax, ay = xy_from(lon1, lat1)
    bx, by = xy_from(lon2, lat2)
    dx = bx - ax
    dy = by - ay
    if dx == 0 and dy == 0:
        return lon1, lat1, 0.0
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t_clamped = max(0.0, min(1.0, t))
    lon_proj = lon1 + (lon2 - lon1) * t_clamped
    lat_proj = lat1 + (lat2 - lat1) * t_clamped
    return lon_proj, lat_proj, t_clamped


def decode_polyline(polyline_str):
    index, lat, lng = 0, 0, 0
    coordinates = []
    str_len = len(polyline_str)
    while index < str_len:
        shift = 0
        result = 0
        while True:
            b = ord(polyline_str[index]) - 63
            index += 1
            result |= (b & 0x1f) << shift
            shift += 5
            if b < 0x20: break
        dlat = ~(result >> 1) if (result & 1) else (result >> 1)
        lat += dlat

        shift = 0
        result = 0
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


def cumulative_distances_along(poly):
    if not poly: return []
    cum = [0.0]
    for i in range(1, len(poly)):
        d = haversine_m(poly[i-1][0], poly[i-1][1], poly[i][0], poly[i][1])
        cum.append(cum[-1] + d)
    return cum


def find_projection_on_polyline(lon, lat, poly, cum_dist):
    if not poly:
        return 0.0, 0, lon, lat

    best_dist = float('inf')
    best_proj = None
    best_seg_idx = None
    best_t = None
    proj_along = 0.0

    for i in range(1, len(poly)):
        a_lon, a_lat = poly[i-1]
        b_lon, b_lat = poly[i]
        lon_p, lat_p, t = point_segment_projection(lon, lat, a_lon, a_lat, b_lon, b_lat)
        d_to_proj = haversine_m(lon, lat, lon_p, lat_p)
        if d_to_proj < best_dist:
            best_dist = d_to_proj
            seg_length = cum_dist[i] - cum_dist[i-1] if cum_dist else 0.0
            proj_along = cum_dist[i-1] + t * seg_length if cum_dist else 0.0
            best_proj = (lon_p, lat_p)
            best_seg_idx = i
            best_t = t

    if best_proj is None:
        return 0.0, 1, poly[0][0], poly[0][1]
    return proj_along, best_seg_idx, best_proj[0], best_proj[1]


def request_route_from_osrm_multi(waypoints):
    if len(waypoints) < 2:
        raise ValueError("At least 2 waypoints required")
    coords = ";".join([f"{lon},{lat}" for lon, lat in waypoints])
    params = {"overview": OVERVIEW, "geometries": GEOMETRY, "steps": STEPS}
    url = f"{ROUTE_SERVICE}/{coords}"
    resp = requests.get(url, params=params, timeout=15)
    resp.raise_for_status()
    data = resp.json()
    if not data.get("routes"): raise ValueError("No routes returned by OSRM.")
    route = data["routes"][0]

    geom = route.get("geometry")
    if isinstance(geom, dict) and "coordinates" in geom:
        poly = [(pt[0], pt[1]) for pt in geom["coordinates"]]
    elif isinstance(geom, str):
        poly = decode_polyline(geom)
    else:
        try: poly = [(pt[0], pt[1]) for pt in route["geometry"]["coordinates"]]
        except Exception: raise ValueError("Unsupported geometry format from OSRM.")

    cuml = cumulative_distances_along(poly)

    maneuvers = []
    for leg in route.get("legs", []):
        for step in leg.get("steps", []):
            maneuver = step.get("maneuver", {})
            man_type = maneuver.get("type")
            if man_type in ("turn", "new name", "roundabout", "fork", "merge", "ramp"):
                loc = maneuver.get("location")
                if not loc: continue
                best_along = find_projection_on_polyline(loc[0], loc[1], poly, cuml)[0]
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
        "waypoints": waypoints
    }


@app.route("/route", methods=["POST"])
def create_route():
    body = request.get_json()
    if not body: abort(400, "Missing JSON body")
    if "origin" in body and "destination" in body:
        waypoints = [body["origin"], body["destination"]]
    else:
        waypoints = body.get("waypoints")
    if not waypoints or len(waypoints) < 2: abort(400, "Need at least 2 waypoints")
    try:
        parsed = request_route_from_osrm_multi(waypoints)
    except Exception as e:
        abort(500, f"OSRM error: {e}")
    route_id = str(uuid.uuid4())
    parsed["last_remaining"] = None
    parsed["no_progress_count"] = 0
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

    proj_along, seg_idx, lon_p, lat_p = find_projection_on_polyline(lon, lat, poly, cuml)
    dist_to_route = haversine_m(lon, lat, lon_p, lat_p)
    remaining_to_end = (cuml[-1] - proj_along) if cuml else 0.0

    buffer_m = 0.0
    next_man = None
    for m in maneuvers:
        if m["distance_along"] > proj_along + buffer_m:
            next_man = m
            break

    last_remaining = route.get("last_remaining")
    no_progress_count = route.get("no_progress_count", 0)
    rerouted = False
    new_route_id = None
    new_route_info = None

    if last_remaining is None:
        route["last_remaining"] = remaining_to_end
        route["no_progress_count"] = 0
    else:
        if remaining_to_end >= last_remaining - NO_PROGRESS_EPS_M:
            no_progress_count += 1
        else:
            no_progress_count = 0
        route["last_remaining"] = remaining_to_end
        route["no_progress_count"] = no_progress_count

    # -------------------- Forward-only reroute -------------------- #
    if dist_to_route > REROUTE_THRESHOLD_M or no_progress_count >= NO_PROGRESS_LIMIT:
        original_waypoints = route.get("waypoints", []) or []
        remaining_waypoints = []
        try:
            for wp in original_waypoints:
                wp_lon, wp_lat = wp
                wp_along = find_projection_on_polyline(wp_lon, wp_lat, poly, cuml)[0]
                if wp_along > proj_along + WAYPOINT_PASSED_BUFFER_M:
                    remaining_waypoints.append([wp_lon, wp_lat])
        except Exception:
            remaining_waypoints = []

        # If nothing ahead, just use final destination
        if not remaining_waypoints:
            remaining_waypoints = [original_waypoints[-1]]

        new_waypoints = [[lon, lat]] + remaining_waypoints
        try:
            parsed_new = request_route_from_osrm_multi(new_waypoints)
            new_route_id = str(uuid.uuid4())
            parsed_new["last_remaining"] = None
            parsed_new["no_progress_count"] = 0
            with ROUTES_LOCK:
                ROUTES[new_route_id] = parsed_new
            rerouted = True
            new_route_info = parsed_new
        except Exception as e:
            print("Reroute failed:", e)
            rerouted = False
            new_route_info = None
            new_route_id = None
        route["no_progress_count"] = 0

    if rerouted and new_route_info:
        poly = new_route_info["poly"]
        cuml = new_route_info["cumulative"]
        maneuvers = new_route_info["maneuvers"]
        proj_along, seg_idx, lon_p, lat_p = find_projection_on_polyline(lon, lat, poly, cuml)
        remaining_to_end = (cuml[-1] - proj_along) if cuml else 0.0
        next_man = None
        for m in maneuvers:
            if m["distance_along"] > proj_along + buffer_m:
                next_man = m
                break

    if next_man:
        distance_to_next = max(0.0, haversine_m(lon_p, lat_p,
                                               next_man["location"][0],
                                               next_man["location"][1]))
        next_maneuver_info = {
            "instruction": next_man.get("instruction"),
            "name": next_man.get("name"),
            "type": next_man.get("type"),
            "modifier": next_man.get("modifier"),
            "location": [next_man["location"][0], next_man["location"][1]],
            "distance_to_maneuver_m": distance_to_next
        }
    else:
        distance_to_next = None
        next_maneuver_info = None

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


@app.route('/set_route_id', methods=['POST'])
def set_route_id():
    global active_route_id
    data = request.get_json()
    route_id = data.get("route_id")
    if not route_id: abort(400, "Missing route_id")
    active_route_id = route_id
    return jsonify({"status": "ok", "route_id": route_id})


@app.route("/update_gnss", methods=["POST"])
def update_gnss():
    global LATEST_GNSS
    data = request.get_json()
    if not data or "lat" not in data or "lon" not in data:
        return jsonify({"error": "Missing lat/lon"}), 400
    with LATEST_GNSS_LOCK:
        LATEST_GNSS = {"lat": data["lat"], "lon": data["lon"], "yaw": data.get("yaw")}
    return jsonify({"status": "ok"})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
