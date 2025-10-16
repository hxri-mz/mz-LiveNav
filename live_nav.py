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
LATEST_GNSS = None
LATEST_GNSS_LOCK = Lock()

active_route_id = None
ROUTES = {}
ROUTES_LOCK = Lock()


def haversine_m(lon1, lat1, lon2, lat2):
    lon1_r, lat1_r, lon2_r, lat2_r = map(math.radians, (lon1, lat1, lon2, lat2))
    dlon = lon2_r - lon1_r
    dlat = lat2_r - lat1_r
    a = math.sin(dlat/2)**2 + math.cos(lat1_r)*math.cos(lat2_r)*math.sin(dlon/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return 6371000.0 * c


def point_segment_projection(lon, lat, lon1, lat1, lon2, lat2):
    lat0 = math.radians((lat1 + lat2)/2.0)
    def xy_from(lon_, lat_):
        x = math.radians(lon_) * math.cos(lat0) * 6371000.0
        y = math.radians(lat_) * 6371000.0
        return x, y
    px, py = xy_from(lon, lat)
    ax, ay = xy_from(lon1, lat1)
    bx, by = xy_from(lon2, lat2)
    dx, dy = bx-ax, by-ay
    if dx==0 and dy==0:
        return lon1, lat1, 0.0
    t = ((px-ax)*dx + (py-ay)*dy)/(dx*dx+dy*dy)
    t = max(0.0, min(1.0, t))
    lon_p = lon1 + (lon2-lon1)*t
    lat_p = lat1 + (lat2-lat1)*t
    return lon_p, lat_p, t


def decode_polyline(polyline_str):
    index, lat, lng = 0, 0, 0
    coordinates = []
    str_len = len(polyline_str)
    while index < str_len:
        shift = 0; result = 0
        while True:
            b = ord(polyline_str[index])-63; index+=1
            result |= (b & 0x1f) << shift
            shift += 5
            if b < 0x20: break
        dlat = ~(result>>1) if result&1 else (result>>1)
        lat += dlat
        shift = 0; result = 0
        while True:
            b = ord(polyline_str[index])-63; index+=1
            result |= (b & 0x1f) << shift
            shift +=5
            if b<0x20: break
        dlng = ~(result>>1) if result&1 else (result>>1)
        lng += dlng
        coordinates.append((lng/1e5, lat/1e5))
    return coordinates


def cumulative_distances_along(poly):
    cuml = [0.0]
    for i in range(1, len(poly)):
        cuml.append(cuml[-1] + haversine_m(*poly[i-1], *poly[i]))
    return cuml


def find_projection_on_polyline(lon, lat, poly, cuml):
    best_dist = float('inf')
    best_proj = None
    best_seg_idx = None
    best_t = None
    for i in range(1,len(poly)):
        lon_p, lat_p, t = point_segment_projection(lon, lat, *poly[i-1], *poly[i])
        d = haversine_m(lon, lat, lon_p, lat_p)
        if d < best_dist:
            best_dist = d
            seg_length = cuml[i]-cuml[i-1] if cuml else 0.0
            proj_along = cuml[i-1]+t*seg_length if cuml else 0.0
            best_proj = (lon_p, lat_p)
            best_seg_idx = i
            best_t = t
    if best_proj is None:
        return 0.0, 1, poly[0][0], poly[0][1]
    return proj_along, best_seg_idx, best_proj[0], best_proj[1]


def request_route_from_osrm_multi(waypoints):
    if len(waypoints)<2: raise ValueError("At least 2 waypoints")
    coord_str = ";".join([f"{lon},{lat}" for lon,lat in waypoints])
    params = {"overview":OVERVIEW, "geometries":GEOMETRY, "steps":STEPS}
    url = f"{ROUTE_SERVICE}/{coord_str}"
    resp = requests.get(url, params=params, timeout=10)
    resp.raise_for_status()
    data = resp.json()
    if not data.get("routes"): raise ValueError("No routes returned by OSRM")
    route = data["routes"][0]

    geom = route.get("geometry")
    if isinstance(geom, dict) and "coordinates" in geom:
        poly = [(pt[0],pt[1]) for pt in geom["coordinates"]]
    elif isinstance(geom,str):
        poly = decode_polyline(geom)
    else:
        poly = [(pt[0],pt[1]) for pt in route["geometry"]["coordinates"]]

    cuml = cumulative_distances_along(poly)

    # Extract only turns, ignore waypoints as maneuvers
    maneuvers=[]
    for leg in route.get("legs",[]):
        for step in leg.get("steps",[]):
            man = step.get("maneuver")
            if man and man.get("type") in ["turn","new name","roundabout"]:
                loc = man.get("location")
                if loc:
                    best_along = find_projection_on_polyline(loc[0], loc[1], poly, cuml)[0]
                    maneuvers.append({
                        "instruction": step.get("name") or "",
                        "type": man.get("type"),
                        "modifier": man.get("modifier"),
                        "location": loc,
                        "distance_along": best_along
                    })
    maneuvers.sort(key=lambda m: m["distance_along"])
    return {
        "poly": poly,
        "cumulative": cuml,
        "maneuvers": maneuvers,
        "distance": route.get("distance"),
        "duration": route.get("duration"),
        "geometry": [[p[0],p[1]] for p in poly]
    }


@app.route("/route", methods=["POST"])
def create_route():
    body = request.get_json()
    if not body or "waypoints" not in body:
        abort(400, "Missing 'waypoints'")
    waypoints = body["waypoints"]
    if len(waypoints)<2: abort(400,"Need at least 2 waypoints")
    try:
        parsed = request_route_from_osrm_multi(waypoints)
    except Exception as e:
        abort(500, f"OSRM error: {e}")
    route_id=str(uuid.uuid4())
    with ROUTES_LOCK:
        ROUTES[route_id]=parsed
    return jsonify({
        "route_id": route_id,
        "distance_m": parsed["distance"],
        "duration_s": parsed["duration"],
        "maneuvers":[
            {"instruction":m["instruction"],"type":m["type"],"modifier":m["modifier"],
             "location":[m["location"][0],m["location"][1]],"distance_along_m":m["distance_along"]}
            for m in parsed["maneuvers"]
        ],
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
        LATEST_GNSS={"lon":lon,"lat":lat}
    with ROUTES_LOCK:
        route=ROUTES.get(route_id)
    if not route: abort(404,"Unknown route_id")
    poly = route["poly"]
    cuml = route["cumulative"]
    maneuvers = route["maneuvers"]

    proj_along, seg_idx, lon_p, lat_p = find_projection_on_polyline(lon, lat, poly, cuml)
    remaining_to_end = cuml[-1]-proj_along if cuml else 0.0

    next_man=None
    buffer_m=0.5
    for m in maneuvers:
        if m["distance_along"]>proj_along+buffer_m:
            next_man=m
            break

    if next_man:
        distance_to_next=next_man["distance_along"]-proj_along
        next_maneuver_info={
            "instruction":next_man["instruction"],
            "type":next_man["type"],
            "modifier":next_man["modifier"],
            "location":[next_man["location"][0],next_man["location"][1]],
            "distance_to_maneuver_m":distance_to_next
        }
    else:
        distance_to_next=None
        next_maneuver_info=None

    return jsonify({
        "projected_point":[lon_p,lat_p],
        "distance_to_next_m":distance_to_next,
        "next_maneuver":next_maneuver_info,
        "remaining_distance_m":remaining_to_end
    })


@app.route("/latest_position", methods=["GET"])
def latest_position():
    with LATEST_GNSS_LOCK:
        if LATEST_GNSS is None: return jsonify({"error":"No GNSS data yet"}),404
        return jsonify(LATEST_GNSS)


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status":"ok","routes_stored":len(ROUTES)})


@app.route("/set_route_id", methods=["POST"])
def set_route_id():
    global active_route_id
    data=request.get_json()
    route_id=data.get("route_id")
    if not route_id: abort(400,"Missing route_id")
    active_route_id=route_id
    return jsonify({"status":"ok","route_id":route_id})


@app.route("/update_gnss", methods=["POST"])
def update_gnss():
    global LATEST_GNSS
    data=request.get_json()
    if not data or "lat" not in data or "lon" not in data:
        return jsonify({"error":"Missing lat/lon"}),400
    with LATEST_GNSS_LOCK:
        LATEST_GNSS={"lat":data["lat"],"lon":data["lon"]}
    return jsonify({"status":"ok"})


if __name__=="__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
