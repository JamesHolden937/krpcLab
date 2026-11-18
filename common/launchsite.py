"""Where the active vessel was launched from: kRPC's ``Vessel.launch_site``.

kRPC (main, #1363) reads the vessel's ``launchedFrom`` -- kept by every part
through decoupling, and by FMRS and quick saves -- and gives the launch
site's first spawn point.
"""


def launch_site(conn, vessel=None):
    """``{"source", "body", "lat", "lon", "alt", "site"}`` or None.

    None when kRPC has no ``launch_site`` (an older server, the offline
    fakes) or the vessel names no known site -- the caller falls back to its
    configured coordinates.
    """
    try:
        if vessel is None:
            vessel = conn.space_center.active_vessel
        site = vessel.launch_site
        if site is None:
            return None
        lat, lon, alt = site.latitude, site.longitude, site.altitude
        body = site.body.name
        name = site.name
    except Exception:                                       # noqa: BLE001
        return None
    if lat != lat or lon != lon:                            # NaN: no spawn point
        return None
    return {"source": "site", "body": body, "lat": float(lat),
            "lon": float(lon), "alt": float(alt), "site": name or ""}
