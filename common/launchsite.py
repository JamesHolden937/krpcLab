"""Where the active vessel was launched from, from the krpcLab window mod.

``launcherMod/LaunchSite.cs`` records each launch's position against the root
part's ``launchID`` (kept by every part through decoupling, and by FMRS and
quick saves); failing a record it reads the vessel's ``launchedFrom`` site.
"""

SERVICE = "krpc_lab_launch"


def launch_site(conn):
    """``{"source", "body", "lat", "lon", "alt", "site"}`` or None.

    None when the mod is not installed or neither a record nor the site is
    known -- the caller falls back to its configured coordinates.
    """
    service = getattr(conn, SERVICE, None)
    if service is None:
        return None
    try:
        text = service.launch_site()
    except Exception:                                       # noqa: BLE001
        return None
    fields = text.split("|") if text else []
    if len(fields) < 6:
        return None
    try:
        return {"source": fields[0], "body": fields[1], "lat": float(fields[2]),
                "lon": float(fields[3]), "alt": float(fields[4]),
                "site": fields[5]}
    except ValueError:
        return None
