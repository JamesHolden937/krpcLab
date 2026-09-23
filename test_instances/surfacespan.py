"""surfacespan.py <rpc> <stream> -- which control surfaces are VERTICAL.

The identification half of an airbrake.  A pair of vertical surfaces
deployed in opposite directions is a split rudder: their side forces act at
equal and opposite moment arms, so yaw and roll both cancel and what is left
is drag.  Horizontal surfaces -- elevons, canards -- cannot be used that way,
because cancelling their pitching moments needs a balance of areas and arms
that is different on every aircraft, and this autopilot's bar is any craft a
player could fly the entry in.

So the rule is geometric and refuses rather than guesses:
  * span axis from the part's rotation quaternion;
  * VERTICAL when |span.z| > |span.x| in the vessel frame;
  * require a pair mirrored about the centreline (x-sum ~ 0);
  * no such pair -> no airbrake, and the vehicle flies as it does now.

Measured on ``qs_plane``: the wingtip ``smallCtrlSrf`` pair at x=+-2.50 is
vertical; the ``elevon2`` pair at x=+-1.67 and the canards at y=+2.76 are
not.
"""
import sys, math
sys.path.insert(0, "/home/holden/boosterlandKSP")
import krpc
conn = krpc.connect(name="span", address="127.0.0.1",
                    rpc_port=int(sys.argv[1]), stream_port=int(sys.argv[2]))
v = conn.space_center.active_vessel
f = v.reference_frame

def axes(q):
    """Part local axes in the given frame, from the rotation quaternion."""
    x, y, z, w = q
    def rot(vx, vy, vz):
        # q * v * q^-1
        tx, ty, tz = (2*(y*vz - z*vy), 2*(z*vx - x*vz), 2*(x*vy - y*vx))
        return (vx + w*tx + (y*tz - z*ty),
                vy + w*ty + (z*tx - x*tz),
                vz + w*tz + (x*ty - y*tx))
    return rot(1,0,0), rot(0,1,0), rot(0,0,1)

print("%-12s %-14s %-20s %-26s %s" % ("part","name","position","span axis (part x)","verdict"))
for cs in v.parts.control_surfaces:
    p = cs.part
    pos = p.position(f)
    q = p.rotation(f)
    ax, ay, az = axes(q)
    # In the vessel frame z is the dorsal/ventral axis.  A surface whose span
    # runs along z is VERTICAL; along x it is horizontal.
    vert = abs(ax[2]) > abs(ax[0])
    print("%-12s %-14s (%+.2f,%+.2f,%+.2f) (%+.2f,%+.2f,%+.2f)   %s"
          % (p.title[:12], p.name[:14], pos[0], pos[1], pos[2],
             ax[0], ax[1], ax[2], "VERTICAL" if vert else "horizontal"))
