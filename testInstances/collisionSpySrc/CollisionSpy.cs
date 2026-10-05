// CollisionSpy: logs every collision a part of the active vessel takes, and
// every joint that breaks, to KSP.log as "[CollisionSpy] ...".  An instrument
// for the farm's camera instance: KSP itself logs only "X Exploded!!", never
// what the part hit or whether a joint gave way first.
using UnityEngine;

[KSPAddon(KSPAddon.Startup.Flight, false)]
public class CollisionSpyAddon : MonoBehaviour
{
    float next;
    // Joint watch: while any wheel of the active vessel is within
    // WatchAGL of the ground (or grounded), every physics step, the largest
    // attach-joint force on the vessel, which part's joint it is, and that
    // joint's break force -- one line per step, "[CollisionSpy] ... JOINT".
    const float WatchAGL = 4f;
    void JointWatch(Vessel v)
    {
        if (v.heightFromTerrain < 0 || v.heightFromTerrain > WatchAGL + 6f) return;
        float best = 0f, bestBreak = 0f; string who = "-";
        float bestT = 0f, bestTBreak = 0f; string whoT = "-";
        float wing = 0f, wingBreak = 0f, wingT = 0f, wingTBreak = 0f; int nj = 0;
        foreach (Part p in v.parts)
        {
            if (p == null || p.attachJoint == null || p.attachJoint.joints == null) continue;
            foreach (ConfigurableJoint j in p.attachJoint.joints)
            {
                if (j == null) continue;
                float f = j.currentForce.magnitude, t = j.currentTorque.magnitude;
                if (f / Mathf.Max(j.breakForce, 1f) > best / Mathf.Max(bestBreak, 1f)) { best = f; bestBreak = j.breakForce; who = p.partInfo.name; }
                if (t / Mathf.Max(j.breakTorque, 1f) > bestT / Mathf.Max(bestTBreak, 1f)) { bestT = t; bestTBreak = j.breakTorque; whoT = p.partInfo.name; }
                if (p.partInfo.name == "wingShuttleDelta")
                {
                    nj++;
                    if (f > wing) { wing = f; wingBreak = j.breakForce; }
                    if (t > wingT) { wingT = t; wingTBreak = j.breakTorque; }
                }
            }
        }
        Debug.Log("[CollisionSpy] ut=" + Planetarium.GetUniversalTime().ToString("F3")
            + " JOINT agl=" + v.heightFromTerrain.ToString("F2")
            + " F=" + best.ToString("F0") + "/" + bestBreak.ToString("F0") + " " + who
            + " T=" + bestT.ToString("F0") + "/" + bestTBreak.ToString("F0") + " " + whoT
            + " wingF=" + wing.ToString("F0") + "/" + wingBreak.ToString("F0")
            + " wingT=" + wingT.ToString("F0") + "/" + wingTBreak.ToString("F0")
            + " wingJoints=" + nj + " n=" + v.parts.Count);
    }
    void FixedUpdate()
    {
        Vessel av = FlightGlobals.ActiveVessel;
        if (av != null) JointWatch(av);
        if (Time.time < next) return;
        next = Time.time + 0.5f;
        Vessel v = FlightGlobals.ActiveVessel;
        if (v == null) return;
        foreach (Part p in v.parts)
            if (p != null && p.gameObject.GetComponent<CollisionSpy>() == null)
                p.gameObject.AddComponent<CollisionSpy>().part = p;
    }
}

public class CollisionSpy : MonoBehaviour
{
    public Part part;
    string Ut() { return Planetarium.GetUniversalTime().ToString("F3"); }
    void OnCollisionEnter(Collision c)
    {
        Part other = c.collider != null ? c.collider.GetComponentInParent<Part>() : null;
        Vector3 pt = c.contacts.Length > 0 ? c.contacts[0].point : Vector3.zero;
        float h = part != null && part.vessel != null
            ? (float)FlightGlobals.getAltitudeAtPos(pt) - (float)part.vessel.terrainAltitude : 0f;
        Debug.Log("[CollisionSpy] ut=" + Ut() + " ENTER part=" + (part ? part.partInfo.name : "?")
            + " pid=" + (part ? part.flightID.ToString() : "?")
            + " other=" + (c.collider ? c.collider.name : "?")
            + " otherPart=" + (other ? other.partInfo.name + "/" + other.flightID : "-")
            + " sameVessel=" + (other && part && other.vessel == part.vessel)
            + " relV=" + c.relativeVelocity.magnitude.ToString("F2")
            + " impulse=" + c.impulse.magnitude.ToString("F1")
            + " contactAGL=" + h.ToString("F2")
            + " crashTol=" + (part ? part.crashTolerance.ToString("F0") : "?"));
    }
    void OnJointBreak(float force)
    {
        Debug.Log("[CollisionSpy] ut=" + Ut() + " JOINTBREAK part=" + (part ? part.partInfo.name : "?")
            + " pid=" + (part ? part.flightID.ToString() : "?")
            + " parent=" + (part && part.parent ? part.parent.partInfo.name : "-")
            + " force=" + force.ToString("F0"));
    }
}
