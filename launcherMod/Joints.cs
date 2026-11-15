// The active vessel's part joints and parts, as PhysX flies them, for the
// touchdown impact model (common/impact.py).  kRPC exposes neither a joint's
// break force nor its drive: both are on the live ConfigurableJoints KSP
// builds for each PartJoint (several per joint on a stack node), so they are
// read straight off those, summed per PartJoint.
using System.Collections.Generic;
using System.Globalization;
using KRPC.Service.Attributes;
using UnityEngine;

namespace KrpcLabLauncher
{
    [KRPCService(Name = "KrpcLabJoints", GameScene = KRPC.Service.GameScene.Flight)]
    public static class KrpcLabJoints
    {
        static readonly CultureInfo C = CultureInfo.InvariantCulture;

        static string F(float x) { return x.ToString("R", C); }

        /// <summary>One line per part of the active vessel:
        /// "index|parent index (-1 root)|title|mass t|crashTolerance m/s|
        /// comY|lowY|nJoints|breakForce kN|breakTorque kN m|linearSpring|
        /// linearDamper|angularSpring|angularDamper", positions along the
        /// vessel's own axis (+y the nose, origin the vessel's centre of mass,
        /// metres); the joint columns are the part's attachJoint to its parent,
        /// the forces summed over its ConfigurableJoints, the springs the
        /// sum of the drives' positionSpring (x/y/z or angular X/YZ) -- with
        /// PhysX's infinite (float.MaxValue) break values written as "inf".</summary>
        [KRPCProcedure]
        public static IList<string> Parts()
        {
            var rows = new List<string>();
            var v = FlightGlobals.ActiveVessel;
            if (v == null) return rows;
            var inv = Quaternion.Inverse(v.ReferenceTransform.rotation);
            var index = new Dictionary<Part, int>();
            for (int i = 0; i < v.parts.Count; i++) index[v.parts[i]] = i;
            for (int i = 0; i < v.parts.Count; i++)
            {
                var p = v.parts[i];
                float mass = p.Rigidbody != null ? p.Rigidbody.mass : p.mass + p.GetResourceMass();
                Vector3 com = inv * ((p.Rigidbody != null ? p.Rigidbody.worldCenterOfMass
                                      : p.transform.position) - (Vector3)v.CoMD);
                float low = float.PositiveInfinity;
                foreach (var col in p.GetComponentsInChildren<Collider>())
                {
                    if (col == null || !col.enabled || col.isTrigger) continue;
                    var b = col.bounds;
                    for (int k = 0; k < 8; k++)
                    {
                        var corner = new Vector3((k & 1) == 0 ? b.min.x : b.max.x,
                                                 (k & 2) == 0 ? b.min.y : b.max.y,
                                                 (k & 4) == 0 ? b.min.z : b.max.z);
                        float y = (inv * (corner - (Vector3)v.CoMD)).y;
                        if (y < low) low = y;
                    }
                }
                int n = 0;
                float bf = 0f, bt = 0f, ls = 0f, ld = 0f, asp = 0f, ad = 0f;
                bool infF = false, infT = false;
                var pj = p.attachJoint;
                if (pj != null && pj.joints != null)
                {
                    foreach (var j in pj.joints)
                    {
                        if (j == null) continue;
                        n++;
                        if (j.breakForce >= float.MaxValue * 0.5f || float.IsInfinity(j.breakForce)) infF = true;
                        else bf += j.breakForce;
                        if (j.breakTorque >= float.MaxValue * 0.5f || float.IsInfinity(j.breakTorque)) infT = true;
                        else bt += j.breakTorque;
                        ls += Mathf.Max(j.xDrive.positionSpring, Mathf.Max(j.yDrive.positionSpring, j.zDrive.positionSpring))
                              + j.linearLimitSpring.spring;
                        ld += Mathf.Max(j.xDrive.positionDamper, Mathf.Max(j.yDrive.positionDamper, j.zDrive.positionDamper))
                              + j.linearLimitSpring.damper;
                        asp += Mathf.Max(j.angularXDrive.positionSpring, j.angularYZDrive.positionSpring);
                        ad += Mathf.Max(j.angularXDrive.positionDamper, j.angularYZDrive.positionDamper);
                    }
                }
                int parent = (p.parent != null && index.ContainsKey(p.parent)) ? index[p.parent] : -1;
                rows.Add(string.Join("|", new[] {
                    i.ToString(C), parent.ToString(C),
                    p.partInfo != null ? p.partInfo.title.Replace("|", "/") : p.name,
                    F(mass), F((float)p.crashTolerance), F(com.y),
                    float.IsInfinity(low) ? F(com.y) : F(low), n.ToString(C),
                    infF ? "inf" : F(bf), infT ? "inf" : F(bt), F(ls), F(ld), F(asp), F(ad) }));
            }
            return rows;
        }
    }
}
