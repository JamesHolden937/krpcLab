// Startup speed-up for kRPC: caches the reflection kRPC repeats while it scans
// its services.
//
// kRPC's KRPC.Utils.Reflection.AllTypes() calls Assembly.GetTypes() on every
// loaded assembly each time it is called, and Reflection.GetType(name) walks
// AllTypes() once per <see cref> in kRPC's documentation (~1000 of them).
// Under Mono each GetTypes() pays a slow System.Type lookup per type, so with
// a large mod list this held the main thread for ~26 s right after kRPC loaded
// its docs (the black screen at launch), and again for ~28 s when kRPC's
// main-menu addon started.
//
// The patches keep kRPC's own semantics: AllTypes still skips assemblies whose
// name holds "Assembly-CSharp" and applies the caller's filter, keeps only the
// loadable types of an assembly that throws ReflectionTypeLoadException, and
// GetType still returns the first type in AllTypes() order whose full name
// ('+' read as '.') matches, else null.  Only GetTypes() results are reused;
// they are rebuilt for any assembly loaded later.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using HarmonyLib;
using UnityEngine;

namespace KrpcLab
{
    [KSPAddon(KSPAddon.Startup.Instantly, true)]
    public class KrpcReflectionCache : MonoBehaviour
    {
        static readonly Dictionary<Assembly, Type[]> typesOf = new Dictionary<Assembly, Type[]>();
        static Dictionary<string, Type> byName;
        static int byNameAssemblies = -1;
        static bool patched;

        void Awake()
        {
            if (patched) return;
            patched = true;
            try
            {
                Type reflection = typeof(KRPC.Service.Attributes.KRPCServiceAttribute).Assembly
                    .GetType("KRPC.Utils.Reflection", true);
                var allTypes = reflection.GetMethod("AllTypes", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static);
                var getType = reflection.GetMethod("GetType", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static,
                    null, new[] { typeof(string) }, null);
                if (allTypes == null || getType == null)
                    throw new MissingMethodException("KRPC.Utils.Reflection.AllTypes/GetType not found");
                var harmony = new Harmony("krpclab.krpcReflectionCache");
                harmony.Patch(allTypes, prefix: new HarmonyMethod(typeof(KrpcReflectionCache), "AllTypesPrefix"));
                harmony.Patch(getType, prefix: new HarmonyMethod(typeof(KrpcReflectionCache), "GetTypePrefix"));
                UnityEngine.Debug.Log("[KrpcLab] kRPC reflection cache on");
            }
            catch (Exception e)
            {
                UnityEngine.Debug.LogWarning("[KrpcLab] kRPC reflection cache off (kRPC unchanged): " + e);
            }
        }

        static Type[] TypesOf(Assembly a)
        {
            Type[] types;
            if (typesOf.TryGetValue(a, out types)) return types;
            try { types = a.GetTypes(); }
            catch (ReflectionTypeLoadException e) { types = e.Types.Where(x => x != null).ToArray(); }
            typesOf[a] = types;
            return types;
        }

        static IEnumerable<Type> AllTypes(Func<Assembly, bool> include)
        {
            foreach (var a in AppDomain.CurrentDomain.GetAssemblies())
            {
                if (a.FullName.Contains("Assembly-CSharp")) continue;
                if (include != null && !include(a)) continue;
                foreach (var t in TypesOf(a)) yield return t;
            }
        }

        static bool AllTypesPrefix(Func<Assembly, bool> __0, ref IEnumerable<Type> __result)
        {
            __result = AllTypes(__0);
            return false;
        }

        static bool GetTypePrefix(string __0, ref Type __result)
        {
            int n = AppDomain.CurrentDomain.GetAssemblies().Length;
            if (byName == null || n != byNameAssemblies)
            {
                var index = new Dictionary<string, Type>();
                foreach (var t in AllTypes(null))
                {
                    string full = t.FullName;
                    if (full == null) return true;   // kRPC's own loop would throw here: let it
                    full = full.Replace('+', '.');
                    if (!index.ContainsKey(full)) index[full] = t;
                }
                byName = index;
                byNameAssemblies = n;
            }
            Type found;
            __result = byName.TryGetValue(__0.Replace('+', '.'), out found) ? found : null;
            return false;
        }
    }
}
