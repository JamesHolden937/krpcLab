// DrainSelect: lets kRPC tick a release valve's resources.
//
// ModuleResourceDrain drains only the resources ticked "yes" in its part
// menu, and those ticks are neither a KSPField, an event nor an action -- the
// menu calls TogglePartResource directly, so kRPC cannot reach them.  A valve
// saved with nothing ticked reads 'Resources': '0', opens onto nothing, and
// the autopilot flies its whole entry wet (LOG14998).  This module sits beside
// the valve (DrainSelect.cfg adds it to every part that has one) and gives
// that choice an event kRPC can trigger: "Release propellant" adds every
// resource named in `resources` that the valve offers to what it already
// releases, through the game's own public UpdateResourcesToRelease.
using System.Collections.Generic;

public class ModuleDrainSelect : PartModule
{
    // Comma-separated resource names; set in DrainSelect.cfg.
    [KSPField]
    public string resources = "LiquidFuel,Oxidizer";

    [KSPEvent(guiActive = true, guiActiveEditor = true,
              guiName = "Release propellant")]
    public void ReleasePropellant()
    {
        ModuleResourceDrain drain = part.FindModuleImplementing<ModuleResourceDrain>();
        if (drain == null || drain.resourcesAvailable == null) return;
        List<string> wanted = new List<string>();
        foreach (string name in resources.Split(','))
            if (name.Trim().Length > 0) wanted.Add(name.Trim());
        // UpdateResourcesToRelease sets exactly the list it is given, so the
        // list is what is ticked already plus what is wanted.
        List<string> release = new List<string>();
        foreach (PartResource r in drain.resourcesAvailable)
        {
            if (r == null) continue;
            if (drain.IsResourceDraining(r) || wanted.Contains(r.resourceName))
                release.Add(r.resourceName);
        }
        drain.UpdateResourcesToRelease(release);
        UnityEngine.Debug.Log("[DrainSelect] " + part.partInfo.name
            + " releases " + string.Join(",", release.ToArray()));
    }
}
