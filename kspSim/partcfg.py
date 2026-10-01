"""Part configs as the game loaded them: ModuleManager's ConfigCache.

    cfg = PartConfigs("testInstances/ksp0/GameData/ModuleManager.ConfigCache")
    cfg.modules("RCSBlock.v2")  ->  [{"name": "ModuleRCSFX", "thrusterPower": "1", ...}, ...]

The cache is the configuration **after** every mod's patches, which is what
the parts in the game were built from -- the files under ``GameData`` are
not (``AtmosphereAutopilot`` replaces control-surface modules, ``ReStock``
rewrites parts).  The simulator needs a handful of module fields that kRPC
does not expose: control surfaces' ``actuatorSpeed`` and
``ctrlSurfaceRange``, reaction wheels' ``torqueResponseSpeed``, gimbals'
``gimbalRange`` and ``gimbalResponseSpeed``, RCS ``thrusterPower`` and
``fullThrust``.  ``probe.py`` stores each part's modules in the model.

In-game part names have ``_`` replaced by ``.`` (``RCSBlock_v2`` is
``RCSBlock.v2``); lookups accept either.
"""
import os


def parse(text):
    """ConfigNode text -> nested {"name": node name, "values": [(k, v)], "nodes": [...]}."""
    root = {"name": "root", "values": [], "nodes": []}
    stack = [root]
    pending = None
    for raw in text.splitlines():
        line = raw.split("//")[0].strip()
        if not line:
            continue
        while line:
            if line.startswith("{"):
                node = {"name": pending or "", "values": [], "nodes": []}
                stack[-1]["nodes"].append(node)
                stack.append(node)
                pending = None
                line = line[1:].strip()
                continue
            if line.startswith("}"):
                if len(stack) > 1:
                    stack.pop()
                pending = None
                line = line[1:].strip()
                continue
            brace = min([i for i in (line.find("{"), line.find("}")) if i >= 0], default=-1)
            head = line if brace < 0 else line[:brace]
            rest = "" if brace < 0 else line[brace:]
            head = head.strip()
            if "=" in head:
                k, v = head.split("=", 1)
                stack[-1]["values"].append((k.strip(), v.strip()))
                pending = None
            elif head:
                pending = head
            line = rest.strip()
    return root


def values(node):
    out = {}
    for k, v in node["values"]:
        out.setdefault(k, v)
    return out


class PartConfigs:
    def __init__(self, path):
        with open(path, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        root = parse(text)
        self.parts = {}
        self.urls = {}
        self.resources = {}
        self._walk(root)

    def _walk(self, node, parent_url=None):
        for child in node["nodes"]:
            if child["name"] == "RESOURCE_DEFINITION":
                v = values(child)
                if v.get("name"):
                    self.resources[v["name"]] = {k: v[k] for k in ("density", "hsp") if k in v}
            elif child["name"] == "PART":
                name = values(child).get("name")
                if name:
                    self.parts[name.replace("_", ".")] = child
                    if parent_url:
                        base = parent_url[:-4] if parent_url.endswith(".cfg") else parent_url
                        self.urls[name.replace("_", ".")] = base + "/" + name
            else:
                self._walk(child, values(child).get("parentUrl", parent_url))

    def url(self, name):
        """The part's PartDatabase url (``Squad/Parts/.../mk1-3/mk1-3pod``)."""
        return self.urls.get(name.replace("_", "."))

    def part(self, name):
        return self.parts.get(name.replace("_", "."))

    def modules(self, name):
        node = self.part(name)
        if node is None:
            return None
        out = []
        for child in node["nodes"]:
            if child["name"] == "MODULE":
                mod = values(child)
                subs = {}
                for sub in child["nodes"]:
                    subs.setdefault(sub["name"], []).append(values(sub))
                if subs:
                    mod["_nodes"] = subs
                out.append(mod)
        return out

    def fields(self, name):
        """The part's own top-level values (mass, maxTemp, skinMaxTemp, ...)."""
        node = self.part(name)
        return values(node) if node is not None else None

    def nodes(self, name):
        """{node id: [x, y, z, ox, oy, oz]} in part space: ``node_stack_<id>``
        and ``node_attach`` (id ``srfAttach``).  Orientation only matters to
        the thermal model (which faces a neighbour covers)."""
        node = self.part(name)
        if node is None:
            return {}
        out = {}
        for k, v in node["values"]:
            if k.startswith("node_stack_"):
                nid = k[len("node_stack_"):]
            elif k == "node_attach":
                nid = "srfAttach"
            else:
                continue
            try:
                nums = [float(x) for x in v.split(",")[:6]]
            except ValueError:
                continue
            if len(nums) == 6:
                out.setdefault(nid, nums)
        return out


def default_cache(root):
    for n in range(10):
        p = os.path.join(root, "testInstances", "ksp%d" % n, "GameData", "ModuleManager.ConfigCache")
        if os.path.exists(p):
            return p
    return None


# -- variants: which of a part's transforms are live ---------------------------

def _mu_offsets(data, name):
    """Offsets of a length-prefixed string in a .mu file (the model's
    transform names are .NET BinaryWriter strings, in hierarchy order)."""
    key = bytes([len(name)]) + name.encode()
    out = []
    i = data.find(key)
    while i >= 0:
        out.append(i)
        i = data.find(key, i + 1)
    return out


class Variants:
    """ModulePartVariants: which GameObjects a variant switches on, and which
    of a module's transforms (``thrusterTransformName``) sit under them."""

    def __init__(self, cfg, gamedata):
        self.cfg = cfg
        self.gamedata = gamedata

    def module(self, part):
        for m in self.cfg.modules(part) or []:
            if m["name"] == "ModulePartVariants":
                return m
        return None

    def objects(self, part, variant):
        """{GameObject name: enabled} for a variant (the base one if None)."""
        node = self.cfg.part(part)
        if node is None:
            return None
        for mod in node["nodes"]:
            if mod["name"] != "MODULE" or values(mod).get("name") != "ModulePartVariants":
                continue
            base = values(mod).get("baseVariant")
            want = variant or base
            first = None
            for var in mod["nodes"]:
                if var["name"] != "VARIANT":
                    continue
                vname = values(var).get("name")
                first = first or var
                if vname == want:
                    for g in var["nodes"]:
                        if g["name"] == "GAMEOBJECTS":
                            return {k: v.lower() == "true" for k, v in g["values"]}
                    return {}
            if first is not None and want is None:
                for g in first["nodes"]:
                    if g["name"] == "GAMEOBJECTS":
                        return {k: v.lower() == "true" for k, v in g["values"]}
        return None

    def model_files(self, part):
        node = self.cfg.part(part)
        out = []
        if node is None:
            return out
        for child in node["nodes"]:
            if child["name"] == "MODEL":
                m = values(child).get("model")
                if m:
                    out.append(os.path.join(self.gamedata, m + ".mu"))
        return out

    def transform_owners(self, part, transform, object_names):
        """For each ``transform`` in hierarchy order, the variant object it
        sits under (the nearest preceding object name), or None."""
        for path in self.model_files(part):
            try:
                with open(path, "rb") as fh:
                    data = fh.read()
            except OSError:
                continue
            marks = sorted((off, name) for name in object_names for off in _mu_offsets(data, name))
            owners = []
            for off in _mu_offsets(data, transform):
                owner = None
                for moff, name in marks:
                    if moff < off:
                        owner = name
                    else:
                        break
                owners.append(owner)
            if owners:
                return owners
        return None

    def live_mask(self, part, variant, transform, count):
        """[bool] per transform: is it under an enabled object?  None when
        the part has no variants or the model cannot be read -- then every
        transform is taken as live."""
        objs = self.objects(part, variant)
        if not objs:
            return None
        owners = self.transform_owners(part, transform, list(objs))
        if owners is None or len(owners) != count:
            return None
        return [objs.get(o, True) if o is not None else True for o in owners]


def _active_vessel(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        root = parse(fh.read())

    def find(node, name):
        for c in node["nodes"]:
            if c["name"] == name:
                return c
        for c in node["nodes"]:
            r = find(c, name)
            if r is not None:
                return r
        return None
    game = find(root, "GAME") or root
    fs = find(game, "FLIGHTSTATE")
    if fs is None:
        return None
    active = int(values(fs).get("activeVessel", 0))
    vessels = [c for c in fs["nodes"] if c["name"] == "VESSEL"]
    return vessels[active] if active < len(vessels) else None


def sfs_attachments(path):
    """Per part of the active vessel, in order: {"name", "att": [[node id,
    part index]], "srf": [node id, part index] or None} -- the save's
    ``attN = top, 3`` and ``srfN = srfAttach, 5[,collider]`` lines."""
    v = _active_vessel(path)
    if v is None:
        return None
    out = []
    for p in v["nodes"]:
        if p["name"] != "PART":
            continue
        rec = {"name": values(p).get("name"), "att": [], "srf": None}
        for k, val in p["values"]:
            if k not in ("attN", "srfN"):
                continue
            bits = [b.strip() for b in val.split(",")]
            try:
                target = int(bits[1])
            except (IndexError, ValueError):
                continue
            if target < 0 or not bits[0] or bits[0] == "None":
                continue
            if k == "attN":
                rec["att"].append([bits[0], target])
            else:
                rec["srf"] = [bits[0], target]
        out.append(rec)
    return out


def sfs_active_parts(path):
    """[(part name, {module name: {field: value}})] of the active vessel of a
    save, in the game's part order."""
    with open(path, encoding="utf-8", errors="replace") as fh:
        root = parse(fh.read())

    def find(node, name):
        for c in node["nodes"]:
            if c["name"] == name:
                return c
        for c in node["nodes"]:
            r = find(c, name)
            if r is not None:
                return r
        return None
    game = find(root, "GAME") or root
    fs = find(game, "FLIGHTSTATE")
    if fs is None:
        return None
    active = int(values(fs).get("activeVessel", 0))
    vessels = [c for c in fs["nodes"] if c["name"] == "VESSEL"]
    if active >= len(vessels):
        return None
    out = []
    for p in vessels[active]["nodes"]:
        if p["name"] != "PART":
            continue
        mods = {}
        for m in p["nodes"]:
            if m["name"] == "MODULE":
                v = values(m)
                # Action-group bindings: {action: [group, ...]}.
                for a in m["nodes"]:
                    if a["name"] == "ACTIONS":
                        v["_actions"] = {
                            act["name"]: [g.strip() for g in values(act).get("actionGroup", "")
                                          .split(",") if g.strip() and g.strip() != "None"]
                            for act in a["nodes"]}
                mods.setdefault(v.get("name"), v)
        out.append((values(p).get("name"), mods))
    return out


def drag_cubes(path):
    """{part url: {cube name: [6 faces of (area, drag, depth)] + centre + size}}
    from ``PartDatabase.cfg`` (KSP's cache of every part's drag cubes; face
    order +x -x +y -y +z -z, each face ``area, drag coefficient, depth``)."""
    out = {}
    url = None
    for raw in open(path, encoding="utf-8", errors="replace"):
        line = raw.strip()
        if line.startswith("url ="):
            url = line.split("=", 1)[1].strip()
        elif line.startswith("cube =") and url:
            parts = [x.strip() for x in line.split("=", 1)[1].split(",")]
            name, nums = parts[0], [float(x) for x in parts[1:]]
            faces = [nums[i * 3:i * 3 + 3] for i in range(6)]
            out.setdefault(url, {})[name] = {"faces": faces, "center": nums[18:21],
                                             "size": nums[21:24]}
    return out
