"""Build each vehicle's DBC from vehicles.json: can_descriptor.json's messages, the
vehicle's inverters (each kind's messages, instantiated at its node and named after
its position), and the vendor DBCs every car has, on the buses the config describes.

    python3 vehicles.py --out dbc-output              # car.dbc + vehicles/car_<name>.dbc
    python3 vehicles.py --out dbc-output --vehicle 4wd_dti

car.dbc is the default vehicle's (the one the firmware pins). Inverter messages are
named inv_<position>_<message> (inv_fl_erpm_duty_voltage, inv_rl_setpoints_1) and sent
by the node inv_<position>; the VCU sends the commands.
"""
import argparse
import copy
import json
import os

import cantools

from json_to_dbc import build_database
from utils import get_dbc_files

HERE = os.path.dirname(os.path.abspath(__file__))


def load_config(path: str = os.path.join(HERE, "vehicles.json")) -> dict:
    with open(path) as f:
        return json.load(f)


def default_vehicle(config: dict) -> str:
    defaults = [name for name, v in config["vehicles"].items() if v.get("default")]
    if len(defaults) != 1:
        raise ValueError(f"vehicles.json needs exactly one default vehicle, has {defaults}")
    return defaults[0]


def _node(inv: dict) -> str:
    return f"inv_{inv['name']}"


def _amk_messages(kind: dict, inv: dict, bus: str) -> list:
    with open(os.path.join(HERE, kind["template"])) as f:
        template = json.load(f)
    out = []
    for name, m in template["messages"].items():
        signals = []
        for s in m["signals"]:
            signed = s.get("is_signed", False)
            length = s["length"]
            sig = cantools.db.Signal(
                name=s["name"], start=s["start"], length=length, byte_order="little_endian", is_signed=signed,
                conversion=cantools.database.conversion.BaseConversion.factory(scale=s.get("scale", 1), offset=0),
                minimum=(-(2 ** (length - 1)) if signed else 0) * s.get("scale", 1),
                maximum=((2 ** (length - 1) - 1) if signed else 2 ** length - 1) * s.get("scale", 1),
                unit=s.get("units"), comment=s.get("comment"))
            signals.append(sig)
        sender = _node(inv) if m["sender"] == "inverter" else m["sender"]
        out.append(cantools.db.Message(
            frame_id=m["base_id"] + inv["node"], name=f"{_node(inv)}_{name}", length=m["length"], signals=signals,
            senders=[sender], comment=f"{m['comment']}, {inv['name']} (node {inv['node']})", bus_name=bus))
    return out


def _dti_messages(kind: dict, inv: dict, bus: str) -> list:
    db = cantools.database.load_file(os.path.join(HERE, kind["template_dbc"]), strict=False)
    rename = {k: (_node(inv) if v == "inverter" else v) for k, v in kind["senders"].items()}
    out = []
    for m in db.messages:
        if m.name.startswith("VECTOR__"):
            continue
        packet = m.frame_id >> 8                      # extended ID: packet << 8 | node
        signals = [copy.deepcopy(s) for s in m.signals]
        for s in signals:
            s.receivers = [rename.get(r, r) for r in s.receivers]
        name = m.name[len(kind["strip_prefix"]):] if m.name.startswith(kind["strip_prefix"]) else m.name
        out.append(cantools.db.Message(
            frame_id=(packet << 8) | inv["node"], name=f"{_node(inv)}_{name.lower()}", length=m.length,
            signals=signals, senders=[rename.get(s, s) for s in m.senders], is_extended_frame=True,
            comment=f"DTI HV500 {name}, {inv['name']} (node {inv['node']})", bus_name=bus))
    return out


def inverter_messages(config: dict, inv: dict) -> list:
    """The messages of one inverter instance (none for a kind that comes as a whole DBC)."""
    kind = config["inverter_kinds"][inv["kind"]]
    bus = config["networks"].get(inv.get("network", "inverter"), inv.get("network"))
    if kind.get("ids") == "base_plus_node":
        return _amk_messages(kind, inv, bus)
    if kind.get("ids") == "dti_extended":
        return _dti_messages(kind, inv, bus)
    return []


def build(vehicle: str, config: dict | None = None,
          descriptor: str = os.path.join(HERE, "can_descriptor.json")) -> cantools.db.Database:
    """The vehicle's DBC as a cantools Database."""
    config = config or load_config()
    v = config["vehicles"][vehicle]
    keywords = list(config["common_dbcs"])
    extra, nodes = [], []
    for inv in v["inverters"]:
        kind = config["inverter_kinds"][inv["kind"]]
        if "dbc" in kind:
            if kind["dbc"] not in keywords:
                keywords.insert(1, kind["dbc"])        # where car.dbc always had it
        else:
            extra += inverter_messages(config, inv)
            nodes.append(_node(inv))
    buses = [cantools.db.Bus(name, b.get("comment"), b["bitrate"]) for name, b in config["buses"].items()]
    cwd = os.getcwd()
    os.chdir(HERE)                                     # utils.get_dbc_files reads ./dbc-files
    try:
        dbs = [get_dbc_files(k) for k in keywords]
    finally:
        os.chdir(cwd)
    db = build_database(descriptor, dbs, buses=buses, extra_messages=extra, extra_nodes=nodes)
    for m in db.messages:                              # logical networks onto the physical buses
        if m.bus_name in config["networks"]:
            m.bus_name = config["networks"][m.bus_name]
    return db


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", required=True, help="folder: car.dbc and vehicles/car_<name>.dbc go in it")
    ap.add_argument("--vehicle", action="append", help="only these (repeatable); default: all")
    ap.add_argument("--config", default=os.path.join(HERE, "vehicles.json"))
    args = ap.parse_args()
    config = load_config(args.config)
    os.makedirs(os.path.join(args.out, "vehicles"), exist_ok=True)
    default = default_vehicle(config)
    for name in args.vehicle or config["vehicles"]:
        db = build(name, config)
        path = os.path.join(args.out, "vehicles", f"car_{name}.dbc")
        cantools.db.dump_file(db, path)
        print(f"{name}: {len(db.messages)} messages -> {path}")
        if name == default:
            cantools.db.dump_file(db, os.path.join(args.out, "car.dbc"))
            print(f"  (default) -> {os.path.join(args.out, 'car.dbc')}")


if __name__ == "__main__":
    main()
