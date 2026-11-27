"""The generated DBCs: every vehicle in vehicles.json builds, its messages are
well-formed and don't collide on a bus, the default vehicle is the car.dbc the firmware
pins, and the inverter messages match their sources (AMK's manual, DTI's DBC)."""
import json
import os

import cantools
import pytest

import vehicles
from json_to_dbc import build_database
from utils import get_dbc_files

HERE = vehicles.HERE
CONFIG = vehicles.load_config()
VEHICLES = list(CONFIG["vehicles"])
DESCRIPTOR = os.path.join(HERE, "can_descriptor.json")


@pytest.fixture(scope="module")
def dbs():
    return {name: vehicles.build(name, CONFIG) for name in VEHICLES}


@pytest.fixture(scope="module")
def descriptor():
    with open(DESCRIPTOR) as f:
        return json.load(f)


def _layout(m):
    """A message's definition (signals by name: DBC text has no order for a
    multiplexed message's signals)."""
    return (m.frame_id, m.is_extended_frame, m.length,
            sorted((s.name, s.start, s.length, s.byte_order, s.is_signed, s.scale, s.offset) for s in m.signals))


def _field_range(s):
    """(lowest, highest) value the signal's bits can carry."""
    lo = (-(2 ** (s.length - 1)) if s.is_signed else 0) * s.scale + s.offset
    hi = ((2 ** (s.length - 1) - 1) if s.is_signed else 2 ** s.length - 1) * s.scale + s.offset
    return min(lo, hi), max(lo, hi)


# declared ranges wider than the field can carry, known: the ACU current senses' range
# leaves out their offset (which is right is a calibration question for the ACU board);
# DTI's map version is 0..255 at a 0.1 scale in DTI's own DBC; Cascadia's DBC has the
# fast torque command up to 32767.7 N m (3276.7: a typo)
KNOWN_RANGE_ISSUES = {"acu_sdc_current", "acu_glv_current", "CAN_map_version", "INV_Fast_Torque_Command"}


def test_one_default_vehicle():
    assert vehicles.default_vehicle(CONFIG) == "rwd_emrax"


def test_default_vehicle_is_the_single_dbc_build(dbs):
    """car.dbc as the generator always built it (json_to_dbc with the vendor DBCs)."""
    cwd = os.getcwd()
    os.chdir(HERE)
    try:
        legacy = build_database(DESCRIPTOR, [get_dbc_files(k) for k in
                                             ("Orion", "20240625-Gen5-CAN-DB", "elcon", "Megasquirt", "messageNEW")])
    finally:
        os.chdir(cwd)
    new = dbs[vehicles.default_vehicle(CONFIG)]
    assert [m.name for m in new.messages] == [m.name for m in legacy.messages]
    assert [_layout(m) for m in new.messages] == [_layout(m) for m in legacy.messages]


@pytest.mark.parametrize("name", VEHICLES)
def test_no_id_collisions_on_a_bus(dbs, name):
    seen = {}
    for m in dbs[name].messages:
        key = (m.bus_name, m.is_extended_frame, m.frame_id)
        assert key not in seen, f"{m.name} and {seen[key]} share {hex(m.frame_id)} on {m.bus_name}"
        seen[key] = m.name


@pytest.mark.parametrize("name", VEHICLES)
def test_names_unique(dbs, name):
    names = [m.name for m in dbs[name].messages]
    assert len(names) == len(set(names))


@pytest.mark.parametrize("name", VEHICLES)
def test_messages_on_known_buses(dbs, name):
    buses = set(CONFIG["buses"])
    for m in dbs[name].messages:
        assert m.bus_name is None or m.bus_name in buses, (m.name, m.bus_name)


@pytest.mark.parametrize("name", VEHICLES)
def test_dbc_text_reloads_strictly(dbs, name, tmp_path):
    """Dumped and loaded back with cantools' checks on (overlapping signals, signals
    past the frame): what the firmware's and the logger's code generators read."""
    path = tmp_path / f"{name}.dbc"
    cantools.db.dump_file(dbs[name], str(path))
    again = cantools.database.load_file(str(path), strict=True)
    assert [_layout(m) for m in again.messages] == [_layout(m) for m in dbs[name].messages]


def _example(s):
    """A value inside both the signal's declared range and its field, that survives
    its scaling."""
    if s.choices:
        return next(iter(s.choices))
    lo, hi = _field_range(s)
    lo = max(lo, s.minimum) if s.minimum is not None else lo
    hi = min(hi, s.maximum) if s.maximum is not None else hi
    raw = round(((lo + hi) / 2 - s.offset) / s.scale)
    return raw * s.scale + s.offset


@pytest.mark.parametrize("name", VEHICLES)
def test_messages_round_trip(dbs, name):
    for m in dbs[name].messages:
        if m.is_multiplexed() or not m.signals:
            continue
        data = {s.name: _example(s) for s in m.signals}
        out = m.decode(m.encode(data, strict=False), decode_choices=False, scaling=True)
        for k, v in data.items():
            assert out[k] == pytest.approx(v, abs=abs(m.get_signal_by_name(k).scale) / 2 + 1e-9), (m.name, k)


@pytest.mark.parametrize("name", VEHICLES)
def test_declared_ranges_fit_the_field(dbs, name):
    bad = []
    for m in dbs[name].messages:
        for s in m.signals:
            lo, hi = _field_range(s)
            if s.name not in KNOWN_RANGE_ISSUES and (
                    (s.minimum is not None and s.minimum < lo - 1e-9) or (s.maximum is not None and s.maximum > hi + 1e-9)):
                bad.append(f"{m.name}.{s.name}: field [{lo:g}, {hi:g}], declared [{s.minimum}, {s.maximum}]")
    assert not bad, bad


def test_descriptor_signals_exist_and_fit(descriptor):
    signals = {s["name"]: s for s in descriptor["signals"]}
    assert len(signals) == len(descriptor["signals"]), "signal names repeat"
    for name, m in descriptor["messages"].items():
        for s in m["signals"]:
            assert s in signals, f"{name}: no signal {s}"
            sig = signals[s]
            if sig.get("byte_order", "little_endian") == "little_endian":
                assert sig["start"] + sig["length"] <= 8 * m["length"], f"{name}.{s} past the frame"


def test_every_network_is_on_a_bus(descriptor):
    used = {m.get("bus_name") for m in descriptor["messages"].values()} - {None}
    assert used <= set(CONFIG["networks"]), used - set(CONFIG["networks"])
    assert set(CONFIG["networks"].values()) <= set(CONFIG["buses"])


def test_vehicle_inverters_are_known_kinds_and_nodes_unique():
    for name, v in CONFIG["vehicles"].items():
        positions = [inv["name"] for inv in v["inverters"]]
        assert len(positions) == len(set(positions)), name
        for kind in {inv["kind"] for inv in v["inverters"]}:
            assert kind in CONFIG["inverter_kinds"], (name, kind)
            nodes = [inv["node"] for inv in v["inverters"] if inv["kind"] == kind and "node" in inv]
            assert len(nodes) == len(set(nodes)), (name, kind, nodes)


# --- the inverters against their sources ------------------------------------------

AMK_MANUAL = {   # PDK_205481 p. 61-62: field -> (offset, bits, signed)
    "actual_values_1": {"actual_velocity": (16, 16, True), "torque_current": (32, 16, True),
                        "magnetizing_current": (48, 16, True), "system_ready": (8, 1, False),
                        "error": (9, 1, False), "warning": (10, 1, False), "quit_dc_on": (11, 1, False),
                        "dc_on": (12, 1, False), "quit_inverter_on": (13, 1, False), "inverter_on": (14, 1, False),
                        "derating": (15, 1, False)},
    "actual_values_2": {"temp_motor": (0, 16, True), "temp_inverter": (16, 16, True), "error_info": (32, 16, False),
                        "temp_igbt": (48, 16, True)},
    "setpoints_1": {"inverter_on": (8, 1, False), "dc_on": (9, 1, False), "enable": (10, 1, False),
                    "error_reset": (11, 1, False), "target_velocity": (16, 16, True),
                    "torque_limit_positive": (32, 16, True), "torque_limit_negative": (48, 16, True)},
}
AMK_BASE = {"actual_values_1": 0x282, "actual_values_2": 0x284, "setpoints_1": 0x183}


def test_amk_messages_match_the_manual(dbs):
    db = dbs["4wd_amk"]
    for inv in CONFIG["vehicles"]["4wd_amk"]["inverters"]:
        for msg, fields in AMK_MANUAL.items():
            m = db.get_message_by_name(f"inv_{inv['name']}_{msg}")
            assert m.frame_id == AMK_BASE[msg] + inv["node"] and not m.is_extended_frame
            assert m.length == 8
            for f, (offset, bits, signed) in fields.items():
                s = m.get_signal_by_name(f)
                assert (s.start, s.length, s.is_signed, s.byte_order) == (offset, bits, signed, "little_endian"), (msg, f)
        assert db.get_message_by_name(f"inv_{inv['name']}_actual_values_1").senders == [f"inv_{inv['name']}"]
        assert db.get_message_by_name(f"inv_{inv['name']}_setpoints_1").senders == ["vcu"]


def test_amk_torque_limit_in_tenths_of_a_percent(dbs):
    m = dbs["4wd_amk"].get_message_by_name("inv_fl_setpoints_1")
    data = {s.name: 0 for s in m.signals}
    data.update(target_velocity=-1200, torque_limit_positive=21.4, torque_limit_negative=-100.0)
    raw = m.encode(data)
    assert int.from_bytes(raw[2:4], "little", signed=True) == -1200
    assert int.from_bytes(raw[4:6], "little", signed=True) == 214       # 0.1 % MN units
    assert int.from_bytes(raw[6:8], "little", signed=True) == -1000


def test_dti_instances_are_dtis_dbc_at_their_nodes(dbs):
    dti = cantools.database.load_file(os.path.join(HERE, CONFIG["inverter_kinds"]["dti_hv500"]["template_dbc"]),
                                      strict=False)
    db = dbs["4wd_dti"]
    for inv in CONFIG["vehicles"]["4wd_dti"]["inverters"]:
        for src in dti.messages:
            if src.name.startswith("VECTOR__"):
                continue
            m = db.get_message_by_name(f"inv_{inv['name']}_{src.name.removeprefix('HV500_').lower()}")
            assert m.is_extended_frame and m.frame_id == (src.frame_id >> 8) << 8 | inv["node"]
            assert [(s.name, s.start, s.length, s.byte_order, s.is_signed, s.scale) for s in m.signals] == \
                   [(s.name, s.start, s.length, s.byte_order, s.is_signed, s.scale) for s in src.signals]


def test_dti_ac_current_command_encodes(dbs):
    m = dbs["4wd_dti"].get_message_by_name("inv_rr_setaccurrent")
    assert m.frame_id & 0xFF == 4
    decoded = m.decode(m.encode({s.name: 0 for s in m.signals} | {m.signals[0].name: 12.3}))
    assert decoded[m.signals[0].name] == pytest.approx(12.3, abs=0.05)


def test_pi_motor_torque_command_regen_and_drive(dbs):
    m = dbs["4wd_dti"].get_message_by_name("pi_motor_torque_command")
    data = {"pi_motor_torque_fl": 31.6, "pi_motor_torque_fr": -20.0, "pi_motor_torque_rl": 0.0,
            "pi_motor_torque_rr": -40.96, "pi_motor_engage": 1, "pi_motor_counter": 7, "pi_motor_checksum": 0}
    out = m.decode(m.encode(data))
    for k, v in data.items():
        assert out[k] == pytest.approx(v, abs=0.011)
    assert m.encode(data)[7] == 0                                         # byte 7: the CRC, the VCU's to fill


def test_hub_cars_have_their_inverters_only(dbs):
    names = {n: {m.name for m in db.messages} for n, db in dbs.items()}
    assert not any(n.startswith("inv_") for n in names["rwd_emrax"])
    assert {"inv_fl_setpoints_1", "inv_fr_setpoints_1"} <= names["awd_emrax_amk"]
    assert "inv_rl_setpoints_1" not in names["awd_emrax_amk"]
    assert "M192_Command_Message" in names["awd_emrax_dti"]              # the EMRAX's Cascadia stays
    assert "M192_Command_Message" not in names["4wd_dti"]                 # no EMRAX on a 4WD hub car
