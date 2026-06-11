import json
import sys

import cantools
from utils import get_dbc_files


def cantools_json_to_dbc(input_json: str, outfilename: str, dbs=[]):
    with open(input_json) as file:
        can_json_input = json.load(file)

    new_signal_dict = {}
    nodes = []

    for signal in can_json_input["signals"]:
        print(f"processing signal: {signal['name']}")

        new_signal = cantools.db.Signal(
            name=signal["name"], start=signal["start"], length=signal["length"]
        )

        # Set byte order, if Null assume little endian
        try:
            new_signal.byte_order = signal["byte_order"]
        except KeyError:
            print(f"\tbyte order not specified for {signal['name']}, assuming le")
            new_signal.byte_order = "little_endian"

        # If there is weird conversion math, set it all
        if "conversion" in signal:
            # NOTE: This is only true if the bits of the raw input are IEEE floats
            # not if the scale/offset makes it a float after
            # if "is_float" in signal["conversion"]:
            #     new_signal.is_float = signal["conversion"]["is_float"]

            if "scale" in signal["conversion"]:
                new_signal.scale = signal["conversion"]["scale"]

            if "offset" in signal["conversion"]:
                new_signal.offset = signal["conversion"]["offset"]

            if "choices" in signal["conversion"]:
                new_signal.choices = signal["conversion"]["choices"]

        # Set if its signed, if Null assume it isn't
        try:
            new_signal.is_signed = signal["is_signed"]
        except KeyError:
            print(f"\tSigned not specified for {signal['name']}, assuming no sign")
            new_signal.is_signed = False

        # Set signal minimum, if Null try to guess minimum
        try:
            new_signal.minimum = signal["min"]
        except KeyError:
            # Assume UINT
            min = 0

            if "conversion" in signal:
                # UFLOAT
                if not new_signal.is_signed and signal["conversion"]["is_float"]:
                    min = 0.0

                # FLOAT
                elif new_signal.is_signed and signal["conversion"]["is_float"]:
                    min = ((2 ** (new_signal.length - 1)) * -1) * signal["conversion"][
                        "scale"
                    ]
            else:
                # INT
                if new_signal.is_signed:
                    min = (2 ** (new_signal.length - 1)) * -1

            print(f"\tminimum not specified for {signal['name']}, best guess is {min}")
            new_signal.minimum = min

        # Set signal maximum, if Null try to guess maximum
        try:
            new_signal.maximum = signal["max"]
        except KeyError:
            # Assume UINT
            max = 2 ** (new_signal.length) - 1

            if "conversion" in signal:
                # UFLOAT
                if not new_signal.is_signed and signal["conversion"]["is_float"]:
                    max = (2 ** (new_signal.length) - 1) * signal["conversion"]["scale"]

                # FLOAT
                elif new_signal.is_signed and signal["conversion"]["is_float"]:
                    max = (2 ** (new_signal.length - 1) - 1) * signal["conversion"][
                        "scale"
                    ]

            else:
                # INT
                if new_signal.is_signed:
                    max = 2 ** (new_signal.length - 1) - 1

            # Set max to largest possible value for the length
            print(f"\tmax not specified for {signal['name']}, best guess is {max}")
            new_signal.maximum = max

        # Set its initial value, if Null assume minimum
        try:
            new_signal.initial = signal["initial"]
        except KeyError:
            new_signal.initial = new_signal.minimum
            print(f"\tno initial value specified for {signal['name']}, assuming min")

        # Set if the signal is the multiplexer ID field
        try:
            new_signal.is_multiplexer = signal["is_multiplexer"]
        except KeyError:
            print(f"\tmux not specified for {signal['name']}")

        # Set the sub ID and the controller if it has a multiplexer_signal
        if "multiplexer_signal" in signal:
            new_signal.multiplexer_signal = signal["multiplexer_signal"]
            new_signal.multiplexer_ids = signal["multiplexer_ids"]

        try:
            new_signal.comment = signal["comment"]
        except KeyError:
            print(f"\tno comment specified for {signal['name']}")

        try:
            new_signal.unit = signal["units"]
        except KeyError:
            print(f"\tno units specified for {signal['name']}")
        new_signal_dict[new_signal.name] = new_signal
        print("")

    list_of_cantools_msgs = []

    for message in can_json_input["messages"]:
        message_info = can_json_input["messages"][message]
        signals = []

        for signal in message_info["signals"]:
            signals.append(new_signal_dict[signal])

        senders = message_info.get("senders", [])

        if isinstance(senders, str):
            senders = [senders]

        for sender in senders:
            if sender not in nodes:
                nodes.append(sender)

        new_message = cantools.db.Message(
            frame_id=message_info["id"],
            name=message,
            length=message_info["length"],
            signals=signals,
            senders=(
                message_info["senders"] if "senders" in message_info.keys() else None
            ),
        )
        try:
            new_message.comment = message_info["comment"]
        except:
            print(f"No comment found for message {message}")
        new_message.is_extended_frame = (
            message_info.get("is_extended_frame")
            if message_info.get("is_extended_frame") is not None
            else False
        )
        if new_message.frame_id > 2047:
            new_message.is_extended_frame = True
        try:
            new_message.bus_name = message_info["bus_name"]
        except:
            print(f"No bus specified for message {message}")
        list_of_cantools_msgs.append(new_message)

    for db in dbs:
        for message in db.messages:
            list_of_cantools_msgs.append(message)

        for node in db.nodes:
            name = node.name
            if name not in nodes:
                nodes.append(name)

    buses = [cantools.db.Bus("KSX", "can bus of KSU motorsports vehicles", 500000)]

    node_objs = [cantools.db.Node(name) for name in nodes]

    new_db = cantools.db.Database(list_of_cantools_msgs, nodes=node_objs, buses=buses)

    cantools.db.dump_file(new_db, outfilename + ".dbc")


def json_gen(outfile, infile, dbs):
    filename = outfile
    inputfile = infile
    db_args = dbs
    db_list = []
    for arg in db_args:
        db = get_dbc_files(arg)
        db_list.append(db)
    cantools_json_to_dbc(input_json=inputfile, outfilename=filename, dbs=db_list)


if __name__ == "__main__":
    print(sys.argv)
    args_outfile = sys.argv[1]
    args_infile = sys.argv[2]
    args_other_dbcs = sys.argv[3:]
    print(args_other_dbcs)
    json_gen(args_infile, args_outfile, args_other_dbcs)
