# adapted from https://github.com/odriverobotics/ODriveResources/blob/master/examples/can_restore_config.py
import argparse
import asyncio
import can
from dataclasses import dataclass
import json
import math
import struct
from can_simple_utils import CanSimpleNode, REBOOT_ACTION_SAVE # if this import fails, make sure you copy the whole folder from the git repository

endpoint_dir = "flat_endpoints/"
track_config_file = "config/_track.json"
flipper_config_file = "config/_flipper.json"
config_files = ["config/can.json", "config/encoder.json", "config/power_source.json"]

# tracks_node_ids = [21, 22, 23, 24]
tracks_node_ids = [11, 22, 23, 13]
# flipper_node_ids = [11, 12, 13, 14]
flipper_node_ids = []

IDLE=1
CALIBRATION=3

_OPCODE_READ = 0x00
_OPCODE_WRITE = 0x01

# See https://docs.python.org/3/library/struct.html#format-characters
_FORMAT_LOOKUP = {
    'bool': '?',
    'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h',
    'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q',
    'float': 'f'
}

_GET_VERSION_CMD = 0x00 # Get_Version
_RX_SDO = 0x04 # RxSdo
_TX_SDO = 0x05 # TxSdo


@dataclass
class EndpointAccess():
    node: CanSimpleNode
    endpoint_data: dict

    async def version_check(self):
        self.node.flush_rx()

        # Send read command
        self.node.bus.send(can.Message(
            arbitration_id=(self.node.node_id << 5 | _GET_VERSION_CMD),
            data=b'',
            is_extended_id=False
        ))

        # Await reply
        msg = await self.node.await_msg(_GET_VERSION_CMD)

        _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)
        hw_version_str = f"{hw_product_line}.{hw_version}.{hw_variant}"
        fw_version_str = f"{fw_major}.{fw_minor}.{fw_revision}"

        with open(endpoint_dir + fw_version_str + '.json', 'r') as f:
            self.endpoint_data = json.load(f)

        # If one of these asserts fail, you're probably not using the right flat_endpoints.json file
        if self.endpoint_data['fw_version'] != fw_version_str:
            print(f"the file provided in --endpoints-json does not match the firmware version of the ODrive: {self.endpoint_data['fw_version']} != {fw_version_str}")
            return False
        if self.endpoint_data['hw_version'] != hw_version_str:
            print(f"the file provided in --endpoints-json does not match the firmware version of the ODrive: {self.endpoint_data['hw_version']} != {hw_version_str}")
            return False
        return True

    async def write_and_verify(self, path: str, val):
        endpoint_id = self.endpoint_data['endpoints'][path]['id']
        endpoint_type = self.endpoint_data['endpoints'][path]['type']
        endpoint_fmt = _FORMAT_LOOKUP[endpoint_type]

        self.node.bus.send(can.Message(
            arbitration_id=(self.node.node_id << 5 | _RX_SDO),
            data=struct.pack('<BHB' + endpoint_fmt, _OPCODE_WRITE, endpoint_id, 0, val),
            is_extended_id=False
        ))

        self.node.flush_rx()

        self.node.bus.send(can.Message(
            arbitration_id=(self.node.node_id << 5 | _RX_SDO),
            data=struct.pack('<BHB', _OPCODE_READ, endpoint_id, 0),
            is_extended_id=False
        ))

        msg = await self.node.await_msg(_TX_SDO)

        # Unpack and cpmpare reply
        _, _, _, return_value = struct.unpack_from('<BHB' + endpoint_fmt, msg.data)
        val_pruned = val if endpoint_type != 'float' else struct.unpack('<f', struct.pack('<f', val))[0]
        if return_value == val_pruned:
            pass
        else:
            if math.isnan(return_value) != math.isnan(val_pruned):
                raise Exception(f"failed to write {path}: {return_value} != {val_pruned}")


async def restore_config(odrv: EndpointAccess, config: dict):
    print(f"writing {len(config)} variables...")
    for k, v in config.items():
        print(f"  {k} = {v}")
        await odrv.write_and_verify(k, v)

async def configure(node_id, bus, config, save_config, calibrate):
    with CanSimpleNode(bus=bus, node_id=node_id) as node:
        odrv = EndpointAccess(node=node, endpoint_data={})
        print("Node id:", node_id)
        print("checking version...")
        if await odrv.version_check():
            await restore_config(odrv, config)
            print()
            if save_config:
                print(f"saving configuration...")
                node.reboot_msg(REBOOT_ACTION_SAVE)
        if calibrate:
            odrv.node.set_state_msg(CALIBRATION)
            print(f"calibrating...")
            
            for msg in bus:
                if odrv.node.wait_state(IDLE, msg):
                    break
            node.reboot_msg(REBOOT_ACTION_SAVE)

async def main():
    parser = argparse.ArgumentParser(description='Script to configure ODrive over CAN bus.')
    parser.add_argument('-i', '--interface', type=str, default='socketcan', help='Interface type (e.g., socketcan, slcan). Default is socketcan.')
    parser.add_argument('-c', '--channel', type=str, default='can0', help='Channel/path/interface name of the device (e.g., can0, /dev/tty.usbmodem11201).')
    parser.add_argument('-b', '--bitrate', type=int, default=250000, help='Bitrate for CAN bus. Default is 250000.')
    parser.add_argument("--save-config", action='store_true', help="Save the configuration to NVM and reboot ODrive.")
    parser.add_argument("--calibrate", action='store_true', help="Calibrate the ODrive and save the configuration")
    args = parser.parse_args()
    
    config_list = {}
    track_config_list = {}
    flipper_config_list = {}

    for file in config_files:
        with open(file, 'r') as f:
            config_list.update(json.load(f))

    with open(track_config_file, 'r') as f:
        track_config_list.update(json.load(f))
    track_config_list.update(config_list)

    with open(flipper_config_file, 'r') as f:
        flipper_config_list.update(json.load(f))
    flipper_config_list.update(config_list)

    print("opening CAN bus...")
    with can.interface.Bus(args.channel, bustype=args.interface, bitrate=args.bitrate) as bus:
        #reader = can.AsyncBufferedReader()
        #notifier = can.Notifier(bus, [reader], loop=asyncio.get_running_loop())
        for node_id in tracks_node_ids:
            await configure(node_id, bus, track_config_list, args.save_config, args.calibrate)

        for node_id in flipper_node_ids:
            await configure(node_id, bus, flipper_config_list, args.save_config, args.calibrate)

        await asyncio.sleep(0.1) # needed for last message to get through on SLCAN backend

if __name__ == "__main__":
    asyncio.run(main())