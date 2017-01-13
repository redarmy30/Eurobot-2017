import struct
from cmd_list import REVERSED_CMD_LIST
import numpy as np

SYNC = '\xFA'
ADDR = '\xAF'


CRC_POLY = np.uint32(0x4C11DB7)
CONST_FF = np.uint32(0xFFFFFFFF)
CONST_80 = np.uint32(0x80000000)
CONST_1 = np.uint32(1)

# data must be of bytearray type
def crc(data):
    crc = CONST_FF
    for byte in data:
        crc = crc ^ np.uint32(byte)
        for i in range(32):
            to_poly = crc & CONST_80 != 0
            crc = (crc << CONST_1) & CONST_FF
            if to_poly:
                crc = crc ^ CRC_POLY
    return crc

def cut_crc(msg_crc):
    return struct.pack("<I", msg_crc)[:2]

def encode_params(params):
    res = bytearray()
    for param in params:
        if isinstance(param, str):
            res += param
        elif isinstance(param, int):
            if not 0<= param <= 255:
                raise ValueError("Can't encode int: %d" % param)
            res += chr(param)
        elif isinstance(param, float):
            # TODO check
            res += struct.pack('>f', 0)
        else:
            raise ValueError("Unexpected parameter type: %d" % type(param))
    return res

def encode_packet(cmd_id, params):
    enc_params = encode_params(params)
    msg = bytearray()
    msg += SYNC + ADDR + chr(0) +  chr(cmd_id) + enc_params
    msg += cut_crc(crc(msg))
    msg[2] = len(msg)
    return msg


def decode_params(cmd, params):
    if cmd == 'getCurentCoordinates':
        return [
            struct.unpack('>f', params[i*8:(i+1)*8])[0] # TODO check correctness
            for i in range(3)
        ]
    if cmd in ['closeCubeCollector', 'getADCPinState']:
        return struct.unpack('>B', params)[0] # TODO check correctness
    return str(params)


def decode_packet(data):
    data = bytearray(data)
    if data[0] != SYNC or data[1] != ADDR:
        raise ValueError('Wrong packet header: %s' % data)
    msg_len = data[2]
    if len(data) != msg_len:
        raise ValueError('Length missmatch, expected %d, got %d' %
            (msg_len, len(data)))
    cmd = data[3]
    if cmd not in REVERSED_CMD_LIST:
        raise ValueError('Unexpected command: %02x' % cmd)
    rev_cmd = REVERSED_CMD_LIST[cmd]
    msg_crc = cut_crc(crc(msg[:-2]))
    if msg[-2:] != msg_crc:
        raise ValueError('CRC missmatch')
    params = data[4:-2]
    data = parse_params(rev_cmd, params)
    res = {'cmd': rev_cmd}
    if data:
        res[data] = data
    return res