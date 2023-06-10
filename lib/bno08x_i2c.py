# BNO08X I2C Library by Dobodu
#
# Adapted from
#
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

from struct import pack_into
from machine import I2C
from bno08x import BNO08X, Packet, DATA_BUFFER_SIZE, const, Packet, PacketError

_BNO08X_DEFAULT_ADDRESS = const(0x4A)


class BNO08X_I2C(BNO08X):
    """Library for the BNO08x IMUs from Hillcrest Laboratories"""

    def __init__(self, i2c_bus, reset=None, address=_BNO08X_DEFAULT_ADDRESS, debug=False):
        self.bus_device_obj = i2c_bus
        self.bno_address = address
        super().__init__(reset, debug)

    def _send_packet(self, channel, data):
        #self._dbg("BNO08X_I2C SENDING PACKET...")
        #self._dbg("BNO08X_I2C SENDING Channel : ", channel, " Data : ", data)
        data_length = len(data)
        write_length = data_length + 4

        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte
        #self._dbg("BNO08X_I2C SENDING Data Buffer : ", self._data_buffer[0:write_length])
        packet = Packet(self._data_buffer)
        self._dbg("BNO08X_I2C SENDING Packet : ")
        if self._debug:
            print(packet)
        self.bus_device_obj.writeto(self.bno_address, self._data_buffer[0:write_length], True)

        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        return self._sequence_number[channel]

    # returns true if available data was read
    # the sensor will always tell us how much there is, so no need to track it ourselves

    def _read_header(self):
        self._dbg("BNO08X_I2C READ HEADER...")
        self._dbg("")
        #Reads the first 4 bytes available as a header ==> Expecting a header
        self.bus_device_obj.readfrom_into(self.bno_address, self._data_buffer[0:4])
        packet_header = Packet.header_from_buffer(self._data_buffer[0:4])
        self._dbg("\t",packet_header)
        self._dbg("")
        return packet_header

    def _read_packet(self):
        self._dbg("BNO08X_I2C READ PACKET...")
        self._dbg("")
        #Read a packet
        #Begin with reading a header ==> Expecting a header 
        #Debugging Working better with self._data_buffer but should be self._data_buffer[0:4] ???
        self.bus_device_obj.readfrom_into(self.bno_address, self._data_buffer)
        #self._dbg("\tSHTP READ packet header: ", [hex(x) for x in self._data_buffer[0:4]])

        header = Packet.header_from_buffer(self._data_buffer[0:4])
        packet_byte_count = header.packet_byte_count   
        channel_number = header.channel_number
        sequence_number = header.sequence_number
        data_length = header.data_length #data_lenth equals packet_bytes_count minus 4

        self._sequence_number[channel_number] = sequence_number
        if packet_byte_count == 0:
            self._dbg("\tSKIPPING NO PACKETS AVAILABLE IN i2c._read_packet")
            self._dbg("")
            raise PacketError("No packet available")
        #packet_byte_count -= 4
        #self._dbg("\tChannel", channel_number, "has", packet_byte_count, "bytes available to read")
        self._dbg("\tChannel", channel_number, "has", data_length, "bytes available to read")

        #self._read(packet_byte_count)
        #self.bus_device_obj.readfrom_into(self.bno_address, self._data_buffer[4:packet_byte_count])

        new_packet = Packet(self._data_buffer[0:packet_byte_count],True)
        if self._debug:
            print(new_packet)

        self._update_sequence_number(new_packet)

        return new_packet

    # returns true if all requested data was read
    def _read(self, requested_read_length):
        self._dbg("\tTrying to read", requested_read_length, "bytes")
        # +4 for the header
        total_read_length = requested_read_length + 4
        if total_read_length > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(total_read_length)
            self._dbg("!!!!! Increased _data_buffer to bytearray(%d) !!!!!" % total_read_length)
        self.bus_device_obj.readfrom_into(self.bno_address, self._data_buffer[0:total_read_length])


    @property
    def _data_ready(self):
        header = self._read_header()

        if header.channel_number > 5:
            self._dbg("channel number out of range:", header.channel_number)
        if header.packet_byte_count == 0x7FFF:
            print("Byte count is 0x7FFF/0xFFFF; Error?")
            if header.sequence_number == 0xFF:
                print("Sequence number is 0xFF; Error?")
            ready = False
        else:
            ready = header.data_length > 0

        self._dbg("BNO08X_I2C_DATA READY : ", ready)
        self._dbg("")
        return ready
