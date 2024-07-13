"""
Serial Protocol
Copyright:  Embedded Systems Academy (EmSA) 2024
            All rights reserved. esacademy.com
Version:    1.00, EmSA 27-JUN-24
"""

import serial
import time
from enum import Enum
import crcmod
from typing import Tuple

# import sdoclnt
import SerialProtocolDefines

# default time to wait for command responses in seconds
RESPONSE_TIMEOUT_DEFAULT = 6
# max delay between bytes inside a packet in seconds
INTRAPACKET_TIMEOUT = 2
# start of packet header byte, must match implementation on device
SOH = 0x11

# NMT reset comands [5F01,01]
RESET_APPLICATION = 129
RESET_COMMUNICATION = 130
# constants, must match implementation of device
MAX_PACKET_LENGTH = int(28 + 7)
# max data that can be written to local OD in one go
MAX_WRITE_LENGTH = MAX_PACKET_LENGTH - 4


class State(Enum):
    # Parsing states for packet protocol reception
    START = 0  # Start-of-header
    LENGTH = 1  # Length of data
    DATA = 2  # Data bytes
    CHECKSUM_LOW = 3  # Checksum low
    CHECKSUM_HIGH = 4  # Checksum high


class SerialProtocol:
    m_DataCallback = None
    m_ResponseReceived = False
    m_CommandTimeout = RESPONSE_TIMEOUT_DEFAULT
    m_ResponsePacketData = bytearray()
    m_IncomingPacketData = bytearray()
    m_IncomingPacketLength = 0
    m_PortHandle = None
    m_ReceiveState = State.START
    m_ReceiveTimeout = 0
    # m_SdoClient = None
    m_LastNodeError = 0

    def __init__(self):
        """
        Constructor
        """
        # self.m_SdoClient = sdoclnt.SDOCLNT(self._SdoClientSend)
        pass

    def __del__(self):
        """
        Destructor - disconnects from serial port
        """
        self.Disconnect()
        pass

    def Process(self):
        """
        Checks for received packets and handles them
        """
        result = self._GetPacket()
        if result[0] == True:
            PacketData = result[1]
            if len(PacketData) > 0:
                # new process data
                if PacketData[0] == ord("D"):
                    if self.m_DataCallback != None:
                        self.m_DataCallback(
                            PacketData[1],
                            PacketData[2] | (PacketData[3] << 8),
                            PacketData[4],
                            PacketData[5:],
                        )
                # SDO segmented
                elif PacketData[0] == ord("F"):
                    # TODO:
                    pass
                # custom sdo request response
                elif PacketData[0] == ord("V"):
                    # TODO:
                    pass
                # all other packets
                else:
                    # must be a command response
                    # store and signal
                    self.m_ResponsePacketData = PacketData
                    self.m_ResponseReceived = True
            else:
                print("Packet with no data! ")
        # work on sdo clients
        # TODO: SDOHandleClient

    def RegisterDataCallback(self, callback):
        """
        Register a callback for process data written to COIA device

        Parameters:
            callback : callback function reference
        """
        self.m_DataCallback = callback

    # def RegisterSDORequestCallbacks(self, callback):
    #     # TODO:
    #     pass

    def Connect(self, PortName: str, Baudrate: int) -> bool:
        """
        Connects to the serial port

        Parameters:
            PortName : name of the com port e.g. "COM1" or "/dev/ttyUSB0"
            Baudrate : bitrate of the serial connection
        Returns:
            True for success, False for error
        """
        self.m_PortHandle = serial.Serial()
        self.m_PortHandle.baudrate = Baudrate
        self.m_PortHandle.port = PortName
        self.m_PortHandle.timeout = 0

        try:
            self.m_PortHandle.open()
            return True
        except:
            return False

    def Disconnect(self):
        """
        Disconnects from the serial port
        """
        try:
            self.m_PortHandle.close()
        except:
            pass

    def GetCommandTimeout(self) -> int:
        """
        Retrieves the command execution timeout value (in seconds)

        Returns:
            Timeout value
        """
        return self.m_ReceiveTimeout

    def SetCommandTimeout(self, timeout=RESPONSE_TIMEOUT_DEFAULT):
        """
        Sets the command execution timeout value

        Parameters:
            timeout : timeout in seconds
        """
        self.m_ReceiveTimeout = timeout

    def ReadLocalOD(self, Index: int, Subindex: int) -> Tuple[int, bytearray]:
        """
        Reads from the device's object dictionary
        Blocks until response received. Note that callback functions will
        continue to be called while waiting for the response.

        Parameters:
            Index    : index
            Subindex : subindex
        Returns:
            A Tuple with two values.
            First value is the error code. On success this is ERROR_NOERROR and the second value contains the data.
            On failure the second value is None
        """
        # construct packet
        PacketData = bytearray()
        PacketData.append(ord("R"))
        PacketData.append(Index & 0xFF)
        PacketData.append((Index >> 8) & 0xFF)
        PacketData.append(Subindex)

        # clear receive flag
        self.m_ResponseReceived = False
        # send
        if self._SendPacket(PacketData) != True:
            return SerialProtocolDefines.ERROR_TX, None

        endtime = int(time.time()) + self.m_CommandTimeout

        while True:
            if int(time.time()) >= endtime:
                return SerialProtocolDefines.ERROR_NORESPONSE, None
            # process packet receive
            self.Process()
            if self.m_ResponseReceived == True:
                break
        # if wrong response received then something went wrong
        if self.m_ResponsePacketData[0] != PacketData[0]:
            return SerialProtocolDefines.ERROR_WRONGRESPONSE, None

        # check for an error
        errorcode = self.m_ResponsePacketData[4] | (self.m_ResponsePacketData[5] << 8)
        if errorcode != 0:
            self.m_LastNodeError = errorcode
            return SerialProtocolDefines.ERROR_NODEERROR, None

        # ERROR_NOERROR
        return self.m_ResponsePacketData[4], self.m_ResponsePacketData[6:]

    def ReadRemoteOD(
        self, NodeID: int, Index: int, Subindex: int
    ) -> Tuple[int, bytearray]:
        """
        Reads from a remote device's object dictionary
        Blocks until response received. Note that callback functions will
        continue to be called while waiting for the response.

        Parameters:
            NodeID   : node id
            Index    : index
            Subindex : subindex
        Returns:
            A Tuple with two values.
            First value is the error code. On success this is ERROR_NOERROR and the second value contains the data.
            On failure the second value is None
        """
        # construct packet
        PacketData = bytearray()
        PacketData.append(ord("U"))
        PacketData.append(NodeID)
        PacketData.append(Index & 0xFF)
        PacketData.append((Index >> 8) & 0xFF)
        PacketData.append(Subindex)

        # clear receive flag
        self.m_ResponseReceived = False
        # send
        if self._SendPacket(PacketData) != True:
            return SerialProtocolDefines.ERROR_TX, None

        endtime = int(time.time()) + self.m_CommandTimeout

        while True:
            if int(time.time()) >= endtime:
                return SerialProtocolDefines.ERROR_NORESPONSE, None
            # process packet receive
            self.Process()
            if self.m_ResponseReceived == True:
                break

        # if wrong response received then something went wrong
        if self.m_ResponsePacketData[0] != PacketData[0]:
            return SerialProtocolDefines.ERROR_WRONGRESPONSE, None

        # check for an error
        errorcode = self.m_ResponsePacketData[5] | (self.m_ResponsePacketData[6] << 8)
        if errorcode != 0:
            self.m_LastNodeError = errorcode
            return SerialProtocolDefines.ERROR_NODEERROR, None

        return self.m_ResponsePacketData[5], self.m_ResponsePacketData[7:]

    # def ReadRemoteODExtended(self, NodeID: int, Index: int, Subindex: int, DataLength: int):
    #     if (self.m_SdoClient.SDOCLNT_Read(NodeID, 0, 0, Index, Subindex, DataLength) == True):
    #         return SerialProtocolDefines.ERROR_NOERROR
    #     return SerialProtocolDefines.ERROR_NORESOURCES

    def WriteLocalOD(self, Index: int, Subindex: int, Data: bytearray) -> int:
        """
        Write to the device's object dictionary
        Blocks until response received. Note that callback functions will
        continue to be called while waiting for the response.

        Parameters:
            Index    : index
            Subindex : subindex
            Data     : bytearray of data
        Returns:
            Error code. On success this is ERROR_NOERROR
        """
        # don't allow write of too much data
        if len(Data) > MAX_WRITE_LENGTH:
            return SerialProtocolDefines.ERROR_NORESOURCES

        # construct packet
        PacketData = bytearray()
        PacketData.append(ord("W"))
        PacketData.append(Index & 0xFF)
        PacketData.append((Index >> 8) & 0xFF)
        PacketData.append(Subindex)
        PacketData += Data

        # clear receive flag
        self.m_ResponseReceived = False
        # send
        if self._SendPacket(PacketData) != True:
            return SerialProtocolDefines.ERROR_TX

        endtime = int(time.time()) + self.m_CommandTimeout

        while True:
            if int(time.time()) >= endtime:
                return SerialProtocolDefines.ERROR_NORESPONSE
            # process packet receive
            self.Process()
            if self.m_ResponseReceived == True:
                break

        # if wrong response received then something went wrong
        if self.m_ResponsePacketData[0] != PacketData[0]:
            return SerialProtocolDefines.ERROR_WRONGRESPONSE

        # check for an error
        errorcode = self.m_ResponsePacketData[4] | (self.m_ResponsePacketData[5] << 8)
        if errorcode != 0:
            self.m_LastNodeError = errorcode
            return SerialProtocolDefines.ERROR_NODEERROR

        return self.m_ResponsePacketData[4]  # ERROR_NOERROR

    def WriteRemoteOD(
        self, NodeID: int, Index: int, Subindex: int, Data: bytearray
    ) -> int:
        """
        Write to a remote device's object dictionary
        Blocks until response received. Note that callback functions will
        continue to be called while waiting for the response.

        Parameters:
            NodeID   : node id
            Index    : index
            Subindex : subindex
            Data     : bytearray of data
        Returns:
            Error code. On success this is ERROR_NOERROR
        """
        # don't allow write of too much data
        if len(Data) > MAX_WRITE_LENGTH:
            return SerialProtocolDefines.ERROR_NORESOURCES

        # construct packet
        PacketData = bytearray()
        PacketData.append(ord("S"))
        PacketData.append(NodeID)
        PacketData.append(Index & 0xFF)
        PacketData.append((Index >> 8) & 0xFF)
        PacketData.append(Subindex)
        PacketData += Data

        # clear receive flag
        self.m_ResponseReceived = False
        # send
        if self._SendPacket(PacketData) != True:
            return SerialProtocolDefines.ERROR_TX

        endtime = int(time.time()) + self.m_CommandTimeout

        while True:
            if int(time.time()) >= endtime:
                return SerialProtocolDefines.ERROR_NORESPONSE
            # process packet receive
            self.Process()
            if self.m_ResponseReceived == True:
                break

        # if wrong response received then something went wrong
        if self.m_ResponsePacketData[0] != PacketData[0]:
            return SerialProtocolDefines.ERROR_WRONGRESPONSE

        # check for an error
        errorcode = self.m_ResponsePacketData[5] | (self.m_ResponsePacketData[6] << 8)
        if errorcode != 0:
            self.m_LastNodeError = errorcode
            return SerialProtocolDefines.ERROR_NODEERROR

        return self.m_ResponsePacketData[5]

    # def WriteRemoteODExtended(self, NodeID: int, Index: int, Subindex: int, Data: bytearray) -> int:
    #     if (self.m_SdoClient.SDOCLNT_Write(NodeID, 0, 0, Data, Index, Subindex) == True):
    #         return SerialProtocolDefines.ERROR_NOERROR
    #     return SerialProtocolDefines.ERROR_NORESOURCES

    def WakeUp(self):
        """
        If the node is sleeping, transmit something to wake node up
        """
        # perform read - node will wake up to process serial command
        # if sleeping, this wakes processor up
        self.ReadLocalOD(0x1000, 0x00)

    def SetSleepObjection(self, SleepObjectionOn: bool) -> int:
        """
        Sets the sleep objection state, device will actively object to sleep requests

        Parameters:
            SleepObjectionOn : True for on, False for off
        Returns:
            Error code. On success this is ERROR_NOERROR
        """
        return self.WriteLocalOD(0x5F01, 0x02, bytearray((SleepObjectionOn,)))

    def Reset(self) -> int:
        """
        Resets the node

        Returns:
            Error code. On success this is ERROR_NOERROR
        """
        return self.WriteLocalOD(
            0x5F01, 0x01, RESET_APPLICATION.to_bytes(length=1, byteorder="little")
        )

    def ResetCommunicationLayer(self) -> int:
        """
        Resets the communication layer in the node

        Returns:
            Error code. On success this is ERROR_NOERROR
        """
        return self.WriteLocalOD(
            0x5F01, 0x01, RESET_COMMUNICATION.to_bytes(length=1, byteorder="little")
        )

    def GetLastNodeError(self) -> int:
        """
        Gets the last node error

        Returns:
            Error code from node
        """
        return self.m_LastNodeError

    def _SendPacket(self, Data: bytearray) -> bool:
        """
        Transmits data packet to the device

        Returns:
            True for success, False for error
        """
        # if com port is not open, then nothing to do
        if self.m_PortHandle.is_open == False:
            print("\nERROR _SendPacket: Com port closed\n")
            return False

        # assemble packet
        TxPacketData = bytearray()
        TxPacketData.append(SOH)
        TxPacketData.append(len(Data))
        TxPacketData += Data
        crc16_func = crcmod.predefined.mkCrcFun("xmodem")
        CRCValue = crc16_func(TxPacketData[1 : len(Data) + 2])
        TxPacketData.append(CRCValue & 0xFF)
        TxPacketData.append((CRCValue >> 8) & 0xFF)

        # transmit packet
        if self.m_PortHandle.write(TxPacketData) != len(Data) + 4:
            return False
        return True

    def _GetPacket(self) -> Tuple[bool, bytearray]:
        """
        Attempts to get the next message packet from the device

        Returns:
            A Tuple with two values.
            If packet was received the first value is True and the second includes the packet data.
            On error or if packet is not yet complete the first value is False and the second None.
        """
        if self.m_PortHandle.is_open == False:
            print("\nERROR GetPacket: Com port closed\n")
            return False, None
        ReadResult = self.m_PortHandle.read()
        if len(ReadResult) != 1:
            # failed to receive, check for timeout and reset state machine if needed
            if (
                (int(time.time()) >= self.m_ReceiveTimeout)
                and (self.m_ReceiveState != State.LENGTH)
                and self.m_ReceiveState != State.START
            ):
                print(
                    "\nError GetPacket: Read in state {}\n".format(
                        self.m_ReceiveState.name
                    )
                )
                print(
                    "MSG: " + " ".join(f"0x{x:02X}" for x in self.m_IncomingPacketData)
                )
                self.m_ReceiveState = State.START
            return False, None
        if self.m_ReceiveState == State.START:
            if ReadResult[0] == SOH:
                self.m_IncomingPacketData = bytearray()
                self.m_IncomingPacketLength = 0
                self.m_ReceiveState = State.LENGTH
        elif self.m_ReceiveState == State.LENGTH:
            self.m_IncomingPacketLength = ReadResult[0]
            # if no data then skip to checksum
            if self.m_IncomingPacketLength == 0:
                self.m_ReceiveState = State.CHECKSUM_LOW
                self.m_IncomingCRC = 0x0000
            elif self.m_IncomingPacketLength > MAX_PACKET_LENGTH:
                # packet is too large for us, start over, wait for next
                self.m_ReceiveState = State.START
            else:
                self.m_ReceiveState = State.DATA
        elif self.m_ReceiveState == State.DATA:
            if len(self.m_IncomingPacketData) == 0:
                # first byte, sanity check, is it a supported command byte
                if (
                    (ReadResult[0] != ord("D"))
                    and (ReadResult[0] != ord("R"))
                    and (ReadResult[0] != ord("W"))
                    and (ReadResult[0] != ord("U"))
                    and (ReadResult[0] != ord("S"))
                    and (ReadResult[0] != ord("F"))
                    and (ReadResult[0] != ord("V"))
                ):
                    self.m_ReceiveState = State.START
            self.m_IncomingPacketData.append(ReadResult[0])
            if len(self.m_IncomingPacketData) == self.m_IncomingPacketLength:
                self.m_ReceiveState = State.CHECKSUM_LOW
                self.m_IncomingCRC = 0x0000
        elif self.m_ReceiveState == State.CHECKSUM_LOW:
            self.m_IncomingCRC |= ReadResult[0]
            self.m_ReceiveState = State.CHECKSUM_HIGH
        elif self.m_ReceiveState == State.CHECKSUM_HIGH:
            self.m_IncomingCRC |= ReadResult[0] << 8

            # calculate CRC
            crc16_func = crcmod.predefined.mkCrcFun("xmodem")
            CRCValue = crc16_func(
                self.m_IncomingPacketLength.to_bytes(length=1, byteorder="little")
            )
            CRCValue = crc16_func(self.m_IncomingPacketData, CRCValue)
            if CRCValue == self.m_IncomingCRC:
                self.m_ReceiveState = State.START
                return True, self.m_IncomingPacketData
            else:
                print("\nERROR GetPacket: CRC fail \n")
                print(
                    "MSG: " + " ".join(f"0x{x:02X}" for x in self.m_IncomingPacketData)
                )
                self.m_ReceiveState = State.START
                return False, None

        self.m_ReceiveTimeout = int(time.time()) + INTRAPACKET_TIMEOUT
        return False, None

    # def _SdoClientSend(self, node_id: int, Data: bytearray):
    #     PacketData = bytearray()
    #     PacketData.append(ord("C"))
    #     PacketData.append(node_id)
    #     PacketData += Data
    #     self._SendPacket(PacketData)
