"""
Remote Access App Demo command-line interface
Copyright:  Embedded Systems Academy (EmSA) 2024
            All rights reserved. esacademy.com
Version:    1.22, EmSA 12-JUL-24
"""

import SerialProtocol
import time
import sys
import SerialProtocolDefines
import signal

MAX_NUMBER_OF_NODES = 32

# define to True to enable display of new data on the network
SHOW_NEW_DATA = False

# fmt: off
# node state flags
NODE_STATE_NONE         = 0
NODE_STATE_HBACTIVE     = (1 << 0)
NODE_STATE_HBLOSS       = (1 << 1)
NODE_STATE_SCANNING     = (1 << 2)
NODE_STATE_SCANFINISHED = (1 << 3)
NODE_STATE_PRODUCEDATA  = (1 << 4)
NODE_STATE_BOOTED       = (1 << 7)

# NMT commands
NMT_OPERATIONAL    = 1
NMT_STOP           = 2
NMT_PREOPERATIONAL = 128
NMT_RESETAPP       = 129
NMT_RESETCOM       = 130
# fmt: on

# baudrate to connect at, select based on your CANgineBerry configuration
#BAUDRATE = 115200
BAUDRATE = 921600

COIADevice = SerialProtocol.SerialProtocol()
MyNodeID = 0
MyNMTState = 0
TerminationRequested = False
NodeStates = [NODE_STATE_NONE] * MAX_NUMBER_OF_NODES


def OwnStatusChanged(SubIdx: int, data: int):
    """
    This function is called from the new data received call-back,
    if data received indicates a change in this device node status:
    Change of own node ID or own NMT state or own HW status

    Parameters:
            SubIdx : subindex
            data   : first byte
    """
    if SubIdx == 1:
        print("\n{{Own node ID changed to {}}}".format(data), end="", flush=True)
        global MyNodeID
        MyNodeID = data
    elif SubIdx == 2:
        print("\n{{Own status changed to {}}}".format(data), end="", flush=True)
        global MyNMTState
        MyNMTState = data

        if data == SerialProtocolDefines.NODESTATUS_OPERATIONAL:
            # we are now set to operational
            pass
        # if node was reset then re-initialize it
        if data == SerialProtocolDefines.NODESTATUS_RESETAPP:
            # wait for node to complete restart
            time.sleep(0.5)
    elif SubIdx == 3:
        print(
            "\n{{Hardware status changed to 0x{:02X} - ".format(data),
            end="",
            flush=True,
        )

        if data == SerialProtocolDefines.HWSTATUS_NONE:
            print("NONE} ", end="", flush=True)
            return

        if data & SerialProtocolDefines.HWSTATUS_INITALIZING:
            print("INIT ", end="", flush=True)
        if data & SerialProtocolDefines.HWSTATUS_CANERROR:
            print("CAN-ERROR ", end="", flush=True)
        if data & SerialProtocolDefines.HWSTATUS_ERRORPASSIVE:
            print("ERROR-PASSIVE ", end="", flush=True)
        if data & SerialProtocolDefines.HWSTATUS_RXQUEUEOVERRUN:
            print("RX-OVERRUN ", end="", flush=True)
        if data & SerialProtocolDefines.HWSTATUS_TXQUEUEOVERRUN:
            print("TX-OVERRUN ", end="", flush=True)
        if data & SerialProtocolDefines.HWSTATUS_TXBUSY:
            print("TX-BUSY ", end="", flush=True)
        if data & SerialProtocolDefines.HWSTATUS_BUSOFF:
            print("BUS-OFF ", end="", flush=True)
        print("}} ")
    else:
        print("\n{{Unknown Subindex: {}}}".format(SubIdx))


def NodeStatusChanged(NodeID: int, State: int):
    """
    This function is called from the new data received call-back,
    if data received indicates a change in the node status of any of
    the nodes connected to the network

    Parameters:
            NodeID : node id
            State  : node state
    """
    global NodeStates
    if NodeID >= MAX_NUMBER_OF_NODES:
        return

    print("\n{{Node {} ".format(NodeID & 0x7F), end="", flush=True)

    if NodeID & 0x80:
        print("(self) ", end="", flush=True)

    print("status changed to 0x{:02X} - ".format(State), end="", flush=True)

    if State == SerialProtocolDefines.NODESTATUS_BOOT:
        print("BOOT}} ", end="", flush=True)
    elif State == SerialProtocolDefines.NODESTATUS_STOPPED:
        print("STOP}} ", end="", flush=True)
    elif State == SerialProtocolDefines.NODESTATUS_OPERATIONAL:
        print("OPERATIONAL}} ", end="", flush=True)
    elif State == SerialProtocolDefines.NODESTATUS_PREOP:
        print("PREOP}} ", end="", flush=True)
    elif State == SerialProtocolDefines.NODESTATUS_EMCY_OVER:
        print("EMCY CLEAR}} ", end="", flush=True)
    elif State == SerialProtocolDefines.NODESTATUS_EMCY_NEW:
        print("NEW EMCY}} ", end="", flush=True)
    elif State == SerialProtocolDefines.NODESTATUS_HBACTIVE:
        print("HB ACTIVE}} ", end="", flush=True)
        NodeStates[NodeID - 1] |= NODE_STATE_HBACTIVE  # signal HB active
        NodeStates[NodeID - 1] &= ~NODE_STATE_HBLOSS
    elif State == SerialProtocolDefines.NODESTATUS_HBLOST:
        print("HB LOST}} ", end="", flush=True)
        # signal HB active & loss, reset all other bits
        NodeStates[NodeID - 1] = NODE_STATE_HBACTIVE | NODE_STATE_HBLOSS
    elif State == SerialProtocolDefines.NODESTATUS_SCANSTARTED:
        print("SCAN INIT}} ", end="", flush=True)
        NodeStates[NodeID - 1] |= NODE_STATE_SCANNING  # signal scan init
        NodeStates[NodeID - 1] &= ~NODE_STATE_BOOTED
    elif (
        State == SerialProtocolDefines.NODESTATUS_SCANCOMPLETE
    ):  # App can now access this node
        print("SCANNED}} ", end="", flush=True)
        NodeStates[NodeID - 1] |= NODE_STATE_SCANFINISHED  # signal scan complete
        NodeStates[NodeID - 1] &= ~NODE_STATE_SCANNING
    elif State == SerialProtocolDefines.NODESTATUS_SCANABORTED:
        print("SCAN ABORT}} ", end="", flush=True)
        NodeStates[NodeID - 1] &= ~NODE_STATE_SCANFINISHED  # signal scan abort
        NodeStates[NodeID - 1] &= ~NODE_STATE_SCANNING
    elif State == SerialProtocolDefines.NODESTATUS_RESETAPP:
        print("RESET APP}} ", end="", flush=True)
    elif State == SerialProtocolDefines.NODESTATUS_RESETCOM:
        print("RESET APP}} ", end="", flush=True)
    elif State == SerialProtocolDefines.NODESTATUS_SLEEP:
        print("SLEEP}} ", end="", flush=True)
    else:
        print("UNKNOWN}} ", end="", flush=True)


def NewData(NodeID: int, Index: int, Subindex: int, Data: bytearray):
    """
    Call-back function, data indication, new data arrived in device
    """
    # is this our own status changed?
    if Index == 0x5F00:
        # device status
        OwnStatusChanged(Subindex, Data[0])
    # is this a generic node status change?
    elif Index == 0x5F04:
        # node status
        NodeStatusChanged(Subindex, Data[0])
    elif SHOW_NEW_DATA == True:
        # display raw data received
        print("{{{}:{:04X},{:02X};".format(NodeID, Index, Subindex), end="", flush=True)
        print(" ".join(f"{x:02X}" for x in Data) + "} ")


def Terminate(sig, frame):
    """
    Called when user presses Ctrl-C
    """
    global TerminationRequested
    TerminationRequested = True


def main():
    """
    Main function, open com port, run for 10 minutes
    """
    global NodeStates
    nodes = 1

    print("\nCANopenIA Remote Access by www.esacademy.com\nV1.22 of 12-JUL-2024\n")

    if len(sys.argv) != 2:
        print("Usage: python3 RA_App_Demo_CLI.py <comport>")
        return 1

    ComPort = sys.argv[1]
    print("Connecting to " + ComPort)
    if COIADevice.Connect(ComPort, BAUDRATE) == False:
        print("Failed to connect to " + ComPort)
        return 1
    print("Connected to " + ComPort)

    if SHOW_NEW_DATA == True:
        print(
            "\nData in {{NodeID:Index:Subindex;Data}}-brackets is received in call back functions.\n"
        )

    # register callback functions
    COIADevice.RegisterDataCallback(NewData)
    # TODO: RegisterSDORequestCallbacks (not yet implemented)

    # get NMT state of node
    print("\nRequesting NMT state of COIA node...")
    result = COIADevice.ReadLocalOD(0x5F00, 0x02)
    if result[0] != SerialProtocolDefines.ERROR_NOERROR:
        print(
            "\nFailed to get NMT state of COIA node. Error code = 0x{:08X}".format(
                result[0]
            )
        )
        print("Closing port...")
        # disconnect from COM port, finished with COIA device
        return 0
    else:
        MyNMTState = result[1][0]

    print("\nNMT State = 0x{:02X}".format(MyNMTState), end="", flush=True)

    # wait for packets for 600 seconds
    EndTime = int(time.time()) + 600
    print("\n\nWaiting for a CiA401 device to appear...")
    print("Running for 10min, or until CTRL-C: ", end="", flush=True)

    # look for Ctrl-C - note, this will affect all instances of this class at once
    signal.signal(signal.SIGINT, Terminate)

    PDOTime = int(time.time()) + 1

    # if we are operational then reset all nodes to make them boot up
    # so we find them
    if MyNMTState == SerialProtocolDefines.NODESTATUS_OPERATIONAL:
        NMTCmd = bytearray()
        NMTCmd.append(NMT_RESETCOM & 0xFF)
        NMTCmd.append((NMT_RESETCOM >> 8) & 0xFF)
        result = COIADevice.WriteLocalOD(0x5F0A, 0x01, NMTCmd)
        if result != SerialProtocolDefines.ERROR_NOERROR:
            print("\nFailed to reset all nodes")

    # keep going until termination has been requested
    while TerminationRequested == False:
        if MyNMTState == SerialProtocolDefines.NODESTATUS_OPERATIONAL:
            # only if we have a node ID and are operational
            nodes += 1
            if nodes > MAX_NUMBER_OF_NODES:
                nodes = 1
            # process nodes from 1 to MAX_NUMBER_OF_NODES
            if nodes != MyNodeID:
                if (NodeStates[nodes - 1] & NODE_STATE_SCANFINISHED) != 0:
                    # nodes scan complete
                    # read device type
                    result = COIADevice.ReadRemoteOD(nodes, 0x1000, 0x00)
                    if result[0] == SerialProtocolDefines.ERROR_NOERROR:
                        data = result[1]
                        deviceType = data[0] | data[1] << 8
                        if deviceType == 401:
                            # This is a CiA401 generic I/O device
                            print("[CiA401 device: data producer enabled] ", end="")
                            # produce data for this device
                            NodeStates[nodes - 1] |= NODE_STATE_PRODUCEDATA
                    else:
                        print(
                            "[Error on device type read for node {} - 0x{:08X}] ".format(
                                nodes, result[0]
                            ),
                            end="",
                            flush=True,
                        )
                        if result[0] == SerialProtocolDefines.ERROR_NODEERROR:
                            print(
                                "Node error: 0x{:08X} ".format(
                                    COIADevice.GetLastNodeError()
                                ),
                                end="",
                            )

                    # reset marker
                    NodeStates[nodes - 1] &= ~NODE_STATE_SCANFINISHED
                elif NodeStates[nodes - 1] & NODE_STATE_PRODUCEDATA != 0:
                    # PDO data production enabled
                    if int(time.time()) >= PDOTime:
                        data = bytearray()
                        data.append(int(time.time()) & 0xFF)
                        COIADevice.WriteRemoteOD(nodes, 0x6200, 0x01, data)
                        data = bytearray()
                        data.append((int(time.time()) >> 8) & 0xFF)
                        COIADevice.WriteRemoteOD(nodes, 0x6200, 0x02, data)
                        PDOTime = int(time.time()) + 1

        # keep receiving packets
        COIADevice.Process()

        # EndTime reached?
        if int(time.time()) >= EndTime:
            break

    # de-register callback functions
    COIADevice.RegisterDataCallback(None)
    # TODO: RegisterSDORequestCallbacks (not yet implemented)

    COIADevice.Disconnect()

    print("\nDisconnected from " + ComPort)

    return 0


if __name__ == "__main__":
    sys.exit(main())
