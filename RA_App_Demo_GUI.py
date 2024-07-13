"""
Remote Access App Demo graphical interface
Copyright:  Embedded Systems Academy (EmSA) 2024
            All rights reserved. esacademy.com
Version:    1.00, EmSA 27-JUN-24
"""

from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.clock import Clock
from kivy.uix.scrollview import ScrollView
from kivy.uix.popup import Popup
from threading import Thread, Lock
from functools import partial
from queue import Queue
import sys
import openpyxl
import re

import SerialProtocol
import SerialProtocolDefines

MAX_NUMBER_OF_NODES = 32
BAUDRATE = 921600
COIADevice = SerialProtocol.SerialProtocol()
Disconnect = False
NMT_RESETCOM = 130

# datalist is accessed from separate threads - lock is used to prevent race conditions
datalist_lock = Lock()
datalist = []

detail_queue = Queue()
disconnect_queue = Queue()

vendor_excel_file = openpyxl.load_workbook("CANopen-vendor-Id_xml_2024-05-01.xlsx")


class MainView(GridLayout):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if sys.platform.startswith("win"):
            self.port_input.text = "COM4"
        else:
            self.port_input.text = "/dev/ttyUSB0"

    def connect(self, *args):
        """
        GUI thread

        Function is called if the connect button is pressed.
        If the button is in connect mode, a thread is started which handles the communication with the COIA device.
        The button is disabled to prevent the start of multiple threads.
        If the button is in disconnect mode an item is inserted in the disconnect queue which signals the process thread to perform the disconnect.
        """
        if self.connect_button.text == "Connect":
            # Disable button until connect was finished
            self.connect_button.disabled = True
            process_thread = Thread(target=self.process)
            process_thread.start()
        else:
            # Disable button until disconnect was finished
            self.connect_button.disabled = True
            disconnect_queue.put("Disconnect")

    def process(self):
        """
        Process thread

        A Connection to the COIA device is established.
        Clock.schedule_once() is used to update the GUI in a thread safe manner.
        The network management state is retrieved from the COIA device.
        If the state is operational the COIA device resets all nodes in the network to make them boot up so we find them.
        The Process which handles all communication with the COIA device is called in a loop until a disconnect signal was received.
        If a detail button was pressed an item in the detail_queue is received - the information is retrieved and send back to the GUI thread.
        """
        # Accessing GUI element from Thread is dangerous
        com_port = self.port_input.text.strip()
        if not COIADevice.Connect(com_port, BAUDRATE):
            Clock.schedule_once(
                partial(self.update_info_label, "Failed to connect to " + com_port)
            )
            self.disconnect()
            return
        Clock.schedule_once(partial(self.update_info_label, "Connected to " + com_port))
        COIADevice.RegisterDataCallback(self.NewData)

        # get NMT state of node
        result = COIADevice.ReadLocalOD(0x5F00, 0x02)
        if result[0] != SerialProtocolDefines.ERROR_NOERROR:
            Clock.schedule_once(
                partial(
                    self.update_info_label,
                    "Failed to get NMT state of COIA node. Error code = 0x{:08X} - Closing port...".format(
                        result[0]
                    ),
                )
            )
            # disconnect from COM port, finished with COIA device
            self.disconnect()
            return
        else:
            MyNMTState = result[1][0]

        # Reset all nodes
        if MyNMTState == SerialProtocolDefines.NODESTATUS_OPERATIONAL:
            NMTCmd = bytearray()
            NMTCmd.append(NMT_RESETCOM & 0xFF)
            NMTCmd.append((NMT_RESETCOM >> 8) & 0xFF)
            result = COIADevice.WriteLocalOD(0x5F0A, 0x01, NMTCmd)
            if result != SerialProtocolDefines.ERROR_NOERROR:
                Clock.schedule_once(
                    partial(self.update_info_label, "Failed to reset all nodes")
                )

        # Connected successfully
        Clock.schedule_once(self.set_button_to_disconnect)
        while disconnect_queue.empty():
            # Loop until disconnect signal was received
            COIADevice.Process()
            if not detail_queue.empty():
                # Detail button was pressed in the GUI
                Clock.schedule_once(
                    partial(
                        self.show_detail, self.get_detailed_data(detail_queue.get())
                    )
                )

        disconnect_queue.get()
        self.disconnect()
        Clock.schedule_once(
            partial(self.update_info_label, "Disconnected from " + com_port)
        )

    def get_detailed_data(self, node: int) -> dict:
        """
        Process thread

        Retrieves detailed data from the specified node.
        If a read operation fails, detailed information about the error is displayed in the info label.
        If there are standard errors, they are appended to the dictionary structure below.

        Parameters:
            node : node id of the remote device
        Returns:
            dictionary with all received data
        """
        data = {
            "Vendor-ID": "-",
            "Product code": "-",
            "Revision number": "-",
            "Serial number": "-",
            "Manufacturer hardware version": "-",
            "Manufacturer software version": "-",
            "Producer heartbeat time": "-",
        }

        # Vendor-ID
        result = COIADevice.ReadRemoteOD(node, 0x1018, 0x01)
        if result[0] == SerialProtocolDefines.ERROR_NOERROR:
            data["Vendor-ID"] = "0x" + "".join(f"{x:02X}" for x in reversed(result[1]))
        else:
            Clock.schedule_once(
                partial(
                    self.update_info_label,
                    "Error on vendor id read for node {} - 0x{:08X}".format(
                        node, result[0]
                    ),
                )
            )
        # Product code
        result = COIADevice.ReadRemoteOD(node, 0x1018, 0x02)
        if result[0] == SerialProtocolDefines.ERROR_NOERROR:
            data["Product code"] = "0x" + "".join(
                f"{x:02X}" for x in reversed(result[1])
            )
        else:
            Clock.schedule_once(
                partial(
                    self.update_info_label,
                    "Error on product code read for node {} - 0x{:08X}".format(
                        node, result[0]
                    ),
                )
            )
        # Revision number
        result = COIADevice.ReadRemoteOD(node, 0x1018, 0x03)
        if result[0] == SerialProtocolDefines.ERROR_NOERROR:
            data["Revision number"] = "0x" + "".join(
                f"{x:02X}" for x in reversed(result[1])
            )
        else:
            Clock.schedule_once(
                partial(
                    self.update_info_label,
                    "Error on revision number read for node {} - 0x{:08X}".format(
                        node, result[0]
                    ),
                )
            )
        # Serial number
        result = COIADevice.ReadRemoteOD(node, 0x1018, 0x04)
        if result[0] == SerialProtocolDefines.ERROR_NOERROR:
            data["Serial number"] = "0x" + "".join(
                f"{x:02X}" for x in reversed(result[1])
            )
        else:
            Clock.schedule_once(
                partial(
                    self.update_info_label,
                    "Error on serial number read for node {} - 0x{:08X}".format(
                        node, result[0]
                    ),
                )
            )
        # Manufacturer hardware version
        result = COIADevice.ReadRemoteOD(node, 0x1009, 0x00)
        if result[0] == SerialProtocolDefines.ERROR_NOERROR:
            data["Manufacturer hardware version"] = result[1].decode("ascii")
        else:
            Clock.schedule_once(
                partial(
                    self.update_info_label,
                    "Error on manufacturer hardware version read for node {} - 0x{:08X}".format(
                        node, result[0]
                    ),
                )
            )
        # Manufacturer software version
        result = COIADevice.ReadRemoteOD(node, 0x100A, 0x00)
        if result[0] == SerialProtocolDefines.ERROR_NOERROR:
            data["Manufacturer software version"] = result[1].decode("ascii")
        else:
            Clock.schedule_once(
                partial(
                    self.update_info_label,
                    "Error on manufacturer software version read for node {} - 0x{:08X}".format(
                        node, result[0]
                    ),
                )
            )
        # Producer heartbeat time
        result = COIADevice.ReadRemoteOD(node, 0x1017, 0x00)
        if result[0] == SerialProtocolDefines.ERROR_NOERROR:
            data["Producer heartbeat time"] = "{} ms".format(
                (result[1][0] << 0) | (result[1][1] << 8)
            )
        else:
            Clock.schedule_once(
                partial(
                    self.update_info_label,
                    "Error on producer heartbeat time read for node {} - 0x{:08X}".format(
                        node, result[0]
                    ),
                )
            )

        # Error field
        result = COIADevice.ReadRemoteOD(node, 0x1003, 0x00)
        if result[0] == SerialProtocolDefines.ERROR_NOERROR:
            # result contains number of errors available
            for i in range(1, int(result[1][0]) + 1):
                # read all available errors
                result = COIADevice.ReadRemoteOD(node, 0x2222, i)
                if result[0] == SerialProtocolDefines.ERROR_NOERROR:
                    # append errors to data dictionary
                    data["Error {}".format(i)] = "0x" + "".join(
                        f"{x:02X}" for x in reversed(result[1])
                    )
                else:
                    Clock.schedule_once(
                        partial(
                            self.update_info_label,
                            "Error on read standard error field {} for node {} - 0x{:08X}".format(
                                i, node, result[0]
                            ),
                        )
                    )
        else:
            Clock.schedule_once(
                partial(
                    self.update_info_label,
                    "Error on number of errors read for node {} - 0x{:08X}".format(
                        node, result[0]
                    ),
                )
            )

        return data

    def disconnect(self):
        """
        Process thread

        Resets callback and disconnects from COIA device.
        Connect button is put to connect mode.
        """
        COIADevice.RegisterDataCallback(None)
        COIADevice.Disconnect()
        Clock.schedule_once(self.set_button_to_connect)

    def NewData(self, node: int, Index: int, Subindex: int, Data: bytearray):
        """
        Process thread

        Registered callback for incoming data.

        Parameters:
            node     : node id of the sender device
            Index    : index of the received data
            Subindex : subindex of the received data
            data     : bytearray containing the received data
        """
        if Index == 0x5F04:
            # node status
            self.NodeStatusChanged(Subindex, Data[0])

    def NodeStatusChanged(self, node: int, State: int):
        """
        Process thread

        Handle node status change of all devices.
        If this is the first state change of a node which is received, the node is added to the datalist.
        If heartbeat lost or scan aborted is received the detailed button for this node is disabled.
        If scan complete is received the vendor id and product name is retrieved from the remote device and the detail button is enabled.
        The company name is resolved using the vendor id and an attached xml/excel file.
        All received information are updated in the datalist.

        Parameters:
            node  : node id of the sender device
            State : network state of the device
        """
        if node >= MAX_NUMBER_OF_NODES:
            return

        with datalist_lock:
            # datalist is used in GUI thread and process thread so it is protected with a mutex to prevent race conditions
            if not any(d["node"] == node for d in datalist):
                # If node is not in data list - add it
                self.add_node_to_list(node)

        if State == SerialProtocolDefines.NODESTATUS_BOOT:
            self.update_datalist_entry(node, "status", "BOOT")
        elif State == SerialProtocolDefines.NODESTATUS_STOPPED:
            self.update_datalist_entry(node, "status", "STOP")
        elif State == SerialProtocolDefines.NODESTATUS_OPERATIONAL:
            self.update_datalist_entry(node, "status", "OPERATIONAL")
        elif State == SerialProtocolDefines.NODESTATUS_PREOP:
            self.update_datalist_entry(node, "status", "PREOP")
        elif State == SerialProtocolDefines.NODESTATUS_EMCY_OVER:
            self.update_datalist_entry(node, "status", "EMCY CLEAR")
        elif State == SerialProtocolDefines.NODESTATUS_EMCY_NEW:
            self.update_datalist_entry(node, "status", "NEW EMCY")
        elif State == SerialProtocolDefines.NODESTATUS_HBACTIVE:
            self.update_datalist_entry(node, "status", "HB ACTIVE")
        elif State == SerialProtocolDefines.NODESTATUS_HBLOST:
            # Heartbeat lost - disable detail button
            self.set_detail_button_enabled(node, False)
            self.update_datalist_entry(node, "status", "HB LOST")
        elif State == SerialProtocolDefines.NODESTATUS_SCANSTARTED:
            self.update_datalist_entry(node, "status", "SCAN INIT")
        elif State == SerialProtocolDefines.NODESTATUS_SCANCOMPLETE:
            # App can now access this node - read vendor id and product name

            # Vendor-ID
            result = COIADevice.ReadRemoteOD(node, 0x1018, 0x01)
            if result[0] == SerialProtocolDefines.ERROR_NOERROR:
                self.update_datalist_entry(
                    node, "vendor", "".join(f"{x:02X}" for x in reversed(result[1]))
                )
                # Get vendor name from xslx/xml file
                try:
                    excel_sheet = vendor_excel_file["Tabelle1"]
                    for row in excel_sheet.iter_rows():
                        # search for line with correct vendor id
                        match = re.search(
                            'vendor id="({})"'.format(
                                "".join(f"{x:02X}" for x in reversed(result[1]))
                            ),
                            row[0].value,
                        )
                        if match is None:
                            # no correct vendor id found in this line - continue search in next line
                            continue
                        # search for company name in this line
                        match = re.search(
                            'company="(.*?)"',
                            row[0].value,
                        )
                        if match is not None:
                            # company name found in this line
                            company_string = match.group(1)
                        else:
                            # no company name found in this line - finish search
                            break
                        # search for department name in this line
                        match = re.search(
                            'department="(.*?)"',
                            row[0].value,
                        )
                        if match is not None:
                            # department name found in this line - finish search
                            company_string += " - " + match.group(1)

                        self.update_datalist_entry(node, "vendor", company_string)
                        break
                except:
                    pass
            else:
                Clock.schedule_once(
                    partial(
                        self.update_info_label,
                        "Error on vendor id read for node {} - 0x{:08X}".format(
                            node, result[0]
                        ),
                    )
                )

            # Product name
            result = COIADevice.ReadRemoteOD(node, 0x1008, 0x00)
            if result[0] == SerialProtocolDefines.ERROR_NOERROR:
                self.update_datalist_entry(node, "product", result[1].decode("ascii"))
            else:
                Clock.schedule_once(
                    partial(
                        self.update_info_label,
                        "Error on product name read for node {} - 0x{:08X}".format(
                            node, result[0]
                        ),
                    )
                )

            # Enable detail button
            self.set_detail_button_enabled(node, True)

            self.update_datalist_entry(node, "status", "SCANNED")
        elif State == SerialProtocolDefines.NODESTATUS_SCANABORTED:
            # Scan abort - disable detail button
            self.set_detail_button_enabled(node, False)
            self.update_datalist_entry(node, "status", "SCAN ABORT")
        elif State == SerialProtocolDefines.NODESTATUS_RESETAPP:
            self.update_datalist_entry(node, "status", "RESET APP")
        elif State == SerialProtocolDefines.NODESTATUS_RESETCOM:
            self.update_datalist_entry(node, "status", "RESET APP")
        elif State == SerialProtocolDefines.NODESTATUS_SLEEP:
            self.update_datalist_entry(node, "status", "SLEEP")
        else:
            self.update_datalist_entry(node, "status", "UNKNOWN")

    def update_info_label(self, info: str, *args):
        """
        GUI thread

        Updates the info label at the bottom.
        The reference to this function is mostly given to the Clock.schedule_once() function
        to update the label - thread safe - from the process thread.

        Parameters:
            info : string
        """
        self.info_label.text = info

    def set_button_to_disconnect(self, *args):
        """
        GUI thread

        Changes the mode of the connect button to disconnect.
        The reference to this function is given to the Clock.schedule_once() function
        to update the label - thread safe - from the process thread.
        """
        self.connect_button.text = "Disconnect"
        self.connect_button.disabled = False

    def set_button_to_connect(self, *args):
        """
        GUI thread

        Changes the mode of the connect button to connect.
        The reference to this function is given to the Clock.schedule_once() function
        to update the label - thread safe - from the process thread.
        """
        self.connect_button.text = "Connect"
        self.connect_button.disabled = False

    def add_node_to_list(self, node: int):
        """
        Process thread

        This function may only be called if datalist was previously locked with a mutex.
        Adds a template dictionary entry to datalist only containing the node id and dummy data.
        After adding an entry data list is sorted by node id.

        Parameters:
            node : node id of the device which will be added to datalist
        """
        global datalist
        dict = {
            "node": node,
            "status": "-",
            "vendor": "-",
            "product": "-",
            "detail_enabled": False,
        }
        datalist.append(dict)
        datalist = sorted(datalist, key=lambda d: d["node"])

    def update_datalist_entry(self, node: int, key: str, value: str):
        """
        Process thread

        This function is used to update any string entry in datalist.
        Redrawing the list in GUI thread is triggered.

        Parameters:
            node  : node id of the device whose information is to be updated
            key   : key of the dictionary entry which should be updated
            value : data
        """
        global datalist
        with datalist_lock:
            # datalist is used in GUI thread and process thread so it is protected with a mutex to prevent race conditions
            for element in datalist:
                if element["node"] == node:
                    element[key] = value
                    break
        Clock.schedule_once(self.update_data_list)

    def set_detail_button_enabled(self, node: int, enable: bool, *args):
        """
        Process thread

        This function is used to update the information in datalist if the detailed button is enabled.

        Parameters:
            node   : node id of the device whose information is to be updated
            enable : True to enable button, False to disable button
        """
        global datalist
        with datalist_lock:
            # datalist is used in GUI thread and process thread so it is protected with a mutex to prevent race conditions
            for element in datalist:
                if element["node"] == node:
                    element["detail_enabled"] = enable
                    break

    def update_data_list(self, *args):
        """
        GUI thread

        Redraws table from datalist.
        """
        self.data_list.clear_widgets()
        with datalist_lock:
            # datalist is used in GUI thread and process thread so it is protected with a mutex to prevent race conditions
            for row in datalist:
                nodeId_label = Label(text=str(row["node"]), size_hint=(0.1, 1))
                self.data_list.add_widget(nodeId_label)

                nmtStatus_label = Label(text=row["status"], size_hint=(0.1, 1))
                self.data_list.add_widget(nmtStatus_label)

                vendor_label = Label(text=row["vendor"], size_hint=(0.35, 1))
                self.data_list.add_widget(vendor_label)

                product_label = Label(text=row["product"], size_hint=(0.35, 1))
                self.data_list.add_widget(product_label)

                detail_button = Button(text="i", size_hint=(0.1, 1))
                if not row["detail_enabled"]:
                    detail_button.disabled = True
                detail_button.bind(on_press=self.request_detail)
                self.data_list.add_widget(detail_button)

    def request_detail(self, *args):
        """
        GUI thread

        args[0] contains the object of the button which was pressed.
        args[0].parent.children is a list of all widgets in the table/layout.
        By iterating through this list and searching for the pressed button we get the index of the button widget in the table.
        Because the indices are counted from right to left,
        the widget with the index of the button + 4 contains the node id we are searching for.

        |  4   |   3   |   2    |    1    |   0    |
        --------------------------------------------
        | node | state | vendor | product | button |

        The node id is sent via detail_queue to the process thread.
        """
        for count, element in enumerate(args[0].parent.children):
            if args[0] == element:
                detail_queue.put(int(args[0].parent.children[count + 4].text))
        # if there is a lot of detail data, it can take some time until something is displayed - so we give a hint
        self.update_info_label("Fetch detail data...")

    def show_detail(self, data: dict, *args):
        """
        GUI thread

        To display the detailed data a popup window is used.
        A scrollable table is displayed with all keys in the data dictionary in the first colum and the values in the second.

        Parameters:
            data   : dictionary with detailed data
        """
        popupView = PopupView()

        for i in data:
            key = Label(text=i, size_hint=(0.5, 1), halign="left")
            key.bind(size=key.setter("text_size"))
            value = Label(text=data[i], size_hint=(0.5, 1), halign="left")
            value.bind(size=value.setter("text_size"))
            popupView.detail_list.add_widget(key)
            popupView.detail_list.add_widget(value)

        popup = Popup(title="Details", content=popupView, size_hint=(0.8, 0.8))
        popup.open()

    def clear_list(self, *args):
        """
        GUI thread

        This function is called if the clear button is pressed.
        The datalist, the displayed table and the info label is cleared.

        """
        with datalist_lock:
            # datalist is used in GUI thread and process thread so it is protected with a mutex to prevent race conditions
            datalist.clear()
        self.update_data_list()
        self.update_info_label("")


class PopupView(ScrollView):
    pass


class MyApp(App):
    def build(self):
        self.title = "CANgineBerry CANopen Manager Demonstration"
        return MainView()


if __name__ == "__main__":
    app = MyApp()
    app.run()
    disconnect_queue.put("Disconnect")
