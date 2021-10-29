#!/usr/bin/env python3
"""! @brief Main.

This is the main file of the program, which implements a graphic user interface based on PyQt5 framework, to
interact with the bootloader using the serial port of the host computer.

This program requires that PyQt5 and pyserial be installed within the Python environment you are running
this program in.
"""

##
# @file main.py
#

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtCore
import boot_serial
import boot_cmd

## Variable for storing the name of the binary file used for writing a new application
file_name = ""
## Variable for storing the size of the binary file used for writing a new application
file_size = 0
## Variable for storing the initial address where the new application will be stored
address = 0
## List used for selecting the flash protection mode depending of the PCROP bit
protection_mode = ["Write Protection", "Read/Write Protection","No protection"]

class MainApp(QMainWindow):
    """! A class for implementing the main window and managing the graphic user interface."""

    def __init__(self, parent=None):
        """! The MainApp base class initializer.
        @param parent Is a reference to the parent object (in Qt terminology).
        @return An instance of the MainApp class initialized.
        """

        super(MainApp, self).__init__(parent)

        self.setWindowTitle("Bootloader Host Application")

        # Create a Qt widget, which will be our main window
        widget = QWidget()

        # Instances of different menu layers
        grid = QGridLayout()
        ## Button group for managing the serial port connection
        self.btn_grp_cnt = ConnectButtonnGroup("Connection:")
        grid.addWidget(self.btn_grp_cnt, 0, 0, 1, 2)
        ## Button group for managing the buttons related with the commands
        self.btn_grp_cmd = CommandButtonnGroup("Commands:")
        grid.addWidget(self.btn_grp_cmd, 1, 0)
        ## General display
        self.display = Display("Display:")
        ## For displaying the erase menu
        self.erase_menu = EraseMenu("Erase Menu:")
        ## For displaying the write menu
        self.write_display = WriteDisplay("Write Memory:")
        ## For display the enable read/write protection menu
        self.rw_prot_display = RWProtectDisplay("Read/Write Protection Menu:")
        ## For display the read menu
        self.read_display = MemReadDisplay("Read Memory:")
        ## Stacked layout for overlaying the different menus
        self.stack_lay = QStackedLayout()
        self.stack_lay.addWidget(self.display)
        self.stack_lay.addWidget(self.erase_menu)
        self.stack_lay.addWidget(self.write_display)
        self.stack_lay.addWidget(self.rw_prot_display)
        self.stack_lay.addWidget(self.read_display)
        grid.addLayout(self.stack_lay, 1, 1, 1, 2)
        widget.setLayout(grid)
        self.setCentralWidget(widget)

        # Connections between buttons and slots
        self.btn_grp_cnt.btn_connect.clicked.connect(self.slot_connect)
        self.btn_grp_cmd.btn_cmd_ver.clicked.connect(self.slot_version)
        self.btn_grp_cmd.btn_cmd_help.clicked.connect(self.slot_help)
        self.btn_grp_cmd.btn_cmd_cid.clicked.connect(self.slot_cid)
        self.btn_grp_cmd.btn_cmd_rdp.clicked.connect(self.slot_rdp)
        self.btn_grp_cmd.btn_cmd_go.clicked.connect(self.slot_go)
        self.btn_grp_cmd.btn_cmd_erase.clicked.connect(self.slot_erase)
        self.erase_menu.btn_cancel.clicked.connect(self.slot_cancel_erase)
        self.erase_menu.check_mass_erase.stateChanged.connect(self.slot_mass_erase)
        self.erase_menu.btn_ok.clicked.connect(self.slot_ok_erase)
        self.btn_grp_cmd.btn_cmd_write.clicked.connect(self.slot_write)
        self.write_display.btn_start.clicked.connect(self.slot_write_start)
        self.btn_grp_cmd.btn_cmd_read_sector_status.clicked.connect(self.slot_read_sector_st)
        self.btn_grp_cmd.btn_cmd_rw_protect.clicked.connect(self.slot_rw_protect)
        self.rw_prot_display.check_dis_protect.stateChanged.connect(self.slot_rw_disable)
        self.rw_prot_display.btn_write_protect.clicked.connect(self.slot_rw_protect_write)
        self.btn_grp_cmd.btn_cmd_read.clicked.connect(self.slot_read)
        self.read_display.btn_read.clicked.connect(self.slot_read_start)

    def slot_connect(self):
        """! Slot for managing the connection regarding to the serial port."""

        if boot_serial.connect_serial(self.btn_grp_cnt.usb_list.currentText()):
            self.btn_grp_cmd.setEnabled(True)
        else:
            pass

    def slot_version(self):
        """! Slot for displaying the bootloader version."""

        self.stack_lay.setCurrentIndex(0)
        error, timeout, value = boot_cmd.cmd_ver()
        if not error and not timeout:
            self.display.lab_display.setText('Bootloader Version: ' + hex(value[0]))
        else:
            if timeout:
                self.display.lab_display.setText('Error: timeout')
            else:
                self.display.lab_display.setText('Error: command returned Non ACK')

    def slot_help(self):
        """! Slot for displaying the bootloader supported commands."""

        self.stack_lay.setCurrentIndex(0)
        error, timeout, value = boot_cmd.cmd_help()
        if not error and not timeout:
            str_cmp = []
            str_cmp.append('Commands Supported:\n')
            for x in value:
                str_cmp.append('\n' + hex(x))
            self.display.lab_display.setText(''.join(str_cmp))
        else:
            if timeout:
                self.display.lab_display.setText('Error: timeout')
            else:
                self.display.lab_display.setText('Error: command returned Non ACK')

    def slot_cid(self):
        """! Slot for displaying the chip identifier."""

        self.stack_lay.setCurrentIndex(0)
        error, timeout, value = boot_cmd.cmd_cid()
        if not error and not timeout:
            str_cmp = []
            str_cmp.append('Chip Identifier: ')
            ci = (value[1] << 8 ) + value[0]
            str_cmp.append(hex(ci))
            self.display.lab_display.setText(''.join(str_cmp))
        else:
            if timeout:
                self.display.lab_display.setText('Error: timeout')
            else:
                self.display.lab_display.setText('Error: command returned Non ACK')

    def slot_rdp(self):
        """! Slot for displaying the read protection option level."""

        self.stack_lay.setCurrentIndex(0)
        error, timeout, value = boot_cmd.cmd_rdp()
        if not error and not timeout:
            self.display.lab_display.setText('Read Protection Opt Lvl: ' + hex(value[0]))
        else:
            if timeout:
                self.display.lab_display.setText('Error: timeout')
            else:
                self.display.lab_display.setText('Error: command returned Non ACK')

    def slot_go(self):
        """! Slot for managing the go to adrress command."""

        self.stack_lay.setCurrentIndex(0)
        text, ok = QInputDialog().getText(self, \
                                          'Enter address to jump:', \
                                          'Address (hex format):', \
                                          QLineEdit.Normal, \
                                          '0x08008346')
        if ok and text:
            error, timeout, ret = boot_cmd.cmd_go(text)
            if not error and not timeout:
                if ret[0] == 0:
                    str_cmp = []
                    str_cmp.append('Jump to address: ')
                    str_cmp.append(text)
                    self.display.lab_display.setText(''.join(str_cmp))
                else:
                    self.display.lab_display.setText('Error: invalid address')
            else:
                if timeout:
                    self.display.lab_display.setText('Error: timeout')
                else:
                    self.display.lab_display.setText('Error: command returned Non ACK')

    def slot_erase(self):
        """! Slot for displaying the erase menu."""

        self.btn_grp_cmd.setEnabled(False)
        self.stack_lay.setCurrentIndex(1)

    def slot_mass_erase(self, state : bool):
        """! Slot for managing the mass erase checkbox in the erase menu."""

        if(QtCore.Qt.Checked == state):
            self.erase_menu.spin_sector.setEnabled(False)
            self.erase_menu.spin_num_sector.setEnabled(False)
        else:
            self.erase_menu.spin_sector.setEnabled(True)
            self.erase_menu.spin_num_sector.setEnabled(True)

    def slot_cancel_erase(self):
        """! Slot for managing the cancel button in the erase menu."""

        self.stack_lay.setCurrentIndex(0)
        self.btn_grp_cmd.setEnabled(True)

    def slot_ok_erase(self):
        """! Slot for managing the showed message box in the erase menu."""

        if self.erase_menu.check_mass_erase.isChecked():
            msg_mass_erase = QMessageBox()
            msg_mass_erase.setWindowTitle("Warning Message")
            msg_mass_erase.setIcon(QMessageBox.Warning)
            msg_mass_erase.setText("A Mass Erase Will Be Done!!!")
            msg_mass_erase.setInformativeText("Are you sure to continue?")
            msg_mass_erase.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            msg_mass_erase.buttonClicked.connect(self.slot_mass_erase_ok)
            msg_mass_erase.exec_()
        else:
            sector = self.erase_menu.spin_sector.value()
            offset = self.erase_menu.spin_num_sector.value()
            if((sector + offset) > 8):
                msg_error_erase = QMessageBox()
                msg_error_erase.setWindowTitle("Error Message")
                msg_error_erase.setIcon(QMessageBox.Critical)
                msg_error_erase.setText("Invalid Number of Sectors")
                msg_error_erase.setStandardButtons(QMessageBox.Ok)
                msg_error_erase.exec_()
            else:
                msg_sector_erase = QMessageBox()
                msg_sector_erase.setWindowTitle("Information Message")
                msg_sector_erase.setIcon(QMessageBox.Information)
                if(offset == 1):
                    msg_sector_erase.setText("Sector " + str(sector) + " will be erased")
                else:
                    msg_sector_erase.setText("Sectors from " + \
                                             str(sector) + \
                                             " to " + \
                                             str(sector + offset - 1) + \
                                             " will be erased")
                msg_sector_erase.setInformativeText("Are you sure to continue?")
                msg_sector_erase.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
                msg_sector_erase.buttonClicked.connect(self.slot_sector_erase_ok)
                msg_sector_erase.exec_()

    def slot_mass_erase_ok(self, i : QPushButton):
        """! Slot for managing the mass erase command.
        @param i Is a push button instance for confirming the erase process.
        """

        if(i.text() == 'OK'):
            error, timeout, value = boot_cmd.cmd_erase(0xFF, 0)
            self.check_process('Erase', error, timeout, value)

    def slot_sector_erase_ok(self, i : QPushButton):
        """! Slot for managing the sector erase command.
        @param i Is a push button instance for confirming the erase process.
        """

        if(i.text() == 'OK'):
            error, timeout, value = boot_cmd.cmd_erase(self.erase_menu.spin_sector.value(), \
                                                       self.erase_menu.spin_num_sector.value())
            self.check_process('Erase', error, timeout, value)

    def slot_write(self):
        """! Slot for displaying the write menu."""

        global file_name
        global file_size
        global address

        self.stack_lay.setCurrentIndex(2)
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        dlg = QFileDialog()
        file_name, _filter = dlg.getOpenFileName(None, \
                                                 'Select Binary File', \
                                                 './', \
                                                 filter = '*.bin', \
                                                 options = options)
        if file_name:
            file_size = os.path.getsize(file_name)
            address, ok = QInputDialog().getText(self, \
                                                 'Enter Starting Address to Write:', \
                                                 'Address (hex format):', \
                                                 QLineEdit.Normal, \
                                                 '0x08008000')
            if ok and address:
                self.write_display.lab_file.setText("File: " + file_name)
                self.write_display.lab_size.setText("Size: " + str(file_size) + " Bytes")
                self.write_display.lab_addr.setText("Starting Address: " + address)

    def slot_write_start(self):
        """! Slot for managing the write process."""

        global address

        address_flash = int(address, 16)
        bytes_remaining = 0
        bytes_to_sent = 0
        len_to_read = 0

        bytes_remaining = file_size - bytes_to_sent

        bin_file = open(file_name,'rb')

        while(bytes_remaining):
            if(bytes_remaining >= 128):
                len_to_read = 128
            else:
                len_to_read = bytes_remaining

            error, timeout, value = boot_cmd.cmd_write(len_to_read, address_flash, bin_file)

            if(error or (value[0] == 1)):
                break

            address_flash += len_to_read
            bytes_to_sent += len_to_read
            bytes_remaining = file_size - bytes_to_sent

            self.write_display.pbar.setValue(int((bytes_to_sent/file_size)*100))

        self.check_process('Write', error, timeout, value)

    def slot_read_sector_st(self):
        """! Slot for managing the read sector status command and displaying the result."""

        self.stack_lay.setCurrentIndex(0)
        error, timeout, value = boot_cmd.cmd_read_sector_st()

        if not error and not timeout:
            if(value[1] & (1 << 7)):
                self.display.lab_display.setText('Flash Protection Mode: Read/Write Protection (PCROP)\n')
            else:
                self.display.lab_display.setText('Flash Protection Mode: Write Protection\n')

            for x in range(8):
                self.display.lab_display.setText(self.display.lab_display.text() + \
                                                 "\nSector " + \
                                                 str(x) + \
                                                 ":\t" + \
                                                 protect_type(value, x))
        else:
            if timeout:
                self.display.lab_display.setText('Error: timeout')
            else:
                self.display.lab_display.setText('Error: command returned Non ACK')

    def slot_rw_protect(self):
        """! Slot for displaying the enable read/write protection menu."""

        self.stack_lay.setCurrentIndex(3)

    def slot_rw_disable(self, state : bool):
        """! Slot for enabling/disabling the check boxes of the enable read/write protection menu depending
        on the disabling protetion for all sectors check box.
        @param state The state of the "Disable R/W Protection for All Sectors" check box.
        """

        if(QtCore.Qt.Checked == state):
            self.rw_prot_display.check_sprmod.setEnabled(False)
            self.rw_prot_display.check_sector0.setEnabled(False)
            self.rw_prot_display.check_sector1.setEnabled(False)
            self.rw_prot_display.check_sector2.setEnabled(False)
            self.rw_prot_display.check_sector3.setEnabled(False)
            self.rw_prot_display.check_sector4.setEnabled(False)
            self.rw_prot_display.check_sector5.setEnabled(False)
            self.rw_prot_display.check_sector6.setEnabled(False)
            self.rw_prot_display.check_sector7.setEnabled(False)
        else:
            self.rw_prot_display.check_sprmod.setEnabled(True)
            self.rw_prot_display.check_sector0.setEnabled(True)
            self.rw_prot_display.check_sector1.setEnabled(True)
            self.rw_prot_display.check_sector2.setEnabled(True)
            self.rw_prot_display.check_sector3.setEnabled(True)
            self.rw_prot_display.check_sector4.setEnabled(True)
            self.rw_prot_display.check_sector5.setEnabled(True)
            self.rw_prot_display.check_sector6.setEnabled(True)
            self.rw_prot_display.check_sector7.setEnabled(True)

    def slot_rw_protect_write(self):
        """! Slot for manaing the enable read/write protection command."""

        mode = 1
        sectors = 0

        if self.rw_prot_display.check_dis_protect.isChecked():
            error, timeout, value = boot_cmd.cmd_dis_rw_protect()
            self.check_process('Disable R/W Protection', error, timeout, value)
        else:
            if self.rw_prot_display.check_sprmod.isChecked():
                mode = 2

            if self.rw_prot_display.check_sector0.isChecked():
                sectors |= 0x01
            if self.rw_prot_display.check_sector1.isChecked():
                sectors |= 0x02
            if self.rw_prot_display.check_sector2.isChecked():
                sectors |= 0x04
            if self.rw_prot_display.check_sector3.isChecked():
                sectors |= 0x08
            if self.rw_prot_display.check_sector4.isChecked():
                sectors |= 0x10
            if self.rw_prot_display.check_sector5.isChecked():
                sectors |= 0x20
            if self.rw_prot_display.check_sector6.isChecked():
                sectors |= 0x40
            if self.rw_prot_display.check_sector7.isChecked():
                sectors |= 0x80

            error, timeout, value = boot_cmd.cmd_en_rw_protect(sectors, mode)
            self.check_process('Enable R/W Protection', error, timeout, value)

    def slot_read(self):
        """! Slot for displaying the read menu."""

        self.stack_lay.setCurrentIndex(4)

    def slot_read_start(self):
        """! Slot for managing the read command."""

        self.read_display.lab_mem_read.clear()
        addr = int(self.read_display.text_addr.text(), 16)
        offset = int(self.read_display.text_offset.text(), 10)
        if(4*offset < 256):
            error, timeout, value = boot_cmd.cmd_mem_read(addr, 4*offset)
            if timeout:
                self.read_display.lab_mem_read.setText('Error: timeout')
            elif error:
                self.read_display.lab_mem_read.setText('Error: command returned Non ACK')
            elif(value[0] == 0):
                for i,_ in enumerate(value[1:]):
                    i += 1
                    if(not i%4):
                        mem_read_value = (value[i] << 24) + \
                                         (value[i-1] << 16) + \
                                         (value[i-2] << 8) + \
                                         value[i-3]
                        self.read_display.lab_mem_read.setText(self.read_display.lab_mem_read.text() + \
                                                               '0x{0:0{1}X}'.format(addr + i - 4, 8) + \
                                                               "\t" + \
                                                               '0x{0:0{1}X}'.format(mem_read_value, 8) + \
                                                               "\n")
            else:
                self.read_display.lab_mem_read.setText("Error: try accessing to an invalid memory address!")
        else:
            self.read_display.lab_mem_read.setText("Error: too size for offset value!")

    def check_process(self, process : str, error : bool, timeout : bool, value : bytearray):
        """! Show a popup message with the result of a process, for example an erase process.
        @param process Is the name of the process: erase, write.
        @param error Is the returned error flag from the command of the process.
        @param timeout Is the returned timeout flag from the command of the process.
        @param value Is the returned value from the command of the process.
        """

        msg_popup = QMessageBox()
        msg_popup.setStandardButtons(QMessageBox.Ok)
        if(not timeout and not error and (value[0] == 0)):
            msg_popup.setWindowTitle('Info Message')
            msg_popup.setIcon(QMessageBox.Information)
            msg_popup.setText(process + ' Process Success!!!')
        else:
            msg_popup.setWindowTitle('Error Message')
            msg_popup.setIcon(QMessageBox.Warning)
            if timeout:
                msg_popup.setText('Error: timeout')
            elif value[0] == 1:
                msg_popup.setText(process + ' Process Failed!!!')
            else:
                msg_popup.setText('Error: command returned Non ACK')
        msg_popup.exec_()

def protect_type(status : list, n : int) -> str:
    """! Return the protection status for a sector of the flash depending on the protection mode.

    @param status Is a byte array with the nWRP register value from the bootloader.
    @param n Is the number of the flash sector.

    @return String with the protection mode description.
    """

    if(status[1] & (1 << 7)):
        if(status[0] & (1 << n)):
            return protection_mode[1]
        else:
            return protection_mode[2]
    else:
        if(status[0] & (1 << n)):
            return protection_mode[2]
        else:
            return protection_mode[0]

class ConnectButtonnGroup(QGroupBox):
    """! A class for managing the buttons related with the connection via serial port."""

    def __init__(self, name):
        """! The ConnectButtonnGroup base class initializer.
        @param name The name of the ConnectButtonnGroup.
        @return An instance of the ConnectButtonnGroup class initialized with the specified name.
        """

        super(ConnectButtonnGroup, self).__init__(name)

        ## Push button for connect the serial port
        self.btn_connect = QPushButton("Connect")
        ## Combo box for selecting the serial port
        self.usb_list = QComboBox()
        self.usb_list.addItems(boot_serial.serial_ports())

        hbox = QHBoxLayout()
        hbox.addWidget(self.btn_connect)
        hbox.addWidget(self.usb_list)
        hbox.addStretch(1)
        self.setLayout(hbox)

class CommandButtonnGroup(QGroupBox):
    """! A class for managing the buttons related with sending a command to the bootloader."""

    def __init__(self, name):
        """! The CommandButtonnGroup base class initializer.
        @param name The name of the CommandButtonnGroup.
        @return An instance of the CommandButtonnGroup class initialized with the specified name.
        """

        super(CommandButtonnGroup, self).__init__(name)

        ## Push button for sending get version command
        self.btn_cmd_ver = QPushButton("Get Version")
        ## Push button for sending get help command
        self.btn_cmd_help = QPushButton("Get Help")
        ## Push button for sending chip identifier command
        self.btn_cmd_cid = QPushButton("Get CID")
        ## Push button for sending read protection command
        self.btn_cmd_rdp = QPushButton("Get RDP")
        ## Push button for sending go to address command
        self.btn_cmd_go = QPushButton("Go Addr")
        ## Push button for sending erase flash command
        self.btn_cmd_erase = QPushButton("Erase")
        ## Push button for sending write command
        self.btn_cmd_write = QPushButton("Write")
        ## Push button for sending read sector status command
        self.btn_cmd_read_sector_status = QPushButton("Read Sector Status")
        ## Push button for sending enable read/write protection flash command
        self.btn_cmd_rw_protect = QPushButton("R/W Protection")
        ## Push button for sending read command
        self.btn_cmd_read = QPushButton("Read")
        self.setEnabled(False)

        vbox = QVBoxLayout()
        vbox.addWidget(self.btn_cmd_ver)
        vbox.addWidget(self.btn_cmd_help)
        vbox.addWidget(self.btn_cmd_cid)
        vbox.addWidget(self.btn_cmd_rdp)
        vbox.addWidget(self.btn_cmd_go)
        vbox.addWidget(self.btn_cmd_erase)
        vbox.addWidget(self.btn_cmd_write)
        vbox.addWidget(self.btn_cmd_read_sector_status)
        vbox.addWidget(self.btn_cmd_rw_protect)
        vbox.addWidget(self.btn_cmd_read)
        vbox.addStretch(1)
        self.setLayout(vbox)

class Display(QGroupBox):
    """! A class for implementing the panel for showing the output for most commands."""

    def __init__(self, name):
        """! The Display base class initializer.
        @param name The name of the Display.
        @return An instance of the Display class initialized with the specified name.
        """

        super(Display, self).__init__(name)

        ## Label for displaying the information from commands
        self.lab_display = QLabel()
        self.lab_display.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        vbox = QVBoxLayout()
        vbox.addWidget(self.lab_display)
        vbox.addStretch(1)
        self.setLayout(vbox)

class EraseMenu(QGroupBox):
    """! A class used for implementing the panel for the erase command."""

    def __init__(self, name):
        """! The EraseMenu base class initializer.
        @param name The name of the EraseMenu.
        @return An instance of the EraseMenu class initialized with the specified name.
        """

        super(EraseMenu, self).__init__(name)

        ## Check box for selecting a mass erase
        self.check_mass_erase = QCheckBox("Mass Erase")
        ## Label for information about sector selection
        self.label1 = QLabel("Starting Sector to Erase:")
        ## Spin box for selecting the starting sector to erase
        self.spin_sector = QSpinBox()
        self.spin_sector.setFixedWidth(60)
        self.spin_sector.setRange(0, 7)
        ## Label for information about number of sector to erase
        self.label2 = QLabel("Number of Consecutive Sectors to Erase:")
        ## Spin box for selecting the number of consecutive sectors to erase
        self.spin_num_sector = QSpinBox()
        self.spin_num_sector.setRange(1, 8)
        self.spin_num_sector.setFixedWidth(60)
        ## Push button for starting the erase process
        self.btn_ok = QPushButton("OK")
        self.btn_ok.setFixedWidth(120)
        ## Push button for canceling the erase (return from erase menu)
        self.btn_cancel = QPushButton("Cancel")
        self.btn_cancel.setFixedWidth(120)

        vbox = QVBoxLayout()
        hbox = QHBoxLayout()
        vbox.addWidget(self.check_mass_erase)
        vbox.addWidget(self.label1)
        vbox.addWidget(self.spin_sector)
        vbox.addWidget(self.label2)
        vbox.addWidget(self.spin_num_sector)
        hbox.addWidget(self.btn_ok)
        hbox.addWidget(self.btn_cancel)
        hbox.addStretch(1)
        vbox.addLayout(hbox)
        vbox.addStretch(1)
        self.setLayout(vbox)

class WriteDisplay(QGroupBox):
    """! A class used for implementing the panel for the write command."""

    def __init__(self, name):
        """! The WriteDisplay base class initializer.
        @param name The name of the WriteDisplay.
        @return An instance of the WriteDisplay class initialized with the specified name.
        """

        super(WriteDisplay, self).__init__(name)

        ## Label for informing about the name of the selected file
        self.lab_file = QLabel("File:")
        ## Label for informing about the size of the selected file
        self.lab_size = QLabel("Size:")
        ## Label for informing about the selected starting address
        self.lab_addr = QLabel("Starting Address:")
        ## Label for progress bar
        self.lab_pbar = QLabel("Progress:")
        ## Progress bar for informing about the writing progess
        self.pbar = QProgressBar()
        self.pbar.setMaximum(100)
        self.pbar.setFixedWidth(600)
        ## Push button for starting the write process
        self.btn_start = QPushButton("Start")
        self.btn_start.setFixedWidth(120)

        vbox = QVBoxLayout()
        vbox.addWidget(self.lab_file)
        vbox.addWidget(self.lab_size)
        vbox.addWidget(self.lab_addr)
        vbox.addWidget(self.lab_pbar)
        vbox.addWidget(self.pbar)
        vbox.addWidget(self.btn_start)
        vbox.addStretch(1)
        self.setLayout(vbox)

class RWProtectDisplay(QGroupBox):
    """! A class used for implementing the panel for the enable read/write protection command."""

    def __init__(self, name):
        """! The RWProtectDisplay base class initializer.
        @param name The name of the RWProtectDisplay.
        @return An instance of the RWProtectDisplay class initialized with the specified name.
        """

        super(RWProtectDisplay, self).__init__(name)

        ## Check box for disabling the protecton for all flash sectors
        self.check_dis_protect = QCheckBox("Disable R/W Protection for All Sectors")
        ## Chec box for selecting the SPRMOD bit
        self.check_sprmod = QCheckBox("Set SPRMOD Bit in OPTCR Flash Register (Set for R/W Protection)")
        ## Label for sector selection
        self.lab_info = QLabel("Select Sectors to Apply Protection:")
        ## Check box for selecting flash sector 0
        self.check_sector0 = QCheckBox("Sector 0")
        ## Check box for selecting flash sector 1
        self.check_sector1 = QCheckBox("Sector 1")
        ## Check box for selecting flash sector 2
        self.check_sector2 = QCheckBox("Sector 2")
        ## Check box for selecting flash sector 3
        self.check_sector3 = QCheckBox("Sector 3")
        ## Check box for selecting flash sector 4
        self.check_sector4 = QCheckBox("Sector 4")
        ## Check box for selecting flash sector 5
        self.check_sector5 = QCheckBox("Sector 5")
        ## Check box for selecting flash sector 6
        self.check_sector6 = QCheckBox("Sector 6")
        ## Check box for selecting flash sector 7
        self.check_sector7 = QCheckBox("Sector 7")
        ## Push button for strating configuration process
        self.btn_write_protect = QPushButton("Write")
        self.btn_write_protect.setFixedWidth(120)

        vbox = QVBoxLayout()
        vbox.addWidget(self.check_dis_protect)
        vbox.addWidget(self.check_sprmod)
        vbox.addWidget(self.lab_info)
        vbox.addWidget(self.check_sector0)
        vbox.addWidget(self.check_sector1)
        vbox.addWidget(self.check_sector2)
        vbox.addWidget(self.check_sector3)
        vbox.addWidget(self.check_sector4)
        vbox.addWidget(self.check_sector5)
        vbox.addWidget(self.check_sector6)
        vbox.addWidget(self.check_sector7)
        vbox.addWidget(self.btn_write_protect)
        vbox.addStretch(1)
        self.setLayout(vbox)

class MemReadDisplay(QGroupBox):
    """! A class used for implementing the panel for the memory read command output."""

    def __init__(self, name):
        """! The MemReadDisplay base class initializer.
        @param name The name of the MemReadDisplay.
        @return An instance of the MemReadDisplay class initialized with the specified name.
        """

        super(MemReadDisplay, self).__init__(name)

        ## Label for base address
        self.lab_addr = QLabel("Base Address")
        ## Field for entering a base address value
        self.text_addr = QLineEdit(self)
        self.text_addr.setFixedWidth(120)
        ## Label for length to read
        self.lab_offset = QLabel("Length (in word)")
        ## Field for entering the length to read
        self.text_offset = QLineEdit(self)
        self.text_offset.setFixedWidth(120)
        ## Push button for starting the read process
        self.btn_read = QPushButton("Read")
        self.btn_read.setFixedWidth(120)
        ## Label for displaying the memory content
        self.lab_mem_read = QLabel()
        self.lab_mem_read.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        ## Scroll area for showing the result of the memory read command
        self.scroll = QScrollArea()
        self.scroll.setWidget(self.lab_mem_read)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setWidgetResizable(True)
        self.scroll.setFixedHeight(350)

        hbox = QHBoxLayout()
        hbox.addWidget(self.lab_addr)
        hbox.addWidget(self.text_addr)
        hbox.addWidget(self.lab_offset)
        hbox.addWidget(self.text_offset)
        hbox.addWidget(self.btn_read)
        hbox.addStretch(1)
        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addWidget(self.scroll)
        vbox.addStretch(1)
        self.setLayout(vbox)

if __name__ == '__main__':
    ## For managing the Qt event loop
    app = QApplication(sys.argv)
    ## Instance the main window class
    window = MainApp()
    # Show the main window, it is hidden by default
    window.show()
    # Start the event loop
    sys.exit(app.exec_())
