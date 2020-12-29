"""

"""

import os
import sys
import pprint
import json
from PyQt5.QtWidgets import QApplication, QDialog, QDialogButtonBox, QFileDialog, QInputDialog, QLineEdit
from configGUI import Ui_Dialog


def reverse_dict(ref_dict):
    return_dict = {}
    
    for key in ref_dict.keys():
        return_dict[str(ref_dict[key])] = key
    
    return return_dict


boolRep_to_index = {'1':     0,
                    '0':     1,
                    'true':  0,
                    'false': 1}
index_to_boolRep = reverse_dict(boolRep_to_index)

boolStr_to_digStr = {'True':  '1',
                     'False': '0'}
digStr_to_boolStr = reverse_dict(boolStr_to_digStr)

dNum_to_index = {'22': 0,
                 '23': 1,
                 '40': 2,
                 '41': 3,
                 '2':  4,
                 '3':  5,
                 '4':  6,
                 '5':  7,
                 '6':  8,
                 '9':  9,
                 '30': 10,
                 '31': 11,
                 '32': 12,
                 '33': 13,
                 '36': 14,
                 '37': 15}
index_to_dNum = reverse_dict(dNum_to_index)

sNum_to_index = {'s0':  0,
                 's1':  1,
                 's2':  2,
                 's3':  3,
                 's4':  4,
                 's5':  5,
                 's6':  6,
                 's7':  7,
                 's8':  8,
                 's9':  9,
                 's10': 10,
                 's11': 11,
                 's12': 12,
                 's13': 13,
                 's14': 14,
                 's15': 15}
index_to_sNum = reverse_dict(sNum_to_index)

sNum_to_dNum = {'s0':  '22',
                's1':  '23',
                's2':  '40',
                's3':  '41',
                's4':  '2',
                's5':  '3',
                's6':  '4',
                's7':  '5',
                's8':  '6',
                's9':  '9',
                's10': '30',
                's11': '31',
                's12': '32',
                's13': '33',
                's14': '36',
                's15': '37'}
dNum_to_sNum = reverse_dict(sNum_to_dNum)

aNum_to_index = {'A8':  0,
                 'A9':  1,
                 'A16': 2,
                 'A17': 3}
index_to_aNum = reverse_dict(aNum_to_index)

sNum_to_aNum = {'s0': 'A8',
                's1': 'A9',
                's2': 'A16',
                's3': 'A17'}
aNum_to_sNum = reverse_dict(sNum_to_aNum)

baudNum_to_baudIndex = {'300':     0,
                        '1200':    1,
                        '2400':    2,
                        '4800':    3,
                        '9600':    4,
                        '19200':   5,
                        '38400':   6,
                        '57600':   7,
                        '115200':  8,
                        '230400':  9,
                        '250000':  10,
                        '460800':  11,
                        '500000':  12,
                        '921600':  13,
                        '1000000': 14,
                        '2000000': 15}
baudIndex_to_baudNum = reverse_dict(baudNum_to_baudIndex)

portName_to_portNum = {'USB':     '0',
                       'Serial1': '1',
                       'Serial2': '2',
                       'Serial3': '3',
                       'Serial4': '4',
                       'Serial5': '5',
                       'Serial6': '6',
                       'Serial7': '7',
                       'Serial8': '8'}
portNum_to_portName = reverse_dict(portName_to_portNum)

i2c_portName_to_portIndex = {'Wire':  0,
                             'Wire1': 1,
                             'Wire2': 2}
i2c_portIndex_to_portName = reverse_dict(i2c_portName_to_portIndex)


class AppWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.show()

        self.setup_signals()

        self.BASE_LIBRARY_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), 'src')

        if not os.path.exists("configs"):
            os.makedirs("configs")

        self.shared_tools_filename = os.path.join(self.BASE_LIBRARY_PATH, "Shared_Tools.h")

        self.currentParameters = {'shared_tools': {'serial_settings': {'debug_baud': "",
                                                                       'command_baud': "",
                                                                       'gps_baud': "",
                                                                       'lidar_baud': "",
                                                                       'telem_baud': ""},
                                                   'reporting_timeout_settings': {'command_report_rate': "",
                                                                                  'telem_report_rate': "",
                                                                                  'loss_link_timeout': ""}},
                                  'gs_tools': {'serial_settings': {'gs_command_port': "",
                                                                   'gs_debug_enable': "",
                                                                   'gs_debug_port': "",
                                                                   'gs_telem_enable': "",
                                                                   'gs_telem_port': ""},
                                               'throttle_settings': {'throttle_an_pin': "",
                                                                     'max_throttle_adc': "",
                                                                     'min_throttle_adc': "",
                                                                     'max_throttle_servo': "",
                                                                     'min_throttle_servo': "",
                                                                     'throttle_reverse': ""},
                                               'elevator_settings': {'elevator_an_pin': "",
                                                                     'max_elevator_adc': "",
                                                                     'min_elevator_adc': "",
                                                                     'max_elevator_servo': "",
                                                                     'min_elevator_servo': "",
                                                                     'elevator_reverse': ""},
                                               'aileron_settings': {'aileron_an_pin': "",
                                                                    'max_aileron_adc': "",
                                                                    'min_aileron_adc': "",
                                                                    'max_aileron_servo': "",
                                                                    'min_aileron_servo': "",
                                                                    'aileron_reverse': ""},
                                               'rudder_settings': {'rudder_an_pin': "",
                                                                   'max_rudder_adc': "",
                                                                   'min_rudder_adc': "",
                                                                   'max_rudder_servo': "",
                                                                   'min_rudder_servo': "",
                                                                   'rudder_reverse': ""}},
                                  'ifc_tools': {'serial_settings': {'ifc_command_port': "",
                                                                    'ifc_debug_enable': "",
                                                                    'ifc_debug_port': "",
                                                                    'ifc_telem_enable': "",
                                                                    'ifc_telem_port': ""},
                                                'control_surface_settings': {'throttle_servo_pin': "",
                                                                             'elevator_servo_pin': "",
                                                                             'aileron_servo_pin': "",
                                                                             'rudder_servo_pin': ""},
                                                'sensor_settings': {'gps': {'enable_gps': "",
                                                                            'gps_port': "",
                                                                            'gps_report_rate': "",
                                                                            'gps_timeout': "",
                                                                            'enable_gpgga': "",
                                                                            'enable_gpgll': "",
                                                                            'enable_gpglv': "",
                                                                            'enable_gpgsa': "",
                                                                            'enable_gprmc': "",
                                                                            'enable_gpvtg': ""},
                                                                    'lidar': {'enable_lidar': "",
                                                                              'lidar_port': "",
                                                                              'lidar_fixed_mount': ""},
                                                                    'imu': {'enable_imu': "",
                                                                            'imu_port': "",
                                                                            'imu_address': ""},
                                                                    'pitot': {'enable_pitot': "",
                                                                              'pitot_pin': ""}},
                                                'autopilot_settings': {'unsafe_roll_right': "",
                                                                       'unsafe_roll_left': "",
                                                                       'max_roll_right': "",
                                                                       'max_roll_left': "",
                                                                       'unsafe_pitch_up': "",
                                                                       'unsafe_pitch_down': "",
                                                                       'max_pitch_up': "",
                                                                       'max_pitch_down': ""}}}
        self.get_parameters()

    def setup_signals(self):
        self.ui.get_current_values.clicked.connect(self.get_parameters)
        self.ui.buttonBox.clicked.connect(self.handle_button_click)
        self.ui.load_config.clicked.connect(self.ingest_config)
        self.ui.save_config.clicked.connect(self.write_out_config)
        
        # Shared Settings
        self.ui.debug_baud.currentIndexChanged.connect(self.update_param_dict)
        self.ui.command_baud.currentIndexChanged.connect(self.update_param_dict)
        self.ui.gps_baud.currentIndexChanged.connect(self.update_param_dict)
        self.ui.lidar_baud.currentIndexChanged.connect(self.update_param_dict)
        self.ui.telem_baud.currentIndexChanged.connect(self.update_param_dict)
        self.ui.command_report_rate.valueChanged.connect(self.update_param_dict)
        self.ui.telem_report_rate.valueChanged.connect(self.update_param_dict)
        self.ui.loss_link_timeout.valueChanged.connect(self.update_param_dict)
        
        # GS Settings
        self.ui.gs_command_port.currentIndexChanged.connect(self.update_param_dict)
        self.ui.enable_gs_debugging.clicked.connect(self.update_param_dict)
        self.ui.gs_debug_port.currentIndexChanged.connect(self.update_param_dict)
        self.ui.enable_gs_telem.clicked.connect(self.update_param_dict)
        self.ui.gs_telem_port.currentIndexChanged.connect(self.update_param_dict)
        self.ui.throttle_an_pin.currentIndexChanged.connect(self.update_param_dict)
        self.ui.max_throttle_adc.valueChanged.connect(self.update_param_dict)
        self.ui.min_throttle_adc.valueChanged.connect(self.update_param_dict)
        self.ui.max_throttle_servo.valueChanged.connect(self.update_param_dict)
        self.ui.min_throttle_servo.valueChanged.connect(self.update_param_dict)
        self.ui.throttle_reverse.currentIndexChanged.connect(self.update_param_dict)
        self.ui.elevator_an_pin.currentIndexChanged.connect(self.update_param_dict)
        self.ui.max_elevator_adc.valueChanged.connect(self.update_param_dict)
        self.ui.min_elevator_adc.valueChanged.connect(self.update_param_dict)
        self.ui.max_elevator_servo.valueChanged.connect(self.update_param_dict)
        self.ui.min_elevator_servo.valueChanged.connect(self.update_param_dict)
        self.ui.elevator_reverse.currentIndexChanged.connect(self.update_param_dict)
        self.ui.aileron_an_pin.currentIndexChanged.connect(self.update_param_dict)
        self.ui.max_aileron_adc.valueChanged.connect(self.update_param_dict)
        self.ui.min_aileron_adc.valueChanged.connect(self.update_param_dict)
        self.ui.max_aileron_servo.valueChanged.connect(self.update_param_dict)
        self.ui.min_aileron_servo.valueChanged.connect(self.update_param_dict)
        self.ui.aileron_reverse.currentIndexChanged.connect(self.update_param_dict)
        self.ui.rudder_an_pin.currentIndexChanged.connect(self.update_param_dict)
        self.ui.max_rudder_adc.valueChanged.connect(self.update_param_dict)
        self.ui.min_rudder_adc.valueChanged.connect(self.update_param_dict)
        self.ui.max_rudder_servo.valueChanged.connect(self.update_param_dict)
        self.ui.min_rudder_servo.valueChanged.connect(self.update_param_dict)
        self.ui.rudder_reverse.currentIndexChanged.connect(self.update_param_dict)
        
        # IFC Settings
        self.ui.ifc_command_port.currentIndexChanged.connect(self.update_param_dict)
        self.ui.enable_ifc_debugging.clicked.connect(self.update_param_dict)
        self.ui.ifc_debug_port.currentIndexChanged.connect(self.update_param_dict)
        self.ui.enable_ifc_telem.clicked.connect(self.update_param_dict)
        self.ui.ifc_telem_port.currentIndexChanged.connect(self.update_param_dict)
        self.ui.throttle_servo_pin.currentIndexChanged.connect(self.update_param_dict)
        self.ui.elevator_servo_pin.currentIndexChanged.connect(self.update_param_dict)
        self.ui.aileron_servo_pin.currentIndexChanged.connect(self.update_param_dict)
        self.ui.rudder_servo_pin.currentIndexChanged.connect(self.update_param_dict)
        self.ui.enable_gps.clicked.connect(self.update_param_dict)
        self.ui.gps_port.currentIndexChanged.connect(self.update_param_dict)
        self.ui.gps_report_rate.valueChanged.connect(self.update_param_dict)
        self.ui.gps_timeout.valueChanged.connect(self.update_param_dict)
        self.ui.enable_gpgga.clicked.connect(self.update_param_dict)
        self.ui.enable_gpgll.clicked.connect(self.update_param_dict)
        self.ui.enable_gpglv.clicked.connect(self.update_param_dict)
        self.ui.enable_gpgsa.clicked.connect(self.update_param_dict)
        self.ui.enable_gprmc.clicked.connect(self.update_param_dict)
        self.ui.enable_gpvtg.clicked.connect(self.update_param_dict)
        self.ui.enable_lidar.clicked.connect(self.update_param_dict)
        self.ui.lidar_port.currentIndexChanged.connect(self.update_param_dict)
        self.ui.lidar_fixed_mount.currentIndexChanged.connect(self.update_param_dict)
        self.ui.enable_imu.clicked.connect(self.update_param_dict)
        self.ui.imu_port.currentIndexChanged.connect(self.update_param_dict)
        self.ui.imu_address.valueChanged.connect(self.update_param_dict)
        self.ui.enable_pitot.clicked.connect(self.update_param_dict)
        self.ui.pitot_pin.currentIndexChanged.connect(self.update_param_dict)
        self.ui.unsafe_roll_right.valueChanged.connect(self.update_param_dict)
        self.ui.unsafe_roll_left.valueChanged.connect(self.update_param_dict)
        self.ui.max_roll_right.valueChanged.connect(self.update_param_dict)
        self.ui.max_roll_left.valueChanged.connect(self.update_param_dict)
        self.ui.unsafe_pitch_up.valueChanged.connect(self.update_param_dict)
        self.ui.unsafe_pitch_down.valueChanged.connect(self.update_param_dict)
        self.ui.max_pitch_up.valueChanged.connect(self.update_param_dict)
        self.ui.max_pitch_down.valueChanged.connect(self.update_param_dict)

    def handle_button_click(self, button):
        sb = self.ui.buttonBox.standardButton(button)
        if sb == QDialogButtonBox.Apply:
            self.write_out_parameters()

        elif sb == QDialogButtonBox.Cancel:
            exit()

    def ingest_config(self):
        openfile = QFileDialog.getOpenFileName(self, directory="configs", filter="*.json")[0]

        try:
            if openfile:
                with open(openfile, "r") as _file:
                    tempParameters = json.loads(_file.read().replace("'", '"'))
                    
                copy = True
                key_error = ''
                
                for keys_1 in tempParameters.keys():
                    if keys_1 not in self.currentParameters.keys():
                        copy = False
                        key_error = keys_1
                    
                    for keys_2 in tempParameters[keys_1].keys():
                        if keys_2 not in self.currentParameters[keys_1].keys():
                            copy = False
                            key_error = keys_2
                        
                        try:
                            for keys_3 in tempParameters[keys_1][keys_2].keys():
                                if keys_3 not in self.currentParameters[keys_1][keys_2].keys():
                                    copy = False
                                    key_error = keys_3
                        except AttributeError:
                            pass
                
                if copy:
                    self.currentParameters = tempParameters
                    self.update_gui()
    
                    print("--------------------------------")
                    print("Loaded config file: {}".format(os.path.basename(openfile)))
                else:
                    print("--------------------------------")
                    print("JSON key error: '{}' in config file: {}".format(key_error, os.path.basename(openfile)))
        except:
            import traceback
            traceback.print_exc()
    
    def update_param_dict(self):
        # Shared Settings
        self.currentParameters['shared_tools']['serial_settings']['debug_baud']         = self.ui.debug_baud.currentText()
        self.currentParameters['shared_tools']['serial_settings']['command_baud']       = self.ui.command_baud.currentText()
        self.currentParameters['shared_tools']['serial_settings']['gps_baud']           = self.ui.gps_baud.currentText()
        self.currentParameters['shared_tools']['serial_settings']['lidar_baud']         = self.ui.lidar_baud.currentText()
        self.currentParameters['shared_tools']['serial_settings']['telem_baud']         = self.ui.telem_baud.currentText()
        self.currentParameters['shared_tools']['reporting_timeout_settings']['command_report_rate'] = self.ui.command_report_rate.text()
        self.currentParameters['shared_tools']['reporting_timeout_settings']['telem_report_rate']   = self.ui.telem_report_rate.text()
        self.currentParameters['shared_tools']['reporting_timeout_settings']['loss_link_timeout']   = self.ui.loss_link_timeout.text()

        #GS Settings
        self.currentParameters['gs_tools']['serial_settings']['gs_command_port']        = self.ui.gs_command_port.currentText()
        self.currentParameters['gs_tools']['serial_settings']['gs_debug_enable']        = str(self.ui.enable_gs_debugging.isChecked())
        self.currentParameters['gs_tools']['serial_settings']['gs_debug_port']          = self.ui.gs_debug_port.currentText()
        self.currentParameters['gs_tools']['serial_settings']['gs_telem_enable']        = str(self.ui.enable_gs_telem.isChecked())
        self.currentParameters['gs_tools']['serial_settings']['gs_telem_port']          = self.ui.gs_telem_port.currentText()
        self.currentParameters['gs_tools']['throttle_settings']['throttle_an_pin']      = self.ui.throttle_an_pin.currentText()
        self.currentParameters['gs_tools']['throttle_settings']['max_throttle_adc']     = self.ui.max_throttle_adc.text()
        self.currentParameters['gs_tools']['throttle_settings']['min_throttle_adc']     = self.ui.min_throttle_adc.text()
        self.currentParameters['gs_tools']['throttle_settings']['max_throttle_servo']   = self.ui.max_throttle_servo.text()
        self.currentParameters['gs_tools']['throttle_settings']['min_throttle_servo']   = self.ui.min_throttle_servo.text()
        self.currentParameters['gs_tools']['throttle_settings']['throttle_reverse']     = self.ui.throttle_reverse.currentText()
        self.currentParameters['gs_tools']['elevator_settings']['elevator_an_pin']      = self.ui.elevator_an_pin.currentText()
        self.currentParameters['gs_tools']['elevator_settings']['max_elevator_adc']     = self.ui.max_elevator_adc.text()
        self.currentParameters['gs_tools']['elevator_settings']['min_elevator_adc']     = self.ui.min_elevator_adc.text()
        self.currentParameters['gs_tools']['elevator_settings']['max_elevator_servo']   = self.ui.max_elevator_servo.text()
        self.currentParameters['gs_tools']['elevator_settings']['min_elevator_servo']   = self.ui.min_elevator_servo.text()
        self.currentParameters['gs_tools']['elevator_settings']['elevator_reverse']     = self.ui.elevator_reverse.currentText()
        self.currentParameters['gs_tools']['aileron_settings']['aileron_an_pin']        = self.ui.aileron_an_pin.currentText()
        self.currentParameters['gs_tools']['aileron_settings']['max_aileron_adc']       = self.ui.max_aileron_adc.text()
        self.currentParameters['gs_tools']['aileron_settings']['min_aileron_adc']       = self.ui.min_aileron_adc.text()
        self.currentParameters['gs_tools']['aileron_settings']['max_aileron_servo']     = self.ui.max_aileron_servo.text()
        self.currentParameters['gs_tools']['aileron_settings']['min_aileron_servo']     = self.ui.min_aileron_servo.text()
        self.currentParameters['gs_tools']['aileron_settings']['aileron_reverse']       = self.ui.aileron_reverse.currentText()
        self.currentParameters['gs_tools']['rudder_settings']['rudder_an_pin']          = self.ui.rudder_an_pin.currentText()
        self.currentParameters['gs_tools']['rudder_settings']['max_rudder_adc']         = self.ui.max_rudder_adc.text()
        self.currentParameters['gs_tools']['rudder_settings']['min_rudder_adc']         = self.ui.min_rudder_adc.text()
        self.currentParameters['gs_tools']['rudder_settings']['max_rudder_servo']       = self.ui.max_rudder_servo.text()
        self.currentParameters['gs_tools']['rudder_settings']['min_rudder_servo']       = self.ui.min_rudder_servo.text()
        self.currentParameters['gs_tools']['rudder_settings']['rudder_reverse']         = self.ui.rudder_reverse.currentText()

        #IFC Settings
        self.currentParameters['ifc_tools']['serial_settings']['ifc_command_port']      = self.ui.ifc_command_port.currentText()
        self.currentParameters['ifc_tools']['serial_settings']['ifc_debug_enable']      = str(self.ui.enable_ifc_debugging.isChecked())
        self.currentParameters['ifc_tools']['serial_settings']['ifc_debug_port']        = self.ui.ifc_debug_port.currentText()
        self.currentParameters['ifc_tools']['serial_settings']['ifc_telem_enable']      = str(self.ui.enable_ifc_telem.isChecked())
        self.currentParameters['ifc_tools']['serial_settings']['ifc_telem_port']        = self.ui.ifc_telem_port.currentText()
        self.currentParameters['ifc_tools']['control_surface_settings']['throttle_servo_pin'] = self.ui.throttle_servo_pin.currentText()
        self.currentParameters['ifc_tools']['control_surface_settings']['elevator_servo_pin'] = self.ui.elevator_servo_pin.currentText()
        self.currentParameters['ifc_tools']['control_surface_settings']['aileron_servo_pin']  = self.ui.aileron_servo_pin.currentText()
        self.currentParameters['ifc_tools']['control_surface_settings']['rudder_servo_pin']   = self.ui.rudder_servo_pin.currentText()
        self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gps']      = str(self.ui.enable_gps.isChecked())
        self.currentParameters['ifc_tools']['sensor_settings']['gps']['gps_port']        = self.ui.gps_port.currentText()
        self.currentParameters['ifc_tools']['sensor_settings']['gps']['gps_report_rate'] = self.ui.gps_report_rate.text()
        self.currentParameters['ifc_tools']['sensor_settings']['gps']['gps_timeout']     = self.ui.gps_timeout.text()
        self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gpgga']    = str(self.ui.enable_gpgga.isChecked())
        self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gpgll']    = str(self.ui.enable_gpgll.isChecked())
        self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gpglv']    = str(self.ui.enable_gpglv.isChecked())
        self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gpgsa']    = str(self.ui.enable_gpgsa.isChecked())
        self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gprmc']    = str(self.ui.enable_gprmc.isChecked())
        self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gpvtg']    = str(self.ui.enable_gpvtg.isChecked())
        self.currentParameters['ifc_tools']['sensor_settings']['lidar']['enable_lidar']  = str(self.ui.enable_lidar.isChecked())
        self.currentParameters['ifc_tools']['sensor_settings']['lidar']['lidar_port']    = self.ui.lidar_port.currentText()
        self.currentParameters['ifc_tools']['sensor_settings']['lidar']['lidar_fixed_mount'] = self.ui.lidar_fixed_mount.currentText()
        self.currentParameters['ifc_tools']['sensor_settings']['imu']['enable_imu']      = str(self.ui.enable_imu.isChecked())
        self.currentParameters['ifc_tools']['sensor_settings']['imu']['imu_port']        = self.ui.imu_port.currentText()
        self.currentParameters['ifc_tools']['sensor_settings']['imu']['imu_address']     = self.ui.imu_address.text()
        self.currentParameters['ifc_tools']['sensor_settings']['pitot']['enable_pitot']  = str(self.ui.enable_pitot.isChecked())
        self.currentParameters['ifc_tools']['sensor_settings']['pitot']['pitot_pin']     = self.ui.pitot_pin.currentText()
        self.currentParameters['ifc_tools']['autopilot_settings']['unsafe_roll_right']   = self.ui.unsafe_roll_right.text()
        self.currentParameters['ifc_tools']['autopilot_settings']['unsafe_roll_left']    = self.ui.unsafe_roll_left.text()
        self.currentParameters['ifc_tools']['autopilot_settings']['max_roll_right']      = self.ui.max_roll_right.text()
        self.currentParameters['ifc_tools']['autopilot_settings']['max_roll_left']       = self.ui.max_roll_left.text()
        self.currentParameters['ifc_tools']['autopilot_settings']['unsafe_pitch_up']     = self.ui.unsafe_pitch_up.text()
        self.currentParameters['ifc_tools']['autopilot_settings']['unsafe_pitch_down']   = self.ui.unsafe_pitch_down.text()
        self.currentParameters['ifc_tools']['autopilot_settings']['max_pitch_up']        = self.ui.max_pitch_up.text()
        self.currentParameters['ifc_tools']['autopilot_settings']['max_pitch_down']      = self.ui.max_pitch_down.text()

    def get_custom_config_name(self):
        text, okPressed = QInputDialog.getText(self, 'Config Name', 'Enter config name (leave blank to cancel):', QLineEdit.Normal, '')
        
        if okPressed and text != '':
            return text
        return False

    def write_out_config(self):
        openfile = QFileDialog.getSaveFileName(self, directory='configs', filter='*.json')[0]
        
        if openfile:
            contents = pprint.pformat(self.currentParameters).replace("'", '"')
            
            with open(openfile, 'w') as config:
                config.write(contents)
    
            print('--------------------------------')
            print('Config exported to {}'.format(openfile))

    def get_parameters(self):
        try:
            self.read_shared_tools()
            self.update_param_dict()
        except:
            import traceback
            traceback.print_exc()

    def read_shared_tools(self):
        with open(self.shared_tools_filename, 'r') as inFile:
            contents = inFile.readlines()

        for line in contents:
            if len(line.split()) >= 3:
                define       = line.split()[0]
                setting_name = line.split()[1]
                setting_val  = line.split()[2]
                
                if define == '#define':
                    if setting_name == 'USE_GS_DEBUG':
                        self.ui.enable_gs_debugging.setChecked(bool(int(setting_val)))
    
                    elif setting_name == 'USE_IFC_DEBUG':
                        self.ui.enable_ifc_debugging.setChecked(bool(int(setting_val)))
                    
                    elif setting_name == 'USE_GS_TELEM':
                        self.ui.enable_gs_telem.setChecked(bool(int(setting_val)))
                        
                    elif setting_name == 'USE_IFC_TELEM':
                        self.ui.enable_ifc_telem.setChecked(bool(int(setting_val)))
                        
                    elif setting_name == 'USE_IMU':
                        self.ui.enable_imu.setChecked(bool(int(setting_val)))
                        
                    elif setting_name == 'USE_GPS':
                        self.ui.enable_gps.setChecked(bool(int(setting_val)))
                        
                    elif setting_name == 'USE_PITOT':
                        self.ui.enable_pitot.setChecked(bool(int(setting_val)))
                        
                    elif setting_name == 'USE_LIDAR':
                        self.ui.enable_lidar.setChecked(bool(int(setting_val)))
                        
                    elif setting_name == 'IMU_PORT':
                        self.ui.imu_port.setCurrentIndex(i2c_portName_to_portIndex[setting_val])
                        
                    elif setting_name == 'IMU_ID':
                        self.ui.imu_address.setValue(int(setting_val))
                    
                    elif setting_name == 'DEBUG_PORT_BAUD':
                        self.ui.debug_baud.setCurrentIndex(baudNum_to_baudIndex[setting_val])
                        
                    elif setting_name == 'COMMAND_PORT_BAUD':
                        self.ui.command_baud.setCurrentIndex(baudNum_to_baudIndex[setting_val])
                        
                    elif setting_name == 'GPS_PORT_BAUD':
                        self.ui.gps_baud.setCurrentIndex(baudNum_to_baudIndex[setting_val])
                        
                    elif setting_name == 'LIDAR_PORT_BAUD':
                        self.ui.lidar_baud.setCurrentIndex(baudNum_to_baudIndex[setting_val])
                        
                    elif setting_name == 'TELEM_PORT_BAUD':
                        self.ui.telem_baud.setCurrentIndex(baudNum_to_baudIndex[setting_val])
                        
                    elif setting_name == 'IFC_DEBUG_PORT_NUMBER':
                        self.ui.ifc_debug_port.setCurrentIndex(int(setting_val))
                        
                    elif setting_name == 'IFC_COMMAND_PORT_NUMBER':
                        self.ui.ifc_command_port.setCurrentIndex(int(setting_val))
                        
                    elif setting_name == 'IFC_GPS_PORT_NUMBER':
                        self.ui.gps_port.setCurrentIndex(int(setting_val))
                        
                    elif setting_name == 'IFC_LIDAR_PORT_NUMBER':
                        self.ui.lidar_port.setCurrentIndex(int(setting_val))
                        
                    elif setting_name == 'IFC_TELEM_PORT_NUMBER':
                        self.ui.ifc_telem_port.setCurrentIndex(int(setting_val))
                        
                    elif setting_name == 'GS_DEBUG_PORT_NUMBER':
                        self.ui.gs_debug_port.setCurrentIndex(int(setting_val))
                        
                    elif setting_name == 'GS_COMMAND_PORT_NUMBER':
                        self.ui.gs_command_port.setCurrentIndex(int(setting_val))
                        
                    elif setting_name == 'GS_TELEM_PORT_NUMBER':
                        self.ui.gs_telem_port.setCurrentIndex(int(setting_val))
                        
                    elif setting_name == 'REPORT_COMMANDS_FREQ':
                        self.ui.command_report_rate.setValue(float(setting_val))
                        
                    elif setting_name == 'REPORT_TELEM_FREQ':
                        self.ui.telem_report_rate.setValue(float(setting_val))
                        
                    elif setting_name == 'LOSS_LINK_TIMEOUT':
                        self.ui.loss_link_timeout.setValue(int(setting_val))
                        
                    elif setting_name == 'LOSS_GPS_TIMEOUT':
                        self.ui.gps_timeout.setValue(int(setting_val))
                        
                    elif setting_name == 'ROLL_ANALOG_PIN':
                        self.ui.aileron_an_pin.setCurrentIndex(aNum_to_index[setting_val])
                        
                    elif setting_name == 'PITCH_ANALOG_PIN':
                        self.ui.elevator_an_pin.setCurrentIndex(aNum_to_index[setting_val])
                        
                    elif setting_name == 'YAW_ANALOG_PIN':
                        self.ui.rudder_an_pin.setCurrentIndex(aNum_to_index[setting_val])
                        
                    elif setting_name == 'THROTTLE_ANALOG_PIN':
                        self.ui.throttle_an_pin.setCurrentIndex(aNum_to_index[setting_val])
                        
                    elif setting_name == 'AILERON_MAX_ADC':
                        self.ui.max_aileron_adc.setValue(int(setting_val))
                        
                    elif setting_name == 'ELEVATOR_MAX_ADC':
                        self.ui.max_elevator_adc.setValue(int(setting_val))
                        
                    elif setting_name == 'RUDDER_MAX_ADC':
                        self.ui.max_rudder_adc.setValue(int(setting_val))
                        
                    elif setting_name == 'THROTTLE_MAX_ADC':
                        self.ui.max_throttle_adc.setValue(int(setting_val))
                        
                    elif setting_name == 'AILERON_MIN_ADC':
                        self.ui.min_aileron_adc.setValue(int(setting_val))
                        
                    elif setting_name == 'ELEVATOR_MIN_ADC':
                        self.ui.min_elevator_adc.setValue(int(setting_val))
                        
                    elif setting_name == 'RUDDER_MIN_ADC':
                        self.ui.min_rudder_adc.setValue(int(setting_val))
                        
                    elif setting_name == 'THROTTLE_MIN_ADC':
                        self.ui.min_throttle_adc.setValue(int(setting_val))
                        
                    elif setting_name == 'AILERON_REVERSE':
                        self.ui.aileron_reverse.setCurrentIndex(boolRep_to_index[setting_val])
                    
                    elif setting_name == 'ELEVATOR_REVERSE':
                        self.ui.elevator_reverse.setCurrentIndex(boolRep_to_index[setting_val])
                    
                    elif setting_name == 'RUDDER_REVERSE':
                        self.ui.rudder_reverse.setCurrentIndex(boolRep_to_index[setting_val])
                    
                    elif setting_name == 'THROTTLE_REVERSE':
                        self.ui.throttle_reverse.setCurrentIndex(boolRep_to_index[setting_val])
                    
                    elif setting_name == 'AILERON_MAX':
                        self.ui.max_aileron_servo.setValue(int(setting_val))
                    
                    elif setting_name == 'ELEVATOR_MAX':
                        self.ui.max_elevator_servo.setValue(int(setting_val))
                    
                    elif setting_name == 'RUDDER_MAX':
                        self.ui.max_rudder_servo.setValue(int(setting_val))
                    
                    elif setting_name == 'THROTTLE_MAX':
                        self.ui.max_throttle_servo.setValue(int(setting_val))
                    
                    elif setting_name == 'AILERON_MIN':
                        self.ui.min_aileron_servo.setValue(int(setting_val))
                    
                    elif setting_name == 'ELEVATOR_MIN':
                        self.ui.min_elevator_servo.setValue(int(setting_val))
                    
                    elif setting_name == 'RUDDER_MIN':
                        self.ui.min_rudder_servo.setValue(int(setting_val))
                    
                    elif setting_name == 'THROTTLE_MIN':
                        self.ui.min_throttle_servo.setValue(int(setting_val))
                    
                    elif setting_name == 'AILERON_SERVO_PIN':
                        self.ui.aileron_servo_pin.setCurrentIndex(dNum_to_index[setting_val])
                    
                    elif setting_name == 'ELEVATOR_SERVO_PIN':
                        self.ui.elevator_servo_pin.setCurrentIndex(dNum_to_index[setting_val])
                    
                    elif setting_name == 'RUDDER_SERVO_PIN':
                        self.ui.rudder_servo_pin.setCurrentIndex(dNum_to_index[setting_val])
                    
                    elif setting_name == 'THROTTLE_SERVO_PIN':
                        self.ui.throttle_servo_pin.setCurrentIndex(dNum_to_index[setting_val])
                    
                    elif setting_name == 'LIDAR_FIXED_MOUNT':
                        self.ui.lidar_fixed_mount.setCurrentIndex(boolRep_to_index[setting_val])
                    
                    elif setting_name == 'GPS_REFRESH':
                        self.ui.gps_report_rate.setValue(int(setting_val))
                    
                    elif setting_name == 'GPGGA_ENABLED':
                        self.ui.enable_gpgga.setChecked(bool(int(setting_val)))
                    
                    elif setting_name == 'GPGLL_ENABLED':
                        self.ui.enable_gpgll.setChecked(bool(int(setting_val)))
                    
                    elif setting_name == 'GPGLV_ENABLED':
                        self.ui.enable_gpglv.setChecked(bool(int(setting_val)))
                    
                    elif setting_name == 'GPGSA_ENABLED':
                        self.ui.enable_gpgsa.setChecked(bool(int(setting_val)))
                    
                    elif setting_name == 'GPRMC_ENABLED':
                        self.ui.enable_gprmc.setChecked(bool(int(setting_val)))
                    
                    elif setting_name == 'GPVTG_ENABLED':
                        self.ui.enable_gpvtg.setChecked(bool(int(setting_val)))
                    
                    elif setting_name == 'PITOT_PIN':
                        self.ui.pitot_pin.setCurrentIndex(aNum_to_index[setting_val])
                    
                    elif setting_name == 'UNSAFE_ROLL_R':
                        self.ui.unsafe_roll_right.setValue(int(setting_val))
                    
                    elif setting_name == 'UNSAFE_ROLL_L':
                        self.ui.unsafe_roll_left.setValue(int(setting_val))
                    
                    elif setting_name == 'MAX_ROLL_R':
                        self.ui.max_roll_right.setValue(int(setting_val))
                    
                    elif setting_name == 'MAX_ROLL_L':
                        self.ui.max_roll_left.setValue(int(setting_val))
                    
                    elif setting_name == 'UNSAFE_PITCH_UP':
                        self.ui.unsafe_pitch_up.setValue(int(setting_val))
                    
                    elif setting_name == 'UNSAFE_PITCH_DOWN':
                        self.ui.unsafe_pitch_down.setValue(int(setting_val))
                    
                    elif setting_name == 'MAX_PITCH_UP':
                        self.ui.max_pitch_up.setValue(int(setting_val))
                    
                    elif setting_name == 'MAX_PITCH_DOWN':
                        self.ui.max_pitch_down.setValue(int(setting_val))

    def update_gui(self):
        self.update_gui_shared_tools()

    def update_gui_shared_tools(self):
        self.value = self.currentParameters['shared_tools']['serial_settings']['debug_baud']
        self.ui.debug_baud.setCurrentIndex(baudNum_to_baudIndex[self.value])
        
        self.value = self.currentParameters['shared_tools']['serial_settings']['command_baud']
        self.ui.command_baud.setCurrentIndex(baudNum_to_baudIndex[self.value])
        
        self.value = self.currentParameters['shared_tools']['serial_settings']['gps_baud']
        self.ui.gps_baud.setCurrentIndex(baudNum_to_baudIndex[self.value])
        
        self.value = self.currentParameters['shared_tools']['serial_settings']['lidar_baud']
        self.ui.lidar_baud.setCurrentIndex(baudNum_to_baudIndex[self.value])
        
        self.value = self.currentParameters['shared_tools']['serial_settings']['telem_baud']
        self.ui.lidar_baud.setCurrentIndex(baudNum_to_baudIndex[self.value])
        
        self.value = self.currentParameters['shared_tools']['reporting_timeout_settings']['command_report_rate']
        self.ui.command_report_rate.setValue(float(self.value))
        
        self.value = self.currentParameters['shared_tools']['reporting_timeout_settings']['telem_report_rate']
        self.ui.telem_report_rate.setValue(float(self.value))
        
        self.value = self.currentParameters['shared_tools']['reporting_timeout_settings']['loss_link_timeout']
        self.ui.loss_link_timeout.setValue(float(self.value))
        
        self.value = self.currentParameters['gs_tools']['serial_settings']['gs_command_port']
        self.ui.gs_command_port.setCurrentIndex(int(portName_to_portNum[self.value]))
        
        self.value = self.currentParameters['gs_tools']['serial_settings']['gs_debug_enable']
        self.ui.enable_gs_debugging.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['gs_tools']['serial_settings']['gs_debug_port']
        self.ui.gs_debug_port.setCurrentIndex(int(portName_to_portNum[self.value]))
        
        self.value = self.currentParameters['gs_tools']['serial_settings']['gs_telem_enable']
        self.ui.enable_gs_telem.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['gs_tools']['serial_settings']['gs_telem_port']
        self.ui.gs_telem_port.setCurrentIndex(int(portName_to_portNum[self.value]))
        
        self.value = self.currentParameters['gs_tools']['throttle_settings']['throttle_an_pin']
        self.ui.throttle_an_pin.setCurrentIndex(aNum_to_index[sNum_to_aNum[self.value]])
        
        self.value = self.currentParameters['gs_tools']['throttle_settings']['max_throttle_adc']
        self.ui.max_throttle_adc.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['throttle_settings']['min_throttle_adc']
        self.ui.min_throttle_adc.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['throttle_settings']['max_throttle_servo']
        self.ui.max_throttle_servo.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['throttle_settings']['min_throttle_servo']
        self.ui.min_throttle_servo.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['throttle_settings']['throttle_reverse']
        self.ui.throttle_reverse.setCurrentIndex(boolRep_to_index[self.value.lower()])
        
        self.value = self.currentParameters['gs_tools']['elevator_settings']['elevator_an_pin']
        self.ui.elevator_an_pin.setCurrentIndex(aNum_to_index[sNum_to_aNum[self.value]])
        
        self.value = self.currentParameters['gs_tools']['elevator_settings']['max_elevator_adc']
        self.ui.max_elevator_adc.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['elevator_settings']['min_elevator_adc']
        self.ui.min_elevator_adc.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['elevator_settings']['max_elevator_servo']
        self.ui.max_elevator_servo.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['elevator_settings']['min_elevator_servo']
        self.ui.min_elevator_servo.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['elevator_settings']['elevator_reverse']
        self.ui.elevator_reverse.setCurrentIndex(boolRep_to_index[self.value.lower()])
        
        self.value = self.currentParameters['gs_tools']['aileron_settings']['aileron_an_pin']
        self.ui.aileron_an_pin.setCurrentIndex(aNum_to_index[sNum_to_aNum[self.value]])
        
        self.value = self.currentParameters['gs_tools']['aileron_settings']['max_aileron_adc']
        self.ui.max_aileron_adc.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['aileron_settings']['min_aileron_adc']
        self.ui.min_aileron_adc.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['aileron_settings']['max_aileron_servo']
        self.ui.max_aileron_servo.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['aileron_settings']['min_aileron_servo']
        self.ui.min_aileron_servo.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['aileron_settings']['aileron_reverse']
        self.ui.aileron_reverse.setCurrentIndex(boolRep_to_index[self.value.lower()])
        
        self.value = self.currentParameters['gs_tools']['rudder_settings']['rudder_an_pin']
        self.ui.rudder_an_pin.setCurrentIndex(aNum_to_index[sNum_to_aNum[self.value]])
        
        self.value = self.currentParameters['gs_tools']['rudder_settings']['max_rudder_adc']
        self.ui.max_rudder_adc.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['rudder_settings']['min_rudder_adc']
        self.ui.min_rudder_adc.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['rudder_settings']['max_rudder_servo']
        self.ui.max_rudder_servo.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['rudder_settings']['min_rudder_servo']
        self.ui.min_rudder_servo.setValue(int(self.value))
        
        self.value = self.currentParameters['gs_tools']['rudder_settings']['rudder_reverse']
        self.ui.rudder_reverse.setCurrentIndex(boolRep_to_index[self.value.lower()])
        
        self.value = self.currentParameters['ifc_tools']['serial_settings']['ifc_command_port']
        self.ui.ifc_command_port.setCurrentIndex(int(portName_to_portNum[self.value]))
        
        self.value = self.currentParameters['ifc_tools']['serial_settings']['ifc_debug_enable']
        self.ui.enable_ifc_debugging.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['serial_settings']['ifc_debug_port']
        self.ui.ifc_debug_port.setCurrentIndex(int(portName_to_portNum[self.value]))
        
        self.value = self.currentParameters['ifc_tools']['serial_settings']['ifc_telem_enable']
        self.ui.enable_ifc_telem.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['serial_settings']['ifc_telem_port']
        self.ui.ifc_telem_port.setCurrentIndex(int(portName_to_portNum[self.value]))
        
        self.value = self.currentParameters['ifc_tools']['control_surface_settings']['throttle_servo_pin']
        self.ui.throttle_servo_pin.setCurrentIndex(sNum_to_index[self.value])
        
        self.value = self.currentParameters['ifc_tools']['control_surface_settings']['elevator_servo_pin']
        self.ui.elevator_servo_pin.setCurrentIndex(sNum_to_index[self.value])
        
        self.value = self.currentParameters['ifc_tools']['control_surface_settings']['aileron_servo_pin']
        self.ui.aileron_servo_pin.setCurrentIndex(sNum_to_index[self.value])
        
        self.value = self.currentParameters['ifc_tools']['control_surface_settings']['rudder_servo_pin']
        self.ui.rudder_servo_pin.setCurrentIndex(sNum_to_index[self.value])
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gps']
        self.ui.enable_gps.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['gps']['gps_port']
        self.ui.gps_port.setCurrentIndex(int(portName_to_portNum[self.value]))
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['gps']['gps_report_rate']
        self.ui.gps_report_rate.setValue(int(self.value))
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['gps']['gps_timeout']
        self.ui.gps_timeout.setValue(int(self.value))
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gpgga']
        self.ui.enable_gpgga.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gpgll']
        self.ui.enable_gpgll.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gpglv']
        self.ui.enable_gpglv.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gpgsa']
        self.ui.enable_gpgsa.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gprmc']
        self.ui.enable_gprmc.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['gps']['enable_gpvtg']
        self.ui.enable_gpvtg.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['lidar']['enable_lidar']
        self.ui.enable_lidar.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['lidar']['lidar_port']
        self.ui.lidar_port.setCurrentIndex(int(portName_to_portNum[self.value]))
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['lidar']['lidar_fixed_mount']
        self.ui.lidar_fixed_mount.setCurrentIndex(boolRep_to_index[self.value.lower()])
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['imu']['enable_imu']
        self.ui.enable_imu.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['imu']['imu_port']
        self.ui.imu_port.setCurrentIndex(i2c_portName_to_portIndex[self.value])
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['imu']['imu_address']
        self.ui.imu_address.setValue(int(self.value))
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['pitot']['enable_pitot']
        self.ui.enable_pitot.setChecked(self.value.capitalize() == 'True')
        
        self.value = self.currentParameters['ifc_tools']['sensor_settings']['pitot']['pitot_pin']
        self.ui.pitot_pin.setCurrentIndex(aNum_to_index[sNum_to_aNum[self.value]])
        
        self.value = self.currentParameters['ifc_tools']['autopilot_settings']['unsafe_roll_right']
        self.ui.unsafe_roll_right.setValue(int(self.value))
        
        self.value = self.currentParameters['ifc_tools']['autopilot_settings']['unsafe_roll_left']
        self.ui.unsafe_roll_left.setValue(int(self.value))
        
        self.value = self.currentParameters['ifc_tools']['autopilot_settings']['max_roll_right']
        self.ui.max_roll_right.setValue(int(self.value))
        
        self.value = self.currentParameters['ifc_tools']['autopilot_settings']['max_roll_left']
        self.ui.max_roll_left.setValue(int(self.value))
        
        self.value = self.currentParameters['ifc_tools']['autopilot_settings']['unsafe_pitch_up']
        self.ui.unsafe_pitch_up.setValue(int(self.value))
        
        self.value = self.currentParameters['ifc_tools']['autopilot_settings']['unsafe_pitch_down']
        self.ui.unsafe_pitch_down.setValue(int(self.value))
        
        self.value = self.currentParameters['ifc_tools']['autopilot_settings']['max_pitch_up']
        self.ui.max_pitch_up.setValue(int(self.value))
        
        self.value = self.currentParameters['ifc_tools']['autopilot_settings']['max_pitch_down']
        self.ui.max_pitch_down.setValue(int(self.value))

    def write_out_parameters(self):
        try:
            self.write_out_shared_tools()

            print('--------------------------------')
            print('Library Updated')
        except:
            import traceback
            traceback.print_exc()

    def write_out_shared_tools(self):
        with open(self.shared_tools_filename, 'r') as inFile:
            contents = inFile.readlines()

        new_contents = []

        for line in contents:
            if len(line.split()) >= 3:
                define       = line.split()[0]
                setting_name = line.split()[1]
                setting_val  = line.split()[2]
                
                if define == '#define':
                    if setting_name == 'USE_GS_DEBUG':
                        new_val = boolStr_to_digStr[str(self.ui.enable_gs_debugging.isChecked())]
                        line = line.replace(setting_val, new_val)
    
                    elif setting_name == 'USE_IFC_DEBUG':
                        new_val = boolStr_to_digStr[str(self.ui.enable_ifc_debugging.isChecked())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'USE_GS_TELEM':
                        new_val = boolStr_to_digStr[str(self.ui.enable_gs_telem.isChecked())]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'USE_IFC_TELEM':
                        new_val = boolStr_to_digStr[str(self.ui.enable_ifc_telem.isChecked())]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'USE_IMU':
                        new_val = boolStr_to_digStr[str(self.ui.enable_imu.isChecked())]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'USE_GPS':
                        new_val = boolStr_to_digStr[str(self.ui.enable_gps.isChecked())]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'USE_PITOT':
                        new_val = boolStr_to_digStr[str(self.ui.enable_pitot.isChecked())]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'USE_LIDAR':
                        new_val = boolStr_to_digStr[str(self.ui.enable_lidar.isChecked())]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'IMU_PORT':
                        new_val = self.ui.imu_port.currentText()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'IMU_ID':
                        new_val = self.ui.imu_address.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'DEBUG_PORT_BAUD':
                        new_val = self.ui.debug_baud.currentText()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'COMMAND_PORT_BAUD':
                        new_val = self.ui.command_baud.currentText()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'GPS_PORT_BAUD':
                        new_val = self.ui.gps_baud.currentText()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'LIDAR_PORT_BAUD':
                        new_val = self.ui.lidar_baud.currentText()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'TELEM_PORT_BAUD':
                        new_val = self.ui.telem_baud.currentText()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'IFC_DEBUG_PORT_NUMBER':
                        new_val = portName_to_portNum[self.ui.ifc_debug_port.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'IFC_COMMAND_PORT_NUMBER':
                        new_val = portName_to_portNum[self.ui.ifc_command_port.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'IFC_GPS_PORT_NUMBER':
                        new_val = portName_to_portNum[self.ui.gps_port.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'IFC_LIDAR_PORT_NUMBER':
                        new_val = portName_to_portNum[self.ui.lidar_port.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'IFC_TELEM_PORT_NUMBER':
                        new_val = portName_to_portNum[self.ui.ifc_telem_port.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'GS_DEBUG_PORT_NUMBER':
                        new_val = portName_to_portNum[self.ui.gs_debug_port.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'GS_COMMAND_PORT_NUMBER':
                        new_val = portName_to_portNum[self.ui.gs_command_port.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'GS_TELEM_PORT_NUMBER':
                        new_val = portName_to_portNum[self.ui.gs_telem_port.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'REPORT_COMMANDS_FREQ':
                        new_val = self.ui.command_report_rate.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'REPORT_TELEM_FREQ':
                        new_val = self.ui.telem_report_rate.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'LOSS_LINK_TIMEOUT':
                        new_val = self.ui.loss_link_timeout.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'LOSS_GPS_TIMEOUT':
                        new_val = self.ui.gps_timeout.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'AILERON_SERVO_PIN':
                        new_val = sNum_to_dNum[self.ui.aileron_servo_pin.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'ELEVATOR_SERVO_PIN':
                        new_val = sNum_to_dNum[self.ui.elevator_servo_pin.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'RUDDER_SERVO_PIN':
                        new_val = sNum_to_dNum[self.ui.rudder_servo_pin.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'THROTTLE_SERVO_PIN':
                        new_val = sNum_to_dNum[self.ui.throttle_servo_pin.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'ROLL_ANALOG_PIN':
                        new_val = sNum_to_aNum[self.ui.aileron_an_pin.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'PITCH_ANALOG_PIN':
                        new_val = sNum_to_aNum[self.ui.elevator_an_pin.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'YAW_ANALOG_PIN':
                        new_val = sNum_to_aNum[self.ui.rudder_an_pin.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'THROTTLE_ANALOG_PIN':
                        new_val = sNum_to_aNum[self.ui.throttle_an_pin.currentText()]
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'AILERON_MAX_ADC':
                        new_val = self.ui.max_aileron_adc.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'ELEVATOR_MAX_ADC':
                        new_val = self.ui.max_elevator_adc.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'RUDDER_MAX_ADC':
                        new_val = self.ui.max_rudder_adc.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'THROTTLE_MAX_ADC':
                        new_val = self.ui.max_throttle_adc.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'AILERON_MIN_ADC':
                        new_val = self.ui.min_aileron_adc.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'ELEVATOR_MIN_ADC':
                        new_val = self.ui.min_elevator_adc.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'RUDDER_MIN_ADC':
                        new_val = self.ui.min_rudder_adc.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'THROTTLE_MIN_ADC':
                        new_val = self.ui.min_throttle_adc.text()
                        line = line.replace(setting_val, new_val)
                        
                    elif setting_name == 'AILERON_REVERSE':
                        new_val = boolStr_to_digStr[str(self.ui.aileron_reverse.currentText())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'ELEVATOR_REVERSE':
                        new_val = boolStr_to_digStr[str(self.ui.elevator_reverse.currentText())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'RUDDER_REVERSE':
                        new_val = boolStr_to_digStr[str(self.ui.rudder_reverse.currentText())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'THROTTLE_REVERSE':
                        new_val = boolStr_to_digStr[str(self.ui.throttle_reverse.currentText())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'AILERON_MAX':
                        new_val = self.ui.max_aileron_servo.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'ELEVATOR_MAX':
                        new_val = self.ui.max_elevator_servo.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'RUDDER_MAX':
                        new_val = self.ui.max_rudder_servo.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'THROTTLE_MAX':
                        new_val = self.ui.max_throttle_servo.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'AILERON_MIN':
                        new_val = self.ui.min_aileron_servo.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'ELEVATOR_MIN':
                        new_val = self.ui.min_elevator_servo.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'RUDDER_MIN':
                        new_val = self.ui.min_rudder_servo.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'THROTTLE_MIN':
                        new_val = self.ui.min_throttle_servo.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'LIDAR_FIXED_MOUNT':
                        new_val = boolStr_to_digStr[str(self.ui.lidar_fixed_mount.currentText())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'GPS_REFRESH':
                        new_val = self.ui.gps_report_rate.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'GPGGA_ENABLED':
                        new_val = boolStr_to_digStr[str(self.ui.enable_gpgga.isChecked())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'GPGLL_ENABLED':
                        new_val = boolStr_to_digStr[str(self.ui.enable_gpgll.isChecked())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'GPGLV_ENABLED':
                        new_val = boolStr_to_digStr[str(self.ui.enable_gpglv.isChecked())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'GPGSA_ENABLED':
                        new_val = boolStr_to_digStr[str(self.ui.enable_gpgsa.isChecked())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'GPRMC_ENABLED':
                        new_val = boolStr_to_digStr[str(self.ui.enable_gprmc.isChecked())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'GPVTG_ENABLED':
                        new_val = boolStr_to_digStr[str(self.ui.enable_gpvtg.isChecked())]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'PITOT_PIN':
                        new_val = sNum_to_aNum[self.ui.pitot_pin.currentText()]
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'UNSAFE_ROLL_R':
                        new_val = self.ui.unsafe_roll_right.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'UNSAFE_ROLL_L':
                        new_val = self.ui.unsafe_roll_left.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'MAX_ROLL_R':
                        new_val = self.ui.max_roll_right.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'MAX_ROLL_L':
                        new_val = self.ui.max_roll_left.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'UNSAFE_PITCH_UP':
                        new_val = self.ui.unsafe_pitch_up.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'UNSAFE_PITCH_DOWN':
                        new_val = self.ui.unsafe_pitch_down.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'MAX_PITCH_UP':
                        new_val = self.ui.max_pitch_up.text()
                        line = line.replace(setting_val, new_val)
                    
                    elif setting_name == 'MAX_PITCH_DOWN':
                        new_val = self.ui.max_pitch_down.text()
                        line = line.replace(setting_val, new_val)
                    
            new_contents.append(line)

        new_contents = ''.join(new_contents)

        with open(self.shared_tools_filename, 'w') as outFile:
            outFile.write(new_contents)

    def close(self):
        exit()


def main():
    app = QApplication(sys.argv)
    w = AppWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        main()
    except SystemExit:
        pass
