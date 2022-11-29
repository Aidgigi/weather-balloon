import serial
import serial.tools.list_ports
import requests

from prom_server import observe_payload, start_server

print([comport.device for comport in serial.tools.list_ports.comports()])

PORT = "COM3"

webhook_url = "https://discord.com/api/webhooks/1046204968971022346/B3S8yd6HqeaZAUVU9dkwyHTj5jLvlJSkW71mR-Q55n0wRcEQrBhNo2fY_HHVxRspXNmm"
message_buffer = []



def main_loop(port, baud):
    ser = serial.Serial(port = port, baudrate = baud)
    start_server(8009)

    while True:

        if (line := ser.readline()):
            if type(line) == bytes:
                line = line.decode('utf-8')
            
            line = line.rstrip()
            print(line)
            parsed_data = parse_line(line)
            observe_payload(parsed_data)            


def parse_line(line):
    # outline data structure
    data = {
        "time": "",
        "latitude": "",
        "longitude": "",
        "gps_altitude": "",
        "speed": "",
        "course": "",
        "satellite_count": "",
        "x_angle": "",
        "y_angle": "",
        "z_angle": "",
        "internal_temperature": "",
        "external_temperature": "",
        "pressure": "",
        "humidity": "",
        "barometric_altitude": "",
        "main_voltage": "",
        "main_current": "",
        "main_power": "",
        "tx_voltage": "",
        "tx_current": "",
        "tx_power": ""
    }

    line_list = line.split("|")
    keys_list = list(data.keys())

    for unparsed in line_list:
        telem_index = unparsed.split(":")[0]

        data[keys_list[int(telem_index) - 1]] = unparsed.split('{')[-1].split('}')[0]
    
    for i in range(len(keys_list)):
        if i == 0:
            continue
        
        if (value := data[keys_list[i]]):
            if "." in value:
                data[keys_list[i]] = float(value)
        
            else:
                data[keys_list[i]] = int(value)

        else:
            data[keys_list[i]] = 0.0
    
    return data
    


main_loop("COM3", 57600)


while True:
    line = ser.readline()
    if (line):
        if type(line) == bytes:
            line = line.decode('utf-8')

        print(line.rstrip())

        message_buffer.append(line)
    
    if len(message_buffer) >= 6:
        data = {
            "content": "\n".join(message_buffer),
            "username": "wb"
        }

        req = requests.post(webhook_url, json = data)

        message_buffer = []




    