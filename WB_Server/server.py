import serial
import serial.tools.list_ports
import time

from prom_server import observe_payload, start_server, observe_rssi

print([comport.device for comport in serial.tools.list_ports.comports()])

PORT = "/dev/ttyUSB0"

webhook_url = "https://discord.com/api/webhooks/1046204968971022346/B3S8yd6HqeaZAUVU9dkwyHTj5jLvlJSkW71mR-Q55n0wRcEQrBhNo2fY_HHVxRspXNmm"
message_buffer = []

def main_loop(port, baud):
    ser = serial.Serial(port = port, baudrate = baud)

    loop_count = 0
    
    while True:
        if loop_count >= 5:
            rssis = fetch_rssi(ser)
            observe_rssi(rssis[0], rssis[1])
            loop_count = 0

        if (line := ser.readline()):
            try:
                if type(line) == bytes:
                    line = line.decode('utf-8')
            except Exception as e:
                print(line)
            
            if '{' in line:
                line = line.rstrip()
                print(line)
                parsed_data = parse_line(line)
                observe_payload(parsed_data)
                #send_to_kafka(parsed_data)

                loop_count += 1            


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

def fetch_rssi(ser: serial.Serial):
    start = time.time()
    ser.read_all()
    ser.write("+++".encode('utf-8'))
    time.sleep(2)

    ser.read_all()

    ser.write("ATI7\r\n".encode('utf-8'))

    ser.readline()

    nl = False
    while not nl:
        if (res := ser.readline()):
            res = res.decode('utf-8')
            local, remote = res.split("RSSI: ")[-1].split(' ')[0].split('/')

            return (int(local), int(remote))

            nl = True

    time.sleep(0.5)
    ser.write("ATO\r\n".encode('utf-8'))
    time.sleep(0.5)

    ser.readline()

start_server(8009)
main_loop(PORT, 57600)

    