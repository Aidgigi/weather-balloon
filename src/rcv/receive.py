from serial import Serial
import os

def read_and_write(s):
    count = 0
    while True:
        try:
            line = s.readline()

            if line:
                line = line.decode()
                print(line)
                
                with open("/Users/aidan/Documents/wb/src/rcv/log.txt", 'a') as f:
                    f.write(line)
        
        except Exception as e:
            print(e)


if __name__ == "__main__":
    os.chdir('/')
    s = Serial('/dev/cu.usbserial-0001', 9600)
    read_and_write(s)
