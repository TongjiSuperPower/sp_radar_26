import serial

port = "/dev/ttyUSB0"
baudrate = 115200

ser = serial.Serial(port, baudrate)
cnt = 0

while True:
    while ser.read() != b"\xA5":
        pass
    data = ser.read(24)
    for byte in data:
        print(f"{byte:02X}",end=" ")
    cnt+=1
    print(f"cnt:{cnt}")

ser.close