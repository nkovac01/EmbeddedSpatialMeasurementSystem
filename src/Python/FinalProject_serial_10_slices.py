import serial

#   Currently set COM3 as serial port at 115.2kbps 8N1

s = serial.Serial("COM3", 115200)

print("Opening: " + s.name)

# Commenting this portion out in case the TA marking it runs this code, it would wipe the XYZ file and its contents
#with open('tof_radar.xyz', 'w') as file:
#    file.close()                            # open the file for writing and then close, this will wipe it

with open('tof_radar.xyz', 'a') as file:    # open the file for appending
    for i in range(320):
        x = s.readline()                    # read one line of data from serial connection
        xyz = x.decode()                    # convert byte type to str
        file.write(xyz)                     # write the xyz coordinates to the file
        print(xyz)
           
print("Closing: " + s.name)
s.close();
