import serial
LOCALE = 'UTF-8'

#communication with STM board
ser = serial.Serial('/dev/ttyS0', timeout=1, baudrate=115200)

#Bluetooth connection
RFCOMM_CHANNEL = 4
RPI_MAC_ADDR = '54:21:9D:18:43:D1' # android tablet BT ID
#UUID = 'a23d00bc-217c-123b-9c00-fc44577136ee' #UUID: Vendor specific
UUID = '94f39d29-7d6d-437d-973b-fba39e49d4ee'
ANDROID_SOCKET_BUFFER_SIZE = 512

