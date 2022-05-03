from machine import Pin, UART, ADC
import time

uart1 = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))
led1 = Pin(18, Pin.OUT)
led2 = Pin(19, Pin.OUT)
led3 = Pin(20, Pin.OUT)

arr = bytearray([0xA5, 0x5A, 0x05, 0x82, 0x19, 0x9A, 0x00, 0x20])
arrOn = bytearray([0xA5,0x5A,0x0C,0x82,0x1A,0x1B,0x4C,0x45,0x44,0x20,0x59,0x41,0x4E,0x44,0x49])
arrOff = bytearray([0xA5,0x5A,0x0C,0x82,0x1A,0x1B,0x4C,0x45,0x44,0x20,0x53,0x4F,0x4E,0x44,0x55])

adc = machine.ADC(2)

# returns system time when called
# this function used for making async wait operations
def millis():
    return round(time.time() * 1000)

# parse data from received packet
def parseData(pData):
    cmdType = pData[0]
    
    # check data type is 0x83
    # if yes, get value and adress from packet
    if cmdType == 0x83:
        adr = (pData[1] << 8) | pData[2]
        dLen = pData[3]
        dArr = []
        for i in range(dLen):
            tt = (pData[4 + i * 2] << 8) | pData[i * 2 + 5]
            dArr.append(tt)
        return [adr, dArr]
    else: return None
    
# try parse received buffer
# if succesfully parse received buffer returns 1 else returns -1
def parseSTONEData(pBuffer, pData):
    for i in range(len(pBuffer)):
        
        # check first and second byte of received buffer
        if((pBuffer[i] == 0xA5) and (pBuffer[i + 1] == 0x5A)):
            
            # get data len from third index from buffer
            dLen = pBuffer[i + 2]
            
            # check buffer len smaller then data len
            # if yes return -1
            if dLen + 2 > len(pBuffer): return -1
            
            # get data from buffer
            for j in range(dLen):
                pData.append(pBuffer[i + 3 + j])
                
            return 1
        
        
recBuffer = b''
lastTime = 0

# main loop
while True:

    # if data available in uart buffer read, it and append recBuffer
    while(uart1.any() > 0):
        recBuffer += uart1.read(1)
        time.sleep(0.01)
        
    # if data available in recBuffer try to parse
    if(len(recBuffer) > 0):
#         print([a for a in aa])
        
        recData = []

        # if packet suffesfully parsed, function retuns 1
        if(parseSTONEData(recBuffer, recData) == 1):

            # if received packet succesfully parsed, read data from packet
            [adr, data] = parseData(recData)
            print(hex(adr), data)

            # toggle LED's according to parse data address and data
            if adr == 0x16a1:
                print("Led1 Toggle")
                led1.toggle()
            elif adr == 0x16a2:
                print("Led2 Toggle")
                led2.toggle()
            elif adr == 0x1992:
                led3.value(data[0])
                if(data[0] == 1): uart1.write(arrOn)            # send "LED Yandı" text to screen
                elif(data[0] == 0): uart1.write(arrOff)         # send "LED Sondu" text to screen
                
        # clear recBuffer after parsing process         
        recBuffer = b''    
        
    # send gauge data in every 100 miliseconds
    if(millis() - lastTime > 100):
        adc_val = adc.read_u16()                # read adc value
        aVal = int(adc_val * 380 / 65535)       # scale adc value range to 0 - 380
        arr[7] = aVal & 0xFF                    # split Low byte first
        arr[6] = (aVal & 0xFF00) >> 8           # split high byte
        uart1.write(arr)                        # send data to lcd
        lastTime = millis()
        
    
    
    