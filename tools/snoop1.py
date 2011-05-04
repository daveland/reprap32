import serial
ser=serial.Serial('com10',38400)

def lf1():
    global ser
    c=1
    while(1):
        c=ser.read(1)
        if (ord(c) == 0xd5):
            print 
        print hex(ord(c)) ,
        

        


    
        
    
