import cv2
from numpy import uint8
import os

#Converts to string representation like "011010010"
def convert_to_binary(value, lenght):
    binval = bin(value)[2:]
    if len(binval) > lenght:
        raise Exception("unable to convert to binary. Value is too large")
    while len(binval) < lenght:
        binval = "0"+binval
    return binval


#First 32 bits shows the embedded text size
class Encoder():
    def __init__(self, image):
        self.image = image
        self.x, self.y, self.z = image.shape
        
        #Every 2 lsb of byte used to hide message
        self.max_size_binary = self.x * self.y * self.z * 2 
        
        self.currentX = 0 
        self.currentY = 0
        self.currentZ = 0
        self.write_to_first = True
        
    def move_next(self):
        #Toogle bit preference
        if self.write_to_first:
            self.write_to_first = False
            return #Second bit of byte is not used continue
        else:
            self.write_to_first = True 
                
        if self.currentZ == self.z - 1:
            self.currentZ = 0
            
            if self.currentX == self.x -1:
                self.currentX = 0
                
                if self.currentY == self.y - 1:
                    self.currentY = 0
                    
                else:
                    self.currentY += 1
                
            else:
                 self.currentX += 1
        else:
            self.currentZ += 1
                    
                         
    def write_binary(self, bits):
        for b in bits:
            pixel = list(self.image[self.currentX,self.currentY])
            if int(b) == 1:
                pixel[self.currentZ] =  pixel[self.currentZ] | (0b00000001 if  self.write_to_first else 0b00000010)
            else:
                pixel[self.currentZ] =  pixel[self.currentZ] & (0b11111110 if  self.write_to_first else 0b11111101)
                
            self.image[self.currentX,self.currentY] = tuple(pixel)
            self.move_next()
                
        
    def encode_binary(self, data):
        size = len(data)
        if size > self.max_size_binary:
            raise Exception("Text is too big to embed in image!!!")
        self.write_binary(convert_to_binary(size, 32))
        for byte in data:
            self.write_binary(convert_to_binary(byte, 8))
        return self.image
        

#Extract secret text from images
class Decoder():
    def __init__(self, image):
        self.image = image
        self.x, self.y, self.z = image.shape
        
        self.currentX = 0 
        self.currentY = 0
        self.currentZ = 0
        self.read_first_bit = True
        
    def read_binary(self, n):
        byte = ""
        for i in range(n):
            pixel = self.image[self.currentX, self.currentY, self.currentZ]
            b = int(pixel) & (0b00000001 if self.read_first_bit else 0b00000010)
            if (b == 1 or b == 2):
                byte += "1"
            else:
                byte += "0"
            self.move_next()
        return byte
    
    def move_next(self):
        #Toogle bit preference
        if self.read_first_bit:
            self.read_first_bit = False
            return #Second bit of byte is not used continue
        else:
            self.read_first_bit = True 
                
        if self.currentZ == self.z - 1:
            self.currentZ = 0
            
            if self.currentX == self.x -1:
                self.currentX = 0
                
                if self.currentY == self.y - 1:
                    self.currentY = 0
                    
                else:
                    self.currentY += 1
                
            else:
                 self.currentX += 1
        else:
            self.currentZ += 1
            
        
    def decode_binary(self):
        size = int(self.read_binary(32),2)
        bytes_array = [0]*size
        for i in range(size):
            bytes_array[i] = int(self.read_binary(8),2)
        return bytes_array
        

import argparse
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--task")
    parser.add_argument("--image")
    parser.add_argument("--txt")
    parser.add_argument("--out")
    args = parser.parse_args()
    ENCODE = True
    if not os.path.exists("./output"):
        os.mkdir("./output")
    if args.task == "encode": 
        ENCODE = True
    elif args.task == "decode":
        ENCODE = False
    else:
        print("[ERROR] specifie task: encode/decode")
        exit(1)
    Carrier_Image = ""
    if args.image != "":
        Carrier_Image = args.image
    else:
        print("[ERROR] specifie carrier image (--image cat.png)")
        exit(1)
    Hidden_text = ""
    if ENCODE:
        Hidden_text = args.txt
        if Hidden_text == "":
            print("[ERROR] specifie txt file to encode")
            exit(1)
    
    
    if ENCODE:
        image = cv2.imread(Carrier_Image)
        with open(Hidden_text, "rb") as f:
            data = f.read()
        encoder = Encoder(image)
        image_written = encoder.encode_binary(data)
        default_name = "hidden_{}".format(Carrier_Image)
        if args.out != None:
            if args.out[-3:] == Carrier_Image[-3:]:
                default_name = args.out
        cv2.imwrite("./output/" + default_name, image_written )
    else:
        image = cv2.imread(Carrier_Image)
        decoder =  Decoder(image)
        bytes_array = decoder.decode_binary()
        default_name = "encrypted.txt"
        if args.out != None:
            if args.out[-3:] == "txt":
                default_name = args.out
        with open("./output/" + default_name, "wb") as f:
            for b in bytes_array:
                f.write(uint8(b))
                
        
        
