# Title: Foolproof temperature checker
# Description: OpenMV run inference check if mask detected then start
#              temperature measurement using MLX90614
#              & display result on the lcd shield --> removed
#              & communicate with core2 --> added
# Connection:
#   MLX90614  OpenMV    Core2
#   VCC       VCC
#   GND       GND
#   SCL       SCL
#   SDA       SDA
#             VIN       PortC.5V
#             GND       PortC.GND
#             P0.RX     PortC.TX
#             P1.TX     PortC.RX
#
# R1 - Initial release for testing
# R2 - Flow:
#       Check MaskYes for consecutive 5 frames
#       If yes, check temp if above 36degC
#       Average those temp above 36degC
#       Display result at LCD for 3sec. --> removed
#       Add led to lit according to the status (green led not working)
# R3 - Add LED toggle
# R4 - Remove lcd part
#      Add uart for comm to core2. Design:
#       bit 0:      int status_to_core2,
#       bit 123:    int bodyTemp_to_core2
#      Change led trigger timing and condition
#      Change check_temp logic to also calculate
#       tAvg when tObj below 36degC, to show number at least
# R5 - Add null terminator to uart string

from pyb import I2C, LED, UART
import sensor, image, time, os, tf, lcd, micropython, pyb

micropython.alloc_emergency_exception_buf(100)

# LED flash ----------------------------------------------------------------------------------------
class Foo(object):
    def __init__(self, timer, led):
        self.led = led
        self.timer = timer
        timer.callback(self.cb)
    def cb(self, tim):
        self.led.toggle()
    def deinit(self):
        self.timer.deinit()

# LED selection ------------------------------------------------------------------------------------
def light(r, g, b):
    if r == True:
        red_led.on()
    else:
        red_led.off()
    if g == True:
        green_led.on()
    else:
        green_led.off()
    if b == True:
        blue_led.on()
    else:
        blue_led.off()

# Temperature function------------------------------------------------------------------------------
def check_temp():   # To take only measurement above 36degC into rolling average of ~4s

    global temp_count
    global tAvg
    global tSum
    global avg_count

    global low_tAvg
    global low_tSum
    global low_count

    done = False

    if (temp_count < max_count):
        temp_count += 1
        tObj = ir.getObjCelsius()

        if tObj > 36.0:
            avg_count += 1
            tSum = tSum + tObj
            tAvg = round(tSum/avg_count, 1)
            print("HI_cnt: %d   tObj: %s *C   tSum: %s *C   tAvg: %s *C" % (avg_count, tObj, tSum, tAvg))
        else:
            low_count +=1
            low_tSum = low_tSum + tObj
            low_tAvg = round(low_tSum/low_count, 1)
            print("LOW_cnt: %d   tObj: %s *C   tSum: %s *C   tAvg: %s *C" % (low_count, tObj, low_tSum, low_tAvg))


        if (temp_count == max_count):
            if (avg_count < 1):
                tAvg = low_tAvg
            uart_result = "2" + str(int(tAvg * 10)) + "\0"
            uart.write(uart_result)
            print("UART to Core2: %s" % uart_result)
            temp_count = 0
            avg_count = 0
            tObj = 0.0
            tSum = 0.0

            low_count = 0
            low_tObj = 0.0
            low_tSum = 0.0

            done = True
    return done

# MLX90614 setup -----------------------------------------------------------------------------------
_MLX90614_IIC_ADDR   = (0x5A)
_MLX90614_TA         = (0x06)
_MLX90614_TOBJ1      = (0x07)

class MLX90614:
  def __init__(self,i2c,addr=_MLX90614_IIC_ADDR):
    self.addr=addr
    self.i2c=i2c

  def getObjCelsius(self):
    return self.getTemp(_MLX90614_TOBJ1) + 3	#Get celsius temperature of the object

  def getEnvCelsius(self):
    return self.getTemp(_MLX90614_TA) + 1   #Get celsius temperature of the ambient

  def getTemp(self,reg):
    temp = self.getReg(reg)*0.02-273.15             #Temperature conversion
    return temp

  def getReg(self,reg):
    data = self.i2c.mem_read(3,self.addr,reg)               #Receive DATA
    result = (data[1]<<8) | data[0]
    return result

# I2C setup ----------------------------------------------------------------------------------------
i2c = I2C(2,I2C.MASTER,baudrate=100000)
ir = MLX90614(i2c)

# Set uart settings --------------------------------------------------------------------------------
uart = UART(1, 115200, timeout_char=1000)       #UART 1 RX -> P0; TX -> P1
uart.init(115200, bits=8, parity=None, stop=1)

# LED setup ----------------------------------------------------------------------------------------
red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)

# Sensor setup -------------------------------------------------------------------------------------
sensor.reset()                         # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.VGA)       # Set frame size to QVGA (320x240)
sensor.set_windowing((360, 360))       # Set 240x240 window.
sensor.skip_frames(time=2000)          # Let the camera adjust.

# tensorflow lite setup ----------------------------------------------------------------------------
net = "trained.tflite"
labels = [line.rstrip('\n') for line in open("labels.txt")]

# --------------------------------------------------------------------------------------------------
detected = ""
temp_count = 0
avg_count = 0
max_count = 12      # if 4fps then 6count is ~3sec.
detected_count = 0

tObj = 0.0
tAvg = 0.0
tSum = 0.0

low_count = 0
low_tAvg = 0.0
low_tSum = 0.0

clock = time.clock()
isStart = False

isEnd = False
isEnd_count = 0
isEnd_count_max = 16
enableFlag_fr_core2 = '0'
line = ""

while(True):
    clock.tick()

    # ---------------------------------
    if uart.any():
        buf = uart.read()

        if (buf!= '\n'):
            line += str(buf)
        else:
            break
        print(line)
        enableFlag_fr_core2 = line[2]
        print(enableFlag_fr_core2)
        line = ""

    # If enable flag received from core2
    if(enableFlag_fr_core2 == '1'):
        light(False, False, True)
        img = sensor.snapshot().replace(vflip=True, hmirror=True)

        for obj in tf.classify(net, img, min_scale=1.0, scale_mul=0.8, x_overlap=0.5, y_overlap=0.5):
            label_list = list(labels)
            score_list = list(obj.output())
            max_num = max(score_list)
            max_ind = score_list.index(max_num)

            detected = label_list[max_ind]

            # To confirm if really MaskYes detected for continuous 5fps (~1.5sec)
            if isStart == False and isEnd == False:
                if detected == "MaskYes":
                    if detected_count < 5:
                        detected_count +=1
                        print("Detected MaskYes count : %d" % detected_count)
                        img.draw_string(125, 30, detected , color = (100, 250, 250), scale = 5, mono_space = False)
                        img.draw_rectangle(116, 116, 128, 128, color=(100, 250, 100), thickness=1)
                        img.replace(vflip=True, hmirror=True)
                        light(False, False, True)

                        if detected_count == 5:
                            isStart = True
                            uart.write("1000\n")
                            print("UART to Core2: 1")
                            light(False, False, False)
                            blueFlash = Foo(pyb.Timer(4, freq=4), blue_led)

                else:
                    detected_count = 0
                    print("Detected object is %s" % detected)
                    img.draw_string(125, 30, detected , color = (100, 250, 250), scale = 5, mono_space = False)
                    img.replace(vflip=True, hmirror=True)

            # Once confirm "MaskYes", start to take measurement
            elif isStart == True and isEnd == False :
                print("Measuring...")
                img.draw_string(115, 150, "Measuring..." , color = (100, 250, 250), scale = 3, mono_space = False)
                img.draw_rectangle(116, 116, 128, 128, color=(100, 250, 100), thickness=5)
                img.replace(vflip=True, hmirror=True)

                if check_temp() == True:
                    isStart = False
                    isEnd = True

            # Once done measurement, display it in frame buffer
            elif isStart == False and isEnd == True:
                if isEnd_count < isEnd_count_max:
                    isStart = False
                    isEnd_count += 1
                    print("%d: Final temperature : %s *C" % (isEnd_count, tAvg))
                    img.draw_string(130, 150, str(tAvg) , color = (100, 250, 250), scale = 5, mono_space = False)
                    img.draw_rectangle(116, 116, 128, 128, color=(100, 250, 100), thickness=5)
                    img.replace(vflip=True, hmirror=True)
                    blueFlash.deinit()
                    light(False, False, True)
                else:
                    isEnd = False
                    isEnd_count = 0
                    tAvg = 0.0
                    light(False, False, True)

            print(clock.fps(), "fps")

    else:
        light(False, False, False)
        time.sleep_ms(1000)

