# Risearch Documentation
## Contributor
Agung Pambudi - agungpambudi55 *agung.pambudi5595@gmail.com*

## The Third Week of December 2020
### Setup UART3 comm BTT TFT70 V3.0 with another board
Edit file TFT\src\User\Variants\pin_TFT70_V3_0.h on line 61
```
#define SERIAL_PORT   _USART2  // default usart port for marlin
#define SERIAL_PORT_2 _USART3  // usart port for another board
```

Finally build the firmware in the folder with name U2DFWTFT70V3.0, to update the BTT TFT70 v3.0 firmware with copying the files in the U2C folder to the SD card, then press the reset button.

### Setup 3DPrint CentraLab
- Use configuration.

## The Second Week of December 2020
### G-Code summary
- G-Code in Marlin provides 4 commands to a buffer of length 96
- Read every 4 commands then save it in the queue buffer, run per command until it's finished then read the command again
- RTOS under Marlin : https://github.com/MarlinFirmware/Marlin/issues/10549
- For the technically-minded, G-code line endings are Unix Line Endings (\n), but will accept Windows Line Endings (\r\n), so you should not need to worry about converting between the two, but it is best practice to use Unix Line Endings where possible.

* Example of a communication error with resend request
`>>>` are lines sent from the host to the RepRap machine, `<<<` are lines sent from the RepRap machine to the host.
```
>>> N66555 G1 X131.338 Y133.349 E0.0091*91
<<< ok
>>> N66556 G1 X131.574 Y133.428 E0.0046*92
<<< Error:checksum mismatch, Last Line: 66555
<<< Resend: 66556
<<< ok
>>> N66556 G1 X131.574 Y133.428 E0.0046*92
<<< ok
```

### Create G-Code communication via I2C
#### Board LPC1789 for Marlin
* Add this header declaration function in queue.h file of private class GCodeQueue
```
static void get_i2c_commands();
```

* Add func get_i2c_commands() in queue.cpp file on func get_available_commands()
```
void GCodeQueue::get_available_commands() {
  get_serial_commands();
  //### mysourcecode
  get_i2c_commands();
  //### mysourcecode
  TERN_(SDSUPPORT, get_sdcard_commands());
}
```

* Add this func get_i2c_commands() in queue.cpp file after func get_serial_commands()
```
void GCodeQueue::get_i2c_commands() {
  static char serial_line_buffer[NUM_SERIAL][MAX_CMD_SIZE];
  static uint8_t serial_input_state[NUM_SERIAL] = { PS_NORMAL };

  i2c.address(0x63);
  i2c.request(32);

  while(1<Wire.available() && length < BUFSIZE) {
    int i = 0;

    const int c = Wire.read();
    if (c < 0) continue;

    const char serial_char = c;

    if (ISEOL(serial_char)) {

      // Reset our state, continue if the line was empty
      if (process_line_done(serial_input_state[i], serial_line_buffer[i], serial_count[i]))
        continue;

      char* command = serial_line_buffer[i];

      while (*command == ' ') command++;         // Skip leading spaces
      char *npos=(*command=='N')?command:nullptr;// Require the N parameter to start the line

      if (npos) {

        bool M110 = strstr_P(command, PSTR("M110")) != nullptr;

        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        const long gcode_N = strtol(npos + 1, nullptr, 10);

        if (gcode_N != last_N[i] + 1 && !M110)
          return gcode_line_error(PSTR(STR_ERR_LINE_NO), i);

        char *apos = strrchr(command, '*');
        if (apos) {
          uint8_t checksum = 0, count = uint8_t(apos - command);
          while (count) checksum ^= command[--count];
          if (strtol(apos + 1, nullptr, 10) != checksum)
            return gcode_line_error(PSTR(STR_ERR_CHECKSUM_MISMATCH), i);
        }
        else
          return gcode_line_error(PSTR(STR_ERR_NO_CHECKSUM), i);

        last_N[i] = gcode_N;
      }
      #if ENABLED(SDSUPPORT)
        // Pronterface "M29" and "M29 " has no line number
        else if (card.flag.saving && !is_M29(command))
          return gcode_line_error(PSTR(STR_ERR_NO_CHECKSUM), i);
      #endif

      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          switch (strtol(gpos + 1, nullptr, 10)) {
            case 0: case 1:
            #if ENABLED(ARC_SUPPORT)
              case 2: case 3:
            #endif
            #if ENABLED(BEZIER_CURVE_SUPPORT)
              case 5:
            #endif
              PORT_REDIRECT(i); // Reply to the serial port that sent the command
              SERIAL_ECHOLNPGM(STR_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }

      #if DISABLED(EMERGENCY_PARSER)
        // Process critical commands early
        if (strcmp_P(command, PSTR("M108")) == 0) {
          wait_for_heatup = false;
          TERN_(HAS_LCD_MENU, wait_for_user = false);
        }
        if (strcmp_P(command, PSTR("M112")) == 0) kill(M112_KILL_STR, nullptr, true);
        if (strcmp_P(command, PSTR("M410")) == 0) quickstop_stepper();
      #endif

      #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
      #endif

      // Add the command to the queue
      _enqueue(serial_line_buffer[i], true
        #if HAS_MULTI_SERIAL
          , i
        #endif
      );
    }
    else
      process_stream_char(serial_char, serial_input_state[i], serial_line_buffer[i], serial_count[i]);
  }
}
```

#### Source code on another board
```
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(0x63);             // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
}

void loop() {
  delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent(const uint8_t bytes) {
  Wire.write("M114\n"); // respond with message of 6 bytesx
}
```

### Send G-Code from another board via UART BTT TFT70
```
long interval = 5000;
long prevMillis = 0;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
}

void loop() {
  // write command to marlin
  unsigned long currentMillis = millis();
 
  if(currentMillis - prevMillis > interval) {
    prevMillis = currentMillis;
    
    Serial3.print("M114");
    Serial3.write(0x0A);
    Serial.println("Send G-Code M114");
  }
 
  // read respon from marlin
  while(Serial3.available()){
    char inChar = (char)Serial3.read();
    Serial.print(inChar);
  }
}
```

## The First Week of December 2020
**1. Create communication I2C / TWI BUS**

Enabling this will allow you to send and receive I2C data from slave devices on the bus.

* Configuration_adv.h on line 2861
```
#define EXPERIMENTAL_I2CBUS
```

1. Example #1
This macro send the string "Marlin" to the slave device with address 0x63 (99). It uses multiple M260 commands with one B<base 10> arg.
```
M260 A99  ; Target slave address
M260 B77  ; M
M260 B97  ; a
M260 B114 ; r
M260 B108 ; l
M260 B105 ; i
M260 B110 ; n
M260 S1   ; Send the current buffer
```

2. Example #2
Request 6 bytes from slave device with address 0x63 (99).
```
M261 A99 B5
```

3. Example #3
Example serial output of a M261 request, echo:i2c-reply: from:99 bytes:5 data:hello


* Set a value from 8 to 127 to act as a slave
```
#define I2C_SLAVE_ADDRESS  8
```

* Example G-Code for set Feedrate
M221 - Set Flow Percentage: "M221 S<percent>"

**2. Create a G-Code test program from the sensor board to the raspberry pi board then to the Marlin LPC1769 board**
```
import serial.tools.list_ports
import time

class extruderBoard():
    def __init__(self):
        self.serCom = None
    
    def searchPort(self):
        ports = list(serial.tools.list_ports.comports())

        for port, desc, hwid in sorted(ports):
            # print(port, desc, hwid)

            if hwid == str('USB VID:PID=2341:0042 SER=55438303439351D072E0 LOCATION=1-1.3:1.0'):
                self.serCom = serial.Serial(
                    port=port,
                    baudrate=115200,
                    parity=serial.PARITY_ODD,
                    stopbits=serial.STOPBITS_TWO,
                    bytesize=serial.SEVENBITS)

    def connect(self):
        if self.serCom.isOpen():
            print("Port [extruder board] is already open")
        else:
            self.serCom.open()
            print("Port is open")

    def disconnect(self):
        self.serCom.close()
        print("Port is close")

    def read(self):
        rxGcode = ''
        
        while self.serCom.inWaiting() > 0:
            rxGcode += self.serCom.read().decode("utf-8")

        return rxGcode

class marlinBoard():
    def __init__(self):
        self.serCom = None

    def searchPort(self):
        ports = list(serial.tools.list_ports.comports())

        for port, desc, hwid in sorted(ports):
            if hwid == str('USB VID:PID=1D50:6029 SER=1500F016AF3C90275C6DC434F50020C3 LOCATION=1-1.4:1.0'):
                self.serCom = serial.Serial(
                    port=port,
                    baudrate=250000,
                    parity=serial.PARITY_ODD,
                    stopbits=serial.STOPBITS_TWO,
                    bytesize=serial.SEVENBITS)

    def connect(self):
        if self.serCom.isOpen():
            print("Port [marlin] is already open")
        else:
            self.serCom.open()
            print("Port is open")

    def disconnect(self):
        self.serCom.close()
        print("Port is close")

    def read(self):
        rxGcode = ''
        
        while self.serCom.inWaiting() > 0:
            rxGcode += self.serCom.read().decode("utf-8")

        return rxGcode

    def write(self, txGcode):
        self.serCom.write(txGcode.encode())

if __name__ == "__main__":
    try:
        ext = extruderBoard()
        ext.searchPort()
        ext.connect()

        lpc = marlinBoard()
        lpc.searchPort()
        lpc.connect()
  
        dataGcode = ''

        while True:            
            dataGcode = ext.read()

            if dataGcode != '':
                lpc.write(dataGcode)
                print('Data Extruder =', dataGcode.encode())

    except KeyboardInterrupt:
        ext.disconnect()
```

## The Four Week of November 2020
- Mode Marlin / RepRapDiscount FULL GRAPHIC Smart Controller
Link : https://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller 
Uncomment this define, Configuration.h on line 2000
```
// LCD12864 Simulator
#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
```

- Setup LCD on line 1440 (Marlin-2.0.x\Marlin\Configuration_adv.h)
```
#define LCD_SERIAL_PORT 3
#define LCD_BAUDRATE 115200
```

- Machine name, uncomment and write name between "" on line 146 (Marlin-2.0.x\Marlin\Configuration.h)
```
#define CUSTOM_MACHINE_NAME "WIDYA 3DCP"
```

- Machine name on line 75 (Marlin-2.0.x\Marlin\src\inc\Version.h)
```
#ifndef MACHINE_NAME
  #define MACHINE_NAME "WIDYA 3DCP"
#endif
```

### Create new serial port for communication with the host
**1.  Board mega2560**
- Configuration.h (Marlin-2.0.x\Marlin\Configuration.h) on line 115
```
#define SERIAL_PORT_3 2
```

- HAL.h (Marlin-2.0.x\Marlin\src\HAL\AVR\HAL.h) on line 106
```
#ifdef SERIAL_PORT_3
  #if !WITHIN(SERIAL_PORT_3, -1, 3)
    #error "SERIAL_PORT_3 must be from -1 to 3. Please update your configuration."
  #endif
  #define MYSERIAL2 customizedSerial3
#endif
```

- MarlinSerial.cpp (Marlin-2.0.x\Marlin\src\HAL\AVR\MarlinSerial.cpp) on line 749
```
#ifdef SERIAL_PORT_3
  // Hookup ISR handlers
  ISR(SERIAL_REGNAME(USART, SERIAL_PORT_3, _RX_vect)) {
    MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>>::store_rxd_char();
  }

  ISR(SERIAL_REGNAME(USART, SERIAL_PORT_3, _UDRE_vect)) {
    MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>>::_tx_udr_empty_irq();
  }

  // Preinstantiate
  template class MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>>;

  // Instantiate
  MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>> customizedSerial3;
#endif
```

- MarlinSerial.h (Marlin-2.0.x\Marlin\src\HAL\AVR\MarlinSerial.h) on line 202
```
#ifdef SERIAL_PORT_3
  extern MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>> customizedSerial3;
#endif
```

- MarlinCore.cpp (Marlin-2.0.x\Marlin\src\MarlinCore.cpp) on setup function
```
MYSERIAL2.begin(57600);
```

- Experimental Result
![Gambar][gambar-3-url]

**2. Board LPC1769**
- HAL.h (Marlin-2.0.x\Marlin\src\HAL\LPC1768\HAL.h) on line 89
```
#ifdef SERIAL_PORT_3
  #if SERIAL_PORT_3 == -1
    #define MYSERIAL2 UsbSerial
  #elif WITHIN(SERIAL_PORT_3, 0, 3)
    #define MYSERIAL2 MSERIAL(SERIAL_PORT_3)
  #else
    #error "SERIAL_PORT_3 must be from -1 to 3. Please update your configuration."
  #endif
#endif
```

## The Third Week of November 2020
**1. Secondary Serial**
- Edit file Configuration.h on line 112 (Marlin-2.0.X/Marlin/Configuration.h)
```
#define SERIAL_PORT_2 <serial-port>
```

- Add this to setup function in MarlinCore.cpp file (baudrate <= 57600)
```
MYSERIAL1.begin(<baudrate>);
```

- Test the print serial to loop function in MarlinCore.cpp file
```
MYSERIAL1.println("Bismillahirrahmanirrahim");

if (MYSERIAL1.available() > 0) {
  char inChar = (char)MYSERIAL1.read();
  MYSERIAL1.println(inChar);
}
```

**2. Create Communication Read and Write to Other Boards**
- Source code
```
unsigned char dataRX[4], dataRX_sum = 4;
int headerFind = 0, indexData = 0;

while(MYSERIAL1.available()){
  char inChar = (char)MYSERIAL1.read();

  if(headerFind == 0 && inChar == 'F'){ 
    headerFind = 1;
    MYSERIAL1.print(inChar);
    MYSERIAL1.println(" < first header"); 
  }else if(headerFind == 1 && inChar == 'F'){
    headerFind = 2; 
    MYSERIAL1.print(inChar);
    MYSERIAL1.println(" < second header");
  }else if(headerFind == 2){
    dataRX[indexData] = (int)inChar;
    indexData++;
    if(indexData >= dataRX_sum){ headerFind = indexData = 0; }
  }else { headerFind = indexData = 0; }
}
```
- Experimental Result
![Gambar][gambar-1-url]

**3. Create communication read and write to other boards**
- Source code for Marlin
```
unsigned int dataSensor1, dataSensor2, dataSensor3, dataSensor4, dataSensor5;
unsigned char dataRX[10], dataRXsum = 10;
int headerFind = 0, indexData = 0;

void processDataSensor(){
  dataRX[0] = dataRX[0] & 0x00ff;
  dataRX[1] = dataRX[1] << 8;
  dataRX[2] = dataRX[2] & 0x00ff;
  dataRX[3] = dataRX[3] << 8;
  dataRX[4] = dataRX[4] & 0x00ff;
  dataRX[5] = dataRX[5] << 8;
  dataRX[6] = dataRX[6] & 0x00ff;
  dataRX[7] = dataRX[7] << 8;
  dataRX[8] = dataRX[8] & 0x00ff;
  dataRX[9] = dataRX[9] << 8;

  dataSensor1 = dataRX[0] | dataRX[1];
  dataSensor2 = dataRX[2] | dataRX[3];
  dataSensor3 = dataRX[4] | dataRX[5];
  dataSensor4 = dataRX[6] | dataRX[7];
  dataSensor5 = dataRX[8] | dataRX[9];

  SERIAL_ECHO("dataSensor1 ");
  SERIAL_ECHOLN(dataSensor1);

  SERIAL_ECHO("dataSensor2 ");
  SERIAL_ECHOLN(dataSensor2);

  SERIAL_ECHO("dataSensor3 ");
  SERIAL_ECHOLN(dataSensor3);

  SERIAL_ECHO("dataSensor4 ");
  SERIAL_ECHOLN(dataSensor4);

  SERIAL_ECHO("dataSensor5 ");
  SERIAL_ECHOLN(dataSensor5);
}

void receiveDataSensor() {
  while(MYSERIAL1.available()){
    char inChar = (char)MYSERIAL1.read();
    SERIAL_ECHOLN(inChar);
    MYSERIAL1.println(inChar);
  
    if(headerFind == 0 && inChar == 'F'){ headerFind = 1; }
    else if(headerFind == 1 && inChar == 'F'){ headerFind = 2; }
    else if(headerFind == 2){      
      dataRX[indexData] = (int)inChar;
      indexData++;
      if(indexData >= dataRXsum){ headerFind = indexData = 0; }
    }else { headerFind = indexData = 0; }
  }
}
```

- Source code format transfer data for sensor extruder
```
unsigned char dataTX[10];
unsigned int dataTXSum = 10;

void transmitDataSensor(int dataSensor1, int dataSensor2, int dataSensor3, int dataSensor4, int dataSensor5) {
  dataTX[0] = dataSensor1 & 0x00ff;
  dataTX[1] = dataSensor1 >> 8;
  dataTX[2] = dataSensor2 & 0x00ff;
  dataTX[3] = dataSensor2 >> 8;
  dataTX[4] = dataSensor3 & 0x00ff;
  dataTX[5] = dataSensor3 >> 8;
  dataTX[6] = dataSensor4 & 0x00ff;
  dataTX[7] = dataSensor4 >> 8;
  dataTX[8] = dataSensor5 & 0x00ff;
  dataTX[9] = dataSensor5 >> 8;

  Serial.print("FF");
  Serial.write((byte*)dataTX, dataTXSum);
}

void setup() {
  Serial.begin(57600);
}

void loop() {
  transmitDataSensor(51,61,95,96,15);
  delay(100);
}
```
- Experimental Result
![Gambar][gambar-2-url]

## The Second Week of November 2020
**1. Board**
- BTT SKR V1.4 Turbo
![Gambar][gambar-4-url]
![Gambar][gambar-5-url]

- BTT SKR V1.3
![Gambar][gambar-6-url]

**2. Building Marlin**
To build Marlin 2.0 you'll need [Arduino IDE 1.8.8 or newer](https://www.arduino.cc/en/main/software) or [PlatformIO](http://docs.platformio.org/en/latest/ide.html#platformio-ide). Detailed build and install instructions are posted at:

- [Installing Marlin (Arduino)](http://marlinfw.org/docs/basics/install_arduino.html)
- [Installing Marlin (VSCode)](http://marlinfw.org/docs/basics/install_platformio_vscode.html).

**3. Supported Platforms**

Platform|MCU|Example Boards
--------|---|-------
[Arduino AVR](https://www.arduino.cc/)|ATmega|RAMPS, Melzi, RAMBo
[Teensy++ 2.0](http://www.microchip.com/wwwproducts/en/AT90USB1286)|AT90USB1286|Printrboard
[Arduino Due](https://www.arduino.cc/en/Guide/ArduinoDue)|SAM3X8E|RAMPS-FD, RADDS, RAMPS4DUE
[LPC1768](http://www.nxp.com/products/microcontrollers-and-processors/arm-based-processors-and-mcus/lpc-cortex-m-mcus/lpc1700-cortex-m3/512kb-flash-64kb-sram-ethernet-usb-lqfp100-package:LPC1768FBD100)|ARM® Cortex-M3|MKS SBASE, Re-ARM, Selena Compact
[LPC1769](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc1700-cortex-m3/512kb-flash-64kb-sram-ethernet-usb-lqfp100-package:LPC1769FBD100)|ARM® Cortex-M3|Smoothieboard, Azteeg X5 mini, TH3D EZBoard
[STM32F103](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html)|ARM® Cortex-M3|Malyan M200, GTM32 Pro, MKS Robin, BTT SKR Mini
[STM32F401](https://www.st.com/en/microcontrollers-microprocessors/stm32f401.html)|ARM® Cortex-M4|ARMED, Rumba32, SKR Pro, Lerdge, FYSETC S6
[STM32F7x6](https://www.st.com/en/microcontrollers-microprocessors/stm32f7x6.html)|ARM® Cortex-M7|The Borg, RemRam V1
[SAMD51P20A](https://www.adafruit.com/product/4064)|ARM® Cortex-M4|Adafruit Grand Central M4
[Teensy 3.5](https://www.pjrc.com/store/teensy35.html)|ARM® Cortex-M4|
[Teensy 3.6](https://www.pjrc.com/store/teensy36.html)|ARM® Cortex-M4|
[Teensy 4.0](https://www.pjrc.com/store/teensy40.html)|ARM® Cortex-M7|
[Teensy 4.1](https://www.pjrc.com/store/teensy41.html)|ARM® Cortex-M7|

**2. Directory File**
```
Marlin-2.0.X
|--Marlin
|  |--lib
|  |  |--readme.txt
|  |--src
|  |  |--core
|  |  |  |--boards.h (name of boards)
|  |  |  |--...
|  |  |--gcode
|  |  |--inc
|  |  |--libs
|  |  |--pins
|  |  |--feature
|  |  |--HAL
|  |  |--lcd
|  |  |--module
|  |  |--sd
|  |  |--MarlinCore.cpp (source main / core program file)
|  |  |--MarlinCore.h (header main / core program file)
|  |--Configuration_adv.cpp (advanced settings file)
|  |--Configuration.h (general settings file)
|  |--...
|--platformio.ini (platformio configuration file)
```

**3. Build Setup**
 - Edit file Configuration.h on line 129 (Marlin-2.0.X/Marlin/Configuration.h) and choose the name from boards.h (Marlin-2.0.X/Marlin/src/core/boards.h) that matches your setup
```
#ifndef MOTHERBOARD
  #define MOTHERBOARD <name-board>
#endif

; EXAMPLE 
; LPC1769
; #define MOTHERBOARD BOARD_BTT_SKR_V1_4_TURBO
; mega2560
; #define MOTHERBOARD BOARD_RAMPS_13_EFB
```

* Edit file platformio.ini on line 21 (Marlin-2.0.X/platformio.ini)
```
default_envs = <name-environment>

; EXAMPLE
; default_envs = LPC1769
; default_envs = mega2560
```

**4. Test Print**
MarlinCore.cpp
```
void loop() {
  idle();
  SERIAL_ECHOLN("Bismillahirrahmanirrahim");
  delay(1000);
}
```

**5. BBT TFT70**
Setting connection baudrate default 250000

**6. G-Codes in Marlin**
* "G" Codes
```
G0   -> G1
G1   - Coordinated Movement X Y Z E
G2   - CW ARC
G3   - CCW ARC
G4   - Dwell S<seconds> or P<milliseconds>
G5   - Cubic B-spline with XYZE destination and IJPQ offsets
G10  - Retract filament according to settings of M207 (Requires FWRETRACT)
G11  - Retract recover filament according to settings of M208 (Requires FWRETRACT)
G12  - Clean tool (Requires NOZZLE_CLEAN_FEATURE)
G17  - Select Plane XY (Requires CNC_WORKSPACE_PLANES)
G18  - Select Plane ZX (Requires CNC_WORKSPACE_PLANES)
G19  - Select Plane YZ (Requires CNC_WORKSPACE_PLANES)
G20  - Set input units to inches (Requires INCH_MODE_SUPPORT)
G21  - Set input units to millimeters (Requires INCH_MODE_SUPPORT)
G26  - Mesh Validation Pattern (Requires G26_MESH_VALIDATION)
G27  - Park Nozzle (Requires NOZZLE_PARK_FEATURE)
G28  - Home one or more axes
G29  - Start or continue the bed leveling probe procedure (Requires bed leveling)
G30  - Single Z probe, probes bed at X Y location (defaults to current XY location)
G31  - Dock sled (Z_PROBE_SLED only)
G32  - Undock sled (Z_PROBE_SLED only)
G33  - Delta Auto-Calibration (Requires DELTA_AUTO_CALIBRATION)
G34  - Z Stepper automatic alignment using probe: I<iterations> T<accuracy> A<amplification> (Requires Z_STEPPER_AUTO_ALIGN)
G35  - Read bed corners to help adjust bed screws: T<screw_thread> (Requires ASSISTED_TRAMMING)
G38  - Probe in any direction using the Z_MIN_PROBE (Requires G38_PROBE_TARGET)
G42  - Coordinated move to a mesh point (Requires MESH_BED_LEVELING, AUTO_BED_LEVELING_BLINEAR, or AUTO_BED_LEVELING_UBL)
G60  - Save current position. (Requires SAVED_POSITIONS)
G61  - Apply/restore saved coordinates. (Requires SAVED_POSITIONS)
G76  - Calibrate first layer temperature offsets. (Requires PROBE_TEMP_COMPENSATION)
G80  - Cancel current motion mode (Requires GCODE_MOTION_MODES)
G90  - Use Absolute Coordinates
G91  - Use Relative Coordinates
G92  - Set current position to coordinates given
```
* "M" Codes
```
M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
M1   -> M0
M3   - Turn ON Laser | Spindle (clockwise), set Power | Speed. (Requires SPINDLE_FEATURE or LASER_FEATURE)
M4   - Turn ON Laser | Spindle (counter-clockwise), set Power | Speed. (Requires SPINDLE_FEATURE or LASER_FEATURE)
M5   - Turn OFF Laser | Spindle. (Requires SPINDLE_FEATURE or LASER_FEATURE)
M7   - Turn mist coolant ON. (Requires COOLANT_CONTROL)
M8   - Turn flood coolant ON. (Requires COOLANT_CONTROL)
M9   - Turn coolant OFF. (Requires COOLANT_CONTROL)
M12  - Set up closed loop control system. (Requires EXTERNAL_CLOSED_LOOP_CONTROLLER)
M16  - Expected printer check. (Requires EXPECTED_PRINTER_CHECK)
M17  - Enable/Power all stepper motors
M18  - Disable all stepper motors; same as M84
M20  - List SD card. (Requires SDSUPPORT)
M21  - Init SD card. (Requires SDSUPPORT)
M22  - Release SD card. (Requires SDSUPPORT)
M23  - Select SD file: "M23 /path/file.gco". (Requires SDSUPPORT)
M24  - Start/resume SD print. (Requires SDSUPPORT)
M25  - Pause SD print. (Requires SDSUPPORT)
M26  - Set SD position in bytes: "M26 S12345". (Requires SDSUPPORT)
M27  - Report SD print status. (Requires SDSUPPORT)
       OR, with 'S<seconds>' set the SD status auto-report interval. (Requires AUTO_REPORT_SD_STATUS)
       OR, with 'C' get the current filename.
M28  - Start SD write: "M28 /path/file.gco". (Requires SDSUPPORT)
M29  - Stop SD write. (Requires SDSUPPORT)
M30  - Delete file from SD: "M30 /path/file.gco"
M31  - Report time since last M109 or SD card start to serial.
M32  - Select file and start SD print: "M32 [S<bytepos>] !/path/file.gco#". (Requires SDSUPPORT)
       Use P to run other files as sub-programs: "M32 P !filename#"
       The '#' is necessary when calling from within sd files, as it stops buffer prereading
M33  - Get the longname version of a path. (Requires LONG_FILENAME_HOST_SUPPORT)
M34  - Set SD Card sorting options. (Requires SDCARD_SORT_ALPHA)
M42  - Change pin status via gcode: M42 P<pin> S<value>. LED pin assumed if P is omitted. (Requires DIRECT_PIN_CONTROL)
M43  - Display pin status, watch pins for changes, watch endstops & toggle LED, Z servo probe test, toggle pins
M48  - Measure Z Probe repeatability: M48 P<points> X<pos> Y<pos> V<level> E<engage> L<legs> S<chizoid>. 
       (Requires Z_MIN_PROBE_REPEATABILITY_TEST)
M73  - Set the progress percentage. (Requires LCD_SET_PROGRESS_MANUALLY)
M75  - Start the print job timer.
M76  - Pause the print job timer.
M77  - Stop the print job timer.
M78  - Show statistical information about the print jobs. (Requires PRINTCOUNTER)
M80  - Turn on Power Supply. (Requires PSU_CONTROL)
M81  - Turn off Power Supply. (Requires PSU_CONTROL)
M82  - Set E codes absolute (default).
M83  - Set E codes relative while in Absolute (G90) mode.
M84  - Disable steppers until next move, or use S<seconds> to specify an idle
       duration after which steppers should turn off. S0 disables the timeout.
M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
M92  - Set planner.settings.axis_steps_per_mm for one or more axes.
M100 - Watch Free Memory (for debugging) (Requires M100_FREE_MEMORY_WATCHER)
M104 - Set extruder target temp.
M105 - Report current temperatures.
M106 - Set print fan speed.
M107 - Print fan off.
M108 - Break out of heating loops (M109, M190, M303). With no controller, breaks out of M0/M1. (Requires EMERGENCY_PARSER)
M109 - S<temp> Wait for extruder current temp to reach target temp. ** Wait only when heating! **
       R<temp> Wait for extruder current temp to reach target temp. ** Wait for heating or cooling. **
       If AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
M110 - Set the current line number. (Used by host printing)
M111 - Set debug flags: "M111 S<flagbits>". See flag bits defined in enum.h.
M112 - Full Shutdown.
M113 - Get or set the timeout interval for Host Keepalive "busy" messages. (Requires HOST_KEEPALIVE_FEATURE)
M114 - Report current position.
M115 - Report capabilities. (Extended capabilities requires EXTENDED_CAPABILITIES_REPORT)
M117 - Display a message on the controller screen. (Requires an LCD)
M118 - Display a message in the host console.
M119 - Report endstops status.
M120 - Enable endstops detection.
M121 - Disable endstops detection.
M122 - Debug stepper (Requires at least one _DRIVER_TYPE defined as TMC2130/2160/5130/5160/2208/2209/2660 or L6470)
M125 - Save current position and move to filament change position. (Requires PARK_HEAD_ON_PAUSE)
M126 - Solenoid Air Valve Open. (Requires BARICUDA)
M127 - Solenoid Air Valve Closed. (Requires BARICUDA)
M128 - EtoP Open. (Requires BARICUDA)
M129 - EtoP Closed. (Requires BARICUDA)
M140 - Set bed target temp. S<temp>
M141 - Set heated chamber target temp. S<temp> (Requires a chamber heater)
M145 - Set heatup values for materials on the LCD. H<hotend> B<bed> F<fan speed> for S<material> (0=PLA, 1=ABS)
M149 - Set temperature units. (Requires TEMPERATURE_UNITS_SUPPORT)
M150 - Set Status LED Color as R<red> U<green> B<blue> W<white> P<bright>. Values 0-255. 
       (Requires BLINKM, RGB_LED, RGBW_LED, NEOPIXEL_LED, PCA9533, or PCA9632).
M155 - Auto-report temperatures with interval of S<seconds>. (Requires AUTO_REPORT_TEMPERATURES)
M163 - Set a single proportion for a mixing extruder. (Requires MIXING_EXTRUDER)
M164 - Commit the mix and save to a virtual tool (current, or as specified by 'S'). (Requires MIXING_EXTRUDER)
M165 - Set the mix for the mixing extruder (and current virtual tool) with parameters ABCDHI. (Requires MIXING_EXTRUDER and DIRECT_MIXING_IN_G1)
M166 - Set the Gradient Mix for the mixing extruder. (Requires GRADIENT_MIX)
M190 - S<temp> Wait for bed current temp to reach target temp. ** Wait only when heating! **
       R<temp> Wait for bed current temp to reach target temp. ** Wait for heating or cooling. **
M200 - Set filament diameter, D<diameter>, setting E axis units to cubic. (Use S0 to revert to linear units.)
M201 - Set max acceleration in units/s^2 for print moves: "M201 X<accel> Y<accel> Z<accel> E<accel>"
M202 - Set max acceleration in units/s^2 for travel moves: "M202 X<accel> Y<accel> Z<accel> E<accel>" ** UNUSED IN MARLIN! **
M203 - Set maximum feedrate: "M203 X<fr> Y<fr> Z<fr> E<fr>" in units/sec.
M204 - Set default acceleration in units/sec^2: P<printing> R<extruder_only> T<travel>
M205 - Set advanced settings. Current units apply:
       <print> T<travel> minimum speeds
       B<minimum segment time>
       X<max X jerk>, Y<max Y jerk>, Z<max Z jerk>, E<max E jerk>
M206 - Set additional homing offset. (Disabled by NO_WORKSPACE_OFFSETS or DELTA)
M207 - Set Retract Length: S<length>, Feedrate: F<units/min>, and Z lift: Z<distance>. (Requires FWRETRACT)
M208 - Set Recover (unretract) Additional (!) Length: S<length> and Feedrate: F<units/min>. (Requires FWRETRACT)
M209 - Turn Automatic Retract Detection on/off: S<0|1> (For slicers that don't support G10/11). (Requires FWRETRACT_AUTORETRACT)
       Every normal extrude-only move will be classified as retract depending on the direction.
M211 - Enable, Disable, and/or Report software endstops: S<0|1> (Requires MIN_SOFTWARE_ENDSTOPS or MAX_SOFTWARE_ENDSTOPS)
M217 - Set filament swap parameters: "M217 S<length> P<feedrate> R<feedrate>". (Requires SINGLENOZZLE)
M218 - Set/get a tool offset: "M218 T<index> X<offset> Y<offset>". (Requires 2 or more extruders)
M220 - Set Feedrate Percentage: "M220 S<percent>" (i.e., "FR" on the LCD)
       Use "M220 B" to back up the Feedrate Percentage and "M220 R" to restore it. (Requires PRUSA_MMU2)
M221 - Set Flow Percentage: "M221 S<percent>"
M226 - Wait until a pin is in a given state: "M226 P<pin> S<state>" (Requires DIRECT_PIN_CONTROL)
M240 - Trigger a camera to take a photograph. (Requires PHOTO_GCODE)
M250 - Set LCD contrast: "M250 C<contrast>" (0-63). (Requires LCD support)
M260 - i2c Send Data (Requires EXPERIMENTAL_I2CBUS)
M261 - i2c Request Data (Requires EXPERIMENTAL_I2CBUS)
M280 - Set servo position absolute: "M280 P<index> S<angle|µs>". (Requires servos)
M281 - Set servo min|max position: "M281 P<index> L<min> U<max>". (Requires EDITABLE_SERVO_ANGLES)
M290 - Babystepping (Requires BABYSTEPPING)
M300 - Play beep sound S<frequency Hz> P<duration ms>
M301 - Set PID parameters P I and D. (Requires PIDTEMP)
M302 - Allow cold extrudes, or set the minimum extrude S<temperature>. (Requires PREVENT_COLD_EXTRUSION)
M303 - PID relay autotune S<temperature> sets the target temperature. Default 150C. (Requires PIDTEMP)
M304 - Set bed PID parameters P I and D. (Requires PIDTEMPBED)
M305 - Set user thermistor parameters R T and P. (Requires TEMP_SENSOR_x 1000)
M350 - Set microstepping mode. (Requires digital microstepping pins.)
M351 - Toggle MS1 MS2 pins directly. (Requires digital microstepping pins.)
M355 - Set Case Light on/off and set brightness. (Requires CASE_LIGHT_PIN)
M380 - Activate solenoid on active extruder. (Requires EXT_SOLENOID)
M381 - Disable all solenoids. (Requires EXT_SOLENOID)
M400 - Finish all moves.
M401 - Deploy and activate Z probe. (Requires a probe)
M402 - Deactivate and stow Z probe. (Requires a probe)
M403 - Set filament type for PRUSA MMU2
M404 - Display or set the Nominal Filament Width: "W<diameter>". (Requires FILAMENT_WIDTH_SENSOR)
M405 - Enable Filament Sensor flow control. "M405 D<delay_cm>". (Requires FILAMENT_WIDTH_SENSOR)
M406 - Disable Filament Sensor flow control. (Requires FILAMENT_WIDTH_SENSOR)
M407 - Display measured filament diameter in millimeters. (Requires FILAMENT_WIDTH_SENSOR)
M410 - Quickstop. Abort all planned moves.
M412 - Enable / Disable Filament Runout Detection. (Requires FILAMENT_RUNOUT_SENSOR)
M413 - Enable / Disable Power-Loss Recovery. (Requires POWER_LOSS_RECOVERY)
M420 - Enable/Disable Leveling (with current values) S1=enable S0=disable (Requires MESH_BED_LEVELING or ABL)
M421 - Set a single Z coordinate in the Mesh Leveling grid. X<units> Y<units> Z<units> 
       (Requires MESH_BED_LEVELING, AUTO_BED_LEVELING_BILINEAR, or AUTO_BED_LEVELING_UBL)
M422 - Set Z Stepper automatic alignment position using probe. X<units> Y<units> A<axis> (Requires Z_STEPPER_AUTO_ALIGN)
M425 - Enable/Disable and tune backlash correction. (Requires BACKLASH_COMPENSATION and BACKLASH_GCODE)
M428 - Set the home_offset based on the current_position. Nearest edge applies. (Disabled by NO_WORKSPACE_OFFSETS or DELTA)
M430 - Read the system current, voltage, and power (Requires POWER_MONITOR_CURRENT, POWER_MONITOR_VOLTAGE, or POWER_MONITOR_FIXED_VOLTAGE)
M486 - Identify and cancel objects. (Requires CANCEL_OBJECTS)
M500 - Store parameters in EEPROM. (Requires EEPROM_SETTINGS)
M501 - Restore parameters from EEPROM. (Requires EEPROM_SETTINGS)
M502 - Revert to the default "factory settings". ** Does not write them to EEPROM! **
M503 - Print the current settings (in memory): "M503 S<verbose>". S0 specifies compact output.
M504 - Validate EEPROM contents. (Requires EEPROM_SETTINGS)
M510 - Lock Printer
M511 - Unlock Printer
M512 - Set/Change/Remove Password
M524 - Abort the current SD print job started with M24. (Requires SDSUPPORT)
M540 - Enable/disable SD card abort on endstop hit: "M540 S<state>". (Requires SD_ABORT_ON_ENDSTOP_HIT)
M569 - Enable stealthChop on an axis. (Requires at least one _DRIVER_TYPE to be TMC2130/2160/2208/2209/5130/5160)
M600 - Pause for filament change: "M600 X<pos> Y<pos> Z<raise> E<first_retract> L<later_retract>". (Requires ADVANCED_PAUSE_FEATURE)
M603 - Configure filament change: "M603 T<tool> U<unload_length> L<load_length>". (Requires ADVANCED_PAUSE_FEATURE)
M605 - Set Dual X-Carriage movement mode: "M605 S<mode> [X<x_offset>] [R<temp_offset>]". (Requires DUAL_X_CARRIAGE)
M665 - Set delta configurations: "M665 H<delta height> L<diagonal rod> R<delta radius> S<segments/s> B<calibration radius> 
       X<Alpha angle trim> Y<Beta angle trim> Z<Gamma angle trim> (Requires DELTA)
M666 - Set/get offsets for delta (Requires DELTA) or dual endstops. (Requires [XYZ]_DUAL_ENDSTOPS)
M672 - Set/Reset Duet Smart Effector's sensitivity. (Requires DUET_SMART_EFFECTOR and SMART_EFFECTOR_MOD_PIN)
M701 - Load filament (Requires FILAMENT_LOAD_UNLOAD_GCODES)
M702 - Unload filament (Requires FILAMENT_LOAD_UNLOAD_GCODES)
M810-M819 - Define/execute a G-code macro (Requires GCODE_MACROS)
M851 - Set Z probe's XYZ offsets in current units. (Negative values: X=left, Y=front, Z=below)
M852 - Set skew factors: "M852 [I<xy>] [J<xz>] [K<yz>]". (Requires SKEW_CORRECTION_GCODE, and SKEW_CORRECTION_FOR_Z for IJ)
M860 - Report the position of position encoder modules.
M861 - Report the status of position encoder modules.
M862 - Perform an axis continuity test for position encoder modules.
M863 - Perform steps-per-mm calibration for position encoder modules.
M864 - Change position encoder module I2C address.
M865 - Check position encoder module firmware version.
M866 - Report or reset position encoder module error count.
M867 - Enable/disable or toggle error correction for position encoder modules.
M868 - Report or set position encoder module error correction threshold.
M869 - Report position encoder module error.
M871 - Print/reset/clear first layer temperature offset values. (Requires PROBE_TEMP_COMPENSATION)
M192 - Wait for probe temp (Requires PROBE_TEMP_COMPENSATION)
M876 - Handle Prompt Response. (Requires HOST_PROMPT_SUPPORT and not EMERGENCY_PARSER)
M900 - Get or Set Linear Advance K-factor. (Requires LIN_ADVANCE)
M906 - Set or get motor current in milliamps using axis codes X, Y, Z, E. Report values if no axis codes given. 
       (Requires at least one _DRIVER_TYPE defined as TMC2130/2160/5130/5160/2208/2209/2660 or L6470)
M907 - Set digital trimpot motor current using axis codes. (Requires a board with digital trimpots)
M908 - Control digital trimpot directly. (Requires HAS_MOTOR_CURRENT_DAC or DIGIPOTSS_PIN)
M909 - Print digipot/DAC current value. (Requires HAS_MOTOR_CURRENT_DAC)
M910 - Commit digipot/DAC value to external EEPROM via I2C. (Requires HAS_MOTOR_CURRENT_DAC)
M911 - Report stepper driver overtemperature pre-warn condition. (Requires at least one _DRIVER_TYPE defined as TMC2130/2160/5130/5160/2208/2209/2660)
M912 - Clear stepper driver overtemperature pre-warn condition flag. (Requires at least one _DRIVER_TYPE defined as TMC2130/2160/5130/5160/2208/2209/2660)
M913 - Set HYBRID_THRESHOLD speed. (Requires HYBRID_THRESHOLD)
M914 - Set StallGuard sensitivity. (Requires SENSORLESS_HOMING or SENSORLESS_PROBING)
M916 - L6470 tuning: Increase KVAL_HOLD until thermal warning. (Requires at least one _DRIVER_TYPE L6470)
M917 - L6470 tuning: Find minimum current thresholds. (Requires at least one _DRIVER_TYPE L6470)
M918 - L6470 tuning: Increase speed until max or error. (Requires at least one _DRIVER_TYPE L6470)
M951 - Set Magnetic Parking Extruder parameters. (Requires MAGNETIC_PARKING_EXTRUDER)
M7219 - Control Max7219 Matrix LEDs. (Requires MAX7219_GCODE)
*
M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
*
************ Custom codes - This can change to suit future G-code regulations
G425 - Calibrate using a conductive object. (Requires CALIBRATION_GCODE)
M928 - Start SD logging: "M928 filename.gco". Stop with M29. (Requires SDSUPPORT)
M993 - Backup SPI Flash to SD
M994 - Load a Backup from SD to SPI Flash
M995 - Touch screen calibration for TFT display
M997 - Perform in-application firmware update
M999 - Restart after being stopped by error
D... - Custom Development G-code. Add hooks to 'gcode_D.cpp' for developers to test features. (Requires MARLIN_DEV_MODE)
```

* "T" Codes
```
T0-T3 - Select an extruder (tool) by index: "T<n> F<units/min>"
```

### Link
* https://marlinfw.org
* https://github.com/bigtreetech/BIGTREETECH-SKR-V1.3
* https://3dwork.io/en/complete-guide-skr-v1-4-and-tmc2209
* https://github.com/bigtreetech/BIGTREETECH-TouchScreenHardware
* https://github.com/bigtreetech/BIGTREETECH-TouchScreenFirmware
* https://reprap.org/wiki/G-code
* https://github.com/MarlinFirmware/MarlinDocumentation
* https://marlinfw.org/meta/gcode
* https://reprap.org/wiki/G-code
* https://linuxcnc.org/docs/html/gcode.html

<!-- MARKDOWN LINKS -->
[gambar-1-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/parsing%20read%20write%20ser0%20ser1.png
[gambar-2-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/read%20write%20parsing%20packet.png
[gambar-3-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/new%20serial%20comm%20port.png
[gambar-4-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/BTT%20SKR%20V1.4%20TURBO.jpg
[gambar-5-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/BTT%20SKR%20V1.4%20TURBO%20BACK.jpg
[gambar-6-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/BTT%20SKR%20V1.3.jpg