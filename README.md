# Risearch Documentation

## Fri, 13 Nov 2020
* Directory File
```
|--Marlin
|  |--lib
|  |  |--readme.txt
|  |--src
|  |  |--core
|  |  |  |--boards.h
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

* Build Setup
File Configuration.h (Marlin/Configuration.h)
```
// Choose the name from boards.h that matches your setup
#ifndef MOTHERBOARD
  #define MOTHERBOARD <name-board>
#endif
```