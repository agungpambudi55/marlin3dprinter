# Risearch Documentation

## Fri, 13 Nov 2020
* Directory File
```
MARLIN-2.0.X
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
Edit file Configuration.h on line 129 (MARLIN-2.0.X/Marlin/Configuration.h)
Choose the name from boards.h (MARLIN-2.0.X/Marlin/src/core/boards.h) that matches your setup
```
#ifndef MOTHERBOARD
  #define MOTHERBOARD <name-board>
#endif
```

Edit file platformio.ini on line 21 (MARLIN-2.0.X/platformio.ini)
```
default_envs = <name-environment>
```