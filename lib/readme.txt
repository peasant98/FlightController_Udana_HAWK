Matthew Strong
CU Boulder 2021 Computer Science

The following code in this repository will use an Adafruit Sensor to calculate
the acceleration, magnetism, and gyro when the code runs. There is a simple LED light that will light up
when the acceleration vector of x, y, and z is higher than 10m/s^2.

The code here is pretty straightforward, there is a class for this sensor and this class
requires the Adafruit_Sensor header file, which can be applied to other adafruit sensors, not just the
lsm9ds1 one that I have. With this code, and once a connection can be established, the code will actually be able to run.




This directory is intended for the project specific (private) libraries.
PlatformIO will compile them to static libraries and link to executable file.

The source code of each library should be placed in separate directory, like
"lib/private_lib/[here are source files]".

For example, see how can be organized `Foo` and `Bar` libraries:

|--lib
|  |
|  |--Bar
|  |  |--docs
|  |  |--examples
|  |  |--src
|  |     |- Bar.c
|  |     |- Bar.h
|  |  |- library.json (optional, custom build options, etc) http://docs.platformio.org/page/librarymanager/config.html
|  |
|  |--Foo
|  |  |- Foo.c
|  |  |- Foo.h
|  |
|  |- readme.txt --> THIS FILE
|
|- platformio.ini
|--src
   |- main.c

Then in `src/main.c` you should use:

#include <Foo.h>
#include <Bar.h>

// rest H/C/CPP code

PlatformIO will find your libraries automatically, configure preprocessor's
include paths and build them.

More information about PlatformIO Library Dependency Finder
- http://docs.platformio.org/page/librarymanager/ldf.html
