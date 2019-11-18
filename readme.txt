This project is a test of I2C communication with a GD32V SoC master
communicating with a BMP085 slave. It was developed on a Longan Nano, so the
results are written to the Nano's LCD display.

My code is released as GPLv2, or later as you choose, but other files such as
the LCD display code, are not. Until the other code has been replaced or
properly licensed, you shouldn't use this commercially.

The project requires a modification to the build configuration in the platform
files so that it links with a math library (needed for the powf() function).

There's a line that defines array LIB, which contains "c". An additional string
should be added to this array with value "m", so that the ELF file gets linked
with 'math'.

See
    ~/.platformio/platforms/gd32v/builder/frameworks/firmware_library.py

env.Append(
    CPPPATH = [
        ...
    ],
    LIBS = [
        "c", "m"  <---- Add "m" for math library.
    ]
)

There may be a better way to do this - the PlatformIO docs describe build
argrument parameters, but I didn't get these to work, nor could I find
reference to the arguments in the PlatformIO source code. I have assumed for
now that they relate to a newer version.

I think we could remove the dependency on on the powf() function with a much simpler
caclulation. It is used in the calculation to compensate pressure to sea-level. If you
need absolute (uncompensated) pressure, then you can remove this calculation.

However, the range of the exponential function that we're concerned with is so
narrow (i.e. we are not interested in pressures at altitudes above maybe 2000,
or even 20000 metres), that the function is almost linear, and I think linear
enough that it would not affect accuracy to treat it as such. This would reduce
the memory footprint to a few floating point multiplications.
