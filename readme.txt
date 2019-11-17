This project requires a modification to the build configuration in the platform files.

There's a line that defines array LIB, which contains "c". An additional string should
be added to this array with value "m", so that the ELF file gets linked with 'math'.
