
#!/bin/bash
cd build
make clean
cd ..
cmake -B build -S .
echo "Done cleaning!"

