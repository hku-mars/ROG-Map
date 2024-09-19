#!/bin/bash

echo "=== GCC Version ==="
gcc --version | head -n 1

echo "=== G++ Version ==="
g++ --version | head -n 1

echo "=== Eigen Version ==="

# Check if Eigen is installed
if dpkg -s libeigen3-dev &> /dev/null; then
    # Extract the version information from the dpkg output
    dpkg -s libeigen3-dev | grep 'Version'
else
    echo "Eigen is not installed or cannot be found."
fi

