#!/bin/bash
# Script to download and set up SPICE toolkit for ILOSS

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
EXTERNAL_DIR="$PROJECT_ROOT/external"
SPICE_DIR="$EXTERNAL_DIR/cspice"

echo "=== SPICE Toolkit Download Script ==="
echo "Project root: $PROJECT_ROOT"
echo "SPICE will be installed to: $SPICE_DIR"

# Create external directory if it doesn't exist
mkdir -p "$EXTERNAL_DIR"

# Detect platform
OS=$(uname -s)
ARCH=$(uname -m)

if [ "$OS" = "Linux" ]; then
    if [ "$ARCH" = "x86_64" ]; then
        SPICE_URL="https://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Linux_GCC_64bit/packages/cspice.tar.Z"
        PLATFORM="PC_Linux_64bit"
    else
        echo "Error: Unsupported Linux architecture: $ARCH"
        exit 1
    fi
elif [ "$OS" = "Darwin" ]; then
    if [ "$ARCH" = "x86_64" ]; then
        SPICE_URL="https://naif.jpl.nasa.gov/pub/naif/toolkit/C/MacIntel_OSX_AppleC_64bit/packages/cspice.tar.Z"
        PLATFORM="MacIntel_64bit"
    elif [ "$ARCH" = "arm64" ]; then
        SPICE_URL="https://naif.jpl.nasa.gov/pub/naif/toolkit/C/MacM1_OSX_clang_64bit/packages/cspice.tar.Z"
        PLATFORM="MacM1_64bit"
    else
        echo "Error: Unsupported macOS architecture: $ARCH"
        exit 1
    fi
else
    echo "Error: Unsupported operating system: $OS"
    echo "For Windows, please download SPICE manually from:"
    echo "https://naif.jpl.nasa.gov/naif/toolkit_C.html"
    exit 1
fi

echo "Platform detected: $PLATFORM"
echo "Download URL: $SPICE_URL"

# Check if SPICE is already downloaded
if [ -d "$SPICE_DIR" ]; then
    echo "SPICE directory already exists at $SPICE_DIR"
    read -p "Do you want to re-download? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Using existing SPICE installation"
        exit 0
    fi
    rm -rf "$SPICE_DIR"
fi

# Download SPICE
echo "Downloading SPICE toolkit..."
cd "$EXTERNAL_DIR"
wget -O cspice.tar.Z "$SPICE_URL" || curl -o cspice.tar.Z "$SPICE_URL"

# Extract SPICE
echo "Extracting SPICE toolkit..."
uncompress cspice.tar.Z || gunzip cspice.tar.Z
tar -xf cspice.tar
rm -f cspice.tar

# Verify installation
if [ -f "$SPICE_DIR/include/SpiceUsr.h" ] && [ -f "$SPICE_DIR/lib/cspice.a" ]; then
    echo "✓ SPICE toolkit successfully installed!"
    echo ""
    echo "To use SPICE in the build, add the following to your CMake command:"
    echo "  -DSPICE_ROOT_DIR=$SPICE_DIR"
    echo ""
    echo "Example:"
    echo "  cd $PROJECT_ROOT/build"
    echo "  cmake .. -DSPICE_ROOT_DIR=$SPICE_DIR"
else
    echo "✗ Error: SPICE installation verification failed"
    exit 1
fi

# Download basic kernels (optional)
echo ""
read -p "Do you want to download basic SPICE kernels (LSK, PCK)? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    KERNELS_DIR="$PROJECT_ROOT/data/spice_kernels"
    mkdir -p "$KERNELS_DIR"
    
    echo "Downloading leap seconds kernel (LSK)..."
    wget -P "$KERNELS_DIR" "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/lsk/naif0012.tls" || \
        curl -o "$KERNELS_DIR/naif0012.tls" "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/lsk/naif0012.tls"
    
    echo "Downloading planetary constants kernel (PCK)..."
    wget -P "$KERNELS_DIR" "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/pck00011.tpc" || \
        curl -o "$KERNELS_DIR/pck00011.tpc" "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/pck00011.tpc"
    
    echo "Downloading DE440 planetary ephemeris..."
    wget -P "$KERNELS_DIR" "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440.bsp" || \
        curl -o "$KERNELS_DIR/de440.bsp" "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440.bsp"
    
    echo "✓ Basic kernels downloaded to $KERNELS_DIR"
fi

echo ""
echo "=== SPICE Setup Complete ==="