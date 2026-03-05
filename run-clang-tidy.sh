#!/bin/bash
# Run clang-tidy on the QSSTV codebase
# Usage: ./run-clang-tidy.sh [--fix]

set -e

# Check if compile_commands.json exists
if [ ! -f "build/compile_commands.json" ]; then
    echo "Error: build/compile_commands.json not found"
    echo "Please run the following commands first:"
    echo "  mkdir -p build && cd build"
    echo "  cmake .."
    echo "  compiledb make"
    exit 1
fi

# Determine clang-tidy path
if command -v clang-tidy &> /dev/null; then
    CLANG_TIDY="clang-tidy"
elif [ -f "/usr/local/opt/llvm/bin/clang-tidy" ]; then
    CLANG_TIDY="/usr/local/opt/llvm/bin/clang-tidy"
else
    echo "Error: clang-tidy not found"
    echo "Install with: brew install llvm"
    exit 1
fi

# Determine run-clang-tidy path
if command -v run-clang-tidy &> /dev/null; then
    RUN_CLANG_TIDY="run-clang-tidy"
elif [ -f "/usr/local/opt/llvm/bin/run-clang-tidy" ]; then
    RUN_CLANG_TIDY="/usr/local/opt/llvm/bin/run-clang-tidy"
else
    echo "Error: run-clang-tidy not found"
    echo "Install with: brew install llvm"
    exit 1
fi

# Parse arguments
FIX_FLAG=""
if [ "$1" == "--fix" ]; then
    FIX_FLAG="-fix"
    echo "Running clang-tidy with automatic fixes..."
else
    echo "Running clang-tidy in check-only mode..."
    echo "Use --fix to automatically apply fixes"
fi

# Run clang-tidy
cd build
$RUN_CLANG_TIDY -p . $FIX_FLAG -header-filter='src/.*' 2>&1 | \
    grep -v "warning.*Qt" | \
    grep -v "warning.*/usr/local" || true

echo ""
echo "Clang-tidy analysis complete!"
