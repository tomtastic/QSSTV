#!/bin/bash
# Run clang-tidy on the QSSTV codebase
# Usage: ./run-clang-tidy.sh [--fix] [--errors-only]

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check if compile_commands.json exists
if [ ! -f "build/compile_commands.json" ]; then
    echo -e "${RED}Error: build/compile_commands.json not found${NC}"
    echo "Please run the following commands first:"
    echo "  cd build && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .."
    exit 1
fi

# Determine clang-tidy path
if command -v clang-tidy &> /dev/null; then
    CLANG_TIDY="clang-tidy"
elif [ -f "/usr/local/opt/llvm/bin/clang-tidy" ]; then
    CLANG_TIDY="/usr/local/opt/llvm/bin/clang-tidy"
else
    echo -e "${RED}Error: clang-tidy not found${NC}"
    echo "Install with: brew install llvm"
    exit 1
fi

# Parse arguments
FIX_FLAG=""
ERRORS_ONLY=false
for arg in "$@"; do
    case $arg in
        --fix)
            FIX_FLAG="-fix"
            echo -e "${YELLOW}Running clang-tidy with automatic fixes...${NC}"
            ;;
        --errors-only)
            ERRORS_ONLY=true
            echo -e "${YELLOW}Showing critical errors only...${NC}"
            ;;
        *)
            echo "Usage: $0 [--fix] [--errors-only]"
            exit 1
            ;;
    esac
done

if [ -z "$FIX_FLAG" ]; then
    echo -e "${YELLOW}Running clang-tidy in check-only mode...${NC}"
    echo "Use --fix to automatically apply fixes"
fi

# Find all C++ source files
FILES=$(find src -name "*.cpp" -o -name "*.cc" -o -name "*.cxx")

# Run clang-tidy on each file
TOTAL=0
ERRORS=0
for FILE in $FILES; do
    TOTAL=$((TOTAL + 1))
    echo -e "${YELLOW}[$TOTAL] Checking $FILE...${NC}"
    
    OUTPUT=$($CLANG_TIDY "$FILE" -p build $FIX_FLAG 2>&1 || true)
    
    if [ "$ERRORS_ONLY" = true ]; then
        # Show only critical errors
        if echo "$OUTPUT" | grep -q "error:"; then
            echo "$OUTPUT" | grep "error:"
            ERRORS=$((ERRORS + 1))
        fi
    else
        # Show all output except Qt and system warnings
        echo "$OUTPUT" | grep -v "warning.*Qt" | grep -v "warning.*/usr/local" || true
    fi
done

echo ""
if [ $ERRORS -gt 0 ]; then
    echo -e "${RED}Found $ERRORS file(s) with critical errors${NC}"
    exit 1
else
    echo -e "${GREEN}Clang-tidy analysis complete! Checked $TOTAL files.${NC}"
fi
