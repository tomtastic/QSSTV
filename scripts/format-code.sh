#!/bin/bash
# Format all C++ code in the project using clang-format

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check if clang-format is available
if ! command -v clang-format &> /dev/null; then
    echo -e "${RED}Error: clang-format not found${NC}"
    echo "Install with: brew install clang-format"
    exit 1
fi

echo -e "${YELLOW}Formatting all C++ files...${NC}"

# Find all C++ source and header files
FILES=$(find src -name "*.cpp" -o -name "*.h" -o -name "*.hpp" -o -name "*.cc" -o -name "*.cxx")

TOTAL=0
for FILE in $FILES; do
    echo "Formatting $FILE..."
    clang-format -i "$FILE"
    TOTAL=$((TOTAL + 1))
done

echo -e "${GREEN}Formatted $TOTAL files!${NC}"
