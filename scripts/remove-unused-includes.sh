#!/bin/bash
# Remove unused includes from C++ files using clang-tidy
# This script processes files in batches and tests the build after each batch

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check if clang-tidy is available
if ! command -v clang-tidy &> /dev/null; then
    echo -e "${RED}Error: clang-tidy not found${NC}"
    exit 1
fi

# Check if compile_commands.json exists
if [ ! -f "build/compile_commands.json" ]; then
    echo -e "${RED}Error: build/compile_commands.json not found${NC}"
    echo "Run: cd build && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .."
    exit 1
fi

BATCH_SIZE=${1:-10}
echo -e "${YELLOW}Processing files in batches of $BATCH_SIZE${NC}"

# Find all C++ source files (excluding xmlrpc)
FILES=$(find src -name "*.cpp" -o -name "*.h" | grep -v "/xmlrpc/" | sort)

TOTAL=$(echo "$FILES" | wc -l | tr -d ' ')
PROCESSED=0
BATCH=0

echo -e "${YELLOW}Found $TOTAL files to process${NC}"

for FILE in $FILES; do
    PROCESSED=$((PROCESSED + 1))
    
    echo "[$PROCESSED/$TOTAL] Checking $FILE..."
    
    # Run clang-tidy with include-cleaner
    clang-tidy "$FILE" -p build \
        --checks='-*,misc-include-cleaner' \
        --fix \
        --format-style=file \
        2>&1 | grep -v "warnings generated" || true
    
    # Every BATCH_SIZE files, test the build
    if [ $((PROCESSED % BATCH_SIZE)) -eq 0 ]; then
        BATCH=$((BATCH + 1))
        echo -e "${YELLOW}Batch $BATCH complete. Testing build...${NC}"
        
        if ! cmake --build build 2>&1 | tail -20; then
            echo -e "${RED}Build failed after processing $PROCESSED files!${NC}"
            echo -e "${YELLOW}Reverting changes...${NC}"
            git checkout .
            exit 1
        fi
        
        echo -e "${GREEN}Build successful! Committing batch $BATCH...${NC}"
        git add -A
        git commit -m "Remove unused includes (batch $BATCH: files $((PROCESSED - BATCH_SIZE + 1))-$PROCESSED)" || true
    fi
done

# Process any remaining files
if [ $((PROCESSED % BATCH_SIZE)) -ne 0 ]; then
    BATCH=$((BATCH + 1))
    echo -e "${YELLOW}Final batch. Testing build...${NC}"
    
    if ! cmake --build build 2>&1 | tail -20; then
        echo -e "${RED}Build failed!${NC}"
        echo -e "${YELLOW}Reverting changes...${NC}"
        git checkout .
        exit 1
    fi
    
    echo -e "${GREEN}Build successful! Committing final batch...${NC}"
    git add -A
    git commit -m "Remove unused includes (batch $BATCH: final files)" || true
fi

echo -e "${GREEN}All done! Processed $TOTAL files in $BATCH batches.${NC}"
