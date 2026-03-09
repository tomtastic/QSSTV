#!/bin/bash
# Remove unused includes from C++ files using clangd diagnostics
# This script processes files in batches and tests the build after each batch

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check if clangd is available
if ! command -v clangd &> /dev/null; then
    echo -e "${RED}Error: clangd not found${NC}"
    echo "Install with: brew install llvm"
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

# Find all C++ source and header files (excluding xmlrpc)
FILES=$(find src -name "*.cpp" -o -name "*.h" | grep -v "/xmlrpc/" | sort)

TOTAL=$(echo "$FILES" | wc -l | tr -d ' ')
PROCESSED=0
BATCH=0
MODIFIED=0

echo -e "${YELLOW}Found $TOTAL files to process${NC}"

# Function to get unused includes from clangd
get_unused_includes() {
    local file=$1
    # Use clangd to check for unused includes
    # This sends a textDocument/didOpen request and gets diagnostics
    clangd --check="$file" 2>&1 | grep -i "unused include" || true
}

# Function to remove an include line
remove_include() {
    local file=$1
    local include=$2
    # Escape special characters for sed
    local escaped=$(echo "$include" | sed 's/[\/&]/\\&/g')
    sed -i.bak "/^#include.*$escaped/d" "$file"
    rm -f "${file}.bak"
}

for FILE in $FILES; do
    PROCESSED=$((PROCESSED + 1))
    
    echo "[$PROCESSED/$TOTAL] Checking $FILE..."
    
    # Get unused includes from clangd
    UNUSED=$(get_unused_includes "$FILE")
    
    if [ -n "$UNUSED" ]; then
        echo -e "${YELLOW}  Found unused includes:${NC}"
        echo "$UNUSED" | head -5
        
        # For now, just report - manual removal is safer
        MODIFIED=$((MODIFIED + 1))
    fi
    
    # Every BATCH_SIZE files, test the build if we made changes
    if [ $((PROCESSED % BATCH_SIZE)) -eq 0 ] && [ $MODIFIED -gt 0 ]; then
        BATCH=$((BATCH + 1))
        echo -e "${YELLOW}Batch $BATCH complete ($MODIFIED files with unused includes). Testing build...${NC}"
        
        if ! cmake --build build 2>&1 | tail -20; then
            echo -e "${RED}Build failed after processing $PROCESSED files!${NC}"
            echo -e "${YELLOW}Reverting changes...${NC}"
            git checkout .
            exit 1
        fi
        
        echo -e "${GREEN}Build successful!${NC}"
        
        # Ask if user wants to commit
        read -p "Commit this batch? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            git add -A
            git commit -m "Remove unused includes (batch $BATCH: files $((PROCESSED - BATCH_SIZE + 1))-$PROCESSED)"
        fi
        
        MODIFIED=0
    fi
done

echo -e "${GREEN}Analysis complete! Processed $TOTAL files.${NC}"
echo -e "${YELLOW}Note: This script currently only reports unused includes.${NC}"
echo -e "${YELLOW}Use your IDE or manually remove them, then test the build.${NC}"
