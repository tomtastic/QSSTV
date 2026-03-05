#!/bin/bash
# Run clang-tidy on all C++ source files to modernize C-style casts

cd build

# Get list of all .cpp files from compile_commands.json
FILES=$(python3 -c "
import json
with open('compile_commands.json') as f:
    data = json.load(f)
    files = set()
    for entry in data:
        if 'file' in entry and entry['file'].endswith('.cpp'):
            files.add(entry['file'])
    for f in sorted(files):
        print(f)
")

echo "Found $(echo "$FILES" | wc -l) C++ files to process"

# Run clang-tidy with fixes
for file in $FILES; do
    echo "Processing: $file"
    /usr/local/opt/llvm/bin/clang-tidy \
        --checks='google-readability-casting' \
        --fix \
        --quiet \
        "$file" 2>&1 | grep -E "(applied|error)" || true
done

echo "Done!"
