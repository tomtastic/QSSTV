#!/usr/bin/env python3
"""
Remove unused includes from C++ files using clangd diagnostics.
This script processes files in batches and tests the build after each batch.
"""

import subprocess
import sys
import os
import re
from pathlib import Path

# Colors
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
NC = '\033[0m'

def find_cpp_files():
    """Find all C++ source and header files, excluding xmlrpc."""
    result = subprocess.run(
        ['find', 'src', '-name', '*.cpp', '-o', '-name', '*.h'],
        capture_output=True, text=True
    )
    files = [f for f in result.stdout.strip().split('\n') if f and '/xmlrpc/' not in f]
    return sorted(files)

def get_unused_includes(filepath):
    """Get list of unused includes from a file using clangd diagnostics."""
    # We'll parse the file and check each include
    # This is a simplified version - in practice, you'd use clangd LSP
    unused = []
    
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
            
        for i, line in enumerate(lines, 1):
            if line.strip().startswith('#include'):
                # Extract the include filename
                match = re.search(r'#include\s+[<"]([^>"]+)[>"]', line)
                if match:
                    include_file = match.group(1)
                    # Check if this include is used in the file
                    # This is a heuristic - proper detection needs clangd
                    base_name = os.path.splitext(os.path.basename(include_file))[0]
                    
                    # Read rest of file
                    rest_of_file = ''.join(lines[i:])
                    
                    # Simple heuristic: if the base name doesn't appear elsewhere, might be unused
                    # This is not perfect but gives us candidates
                    if base_name not in rest_of_file:
                        unused.append((i, line.strip(), include_file))
    except Exception as e:
        print(f"{RED}Error reading {filepath}: {e}{NC}")
    
    return unused

def remove_include_line(filepath, line_number):
    """Remove a specific line from a file."""
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        if 0 < line_number <= len(lines):
            del lines[line_number - 1]
            
            with open(filepath, 'w') as f:
                f.writelines(lines)
            return True
    except Exception as e:
        print(f"{RED}Error modifying {filepath}: {e}{NC}")
    
    return False

def test_build():
    """Test if the project builds successfully."""
    print(f"{YELLOW}Testing build...{NC}")
    result = subprocess.run(
        ['cmake', '--build', 'build'],
        capture_output=True, text=True
    )
    
    if result.returncode != 0:
        print(f"{RED}Build failed!{NC}")
        print(result.stdout[-2000:] if len(result.stdout) > 2000 else result.stdout)
        return False
    
    print(f"{GREEN}Build successful!{NC}")
    return True

def git_commit(message):
    """Commit changes to git."""
    subprocess.run(['git', 'add', '-A'])
    subprocess.run(['git', 'commit', '-m', message])

def main():
    batch_size = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    
    print(f"{YELLOW}Finding C++ files...{NC}")
    files = find_cpp_files()
    total = len(files)
    
    print(f"{YELLOW}Found {total} files to process{NC}")
    print(f"{YELLOW}NOTE: This script uses heuristics. Manual review recommended.{NC}")
    print(f"{YELLOW}Better approach: Use IDE with clangd to see unused includes.{NC}")
    print()
    
    processed = 0
    modified = 0
    batch = 0
    
    for filepath in files:
        processed += 1
        print(f"[{processed}/{total}] Checking {filepath}...")
        
        unused = get_unused_includes(filepath)
        if unused:
            print(f"  {YELLOW}Found {len(unused)} potentially unused includes{NC}")
            for line_num, line, include in unused:
                print(f"    Line {line_num}: {line}")
            modified += 1
        
        # Every batch_size files, offer to test
        if processed % batch_size == 0 and modified > 0:
            batch += 1
            print(f"\n{YELLOW}Batch {batch} complete ({modified} files with potential issues).{NC}")
            print(f"{YELLOW}Review the findings above and manually remove unused includes.{NC}")
            
            response = input(f"Test build now? (y/n): ")
            if response.lower() == 'y':
                if not test_build():
                    print(f"{RED}Stopping due to build failure.{NC}")
                    return 1
            
            modified = 0
    
    print(f"\n{GREEN}Analysis complete! Processed {total} files.{NC}")
    print(f"{YELLOW}Recommendation: Use your IDE with clangd to see and remove unused includes.{NC}")
    print(f"{YELLOW}The IDE will show them with 'Included header X is not used directly' warnings.{NC}")
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
