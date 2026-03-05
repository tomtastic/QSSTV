#!/usr/bin/env python3
"""
Conservative C-style cast to static_cast converter.
Only converts cases where the expression is clearly delimited by parentheses.
Pattern: (type)(expression) -> static_cast<type>(expression)
"""
import os
import re

CAST_TYPES = [
    'int', 'uint', 'unsigned int', 'unsigned',
    'float', 'double', 
    'char', 'unsigned char',
    'short', 'unsigned short',
    'long', 'unsigned long',
    'bool',
    'quint8', 'quint16', 'quint32', 'quint64',
    'qint8', 'qint16', 'qint32', 'qint64',
    'size_t', 'uint8_t', 'uint16_t', 'uint32_t', 'uint64_t',
    'int8_t', 'int16_t', 'int32_t', 'int64_t',
    '_REAL', 'OPJ_INT32', 'OPJ_UINT32', 'CReal', 'OPJ_SIZE_T'
]

def replace_casts_conservative(content):
    """
    Only replace (type)(expr) patterns where expr is in parentheses.
    This is safe because the parentheses clearly delimit the expression.
    """
    changes = 0
    
    for cast_type in CAST_TYPES:
        # Pattern: (type)(  - cast followed immediately by opening paren
        # We'll replace (type)( with static_cast<type>(
        pattern = r'\(\s*' + re.escape(cast_type) + r'\s*\)\s*\('
        replacement = f'static_cast<{cast_type}>('
        
        new_content, count = re.subn(pattern, replacement, content)
        content = new_content
        changes += count
    
    return content, changes

def process_files():
    """Process all C++ files"""
    total_changes = 0
    files_changed = 0
    
    for root, dirs, files in os.walk('src'):
        for filename in files:
            if not filename.endswith(('.cpp', '.h')):
                continue
            
            filepath = os.path.join(root, filename)
            
            try:
                with open(filepath, 'r', encoding='utf-8') as f:
                    content = f.read()
            except UnicodeDecodeError:
                print(f"Skipping {filepath} (encoding issue)")
                continue
            except Exception as e:
                print(f"Error reading {filepath}: {e}")
                continue
            
            new_content, changes = replace_casts_conservative(content)
            
            if changes > 0:
                try:
                    with open(filepath, 'w', encoding='utf-8') as f:
                        f.write(new_content)
                    print(f"Updated {filepath}: {changes} casts")
                    total_changes += changes
                    files_changed += 1
                except Exception as e:
                    print(f"Error writing {filepath}: {e}")
    
    print(f"\nTotal: {total_changes} casts in {files_changed} files")
    return total_changes, files_changed

if __name__ == '__main__':
    process_files()
