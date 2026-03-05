#!/usr/bin/env python3
"""
Convert C-style casts of simple identifiers.
Pattern: (type)identifier -> static_cast<type>(identifier)
where identifier is a simple variable name (no operators, no member access).
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
    '_REAL', 'OPJ_INT32', 'OPJ_UINT32', 'CReal', 'OPJ_SIZE_T',
    'ptt_type_t', 'esstvMode'
]

def replace_casts_identifiers(content):
    """
    Replace (type)identifier with static_cast<type>(identifier).
    Only matches simple identifiers (no dots, arrows, brackets).
    """
    changes = 0
    
    for cast_type in CAST_TYPES:
        # Pattern: (type) followed by identifier (with optional space)
        # Identifier: starts with letter/underscore, contains alphanumeric/underscore
        # Must NOT be followed by ->, ., [, or (
        pattern = r'\(\s*' + re.escape(cast_type) + r'\s*\)\s*([a-zA-Z_][a-zA-Z0-9_]*)(?![a-zA-Z0-9_\[\.\-])'
        
        def replacer(match):
            identifier = match.group(1)
            return f'static_cast<{cast_type}>({identifier})'
        
        new_content, count = re.subn(pattern, replacer, content)
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
            
            new_content, changes = replace_casts_identifiers(content)
            
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
