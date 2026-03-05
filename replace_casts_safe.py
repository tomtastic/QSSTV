#!/usr/bin/env python3
"""
Safe C-style cast to static_cast converter.
Only converts simple, unambiguous cases.
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
    '_REAL', 'OPJ_INT32', 'OPJ_UINT32', 'CReal'
]

def is_safe_to_convert(content, match_start, match_end):
    """Check if this cast is safe to convert"""
    # Look ahead to see what follows
    remaining = content[match_end:match_end+50]
    
    # Skip if followed by -> or . (member access on the cast result)
    # e.g., (Type)ptr->member is ambiguous
    if re.match(r'^\s*[-.]>', remaining):
        return False
    
    # Skip if it's a function-style cast that might be intentional
    # e.g., (int)(x + y) where the parens are significant
    
    return True

def get_cast_expression(content, cast_end):
    """
    Extract the expression being cast.
    Returns (expression, end_position) or (None, -1) if can't parse safely.
    """
    i = cast_end
    
    # Skip whitespace
    while i < len(content) and content[i].isspace():
        i += 1
    
    if i >= len(content):
        return None, -1
    
    # Case 1: (type)(expression)
    if content[i] == '(':
        paren_count = 1
        j = i + 1
        while j < len(content) and paren_count > 0:
            if content[j] == '(':
                paren_count += 1
            elif content[j] == ')':
                paren_count -= 1
            j += 1
        if paren_count == 0:
            return content[i:j], j
        return None, -1
    
    # Case 2: (type)identifier
    if content[i].isalpha() or content[i] == '_':
        j = i
        while j < len(content) and (content[j].isalnum() or content[j] == '_'):
            j += 1
        return content[i:j], j
    
    # Case 3: (type)number
    if content[i].isdigit():
        j = i
        while j < len(content) and (content[j].isdigit() or content[j] in '.eE+-'):
            j += 1
        return content[i:j], j
    
    return None, -1

def replace_casts_in_content(content):
    """Replace C-style casts with static_cast in content"""
    changes = 0
    new_content = content
    offset = 0
    
    for cast_type in CAST_TYPES:
        # Match (type) or (type )
        pattern = r'\(\s*' + re.escape(cast_type) + r'\s*\)'
        
        for match in re.finditer(pattern, content):
            match_start = match.start() + offset
            match_end = match.end() + offset
            
            # Check if safe to convert
            if not is_safe_to_convert(new_content, match_start, match_end):
                continue
            
            # Get the expression being cast
            expr, expr_end = get_cast_expression(new_content, match_end)
            
            if expr is None:
                continue
            
            # Build the replacement
            replacement = f'static_cast<{cast_type}>({expr})'
            
            # Replace in content
            new_content = new_content[:match_start] + replacement + new_content[expr_end:]
            
            # Update offset for subsequent matches
            offset += len(replacement) - (expr_end - match_start)
            changes += 1
    
    return new_content, changes

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
            
            new_content, changes = replace_casts_in_content(content)
            
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
