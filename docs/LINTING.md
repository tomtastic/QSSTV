# Code Linting and Quality Assurance

This document describes the linting infrastructure for QSSTV.

## Overview

QSSTV uses clang-tidy for static code analysis to maintain code quality, catch bugs early, and enforce modern C++ practices.

## Tools

### clang-tidy
Static analyzer that checks C++ code for:
- Common bugs and programming errors
- Security issues (CERT guidelines)
- Performance problems
- Code modernization opportunities
- Readability issues

### clangd
Language server that provides IDE integration:
- Real-time error checking
- Code completion
- Go-to-definition
- Refactoring support

## Configuration Files

### `.clang-tidy`
Main configuration file that defines:
- Which checks are enabled/disabled
- Which warnings are treated as errors
- Naming conventions
- Header filter patterns

### `.clangd`
IDE integration configuration:
- Compilation database location
- Diagnostic settings
- InlayHints preferences

## Severity Levels

### Critical Errors (Blocking)
These will prevent commits and must be fixed:

- **bugprone-use-after-move**: Using an object after it has been moved
- **bugprone-dangling-handle**: References to temporary objects
- **cert-err34-c**: Unchecked return values from functions
- **modernize-use-nullptr**: Using NULL/0 instead of nullptr

### Warnings (Should Fix)
These should be addressed but won't block commits:

- Modernization suggestions (auto, range-based loops)
- Performance improvements (unnecessary copies, etc.)
- Readability improvements (naming, complexity)
- Type safety improvements (explicit constructors, etc.)

## Usage

### Manual Checking

Check all files:
```bash
./run-clang-tidy.sh
```

Check for critical errors only:
```bash
./run-clang-tidy.sh --errors-only
```

Auto-fix issues:
```bash
./run-clang-tidy.sh --fix
```

Check a single file:
```bash
clang-tidy src/path/to/file.cpp -p build
```

### Automatic Checking

Install the pre-commit hook:
```bash
ln -s ../../scripts/pre-commit .git/hooks/pre-commit
```

Now clang-tidy will run automatically on staged files before each commit.

Bypass the hook (not recommended):
```bash
git commit --no-verify
```

### IDE Integration

Most modern IDEs support clangd:

**VS Code:**
1. Install "clangd" extension
2. Disable C/C++ IntelliSense
3. Reload window

**CLion:**
- Built-in support, just enable clang-tidy in settings

**Vim/Neovim:**
- Use coc-clangd or vim-lsp with clangd

**Emacs:**
- Use lsp-mode with clangd

## Common Issues and Fixes

### Issue: "compile_commands.json not found"
**Fix:**
```bash
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
```

### Issue: "clang-tidy not found"
**Fix (macOS):**
```bash
brew install llvm
```

**Fix (Linux):**
```bash
sudo apt-get install clang-tidy  # Debian/Ubuntu
sudo dnf install clang-tools-extra  # Fedora/RHEL
```

### Issue: Too many warnings
Use `--errors-only` to focus on critical issues first:
```bash
./run-clang-tidy.sh --errors-only
```

### Issue: False positives
If a check produces false positives, you can:
1. Disable it in `.clang-tidy`
2. Use `// NOLINT(check-name)` comment to suppress specific warnings
3. Use `// NOLINTNEXTLINE(check-name)` for the next line

Example:
```cpp
// NOLINTNEXTLINE(modernize-use-nullptr)
void* ptr = NULL;  // Required by legacy API
```

## Continuous Integration

GitHub Actions automatically runs clang-tidy on all pull requests. The CI will:
1. Build the project
2. Run clang-tidy on all changed files
3. Report any critical errors
4. Block merge if critical errors are found

## Gradual Adoption

We're gradually improving code quality:

1. **Phase 1** (Current): Critical errors only
   - Focus on bugs and safety issues
   - Block commits with critical errors

2. **Phase 2** (Future): Modernization
   - Gradually fix modernization warnings
   - Update old code to modern C++

3. **Phase 3** (Future): Full compliance
   - All warnings addressed
   - Strict mode enabled

## Resources

- [Clang-Tidy Documentation](https://clang.llvm.org/extra/clang-tidy/)
- [CERT C++ Coding Standard](https://wiki.sei.cmu.edu/confluence/pages/viewpage.action?pageId=88046682)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
- [Modern C++ Features](https://github.com/AnthonyCalandra/modern-cpp-features)

## Questions?

See [CONTRIBUTING.md](../CONTRIBUTING.md) for more information or open an issue.
