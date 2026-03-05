# Contributing to QSSTV

## Code Quality and Linting

This project uses clang-tidy for static analysis and code quality checks.

### Prerequisites

Install the required tools:

```bash
# macOS
brew install llvm compiledb

# Linux (Debian/Ubuntu)
sudo apt-get install clang-tidy python3-pip
pip3 install compiledb
```

### Running Clang-Tidy

1. Generate the compilation database (first time only, or after adding/removing files):

```bash
mkdir -p build && cd build
cmake ..
compiledb make
cd ..
```

2. Run clang-tidy to check for issues:

```bash
./run-clang-tidy.sh
```

3. Run clang-tidy with automatic fixes:

```bash
./run-clang-tidy.sh --fix
```

### Configuration

The `.clang-tidy` file in the project root defines which checks are enabled:

- **bugprone-***: Catches common programming errors
- **cert-***: CERT C++ coding standard checks
- **google-readability-casting**: Enforces static_cast over C-style casts
- **modernize-***: Suggests modern C++ idioms (nullptr, range-based loops, etc.)
- **performance-***: Performance optimization suggestions
- **readability-***: Code readability improvements

### Before Submitting a Pull Request

1. Ensure your code compiles without errors
2. Run clang-tidy and address any warnings: `./run-clang-tidy.sh`
3. Test your changes thoroughly
4. Follow the existing code style

### Modernization Guidelines

We're actively modernizing the codebase to use modern C++ features:

- Use `nullptr` instead of `NULL`
- Use `static_cast<T>()` instead of C-style casts `(T)`
- Use range-based for loops where possible
- Use `true`/`false` instead of `1`/`0` for booleans
- Use modern Qt signal/slot connections with function pointers

See the git history for examples of these modernizations.
