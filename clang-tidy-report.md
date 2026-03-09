# Clang-Tidy Analysis Report for QSSTV

## Critical Errors (Blocking Commits)

These are treated as errors by the pre-commit hook:

| Check | Count | Priority | Notes |
|-------|-------|----------|-------|
| modernize-use-nullptr | 35 | HIGH | Using `0` instead of `nullptr` - modern C++ standard |
| bugprone-use-after-move | 0 | HIGH | Memory safety issue |
| bugprone-dangling-handle | 0 | HIGH | Memory safety issue |
| cert-err34-c | 0 | HIGH | Security issue |

## Top Issues by Category

### Readability (Most Common)

| Check | Count | Fix Difficulty | Auto-fixable |
|-------|-------|----------------|--------------|
| readability-braces-around-statements | 1,157 | Easy | Yes |
| readability-math-missing-parentheses | 1,045 | Easy | Yes |
| readability-identifier-naming | 1,420 | Hard | Partial |
| readability-implicit-bool-conversion | 285 | Medium | Yes |
| readability-isolate-declaration | 263 | Easy | Yes |

### Modernization

| Check | Count | Fix Difficulty | Auto-fixable |
|-------|-------|----------------|--------------|
| modernize-use-nullptr | 35 | Easy | Yes |
| modernize-use-override | 253 | Easy | Yes |
| modernize-avoid-c-arrays | 454 | Hard | No |
| modernize-use-default-member-init | 132 | Medium | Yes |
| modernize-deprecated-headers | 119 | Easy | Yes |
| modernize-use-equals-default | 106 | Easy | Yes |
| modernize-use-auto | 49 | Medium | Yes |

### Performance

| Check | Count | Fix Difficulty | Auto-fixable |
|-------|-------|----------------|--------------|
| performance-unnecessary-value-param | 200 | Easy | Yes |
| bugprone-implicit-widening-of-multiplication-result | 251 | Medium | Yes |

### Bug-prone Patterns

| Check | Count | Fix Difficulty | Auto-fixable |
|-------|-------|----------------|--------------|
| bugprone-branch-clone | 29 | Medium | No |
| bugprone-integer-division | 33 | Medium | No |
| bugprone-switch-missing-default-case | 19 | Easy | No |

## Recommended Action Plan

### Phase 1: Fix Critical Errors (Required for commits)
1. **modernize-use-nullptr** (35 occurrences) - Replace `0` with `nullptr` for pointers

### Phase 2: Easy Wins (High impact, low effort)
1. **modernize-deprecated-headers** (119) - Replace `<math.h>` with `<cmath>`, etc.
2. **modernize-use-override** (253) - Add `override` keyword to virtual functions
3. **modernize-use-equals-default** (106) - Use `= default` for trivial constructors/destructors
4. **readability-delete-null-pointer** (75) - Remove unnecessary null checks before delete

### Phase 3: Code Quality Improvements
1. **performance-unnecessary-value-param** (200) - Pass by const reference instead of by value
2. **readability-convert-member-functions-to-static** (147) - Make non-member-accessing functions static
3. **readability-make-member-function-const** (57) - Add const to functions that don't modify state

### Phase 4: Style Consistency (Optional, but recommended)
1. **readability-braces-around-statements** (1,157) - Add braces to single-line if/for/while
2. **readability-math-missing-parentheses** (1,045) - Add parentheses for operator precedence clarity

### Phase 5: Advanced Refactoring (Low priority)
1. **readability-identifier-naming** (1,420) - Rename to follow naming conventions (camelCase vs snake_case)
2. **modernize-avoid-c-arrays** (454) - Replace C arrays with std::array
3. **modernize-use-auto** (49) - Use auto for type deduction

## Notes

- Total unique issues: ~3,500+
- Most issues are auto-fixable with clang-tidy's `-fix` option
- Identifier naming changes would be the most disruptive (1,420 occurrences)
- Many readability issues are style preferences rather than bugs
