## Section Overview
Expressions, Statements and Operators
- Expressions
- Statements and block statements
- Operators ( assignement, Arithmetic, Increment and Decrement, Equality, Relational, Logical, Compound assignment, Precedence)

### Expression and Statements
- What are expressions? eg. 34 , favorite_number, 1.5 + 2.8 , 2*5 , a>b , a=b
- What are statements . 
```cpp
int x;  // declaration
favorite_number = 12; // assignment
1.5 + 2.8; // expression
x = 2 * 5;
if (a>b) cout << "a is greater than b"; // if
;
```
### Using Operators
C++ has a rich set of operators ( unary, binary,ternary)
- Unary: works on 1 operand     
(Example: -x, ++x, !flag)z
- Binary: works on 2 operands   
(Example: a + b, x == y, x && y)
- Ternary: works on 3 operands  
(Example: condition ? value_if_true : value_if_false)

Common Operators can be grouped as follows:
- assignment
- arithmetic
- increment/decrement
- relational
- logical
- member access
- other

| Category                    | Operators                         | Meaning / Use                                   |                            |                              |
| --------------------------- | --------------------------------- | ----------------------------------------------- | -------------------------- | ---------------------------- |
| **Assignment**              | `=`, `+=`, `-=`, `*=`, `/=`, `%=` | Assign or update values                         |                            |                              |
| **Arithmetic**              | `+`, `-`, `*`, `/`, `%`           | Math operations                                 |                            |                              |
| **Unary**                   | `+`, `-`, `++`, `--`, `!`         | Operate on one value (negate, increment, NOT)   |                            |                              |
| **Increment / Decrement**   | `++x`, `x++`, `--x`, `x--`        | Increase/decrease by 1 (pre/post)               |                            |                              |
| **Relational (Comparison)** | `==`, `!=`, `<`, `>`, `<=`, `>=`  | Compare values → returns `true`/`false`         |                            |                              |
| **Logical**                 | `&&`, `                           |                                                 | `, `!`                     | Boolean logic (AND, OR, NOT) |
| **Member Access**           | `.`, `->`, `::`                   | Access members of objects, pointers, namespaces |                            |                              |
| **Ternary**                 | `?:`                              | Shorthand `if-else`                             |                            |                              |
| **Address / Dereference**   | `&`, `*`                          | Pointer operators: address-of, dereference      |                            |                              |
| **Size**                    | `sizeof`                          | Get size of a variable/type (bytes)             |                            |                              |
| **Bitwise**                 | `&`, `                            | `, `^`, `~`, `<<`, `>>`                         | Operate on bits (advanced) |                              |

### The Assignment Operator
```cpp
int num1 {10};
int num2 {20};
num1 = num2=1000;
/*
 num1 → l-value (left side, a location)
 num2 → r-value (right side, a value)

The assignment operator:
 Copies the r-value into the memory location of the l-value.

also in chaining 
✅ How it works (right to left)
Assignment in C++ is right-associative, meaning:
num2 = 1000     → num2 becomes 1000
num1 = num2     → num1 becomes 1000
*/
```
### Increment and Decrement Operators
Pre-increment (++counter)
- Increases the value first
- Then returns the new value
```cpp
int counter {10};
int result = ++counter;
// Execution:
//  counter becomes 11
//  result becomes 11
```

Post-increment (counter++)
- Returns the current value first
- Then increases the value after
```cpp
int counter {10};
int result = counter++; //avoid this if u can , use it counter++ in a single line;
// Execution:
// result becomes 10
// counter becomes 11 (after assignment)
```
### Mixed Expressions and Conversions

When you mix different data types in an expression, C++ automatically converts them to a common type before evaluating.

Conversions (Type Promotion)
- C++ promotes smaller/lower-precision types into larger/higher-precision types to avoid losing information.

Higher vs Lower Types
- A higher type can store larger or more precise values than a lower type.

Rough hierarchy (lowest → highest)
| Type                    | Notes                    |
| ----------------------- | ------------------------ |
| `bool`                  | lowest                   |
| `char`, `unsigned char` | 1 byte                   |
| `short`                 | 2 bytes                  |
| `int`                   | 4 bytes                  |
| `unsigned int`          | 4 bytes but higher range |
| `long`                  | depends (4 or 8 bytes)   |
| `long long`             | 8 bytes                  |
| `float`                 | 6–7 digits precision     |
| `double`                | **15 digits precision**  |
| `long double`           | highest precision        |

#### Explict Type casting
```
int total_amount {100};
int total_number {8};
double average {0.0};

average = static_cast<double>(total_amount)/ total_number;
```

### Testing for Equality
The == and  != operators

| Operator | Meaning      | Example  |
| -------- | ------------ | -------- |
| `==`     | equal to     | `a == b` |
| `!=`     | not equal to | `a != b` |


### Relational Operators
Used to compare values, results in true or false.
| Operator | Meaning               | Example  |
| -------- | --------------------- | -------- |
| `<`      | less than             | `x < y`  |
| `>`      | greater than          | `x > y`  |
| `<=`     | less than or equal    | `x <= y` |
| `>=`     | greater than or equal | `x >= y` |

### Logical operator
Logical operators are used to combine or invert boolean expressions.
| Operator | Name        | Meaning                                    | Example                |
| -------- | ----------- | ------------------------------------------ | ---------------------- |
| `&&`     | Logical AND | true only if **both** conditions are true  | `(x > 0 && y > 0)`     |
| `\|\|`   | Logical OR  | true if **at least one** condition is true | `(x == 0 \|\| y == 0)` |
| `!`      | Logical NOT | reverses a condition                       | `!is_valid`            |

Short Curcuit Expression
- Short-circuiting means C++ stops evaluating a logical expression as soon as the final result is known.
```
expression1 && expression2 && expression3
expression1 → expression2 → expression3  
(stops early when one is false)

expression1 || expression2 || expression3
expression1 → expression2 → expression3  
(stops early when one is true)
```

Operator Precedence (who gets evaluated first)
```cpp
A || B && C

// Step 1: Precedence groups it into
// A || (B && C)  // because && comes first
// Step 2: Short-circuiting applies during evaluation
// If A is true → skip (B && C)
// If A is false → evaluate (B && C) normally
```

### Compound Assignment Operators
Compound assignment operators combine an operation and assignment in a single shortcut.
| Operator | Meaning                | Equivalent To         |              |
| -------- | ---------------------- | --------------------- | ------------ |
| `+=`     | add and assign         | `x = x + y`           |              |
| `-=`     | subtract and assign    | `x = x - y`           |              |
| `*=`     | multiply and assign    | `x = x * y`           |              |
| `/=`     | divide and assign      | `x = x / y`           |              |
| `%=`     | modulo and assign      | `x = x % y`           |              |
| `<<=`    | left-shift and assign  | `x = x << y`          |              |
| `>>=`    | right-shift and assign | `x = x >> y`          |              |
| `&=`     | bitwise AND and assign | `x = x & y`           |              |
| `        | =`                     | bitwise OR and assign | `x = x \| y` |
| `^=`     | bitwise XOR and assign | `x = x ^ y`           |              |

### Operator Precedence (High → Low)
Higher precedence means the operator is evaluated first.
| Precedence Level | Operators                                          | Description                                |
| ---------------- | -------------------------------------------------- | ------------------------------------------ |
| **Highest**      | `()`, `[]`, `.`, `->`                              | Function call, array access, member access |
|                  | `++` `--` (prefix), `+` `-` (unary), `!`, `sizeof` | Unary operators                            |
|                  | `*`, `/`, `%`                                      | Multiplicative                             |
|                  | `+`, `-`                                           | Additive                                   |
|                  | `<<`, `>>`                                         | Bitwise shift                              |
|                  | `<`, `<=`, `>`, `>=`                               | Relational                                 |
|                  | `==`, `!=`                                         | Equality                                   |
|                  | `&`                                                | Bitwise AND                                |
|                  | `^`                                                | Bitwise XOR                                |
|                  | `\|`                                               | Bitwise OR                                 |
|                  | `&&`                                               | Logical AND                                |
|                  | `\|\|`                                             | Logical OR                                 |
|                  | `?:`                                               | Ternary conditional                        |
| **Lowest**       | `=`, `+=`, `-=`, `*=`, `/=`, `%=`                  | Assignment operators                       |
