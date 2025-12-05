## Controlling Program Flow

- Sequence , Ordering Statements sequentially
- Selection , Making decisions
- Iteration , Looping or repeating

### Selection 
| Selection Type        | Syntax Example                   | Description                         | When to Use                   |
| --------------------- | -------------------------------- | ----------------------------------- | ----------------------------- |
| **if**                | `if (x > 0) { }`                 | Runs code only if condition is true | Single condition              |
| **if–else**           | `if (...) { } else { }`          | Chooses between two paths           | True/false decisions          |
| **Nested if**         | `if (...) { if (...) { } }`      | if inside another if                | Multiple dependent conditions |
| **if–else-if ladder** | `if (...) { } else if (...) { }` | Multiple possible conditions        | 3–5 condition checks          |
| **switch**            | `switch(x) { case 1: ... }`      | Tests multiple fixed values         | Menu options, enums           |
| **Ternary `?:`**      | `(cond) ? a : b`                 | Short if–else in one line           | Small decisions (assignments) |


### Looping
| Loop Type           | Syntax Example            | Description                      | When to Use              |
| ------------------- | ------------------------- | -------------------------------- | ------------------------ |
| **for**             | `for (int i=0; i<5; i++)` | Repeats known number of times    | Fixed count loops        |
| **while**           | `while (x > 0)`           | Runs while condition is true     | Unknown repetitions      |
| **do–while**        | `do { } while(x > 0);`    | Always runs at least once        | Menus, input validation  |
| **range-based for** | `for (int v : arr)`       | Iterates through container items | Arrays, vectors (C++11+) |
| **break**           | `if (x==5) break;`        | Exits the loop immediately       | Stop early               |
| **continue**        | `if (x==5) continue;`     | Skips current iteration          | Skip certain values      |

### If Statement
```cpp
if( selection == 'A'){
    cout << "You selected A";
} else if( hascar){
    cout << "You selected not A and hascar";
} else {
    cout << "You selected not A and no hascar";
}(
```

### switch statement
```cpp
switch (selection) {
    case '1': 
        cout << "1 selected";
        break;
    case '2': 
        cout << "2 selected";
        break;
    case '3': 
    case '4':
        cout << "3 or 4 selected";
        break;
    default: cout << "1,2,3,4 NOT selected";
}
```

```cpp
enum class Direction {
    Left,
    Right,
    Up,
    Down
};
Direction heading = Direction::Left;

switch (heading) {
    case Direction::Left:
        cout << "Going left" << endl;
        break;
    case Direction::Right:
        cout << "Going right" << endl;
        break;
    case Direction::Up:
        cout << "Going up" << endl;
        break;
    case Direction::Down:
        cout << "Going down" << endl;
        break;
}`
```

### Conditional Operator
```cpp
result = (a>b) ? a:b;
result = (a<b) ? (b-a):(a-b);
```

### Looping
#### For Loops
```cpp
for (int i {1} ; i <= 5 ; ++i)
    cout << i << endl;

for (int i {1}, j {5} ; i <= 5 ; ++i, ++j)
    cout << i << j << endl;

// introduced in C++ 11, ranged based for loops
for ( var_type var_name: sequence)
    statement;

int scores [] { 100, 90 , 97};
for (int score : scores)
    cout << score << endl;

for (auto score : scores)
    cout << score << endl;
```

### While Loops
```cpp
while (expression)
    statement;

int i {1};
while (i<=5){
    cout << i <<endl;
    ++i;
}

```

### Do-While Loop
```cpp
do {
    statements;
} while ( expression);

int number {};
do{
    cout << "Enter an integer between 1 and 5: ";
    cin >> number;
} while( number <= 1 || number >= 5);
```

### continue and break
```cpp
for( auto val:values){
    if( val == -99)\
        //break out of loop directly
        break;
    else if (val == -1)
        // skip this step and continue to next iteration
        continue;
    else
        cout << val << endl;
}
```

#### Infinite loops
```cpp
for (;;)
    statement;

while (true)
    statement

do {

}while (true);
```


