## Structure of a C++ Program
- Basic Components
- Preprocessor Directives
- The main function
- Namespaces
- Comments
- Basic i/o

### C++ KEYWORDS(90++)
- https://en.cppreference.com/w/cpp/keywords.html


### #include Preprocessor Directive
🧩  what is a preprocessor? 
It is a tool that runs before the compiler.
It prepares your code for compilation.
It does text manipulation, not real compiling.

🧩 What Does the Preprocessor Do?
- Reads all lines starting with #
- Includes header files
- Expands macros (#define)
- Removes comments
- Handles conditional code (#ifdef, etc.)
- Then it produces a new version of your source code, which gets compiled.


🧩 Preprocessor Directives start with #
- commands to the preprocessor

```cpp
#include <iostream>   // include a system/standard library headers. Compiler looks in system directories.
#include "myFile.h"   // include your own file. Compiler looks in current folder first, then system paths

#define PI 3.14       // define a macro

#ifdef DEBUG          // compile this only if DEBUG is defined
std::cout << "Debug mode\n";
#endif

#ifndef MY_HEADER     // header guard example
#define MY_HEADER
#endif

#pragma once          // another way to prevent double inclusion
```

### The main() function
- Every C++ program must have exactly 1 main() function
- Starting point of program execution 
- return 0 indicates successful program execution 
- 2 versions that are both valid
```cpp
/*  
    version1 ... simple one
    Useage: program.exe
*/

int main(){
    //code 
    return 0;
}
```

```cpp
/*
    version2 ... for cli tools
    Useage: program.exe argument1 argument2
*/

int main(int argc, char *argv[])
{
    //code
    return 0;
}
```

### Namespaces
What is a namespace?
A namespace groups related code together to avoid name conflicts between functions, variables, and classes.
Why use it?
Prevents clashes when two libraries have the same function name
Organizes code into logical module

```cpp
namespace Math {
    int add(int a, int b) {
        return a + b;
    }
}

int result = Math::add(3, 4);
```

```cpp
using namespace std; // bring the entire std library in
using std::cout; // more specific ( better )

int main(){
    cout << "Enter your namespace"
}
```

### Basic I/O
```
int main() {
    cout << "Hello World!";
    cin >> age;
    cout << "You are " << age << " years old.";
    return 0;
}
```