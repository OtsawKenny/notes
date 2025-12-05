## Character and String

- Character Function
- C-style strings
- Working with C-style strings
- C++ Strings
- Working with C++ Strings

### Character functions
| Function           | Meaning                                    |
| ------------------ | ------------------------------------------ |
| `std::isalnum(c)`  | Is letter **or** digit?                    |
| `std::isalpha(c)`  | Is alphabet letter? (A–Z or a–z)           |
| `std::isdigit(c)`  | Is digit? (0–9)                            |
| `std::islower(c)`  | Is lowercase letter?                       |
| `std::isupper(c)`  | Is uppercase letter?                       |
| `std::isspace(c)`  | Is whitespace? (space, tab, newline, etc.) |
| `std::ispunct(c)`  | Is punctuation? (e.g., ! , . ? ; :)        |
| `std::isprint(c)`  | Is printable?                              |
| `std::iscntrl(c)`  | Is control char? (e.g., newline, tab)      |
| `std::isxdigit(c)` | Is hexadecimal (0–9, A–F, a–f)?            |

```cpp
#include <cctype>
#include <iostream>
using namespace std;

int main() {
    char c = 'A';

    if (std::isalpha(c))
        cout << c << " is a letter\n";

    cout << "Lowercase: " << char(std::tolower(c)) << endl;
}

```

### C-Style Strings
Sequnce of characters
- Contiguous in memory
- implemented as an array of characters
- terminated by a null character (null)
- null termianted strings
string literal 

```cpp
// C++ is fun\0

char my_name[] {"Frank"};
my_name[5] = 'y'; // Problem , needs to be null terminated thus this is a problem

cin.get_name(full_name,50);

```
### C++ strings
A better alternative , so u dont have to worry about null-termianted problems

```cpp
#include <string>
using namespace std;

string s1;              // Empty
string s2 {"Frank"};    // Frank
string s3 {s2};         // Frank
string s4 {"Frank",3};  // Fra
string s5 {s3, 0 , 2};  // Fr
string s6 (3,'X');      // XXX

string sentence ;
sentence = part1 + " " + part2 + "language";
sentence = "C++" + " is powerful"; // Illegal , because both are treated as C-style literal , cant contcate c-style strings

string s1 {"Frank"};
for (char c: s1)
    size_t position = alphabet.find(c);
    cout << c << endl;

>> 70   // ascii code for F
>> 114  // ascii code for r
>> 97   //...

s1.substr(0,4)
s1.find("Fr")
```


