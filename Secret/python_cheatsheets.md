# Python Cheatsheet — Quick Reference

> Target: Python 3.10+ (works for 3.8+ unless noted). Copy‑paste friendly.

---

## Basics

```py
# Shebang (scripts)
#!/usr/bin/env python3

# Run a file
#  $ python main.py

# REPL tips
#  >>> help(obj)   # built-in docs
#  >>> dir(obj)    # attributes
```

## Variables & Types

```py
x: int = 42
pi: float = 3.14159
name: str = "Kenny"
flag: bool = True
items: list[int] = [1, 2, 3]
coord: tuple[int, int] = (10, 20)
unique: set[int] = {1, 2, 3}
meta: dict[str, int] = {"a": 1}
none_val = None
```

## Strings

```py
s = "hello"
multiline = """line1\nline2"""
# f-strings
user = "kenny"; score = 99
msg = f"{user=}, {score:.2f}"
# methods
s.upper(); s.lower(); s.title(); s.strip(); s.replace("h", "H")
"abc".startswith("a"); "abc".endswith("c"); "a,b".split(","); ",".join(["a","b"])
```

## Collections

```py
# List
nums = [1, 2, 3]
nums.append(4); nums.extend([5, 6])
nums.insert(0, 0); nums.remove(2)
nums.pop()     # -> 6
nums.sort()    # in-place
sorted_nums = sorted(nums, reverse=True)

# Dict
user = {"name": "Kenny", "age": 30}
user["role"] = "SE"
user.get("age", 0)
for k, v in user.items(): ...

# Set
s = {1, 2, 3}
s.add(4); s.discard(2)
s1 | s2   # union
s1 & s2   # intersection
s1 - s2   # difference
```

## Control Flow

```py
if cond:
    ...
elif other:
    ...
else:
    ...

# Ternary
label = "odd" if n % 2 else "even"

# Match (3.10+)
match event:
    case {"type": "click", "x": x, "y": y}:
        ...
    case _:
        ...
```

## Loops

```py
for i in range(5): ...
for i, v in enumerate(["a","b"]): ...
for k, v in {"a":1}.items(): ...

while cond:
    ...
    if stop: break
else:
    # runs if no break
    ...
```

## Comprehensions

```py
squares = [x*x for x in range(10) if x%2==0]
lookup  = {c: ord(c) for c in "abc"}
unique  = {x%3 for x in range(10)}

def gen():
    yield from (x*x for x in range(3))
```

## Functions

```py
def greet(name: str, /, lang: str = "en", *args, **kwargs) -> str:
    return f"hi {name}"

# Lambda
key_fn = lambda p: (p.age, p.name)

# Docstring
def add(a: int, b: int) -> int:
    """Add two numbers."""
    return a + b
```

## Exceptions

```py
try:
    risky()
except ValueError as e:
    print(e)
except (TypeError, KeyError):
    ...
else:
    print("no errors")
finally:
    cleanup()

# Raise
raise RuntimeError("boom")
```

## With / Context Managers

```py
from pathlib import Path
p = Path("data.txt")
with p.open("w", encoding="utf-8") as f:
    f.write("hello")
```

## Classes & Dataclasses

```py
from dataclasses import dataclass

@dataclass(slots=True)
class User:
    id: int
    name: str
    admin: bool = False

class Counter:
    def __init__(self, start=0):
        self.n = start
    def inc(self):
        self.n += 1
    def __repr__(self):
        return f"Counter(n={self.n})"
```

## Iterators & Generators

```py
def read_lines(path: str):
    with open(path) as f:
        for line in f:
            yield line.rstrip("\n")

class UpTo:
    def __init__(self, n): self.n = n
    def __iter__(self):
        i = 0
        while i < self.n:
            yield i; i += 1
```

## Decorators

```py
import functools

def memo(fn):
    cache = {}
    @functools.wraps(fn)
    def wrapper(*a, **k):
        key = (a, tuple(sorted(k.items())))
        if key not in cache:
            cache[key] = fn(*a, **k)
        return cache[key]
    return wrapper
```

## Typing (PEP 484/PEP 604)

```py
from typing import Iterable, Optional

def total(xs: Iterable[int]) -> int: ...
maybe_num: int | None = None      # PEP 604
```

## Files & Paths

```py
from pathlib import Path
root = Path.cwd()
for p in root.glob("**/*.py"):
    print(p.name)
```

## Date & Time

```py
from datetime import datetime, timedelta, timezone
now = datetime.now(timezone.utc)
one_hour = now + timedelta(hours=1)
now.isoformat()
```

## Regex

```py
import re
m = re.search(r"(\w+)@(\w+)\.com", "a@b.com")
if m:
    user, host = m.groups()
```

## Logging & Debugging

```py
import logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)
log.info("hello %s", "world")

# Debug
#  import pdb; pdb.set_trace()
```

## Virtual Envs & Packages

```sh
# Create & activate venv
python -m venv .venv
source .venv/bin/activate    # Windows: .venv\\Scripts\\activate

# Upgrade pip & install
python -m pip install --upgrade pip
pip install requests==2.*

# Freeze
pip freeze > requirements.txt
```

## Testing (pytest)

```sh
pip install pytest
pytest -q
```

```py
# tests/test_sample.py
import pytest

def test_add():
    assert 1 + 1 == 2
```

## AsyncIO (essentials)

```py
import asyncio

async def work(i):
    await asyncio.sleep(0.1)
    return i

async def main():
    results = await asyncio.gather(*(work(i) for i in range(3)))
    print(results)

asyncio.run(main())
```

## Common Builtins

```py
len(), sum(), min(), max(), any(), all(), map(), filter(), zip(), enumerate(), range()
```

## Handy One‑Liners

```py
# Read file lines to list
lines = Path("f.txt").read_text(encoding="utf-8").splitlines()

# Chunk list
def chunks(xs, n): return [xs[i:i+n] for i in range(0, len(xs), n)]

# Flatten 2D list
flat = [y for x in matrix for y in x]

# Counter
from collections import Counter
c = Counter("abracadabra")
```

## Packaging (minimal)

```
project/
├─ pyproject.toml   # [build-system], [project]
└─ pkg/
   └─ __init__.py
```

```toml
# pyproject.toml (minimal)
[build-system]
requires = ["setuptools", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "pkg"
version = "0.1.0"
```

## Performance Tips

* Use comprehensions over `for` + `append` when simple.
* Prefer `set`/`dict` lookups (O(1)) over lists (O(n)).
* For large loops: consider `itertools`, `collections`, `functools`.

---

### Quick References

* Docs: [https://docs.python.org/3/](https://docs.python.org/3/)
* PEP 8 style: [https://peps.python.org/pep-0008/](https://peps.python.org/pep-0008/)
* Typing: [https://docs.python.org/3/library/typing.html](https://docs.python.org/3/library/typing.html)
