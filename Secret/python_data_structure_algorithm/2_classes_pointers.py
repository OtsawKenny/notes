# Classes
class Cookie:
    def __init__(self, color: str):
        self.color = color

    def get_color(self) -> str:
        return self.color

    def set_color(self, color: str) -> None:
        self.color = color


cookie_one = Cookie('green')
cookie_two = Cookie('blue')
print("Cookie one is", cookie_one.get_color())
print("Cookie two is", cookie_two.get_color())

# Pointer
num1 = 11 # Integers are Immutable
num2 = num1 

print("num1 is", num1)
print("num2 is", num2)
print("\nnum1 points to:",id(num1))
print("num2 points to", id(num2))

num2 = 22
print("num1 is", num1)
print("num2 is", num2)
print("\nnum1 points to:",id(num1))
print("num2 points to", id(num2))

dict1 = {'value':11}
dict2 = dict1
print("Before value is updated:")
print("dict1=", dict1)
print("dict2=", dict2)

print("pointer dict1=", id(dict2))
print("pointer dict2=", id(dict2))

dict2['value']=22
print("Before value is updated:")
print("dict1=", dict1)
print("dict2=", dict2)

print("pointer dict1=", id(dict2))
print("pointer dict2=", id(dict2))

