# 🐍 Python Basic 

```{note}
The only way to learn a new programming language is by writing programs in it. - Dennis Ritchie (inventor of C programming language)
```

## Running Python

### Running Python in VS Code

1. To open the `ece487_wksp` folder in VS Code, right-click on it and choose `Open with Code` from the menu.
1. To open the `Terminal` window, go to View and click on `Terminal`. 
1. Type in `python` and press `Enter`
1. This will launch the Python Shell, where you can input code and see the output. The Shell acts as a sandbox, allowing you to try out code snippets and get immediate feedback. For example, in the Shell, type `2+3` and press Enter, and it will display `5` on the next line.
1. Try the following code in the Shell:
    ```Python
    2+3
    "hello"+" world"
    a = 2
    print(a)
    ```
1. The Python Shell behaves like a calculator, allowing you to perform various mathematical operations.  
1. Now, try the follownig code:
    ```Python
    2+2    # add two integers
    2+2.0  # add an integer to a floating point number
    2*3    # 2 times 3
    2**2   # 2 squared
    2**3   # 2 cubed
    2**128 # Unlike C, integers in Python does not have a size limitation.    
    2/3    # It returns 1 in C, but 0.6666666666666 in Pyton 
    2/3.0  # Same as 2/3 in Python.
    2//3   # modulo (remainder), same as 2%3 in C
    4//3   # modulo (remainder), same as 4%3 in C
    ```
1. To exit the Python Shell, type `exit()` and press `Enter`, or use `Ctrl+z` and press `Enter`.

### Running Python in Command Prompt

1. Open Command Prompt or PowerShell (or Terminal on Mac and Ubuntu), and run `python` on Windows or `python3` on Mac. 
1. This will open the Python Shell. 


## Python Programming

### Variables
- Numeric types: `int`, `float`, `long`, `complex`
- String: `str`
- Boolean: `bool` (`True`/`False`)
    
Python's simple types are summarized in the following table:

**<center>Python Scalar Types</center>**

| Type        | Example        | Description                                                  |
|-------------|----------------|--------------------------------------------------------------|
| ``str``     | ``x = 'abc'``  | String: characters or text                                   | 
| ``int``     | ``x = 1``      | integers (whole numbers)                                     |
| ``float``   | ``x = 1.0``    | floating-point numbers (real numbers)               |                  
| ``complex`` | ``x = 1 + 2j`` | Complex numbers (real and imaginary part) |
| ``bool``    | ``x = True``   | Boolean: `True`/`False` values                              | 
| ``NoneType``| ``x = None``   | Special object indicating nulls                              | 

- Use the `type()` function to find the type for a value or variable

    ```python
    # String
    c = 'hello'
    print(type(c))

    # Integer
    a = 1
    print(type(a))

    # Float
    b = 1.0
    print(type(b))

    # Boolean
    d = True
    print(type(d))

    # None
    e = None
    print(type(e))

    # Cast integer to string
    print(type(str(a)))

    ```

### Math Operators
- `+`, `-`, `*`, and `/`
- Exponentiation: `**`
- Modulo (Remainder): `%`

### Logic Operators

    ```python
    x == y # x is equal to y
    x != y # x is not equal to y
    x > y # x is greater than y
    x < y # x is less than y
    x >= y # x is greater than or equal to y 
    x <= y # x is less than or equal to y
    ```

- Try the following code in Python Console.

    ```python
    2+2
    2+2.0
    2*3
    2**3   # exponentiation 
    2**128 
    2/3
    2/3.0
    2//3   # integer division
    4//3   
    2%3    # modulo
    4%3
    bin(5|2) # convert a number to binary after bit operations
    bin(5^1)
    bin(5&1)
    a = 6
    b = 7
    print(a==6)   # boolean expressions
    print(a==7)
    print(a==6 and b==7)
    print(a==6 or b==6)
    print(not a==6 and b==6)
    print(not (a==6 and b==6))
    ```

### Working with Variables
1. In Python, variables store values like strings or numbers. Strings are blocks of text, such as `"Stan Baek is the legendary pirate captain."` Numbers can be integers or decimals.
1. Try the the following code in Python Console.
    ```python
    # a string variable
    name = "Stan Baek"
    ```
1. You can check the type in use by issuing the type() command
    ```python
    type(name)
    ```
1. It will return `<class 'str'>`
1. Add another string variable: 
    ```python
    title = "the legendary pirate captain"
    ```
1. You can concatenate strings using `+` operator:
    ```python
    character = name + ": " + title
    print(character)  # It will print "Stan Baek: the legendary pirate captain"
    ```
1. Now, try combining a string with an integer:
    ```python
    # Integer variable
    age = 726
    # This will raise an error
    print(character + age)
    ```
1. Convert the integer to a string for proper concatenation:    
    ```python
    print(character + " is " + str(age) + " years old.")
    ```
1. This will print 
    ```bash
    Stan Baek: the legendary pirate captain is 726 years old.
    ```
1. You can also request input from the user: 
    ```python
    age = input("How old are you? ")
    print(type(age))

    # Since age is a string variable. We need to convert it to integer.
    age = int(age)

    # Floating-point numbers
    x = 3.141
    print(type(x))
    print(x)  # Prints 3.141
    print(int(x))  # Prints 3
    ```
1. When creating variables, remember that single or double quotes can be used for strings, but be consistent with your choice.

### Functions and Modules
1. Python has many built-in functions. For example:
    ```python
    name = "Stan Baek"
    len(name)  # Returns the lenght of the string
    type(name) # Returns the type of the variable
    ```
1. To explore more built-in functions, visit [Python's built-in functions documentation](https://docs.python.org/3.8/library/functions.html). You can also use the `help()` function to get descriptions:
    ```python
    help(len)
    ```
1. You can define your own functions in Python:
    ```python
    def add(a, b):
        return a+b
    print(add(3, 4))  # Outputs 7
    ```

### Python Modules
1. You can add extra functionality to Python by importing modules. Modules are collections of additional functions built by others. For example, to access advanced mathematical functions, you can import the `math` module:
    ```python
    import math
    print(math.sqrt(16))    # Outputs 4.0
    print(math.sin(math.pi/2))  # Outputs 1.0
    print(math.cos(0))  # Outputs 1.0
    ```
1. In Python, it's a convention to import all required modules at the top of the script. This ensures all dependencies are loaded before they're used and makes the code easier to understand. You can also import specific functions from a module:
    ```python
    from math import sqrt, sin, pi
    print(sqrt(16))
    print(sin(pi/2))
    ```
1. We can also import individual functions from a module
    ```python
    from math import sqrt, sin, pi
    print(sqrt(16))
    print(sin(pi/2))
    cos(0)   # error because cos was not imported
    ```
1. You can also use an alias for a module:
    ```python
    import math as m
    m.sqrt(16)
    m.sin(m.pi/2)
    ```

### Conditional Statements

1. The `if`, `else`, and `elif` statements control the flow of execution based on conditions. The basic syntax is:
    ```Python
    if expression:
        statements
    elif expression:
        statements
    else:
        statements
    ```
1. If no action is needed for a certain condition, you can use the `pass` statement:
    ```Python
    if expression:
        pass       # Do nothing.
    else:
        statements
    ```
1. Example:
    ```Python
    word = input("Enter a four-letter word: ")
    if len(word) == 4:
        print(word, "is a four-letter word. Well done.")
    elif len(word) == 3:
        print(word, "is a three-letter word.")
    else:
        print(word, "is not a four-letter word.")
    ```

### Loops
1. Let's start with a simple `while` statement.
    ```Python
    x = 1
    while x < 5:
        print(x)
        x = x + 1
    ```
1. We can use `break` to exit a loop:
    ```Python
    while True:
        print(x)
        x -= 1
        if x == 0:
            break
    ```
1. The `for` loop is commonly used in Python:
    ```Python
    for i in [0, 1, 2, 3, 4]:
        print(i)
    ```
1. We can also use `range()`:
    ```Python
    for i in range(5):  # Loops through 0 to 4
        print(i)
    ```
