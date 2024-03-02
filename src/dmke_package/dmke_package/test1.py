import time

def save_integer_to_file(number, file_path):
    if not isinstance(number, int):
        raise ValueError("Input must be an integer.")

    with open(file_path, 'w') as file:
        file.write(str(number))

def read_integer_from_file(file_path):
    with open(file_path, 'r') as file:
        data = file.read().strip()
        try:
            return int(data)
        except ValueError:
            raise ValueError("File does not contain a valid integer.")


# Usage
try:
    # x = read_integer_from_file('data.txt')
    # print(x)
    # time.sleep(1)

    number = 48  # Your integer value
    save_integer_to_file(number, 'data.txt')
    time.sleep(1)
    a=read_integer_from_file('data.txt')
    print(a)
    print("Integer 1 saved successfully!")

    time.sleep(1)
    number = 88
    save_integer_to_file(number, 'data.txt')
    b=read_integer_from_file('data.txt')
    print(b)
    print("Integer 2 saved successfully!")
    # time.sleep(1)
    # number = 48
    # save_integer_to_file(number, 'data.txt')
    # print("Integer saved successfully!")
    # c = read_integer_from_file('dmke_encoder_pos.txt')
    # print(c+b)
except ValueError as e:
    print("Error:", e)