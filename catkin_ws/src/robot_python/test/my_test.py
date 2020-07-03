import numpy as np
matrix = np.eye(4)
matrix = np.array(matrix)
m = len(matrix[:, 0])
n = len(matrix[0, :])
a = []
for i in range(m/2):
    k2 = len(matrix[:, 0])
    k1 = len(matrix[0, :])
    for j in range(k1):
        a.append(matrix[0, j])
    for j in range(k2 - 1):
        a.append(matrix[j + 1, -1])
    for j in range(k1-1):
        a.append(matrix[-1, k1-1-1 - j])
    for j in range(k2 - 1):
        a.append(matrix[k2 - 1 -1 -j, 0])
    matrix = matrix[1:-1,1:-1]

print a

numbers = [1, 2234, 3, 4, 32, 13]
number_str = []
n = len(numbers)
for i in range(n):
    number_str.append(str(numbers[i]))
number_str.sort()
str1 = ""
for i in range(n):
    str1 = str1 + number_str[i]

print "str1", str1