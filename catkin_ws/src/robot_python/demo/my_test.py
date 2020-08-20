#!/usr/bin/python
#-*-coding:utf-8-*-

M = 10
N = 10

matrix = []

mm = []
m = []
for i in range(M):
    for j in range(N):
        mm.append([i, j])
    matrix.append(mm)

list = []
while matrix:
    list += (matrix.pop(0))
    if matrix and matrix[0]:
        for row in matrix:
            list.append(row.pop())
    if matrix and matrix[0]:
        list += matrix.pop()[::-1]
    if matrix and matrix[0]:
        for row in matrix[::-1]:
            list.append(row.pop(0))

list1 = []
h = min(M, N)/2 + min(M, N)%2
s = 0

for i in range(h):
    if(M!=0 and N==0):
        l = M
    if (M == 0 and N!= 0):
        l = N
    l = (M+(N-2))*2
    for j in range(l):
        if ((j+1)%10==7 and (j+1)%100/10/2==1):
            list1.append(list[s + j])
    s = s + l
    M = M-2
    N= N-2
print list1