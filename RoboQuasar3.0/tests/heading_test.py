import sys
from math import pi
sys.path.insert(0, "../")

from analyzers.heading_kalman import HeadingFilter

f1 = HeadingFilter()

#first check without flags
print("CHECKING WITH ALL is")
for i in range(10):
    heading = f1.update(i, True, i,True, 1, True)
    print(heading)

f1 = HeadingFilter()
print("\nCHECKING WITH IMU = 0")
for i in range(10):
    heading = f1.update(i, True, i,True, 0, True)
    print(heading)

print("\nCHECKING WITH BIND_FLAG = FALSE")
f1 = HeadingFilter()
for i in range(10):
    heading = f1.update(i, True, i,False, 1, True)
    print(heading)
print("\nNow update with all flags true")
heading = f1.update(i,True, i,True, 1, True)
print(heading)

print("\nCHECKING WITH GPS_FLAG = FALSE")
f1 = HeadingFilter()
for i in range(10):
    heading = f1.update(i, False, i,True, 1, True)
    print(heading)
print("\nNow update with all flags true")
heading = f1.update(i,True, i,True, 1, True)
print(heading)

print("\nCHECKING WITH IMU_FLAG = FALSE")
f1 = HeadingFilter()
for i in range(10):
    heading = f1.update(i, True, i,True, 1, False)
    print(heading)
print("\nNow update with all flags true")
heading = f1.update(i,True, i,True, 1, True)
print(heading)
