import sys
import math

sys.path.insert(0,"../")

from analyzers.converter import HeadingConverter
from analyzers.converter import PositionConverter

mes = """

First check the heading filter, giving it values that are in the same
direction, then the same point, then moving points

"""

c1 = HeadingConverter(initial_lng = 0, initial_lat = 0)

#have it move in the same direction repeatedly
for i in range(1,5):
    gps_heading, bind_heading = (
        c1.convert(longitude = i, latitude = i, bind_x = i, bind_y = i))
    print("c1\tgps heading: %f\tbind_heading: %f"
            %(gps_heading, bind_heading))
    assert(gps_heading == math.pi/4)
    assert(bind_heading == math.pi/4)

#test edge case of if the values are the same

c2 = HeadingConverter(initial_lng = 0, initial_lat = 0)

for j in range(5):
    gps_heading, bind_heading = (
        c2.convert(1,1,1,1))
    print("c2\tgps heading: %f\tbind_heading: %f"
            %(gps_heading, bind_heading))
    #What should ehadings be in this case? TODO make it same as prev one

c3 = HeadingConverter(initial_lng = 0, initial_lat = 0)

#now check if the values are different
for k in range(5):
    gps_heading, bind_heading = (
        c3.convert(longitude = k, latitude = 5-k, bind_x = k, bind_y = 5-k))
    print("c3\tgps heading: %f\tbind_heading: %f"
            %(gps_heading, bind_heading))


mes = """

Now check the Position Converter

"""
#first, check the gps cooridinates
p1 = PositionConverter(initial_encoder = 0, origin_lng = 0, origin_lat = 0)

for i in range(10):
    x, y, ax, ay, enc_dist = \
        p1.convert(longitude = 0.001*i, latitude = 0.001*(9-i),
            ax = 0, ay = 0, encoder_counts = 0, heading = math.pi/4)
    print("x: %f\ty: %f\tax: %f\tay: %f\tenc_dist: %f"
        %(x,y,ax,ay,enc_dist))
    assert(ax == 0)
    assert(ay == 0)

p2 = PositionConverter(initial_encoder = 0, origin_lng = 0, origin_lat = 0)

#now check the acceleration conversion
for j in range(8):
    x, y, ax, ay, enc_dist = \
        p1.convert(longitude = 0, latitude = 0,
            ax = 10, ay = 10, encoder_counts = 0, heading = math.pi/4 * j)
    print("ax = %f\tay = %f\theading = %f" %(ax, ay, math.pi / 4 * j))
