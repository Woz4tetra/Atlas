# -*- coding: cp1252 -*-
import time
#import numpy

#-*- coding: cp1252 -*-
output ='''Latitude: 40.44167°
Longitude: -79.94172°
Latitude: 40.44137°
Longitude: -79.94176°
Latitude: 40.44112°
Longitude: -79.94187°
Latitude: 40.44091°
Longitude: -79.94196°
Latitude: 40.44069°
Longitude: -79.94205°
Latitude: 40.44045°
Longitude: -79.94219°
Latitude: 40.44023°
Longitude: -79.94236°
Latitude: 40.44018°
Longitude: -79.94263°
Latitude: 40.44012°
Longitude: -79.94281°
Latitude: 40.44008°
Longitude: -79.94305°
Latitude: 40.43995°
Longitude: -79.94321°
Latitude: 40.43985°
Longitude: -79.94352°
Latitude: 40.43961°
Longitude: -79.94361°
Latitude: 40.43957°
Longitude: -79.94367°
Latitude: 40.43948°
Longitude: -79.94388°
Latitude: 40.43938°
Longitude: -79.94395°
Latitude: 40.43927°
Longitude: -79.944°
Latitude: 40.43918°
Longitude: -79.94404°
Latitude: 40.43912°
Longitude: -79.94411°
Latitude: 40.43906°
Longitude: -79.94416°
Latitude: 40.439°
Longitude: -79.9444°
Latitude: 40.43891°
Longitude: -79.94446°
Latitude: 40.4389°
Longitude: -79.94457°
Latitude: 40.43886°
Longitude: -79.94465°
Latitude: 40.43882°
Longitude: -79.94475°
Latitude: 40.43882°
Longitude: -79.94483°
Latitude: 40.4388°
Longitude: -79.94493°
Latitude: 40.43873°
Longitude: -79.94516°
Latitude: 40.4387°
Longitude: -79.94547°
Latitude: 40.43868°
Longitude: -79.94577°
Latitude: 40.43872°
Longitude: -79.94595°
Latitude: 40.43881°
Longitude: -79.94613°
Latitude: 40.43887°
Longitude: -79.94628°
Latitude: 40.43903°
Longitude: -79.94643°
Latitude: 40.43916°
Longitude: -79.94651°
Latitude: 40.43929°
Longitude: -79.94655°
Latitude: 40.43943°
Longitude: -79.94661°
Latitude: 40.43951°
Longitude: -79.94671°
Latitude: 40.43956°
Longitude: -79.9468°
Latitude: 40.43968°
Longitude: -79.94692°
Latitude: 40.43984°
Longitude: -79.94708°
Latitude: 40.43995°
Longitude: -79.94714°
Latitude: 40.44009°
Longitude: -79.94731°
Latitude: 40.44024°
Longitude: -79.94751°
Latitude: 40.44037°
Longitude: -79.94771°
Latitude: 40.44052°
Longitude: -79.94791°
Latitude: 40.44062°
Longitude: -79.94796°
Latitude: 40.44073°
Longitude: -79.948°
Latitude: 40.44085°
Longitude: -79.94796°
Latitude: 40.44099°
Longitude: -79.94791°
Latitude: 40.44109°
Longitude: -79.94779°
Latitude: 40.44122°
Longitude: -79.94759°
Latitude: 40.44136°
Longitude: -79.94741°
Latitude: 40.4415°
Longitude: -79.9472°
Latitude: 40.44149°
Longitude: -79.947°
Latitude: 40.44148°
Longitude: -79.94679°
Latitude: 40.44144°
Longitude: -79.94659°
Latitude: 40.4414°
Longitude: -79.9463°
Latitude: 40.44132°
Longitude: -79.946°
Latitude: 40.44127°
Longitude: -79.94575°
Latitude: 40.4412°
Longitude: -79.94547°
Latitude: 40.44114°
Longitude: -79.9451°
Latitude: 40.4411°
Longitude: -79.9448°
Latitude: 40.44104°
Longitude: -79.94454°
Latitude: 40.44098°
Longitude: -79.94428°
Latitude: 40.44094°
Longitude: -79.94408°
Latitude: 40.44098°
Longitude: -79.94378°
Latitude: 40.4409°
Longitude: -79.94352°
Latitude: 40.4408°
Longitude: -79.94325°
Latitude: 40.44065°
Longitude: -79.94299°
Latitude: 40.44058°
Longitude: -79.94267°
Latitude: 40.44057°
Longitude: -79.94256°'''

def parse(data):
    new_data = data.split("°")
    count = 0
    points = []
    while count < len(new_data)-2:
        lat = new_data[count]
        latL, latR = lat.split(" ")
        count+=1
        lng = new_data[count]
        lngL, lngR = lng.split(" ")
        count += 1
        points += [(float(latR),float(lngR))]
    return points
    

#parse(output)

def close(bind, pos, acc):
    #take a bind spot and position from gps
    #and see if they are close enough to be correct
    dist_lat = abs(float(bind[0])-pos[0])
    dist_lng = abs(float(bind[1])-pos[1])
    dist = ((dist_lat**2) + (dist_lng**2)**(0.5))
    #print dist
    if dist > acc:
        return False
    elif dist <= acc:
        return True
    

def bind(track, pos, acc, prev_bind = 0):
    #REQUIRES: track is a list of tuple (lat, lng)
    #REQUIRES: pos = (lat,lng) are within (acc) units of a bind point
    #REQUIRES prev_bind is in boundary of track
    #ENSURES: will find closest bind point and return position of it and #
    #print track
    print len(track), "bind points"
    if prev_bind >= len(track) or prev_bind <0:
        raise Exception("bind outside track")
    
    for i in xrange(prev_bind,len(track)):
        if close(track[i],pos,acc) == True:
            return (track[i], i)

    for i in xrange(len(track)):
        if close(track[i],pos,acc) == True:
            return (track[i], i)

    return 


def test(string):
    track = parse(string)
    t1 = time.time()
    pos = (40.43881,-79.94446)
    acc = 0.0000002
    (new_pos, num_bind) = bind(track, pos, acc)
    t2 = time.time()
    print new_pos, num_bind, "new_pos and num_bind"
    print t2-t1
test(output)
