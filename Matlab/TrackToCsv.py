
import json
from utm import utmconv
from hermite import cubic_hermite_spline
import math

uc = utmconv()
utmData=[]
stepWidth=0.4
samplingWidth = 0.01

utmDataFixedStepWidth=[]
utmDataSamplingWidth = []
# fileName = '../FinalTrack/FinalTrack'
fileName = 'Crossing/Crossing'
with open(fileName+'.plan') as json_file:
    jsonFile = json.load(json_file)
    #add path
    utmX=0
    utmY=0
    for p in jsonFile['mission']['items']:
        lat = p['params'][4]
        lon = p['params'][5]
        res = uc.geodetic_to_utm(lat, lon)
        utmX =float(res[3])% 5900
        utmY =float(res[4])%5800
        utmData.append((utmX, utmY))


for i in range(1, len(utmData)):
    x0=utmData[i-1][0]
    x1=utmData[i][0]
    y0= utmData[i-1][1]
    y1=utmData[i][1]
    norm = int(math.sqrt((x1-x0)**2+(y1-y0)**2))
    interpolatedSteps = int(norm/samplingWidth)
    if(interpolatedSteps<=1):
        print ("Too close... this could cause a fail in mpc")
    if(interpolatedSteps>0):
        direction = [(x1-x0)/interpolatedSteps, (y1-y0)/interpolatedSteps]
        for i in range(1, interpolatedSteps):
            utmDataSamplingWidth.append((x0+(direction[0]*i), y0+(direction[1]*i)))

#print 
f = open(fileName+".csv", "w")
lastPoint = utmDataSamplingWidth[0]
for point in utmDataSamplingWidth:
    if(math.sqrt((lastPoint[0]-point[0])**2+(lastPoint[1]-point[1])**2))>stepWidth:
        f.write("{0},{1},{2},{3},{4},{5} \r".format(lastPoint[0]-1, lastPoint[1]-1,lastPoint[0]+1, lastPoint[1]+1,lastPoint[0], lastPoint[1]))
        # print ("Distance: {0}\r".format(math.sqrt((lastPoint[0]-point[0])**2+(lastPoint[1]-point[1])**2)))
        lastPoint=point
        
f.close