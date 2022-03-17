#!/usr/bin/env python


from project2_base import *
from filterpy.kalman import KalmanFilter
from traffic.drawing import countries
from traffic.core.projection import Mercator
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.dates import DateFormatter




#############################


def main():
    # TODO: put your code here

    parsedFlights = []
    flights = get_ground_truth_data()
    #print(flights)
    print("\n")
    print("\n")
    for key,value  in flights.items() :
        parsedFlights.append([key,value])
    
    print(parsedFlights)
    print("\n")
    print("\n")    
    
    # radarData = get_radar_data(flights)
    # print(radarData)
    
    # ax = plt.axes(projection=Mercator())
    # ax.add_feature(countries())
    # bx = plt.axes(projection=Mercator())
    # bx.add_feature(countries())
    # plt.subplot(1,2,1)
    # parsedFlights[0][1].plot(ax, color="green")
    # plt.title("Flight " + str(parsedFlights[0][0]) )
    
    # plt.subplot(1,2,2)
    # parsedFlights[1][1].plot(bx, color="red")  
    # plt.title("Flight " + str(parsedFlights[1][0]) )
    
    # plt.show()
    
    fig, ((ax1, ax2),(ax3,ax4)) = plt.subplots(2, 2 , subplot_kw=dict(projection=Mercator()))
    fig.suptitle('Flight visualisation')
    ax1.add_feature(countries())
    ax1.set_title("Flight " + str(parsedFlights[0][0]) )
    ax2.add_feature(countries())
    ax2.set_title("Flight " + str(parsedFlights[1][0]) )
    ax3.add_feature(countries())
    ax3.set_title("Flight " + str(parsedFlights[2][0]) )
    ax4.add_feature(countries())
    ax4.set_title("Flight " + str(parsedFlights[3][0]) )
    parsedFlights[0][1].plot(ax1, color="green")
    parsedFlights[1][1].plot(ax2, color="red")
    parsedFlights[2][1].plot(ax3, color="blue")
    parsedFlights[3][1].plot(ax4, color="orange")
    
    plt.show()
    
    pass


#############################

if __name__ == "__main__":
    main()
