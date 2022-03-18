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
    
    delta_t = 10
    standard_deviation_o = 100
    acceleration_g = 0.3
    acceleration_m_squared_sec = 3
    standard_deviation_p = 1.5
    variance_position = 0.25*(delta_t**4)*standard_deviation_p**2
    variance_velocities = (delta_t**2)*(standard_deviation_p**2)
    co_variance_matching_pair = 0.5*(delta_t**3)*(standard_deviation_p**2)
    
    size4 = (4,4)
    size2 = (2,2)
    F = np.identity(4)
    F[0][2] , F[1][3] = delta_t , delta_t
    print(f"F =\n {F}")
    
    Q = np.zeros(size4)
    Q[0][0] , Q[2][2] = (0.25*delta_t**4) , (0.25*delta_t**4)
    Q[0][1] , Q[1][0] , Q[2][3] , Q[3][2] = 0.5 * delta_t **3 , 0.5 * delta_t**3 , 0.5 * delta_t **3 , 0.5 * delta_t**3
    Q[1][1] , Q[3][3] = delta_t**2 , delta_t**2
    print(f"Q =\n {Q}")
    
    R = np.zeros(size2)
    R[0][0] , R[1][1] = co_variance_matching_pair , co_variance_matching_pair
    print(f"R =\n {R}")
    H = np.zeros((2,4))
    H[0][0] , H[1][2] = 1 , 1
    print(f"H =\n {H}")
    

    parsedFlights = []
    flights = get_ground_truth_data()
    # print(flights)
    # print("\n")
    # print("\n")
    for key,value  in flights.items() :
        parsedFlights.append([key,value])
    
    print(parsedFlights[0])
    print(parsedFlights[0][1].data)
    # print("\n")
    # print("\n")    
    exit()
    
    parsedRadarData = []
    radarData = get_radar_data(flights)
    for key,value  in radarData.items() :
        parsedRadarData.append([key,value])
    
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
    
    plt.rcParams["figure.autolayout"] = True
    plt.style.context("traffic")
    fig, ((ax1, ax2),(ax3,ax4)) = plt.subplots(2, 2 , subplot_kw = dict(projection=Mercator()) )
    fig.suptitle('Ground truth data flight visualisation')
    ax1.add_feature(countries())
    ax1.set_title("Ground truth data of flight " + str(parsedFlights[5][0]) )
    ax1.axis("equal")
    ax2.add_feature(countries())
    ax2.set_title("Ground truth data of flight " + str(parsedFlights[10][0]) )
    ax2.axis("equal")
    ax3.add_feature(countries())
    ax3.set_title("Ground truth data of flight " + str(parsedFlights[15][0]) )
    ax3.axis("equal")
    ax4.add_feature(countries())
    ax4.set_title("Ground truth data of flight " + str(parsedFlights[20][0]) )
    ax4.axis("equal")
    parsedFlights[5][1].plot(ax1, color="green")
    parsedFlights[10][1].plot(ax2, color="red")
    parsedFlights[15][1].plot(ax3, color="blue")
    parsedFlights[20][1].plot(ax4, color="orange")
    plt.show()
        
    fig, ((ax1, ax2),(ax3,ax4)) = plt.subplots(2, 2 , subplot_kw = dict(projection=Mercator()) )
    fig.suptitle('Radar data flight visualisation')
    ax1.add_feature(countries())
    ax1.set_title(" Radar data of flight " + str(parsedRadarData[5][0]) )
    ax1.axis("equal")
    ax2.add_feature(countries())
    ax2.set_title(" Radar data of flight " + str(parsedRadarData[10][0]) )
    ax2.axis("equal")
    ax3.add_feature(countries())
    ax3.set_title(" Radar data of flight " + str(parsedRadarData[15][0]) )
    ax3.axis("equal")
    ax4.add_feature(countries())
    ax4.set_title(" Radar data of flight " + str(parsedRadarData[20][0]) )
    ax4.axis("equal")
    parsedRadarData[5][1].plot(ax1, color="green")
    parsedRadarData[10][1].plot(ax2, color="red")
    parsedRadarData[15][1].plot(ax3, color="blue")
    parsedRadarData[20][1].plot(ax4, color="orange")
    plt.show()
    
    fig, ((ax1, ax2),(ax3,ax4)) = plt.subplots(2, 2 , subplot_kw = dict(projection=Mercator()) )
    fig.suptitle('Comparison of ground truth data and radar data')
    ax1.add_feature(countries())
    ax1.set_title(" Ground truth data of flight " + str(parsedFlights[7][0]) )
    ax1.axis("equal")
    ax2.add_feature(countries())
    ax2.set_title(" Radar data of flight " + str(parsedRadarData[7][0]) )
    ax2.axis("equal")
    ax3.add_feature(countries())
    ax3.set_title(" Ground truth data of flight " + str(parsedFlights[13][0]) )
    ax3.axis("equal")
    ax4.add_feature(countries())
    ax4.set_title(" Radar data of flight " + str(parsedRadarData[13][0]) )
    ax4.axis("equal")
    parsedFlights[7][1].plot(ax1, color="red")
    parsedRadarData[7][1].plot(ax2, color="red")
    parsedFlights[13][1].plot(ax3, color="orange")
    parsedRadarData[13][1].plot(ax4, color="orange")
    
    fig.tight_layout()
    
    plt.show()
    
    pass


#############################

if __name__ == "__main__":
    main()