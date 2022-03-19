#!/usr/bin/env python

from re import X
from project2_base import *
from filterpy.kalman import KalmanFilter
from filterpy.kalman import predict , update
from traffic.drawing import countries
from traffic.core.projection import Mercator
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.dates import DateFormatter

#############################


def main():
    # TODO: put your code here
    delta_t = 10
    sigma0 = 100
    sigmaP = 1.5

    parsedFlights = []
    flights = get_ground_truth_data()
    for key,value  in flights.items() :
        parsedFlights.append([key,value])
    
    parsedRadarData = []
    radarData = get_radar_data(flights)
    for key,value  in radarData.items() :
        parsedRadarData.append([key,value])
    
    # print(f"Ground truth data for flight {parsedFlights[0][0]}")
    # print(parsedFlights[0][1].data)   
    # print(f"Radar data for flight {parsedRadarData[0][0]}")
    # print(parsedRadarData[0][1].data)       
    
    # flightNumber1 = int(input(f"You have {len(parsedFlights)} flights available to plot, enter the number of the first flight you want to plot (number range from 0 to {len(parsedFlights) -1 }) :  "))
    # flightNumber2= int(input(f"You have {len(parsedFlights)} flights available to plot, enter the number of the second flight you want to plot (number range from 0 to {len(parsedFlights) -1 }) :  "))
    # flightNumber3 = int(input(f"You have {len(parsedFlights)} flights available to plot, enter the number of the third flight you want to plot (number range from 0 to {len(parsedFlights) -1 }) :  "))
    # flightNumber4 = int(input(f"You have {len(parsedFlights)} flights available to plot, enter the number of the fourth flight you want to plot (number range from 0 to {len(parsedFlights) -1 }) :  "))
    # plot_flights(parsedFlights, parsedRadarData , flightNumber1,flightNumber2,flightNumber3,flightNumber4)
    
    print(parsedRadarData[10][1].data)
    print(kalman_filtering(parsedRadarData[10][1] , delta_t , sigmaP , sigma0).data)
    filteredFlight = kalman_filtering(parsedRadarData[10][1] , delta_t , sigmaP , sigma0)
    print(filteredFlight.data)
    plot_filtered_flight(parsedFlights[10] ,parsedRadarData[10] , filteredFlight )
    

#================
# Kalman Filter
#================
def kalman_filtering(flight, deltaT, sigma0, sigmap):
    kf = KalmanFilter(dim_x=4, dim_z=2)

    #State Transition matrix
    kf.F = np.array([[1,deltaT,0,0], [0,1,0,0],[0,0,1,deltaT],[0,0,0,1]])

    #State
    kf.x = np.zeros((4,1))

    #Measurement function
    kf.H = np.array([[1,0,0,0],[0,0,1,0]])

    #Covariance matrix for the porecess noise
    kf.Q = np.array([[0.25*deltaT**(4),0.5*deltaT**(3),0,0], [0.5*deltaT**(3),deltaT**(2),0,0],[0,0,0.25*deltaT**(4),0.5*deltaT**(3)],[0,0,0.5*deltaT**(3),deltaT**(2)]])*sigmap**(2)

    #Covariance matrix for the observation noise
    kf.R = np.array([[sigma0**(2),0],[0,sigma0**(2)]])

    #radar data
    flight_radar = get_radar_data_for_flight(flight)
    Xm=flight_radar.data.x
    Ym=flight_radar.data.y
    
    # run the kalman filter and store the results
    xs, cov = [], []
    for i in range(len(flight_radar.data)):
        z=np.array([[Xm[i]],[Ym[i]]])
        kf.predict()
        kf.update(z)
        xs.append(kf.x)
        flight_radar.data.at[i, "x"]= kf.x[0]
        flight_radar.data.at[i, "y"]= kf.x[2]
        cov.append(kf.P)
    xs, cov = np.array(xs), np.array(cov)
    flight_radar_filtered=set_lat_lon_from_x_y(flight_radar)
    return flight_radar_filtered
            
def plot_filtered_flight(flight , radarFlight, filtered_flight):
    # print("plot_filtered_flight")
    # print("ground data")
    # print(flight[1].data)
    # print("unfiltered")
    # print(radarFlight[1].data)
    # print("filtered")
    # print(filtered_flight.data)
    # exit()
    plt.rcParams["figure.autolayout"] = True
    plt.style.context("traffic")
    fig, (ax1, ax2 , ax3) = plt.subplots(1 , 3 , subplot_kw = dict(projection=Mercator()) )
    fig.suptitle('Comparison of ground truth data, radar data and filtered radar data')
    ax1.add_feature(countries())
    ax1.set_title("Ground truth data of flight " + str(flight[0]) )
    ax1.axis("equal")
    ax2.add_feature(countries())
    ax2.set_title("Radar data of flight " + str(flight[0]) )
    ax2.axis("equal")
    ax3.add_feature(countries())
    ax3.set_title("Filtered radar data of flight " + str(flight[0]) )
    ax3.axis("equal")
    flight[1].plot(ax1, color="orange")
    radarFlight[1].plot(ax2, color="orange")
    filtered_flight.plot(ax3, color="orange")
    plt.show()


def plot_flights(parsedFlights , parsedRadarData , flightNumber1,flightNumber2,flightNumber3,flightNumber4):
    
    plt.rcParams["figure.autolayout"] = True
    plt.style.context("traffic")
    fig, ((ax1, ax2),(ax3,ax4)) = plt.subplots(2, 2 , subplot_kw = dict(projection=Mercator()) )
    fig.suptitle('Ground truth data flight visualisation')
    ax1.add_feature(countries())
    ax1.set_title("Ground truth data of flight " + str(parsedFlights[flightNumber1][0]) )
    ax1.axis("equal")
    ax2.add_feature(countries())
    ax2.set_title("Ground truth data of flight " + str(parsedFlights[flightNumber2][0]) )
    ax2.axis("equal")
    ax3.add_feature(countries())
    ax3.set_title("Ground truth data of flight " + str(parsedFlights[flightNumber3][0]) )
    ax3.axis("equal")
    ax4.add_feature(countries())
    ax4.set_title("Ground truth data of flight " + str(parsedFlights[flightNumber4][0]) )
    ax4.axis("equal")
    parsedFlights[flightNumber1][1].plot(ax1, color="green")
    parsedFlights[flightNumber2][1].plot(ax2, color="red")
    parsedFlights[flightNumber3][1].plot(ax3, color="blue")
    parsedFlights[flightNumber4][1].plot(ax4, color="orange")
    plt.show()
        
    fig, ((ax1, ax2),(ax3,ax4)) = plt.subplots(2, 2 , subplot_kw = dict(projection=Mercator()) )
    fig.suptitle('Radar data flight visualisation')
    ax1.add_feature(countries())
    ax1.set_title(" Radar data of flight " + str(parsedRadarData[flightNumber1][0]) )
    ax1.axis("equal")
    ax2.add_feature(countries())
    ax2.set_title(" Radar data of flight " + str(parsedRadarData[flightNumber2][0]) )
    ax2.axis("equal")
    ax3.add_feature(countries())
    ax3.set_title(" Radar data of flight " + str(parsedRadarData[flightNumber3][0]) )
    ax3.axis("equal")
    ax4.add_feature(countries())
    ax4.set_title(" Radar data of flight " + str(parsedRadarData[flightNumber4][0]) )
    ax4.axis("equal")
    parsedRadarData[flightNumber1][1].plot(ax1, color="green")
    parsedRadarData[flightNumber2][1].plot(ax2, color="red")
    parsedRadarData[flightNumber3][1].plot(ax3, color="blue")
    parsedRadarData[flightNumber4][1].plot(ax4, color="orange")
    plt.show()
    
    fig, ((ax1, ax2),(ax3,ax4)) = plt.subplots(2, 2 , subplot_kw = dict(projection=Mercator()) )
    fig.suptitle('Comparison of ground truth data and radar data')
    ax1.add_feature(countries())
    ax1.set_title(" Ground truth data of flight " + str(parsedFlights[flightNumber1][0]) )
    ax1.axis("equal")
    ax2.add_feature(countries())
    ax2.set_title(" Radar data of flight " + str(parsedRadarData[flightNumber1][0]) )
    ax2.axis("equal")
    ax3.add_feature(countries())
    ax3.set_title(" Ground truth data of flight " + str(parsedFlights[flightNumber2][0]) )
    ax3.axis("equal")
    ax4.add_feature(countries())
    ax4.set_title(" Radar data of flight " + str(parsedRadarData[flightNumber2][0]) )
    ax4.axis("equal")
    parsedFlights[flightNumber1][1].plot(ax1, color="red")
    parsedRadarData[flightNumber1][1].plot(ax2, color="red")
    parsedFlights[flightNumber2][1].plot(ax3, color="orange")
    parsedRadarData[flightNumber2][1].plot(ax4, color="orange")
    
    fig.tight_layout()
    
    plt.show()


#############################

if __name__ == "__main__":
    main()