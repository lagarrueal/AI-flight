#!/usr/bin/env python

from re import X
from project2_base import *
from filterpy.kalman import KalmanFilter
from filterpy.kalman import predict , update
from traffic.drawing import countries
from traffic.core.projection import Mercator
from geopy.distance import geodesic
import numpy as np
import matplotlib.pyplot as plt

#############################


def main():
    # TODO: put your code here
    delta_t = 10
    sigma0 = 100
    sigmaP = 1.5
    id = 'DMUPY_052'
    
    flights = get_ground_truth_data()  
    

    plot_filtered_flight(flights , id , delta_t  , sigma0 , sigmaP )
    
    print("Error :")
    print( kalman_error(flights[id] , kalman_filtering(flights[id] , delta_t , sigmaP , sigma0) ) )

#================
# Kalman Filter
#================
#function that apply the Kalman filter on a flight
# input : a flight
# output : the filtered version of the flight
def kalman_filtering(flight, delta_t, sigma0, sigmaP):
    kf = KalmanFilter(dim_x=4, dim_z=2)

    #State Transition matrix
    kf.F = np.array([[1,delta_t,0,0], 
                     [0,1,0,0],
                     [0,0,1,delta_t],
                     [0,0,0,1]])

    #State
    kf.x = np.zeros((4,1))

    #Measurement function
    kf.H = np.array([[1,0,0,0],
                     [0,0,1,0]])

    #Covariance matrix for the porecess noise
    kf.Q = np.array([[0.25*delta_t**(4),0.5*delta_t**(3),0,0], 
                     [0.5*delta_t**(3),delta_t**(2),0,0],
                     [0,0,0.25*delta_t**(4),0.5*delta_t**(3)],
                     [0,0,0.5*delta_t**(3),delta_t**(2)]])*sigmaP**(2)

    #Covariance matrix for the observation noise
    kf.R = np.array([[sigma0**(2),0],
                     [0,sigma0**(2)]])

    #radar data
    flight_radar = get_radar_data_for_flight(flight)
    Xm=flight_radar.data.x
    Ym=flight_radar.data.y
    
    # run the kalman filter and store the results
    for i in range(len(flight_radar.data)):
        z=np.array([[Xm[i]],[Ym[i]]])
        kf.predict()
        kf.update(z)
        flight_radar.data.at[i, "x"]= kf.x[0]
        flight_radar.data.at[i, "y"]= kf.x[2]
    flight_radar_filtered=set_lat_lon_from_x_y(flight_radar)
    return flight_radar_filtered 

 #function to evaluate our model
# input : flight, kalman filtered flight
# output error : mean and max distance
def kalman_error(flight, kalman_flight):
    flight= flight.resample("10s")
    distance = [0]*len(kalman_flight)
    true_lons= flight.data["longitude"]
    true_lats = flight.data["latitude"]
    lons= kalman_flight.data["longitude"]
    lats = kalman_flight.data["latitude"]
    for i in range(len(kalman_flight)):
        distance[i] = geodesic((lats[i],lons[i]), (true_lats[i],true_lons[i])).m
    return np.mean(distance), max(distance)       
    
def plot_filtered_flight(flights , id , delta_t , sigmaP , sigma0):
    
    plt.rcParams["figure.autolayout"] = True
    plt.style.context("traffic")
    fig, (ax1, ax2 , ax3) = plt.subplots(1 , 3 , subplot_kw = dict(projection=Mercator()) )
    fig.suptitle('Comparison of ground truth data, radar data and filtered radar data')
    ax1.add_feature(countries())
    ax1.set_title("Ground truth data of flight " + id) 
    ax1.axis("equal")
    ax2.add_feature(countries())
    ax2.set_title("Radar data of flight " + id )
    ax2.axis("equal")
    ax3.add_feature(countries())
    ax3.set_title("Filtered radar data of flight " + id )
    ax3.axis("equal")
    flights[id].plot(ax1, color="orange")
    get_radar_data_for_flight(flights[id]).plot(ax2, color="orange")
    kalman_filtering(flights[id] , delta_t , sigmaP , sigma0).plot(ax3, color="orange")
    plt.show()


#############################

if __name__ == "__main__":
    main()