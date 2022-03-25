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
    #========================
    #Flights ID used for the different tests :
    # DMUPY_052
    # N441FS_003
    # G-CLPU_016
    # ADA4_031
    # SAMU31_019
    #========================
    
    # TODO: put your code here
    delta_t = 10
    
    sigma0 = 100
    sigmaP = 1.5
    
    #the flight ID to plot and filter
    id = 'DMUPY_052'
    
    flights = get_ground_truth_data()  
    

    plot_filtered_smoothed_flight(flights , id , delta_t  , sigma0 , sigmaP )
    
    print("Error between real flight and radar flight")
    print(kalman_error(flights[id] , get_radar_data_for_flight(flights[id])))
    
    print("Error between real flight and filtered flight:")
    print( kalman_error(flights[id] , kalman_filtering(flights[id] , delta_t , sigma0 , sigmaP) ) )
    
    print("Error between real flight and smoothed flight: ")
    print( kalman_error(flights[id] , kalman_smoothing(flights[id] , delta_t , sigma0 , sigmaP) ) )


def kalman_filtering(flight, delta_t, sigma0, sigmaP):
    """_summary_

    Args:
        flight (object): a specific flight containing various features like flight_id, timestamp, latitude, longitude
        delta_t (float): time interval
        sigma0 (): cription_
        sigmaP (_type_): _description_

    Returns:
        _type_: _description_
    """
    kf = KalmanFilter(dim_x=4, dim_z=2)

    #State Transition matrix
    kf.F = np.array([[1,delta_t,0,0],
                     [0,1,0,0],
                     [0,0,1,delta_t],
                     [0,0,0,1]])

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

    #State
    kf.x = np.array([[flight_radar.data.at[0, "x"]],
                     [0],
                     [flight_radar.data.at[0, "y"]],
                     [0]])

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

def kalman_error(flight, kalman_flight):
    """_summary_

    Args:
        flight (_type_): _description_
        kalman_flight (_type_): _description_

    Returns:
        _type_: _description_
    """
    flight= flight.resample("10s")
    distance = [0]*len(kalman_flight)
    true_lons= flight.data["longitude"]
    true_lats = flight.data["latitude"]
    lons= kalman_flight.data["longitude"]
    lats = kalman_flight.data["latitude"]
    for i in range(len(kalman_flight)):
        distance[i] = geodesic((lats[i],lons[i]), (true_lats[i],true_lons[i])).m
    return np.mean(distance), max(distance)       

def kalman_smoothing(flight,delta_t, sigma0, sigmaP):
    """_summary_

    Args:
        flight (_type_): _description_
        delta_t (_type_): _description_
        sigma0 (_type_): _description_
        sigmaP (_type_): _description_

    Returns:
        _type_: _description_
    """
    kf = KalmanFilter(dim_x=4, dim_z=2)

    #State Transition matrix
    kf.F = np.array([[1,delta_t,0,0],
                     [0,1,0,0],
                     [0,0,1,delta_t],
                     [0,0,0,1]])

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
    #State
    kf.x = np.array([[flight_radar.data.at[0, "x"]],[0],[flight_radar.data.at[0, "y"]],[0]])

    zs=flight_radar.data[['x','y']].values.tolist()
    mean, cov, _,_ = kf.batch_filter(zs)
    
    M, P, C, _ = kf.rts_smoother(mean, cov)
    flight_radar.data.x=M[:,0]
    flight_radar.data.y=M[:,2]
    flight_smoothed =set_lat_lon_from_x_y(flight_radar)
    return flight_smoothed

def plot_filtered_smoothed_flight(flights , id , delta_t , sigma0, sigmaP):
    """_summary_

    Args:
        flights (_type_): _description_
        id (_type_): _description_
        delta_t (_type_): _description_
        sigmaP (_type_): _description_
        sigma0 (_type_): _description_
    """
    # plt.rcParams["figure.autolayout"] = True
    plt.style.context("traffic")
    
    fig, ax = plt.subplots(1, subplot_kw = dict(projection=Mercator()) )
    fig.suptitle('Comparison of ground truth data, radar data, filtered radar data and smoothed data. of flight ' + id + "\n" +
                 "Parameters: delta_t = " + str(delta_t) + ", sigma0 = " + str(sigma0) + ", sigmaP = " + str(sigmaP) )
    labels = ["Ground data" , "Radar Data" , "Filtered Data" , "Smoothed Data"]
    
    ax.add_feature(countries())    
    ax.set_xlabel('Longitude [°]')
    ax.set_ylabel('Latitude [°]')  
    
    flights[id].plot(ax, color="black" , linestyle='-', linewidth=3 ,  label=labels[0])
    get_radar_data_for_flight(flights[id]).plot(ax, color="orange",  label=labels[1])
    kalman_filtering(flights[id] , delta_t , sigmaP , sigma0).plot(ax, color="green",  label=labels[2])

    fig.legend(labels=labels, loc="center right", ncol=1)
    
    plt.show()

#function that apply the 3D Kalman filter on a flight
# input : a flight
# output : the filtered version of the flight
def kalman_filtering_3D(flight, deltaT, sigma0, sigmap):

    kf = KalmanFilter(dim_x=6, dim_z=3)

    #State Transition matrix
    kf.F = np.array([[1,deltaT,0,0,0,0],
                     [0,1,0,0,0,0],
                     [0,0,1,deltaT,0,0],
                     [0,0,0,1,0,0],
                     [0,0,0,0,1,deltaT],
                     [0,0,0,0,0,1]])

    #Measurement function
    kf.H = np.array([[1,0,0,0,0,0],
                     [0,0,1,0,0,0],
                     [0,0,0,0,1,0]])

    #Covariance matrix for the porecess noise
    kf.Q = np.array([[0.25*deltaT**(4),0.5*deltaT**(3),0,0,0,0],
                     [0.5*deltaT**(3),deltaT**(2),0,0,0,0],
                     [0,0,0.25*deltaT**(4),0.5*deltaT**(3),0,0],
                     [0,0,0,0,0.5*deltaT**(3),deltaT**(2)],
                     [0,0,0.25*deltaT**(4),0.5*deltaT**(3),0,0],
                     [0,0,0,0,0.5*deltaT**(3),deltaT**(2)]])*sigmap**(2)

    #Covariance matrix for the observation noise
    kf.R = np.array([[sigma0**(2),0,0],
                     [0,sigma0**(2),0],
                     [0,0,sigma0**(2)]])

    #radar data
    flight_radar = get_radar_data_for_flight(flight)
    Xm=flight_radar.data.x
    Ym=flight_radar.data.y

    #State
    kf.x = np.array([[flight_radar.data.at[0, "x"]],
                     [0],
                     [flight_radar.data.at[0, "y"]],
                     [0],
                     [flight_radar.data.at[0, "altitude"]],
                     [0]])
    
    # run the kalman filter and store the results
    for i in range(len(flight_radar.data)):
        z=np.array([[Xm[i]],[Ym[i]],[flight_radar.data.at[i, "altitude"]]])
        kf.predict()
        kf.update(z)
        flight_radar.data.at[i, "x"]= kf.x[0]
        flight_radar.data.at[i, "y"]= kf.x[2]
        flight_radar.data.at[i, "altitude"]= kf.x[4]
    flight_radar_filtered=set_lat_lon_from_x_y(flight_radar)
    return flight_radar_filtered

#function to evaluate our 3D model
# input : flight, 3D kalman filtered flight
# output error : mean and max distance
def kalman_error_3D(flight, kalman_flight_3D):
    flight= flight.resample("10s")
    distance = [0]*len(kalman_flight_3D)
    true_lons= flight.data["longitude"]
    true_lats = flight.data["latitude"]
    true_alts = flight.data["altitude"]
    lons= kalman_flight_3D.data["longitude"]
    lats = kalman_flight_3D.data["latitude"]
    alts = kalman_flight_3D.data["altitude"]
    for i in range(len(kalman_flight_3D)):
        distance_2d = geodesic((lats[i],lons[i]), (true_lats[i],true_lons[i])).m
        distance[i] = np.sqrt(distance_2d**2 + (true_alts[i] - alts[i])**2)
    return np.mean(distance), max(distance)

#############################

if __name__ == "__main__":
    main()