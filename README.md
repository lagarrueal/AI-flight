# AI-flight
Second AI project of the 12 weeks course of Artificial Intelligence at Reykjavik University, Iceland.

How to install the needed libraries
===================================
The following commands work on a plain Ubuntu installation.

# install needed libraries (Proj, GEOS)
apt install python3 python3-pip  
apt install libproj-dev libgeos-dev proj-data proj-bin
# download traffic-viz 
git clone https://github.com/xoolive/traffic  
cd traffic/
# require that Cartopy version is <0.20 to avoid problems with version of Proj
sed -i 's/^Cartopy = .*/Cartopy = ">=0.19.0,<0.20.0"/' pyproject.toml
# install traffic library
python3 -m pip install .
# install filtery library
python3 -m pip install filterpy
# install geopy library
python3 -m pip install geopy


How to execute the program
==========================
# in the project2.py file's directory
python3 project2.py

# changing the parameters
project2.py => line 27 : changing the value of the parameter delta_t  
            => line 29 : changing the value of the parameter sigma0  
            => line 30 : changing the value of the parameter sigmaP  

# changing the flight we're filtering, smoothing and ploting
project2.py => line 33 : changing the value of the parameter id, different flight ids can be found in the file orderedFlightData.txt
