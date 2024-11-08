### Google Earth Parser
1. First, go to [Google Earth](https://earth.google.com/) to mark objects. You will either use the measurement tool to draw a polygon or add a placemarker to place a proto. With both options you must name based on the object type:
    Using the measurement tool:
        "Building" - Buildings with randomly generated heights
        "Base" - (Optional) Used to mark the entirety of the simulated area
        "Curb" - Non-Drivable areas
        "Sidewalk" - Walkable areas
        "End" - Simulation boundaries
    Using the placemarker tool:
        "tree" - Places a tree proto with a randomly generate height and weidth
        For the following two replace # with n, e, s, w based on the direction it is facing"
            "sl" - street light proto
            "Stop_sign" - places a stop sign proto
    
2. Once you mave placed/traced your objects, select file > Export as KML file

3. Place the .kml file in the [kml_input folder](google_earth_parser/kml_input). Edit the "INPUT_FILE_NAME" constant in [kml_scrape.py](google_earth_parser/kml_scrape.py) to match your file name.

4. In your terminal, Move to the google_earth_parser directory and run kml_scrape.py. This will store the useful information in a .json file placed in [kml_output](google_earth_parser/kml_output).

<pre>
cd google_earth_parser
python3 kml_scrape.py
</pre>

5. Inside of object_generator.py, change the constant "INPUT_FILE" to match your .json file. Then, in the same terminal from step 4, run object_generator.py. This will generate a .wbt file inside of [wbt_output](google_earth_parser/wbt_output).

<pre>
python3 kml_scrape.py
</pre>

### TO INSTALL PACKAGE FOR ASSIGNMENT 

1. Set up environment variables for ROS. Make sure to replace '/home/rpi/shared' with your own shared folder location
<pre>
source /opt/ros/humble/setup.bash
</pre>
Also do any Windows or Mac specific setup

For example in Mac...
<pre>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
</pre>

For example in windows...
<pre>
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
</pre>

2. Fork your own repository of this project (using web interface)

3. Clone your fork
<pre>
git clone <your github url for this repository>
</pre>

4. Make the package (for python, it really just installs the files)
<pre>
cd p1_outdoor
colcon build
</pre>

5. Set up variables to use the package you just created
<pre>
source install/setup.bash
</pre>

6. Start webots simulation with connect back to ROS in the virtual machine
<pre>
ros2 launch outdoor_simulation outdoor_simulation_launch.py
</pre>

7. If you would like to run the simulation at night time, comment out the "# Day" set up and un-comment the "# Night" set up at the beginning of [the .wbt file located at](src/outdoor_simulation/worlds/outdoor.wbt)
<pre>
# Day
WorldInfo {
}
TexturedBackground {
}
TexturedBackgroundLight {
}
</pre>
<pre>
## Night
#WorldInfo {
#}
#Background {
#  skyColor [0.05 0.05 0.1]  
#  luminosity 0.1             
#}
#DirectionalLight {
#  color 0.2 0.2 0.4        
#  intensity 0.1             
#  direction -1 -1 -1
#  castShadows TRUE
#}
</pre>

### Recomendations
Due to the complexity of the simulation, I recommend openning Webots prior to trying to run the simulation and changing the following settings

1. Start up project in a paused state
<pre>
Tools > Preferences > General > Startup Mode: Pause, Rendering: Off
</pre>

2. Start up project in a paused state
<pre>
Tools > Preferences > OpenGL > Texture Quality: Low
</pre>

3. Line 20 in "RidgecrestWest.wbt" sets the basicTimeStep to 32. Increasing this value will reduce the strain but update the simmulation at a slower rate. It may be preferable to decrease this value for a worse performance but a more accurate result.
<pre>
WorldInfo {
  basicTimeStep 32
}
</pre>