catkin_make
source .develsetup.bash
roslaunch cav_simulation cav_simulation_qiyansuo.launch 
cd test13
catkin_make
source .develsetup.bash
cd cav_path_follower
python main_test.py