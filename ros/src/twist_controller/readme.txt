Hot to run dbw_test.py
1. download https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc7
2. rename the file to dbw_test.rosbag.bag
3. move the dbw_test.rosbag.bag file to the project root’s data folder, e.g. CarND-Capstone/data/dbw_test.rosbag.bag
4. Run dbw_test: roslaunch twist_controller dbw_test.launch
5. Compare your results to the dbw_test output logged in ros/src/twist_controller in three files: brakes.csv, steers.csv, throttles.csv
