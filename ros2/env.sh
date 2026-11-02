# B（ROS2＋MoveIt）の端末を開いたら毎回: source ~/piper-bottle-pp/ros2/env.sh
source /opt/ros/humble/setup.bash
source ~/agx_arm_ws/install/setup.bash
source ~/venv_pp/bin/activate
export PYTHONPATH=$PYTHONPATH:$HOME/pyorbbecsdk/install/lib
export LC_NUMERIC=en_US.UTF-8
cd ~/piper-bottle-pp
