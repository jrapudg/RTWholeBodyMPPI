#!/bin/zsh
source /home/ControlUser/ocs2_ws/devel/setup.zsh
cd /home/ControlUser/legged_ctrl_ws
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
chmod +x /home/ControlUser/legged_ctrl_ws/src/legged_controllers/scripts/switch_controller.sh
source /home/ControlUser/legged_ctrl_ws/devel/setup.zsh
pip install -e /home/ControlUser/legged_ctrl_ws/src/legged_mppi/
mkdir -p /home/ControlUser/legged_ctrl_ws/src/bags/
echo 'export PATH=/home/ControlUser/.local/bin:$PATH' >> ~/.zshrc
source ~/.zshrc
pip install notebook jupyterlab ipywidgets matplotlib pandas
sudo apt install ffmpeg
