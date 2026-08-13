dataROOT="$HOME/2025huat/scripts/vehicle/autoStartGkj"
echo '0' > ${dataROOT}/command
sourceBash="${dataROOT}/sourceBash"

# Validate workspace setup.bash exists before sourcing
if [ ! -f "$HOME/2025huat/devel/setup.bash" ]; then
    echo "ERROR: 2025huat workspace not built. Run 'catkin build' first."
    exit 1
fi

succ=0
gnome-terminal --window -- bash -c "source $HOME/.bashrc; source /opt/ros/noetic/setup.bash;roscore"
sleep 2s
# Note: The following paths are updated to $HOME but might point to missing workspaces. Please verify.
gnome-terminal --window -- bash -c "source $HOME/.bashrc;source /opt/ros/noetic/setup.bash; source $HOME/2025huat/devel/setup.bash; bash ${sourceBash}/racingNumCmd.sh ${sourceBash}"
sleep 2s
gnome-terminal --window -- bash -c "source $HOME/.bashrc;source /opt/ros/noetic/setup.bash;bash ${sourceBash}/interface.sh"
sleep 2s

# sudo chmod 777 /dev/ttyUSB0
#gnome-terminal -x bash -c "source $HOME/car/devel/setup.sh; roslaunch rslidar_sdk start.launch"

INS_SETUP="$HOME/Driver/pbox_node_dirve-V3.0.5-20240412/pbox_node_dirve/devel/setup.sh"
if [ -f "$INS_SETUP" ]; then
    gnome-terminal -- bash -c "source $HOME/.bashrc; source $INS_SETUP; roslaunch pbox_node pbox_node.launch"
else
    echo "WARNING: INS driver not found at $INS_SETUP (skipping)"
fi

CAM_SETUP="$HOME/camera/devel/setup.sh"
if [ -f "$CAM_SETUP" ]; then
    gnome-terminal -- bash -c "source $HOME/.bashrc; source $CAM_SETUP; roslaunch pylon_camera pylon_camera_node.launch"
else
    echo "WARNING: Camera driver not found at $CAM_SETUP (skipping)"
fi
sleep 3

while [ $succ -eq 0 ]
do
var=$(cat $dataROOT/command)
echo $var
    case $var in
        0)
	    succ=0
            ;;
        *)
	    echo "Successfully entered control"
	    gnome-terminal --window -- bash -c "source $HOME/.bashrc; source $HOME/2025huat/devel/setup.bash; roslaunch fsd_launch trackdrive.launch"
            succ=2025
            ;;
    esac
sleep 1
done

echo $succ
gnome-terminal --window -- bash -c "source $HOME/.bashrc; source /opt/ros/noetic/setup.bash; source $HOME/2025huat/devel/setup.bash; rostopic echo /vehcileCMDMsg"

#gnome-terminal --window -- bash -c "$HOME/2025huat/scripts/vehicle/autoStartGkj/sourceBash/detect.sh;exec bash"


while [ $succ -eq 2025 ]
do
var=$(cat $dataROOT/command)
echo $var
sleep 1
done
