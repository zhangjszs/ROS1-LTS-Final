dataROOT="/home/tb/autoStartGkj"
echo '0' > ${dataROOT}/command
sourceBash="${dataROOT}/sourceBash"

succ=0
gnome-terminal --window -- bash -c "source /home/tb/.bashrc; source /opt/ros/melodic/setup.bash;roscore"
sleep 2s
gnome-terminal --window -- bash -c "source /home/tb/.bashrc;source /opt/ros/melodic/setup.bash; source /home/tb/line_creat2/devel/setup.bash; bash ${sourceBash}/racingNumCmd.sh ${sourceBash}"
sleep 2s
gnome-terminal --window -- bash -c "source /home/tb/.bashrc;source /opt/ros/melodic/setup.bash;bash ${sourceBash}/interface.sh"
sleep 2s

# sudo chmod 777 /dev/ttyUSB0
#gnome-terminal -x bash -c "source /home/tb/car/devel/setup.sh; roslaunch rslidar_sdk start.launch"
gnome-terminal -x bash -c "source /home/tb/Driver/pbox_node_dirve-V3.0.5-20240412/pbox_node_dirve/devel/setup.sh; roslaunch pbox_node pbox_node.launch"
gnome-terminal -x bash -c "source /home/tb/camera/devel/setup.sh; roslaunch pylon_camera pylon_camera_node.launch"
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
	    gnome-terminal --window -- bash -c "source /home/tb/.bashrc; source /home/tb/huat2025/devel/setup.bash; rosrun control control"
            succ=2025
            ;;
    esac
sleep 1
done

echo $succ
gnome-terminal --window -- bash -c "source /home/tb/.bashrc; source /opt/ros/melodic/setup.bash; source /home/tb/huat2025/devel/setup.bash; rostopic echo /vehcileCMDMsg"

#gnome-terminal --window -- bash -c "/home/tb/autoStartGkj/sourceBash/detect.sh;exec bash"


while [ $succ -eq 2025 ]
do
var=$(cat $dataROOT/command)
echo $var
sleep 1
done
