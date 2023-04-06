# Scarab setup instructions for Ubuntu 20.04 and ROS Noetic

Install Ubunutu 20.04. Choose not to connect to the internet during setup and choose the minimal installation option. Make an admin user `scarab_admin` with hostname `scarab<id>`, replacing `<id>` with the scarab's ID. Throughout these instructions, all instances of `<id>` should be replaced with the scarab's ID.

After restarting and removing the installation media, connect to the 'scarab' wifi network (get password from lab admins). Go to Settings-->Wi-Fi and click on the gear icon next to the scarab network. From there go to IPv4 and set the IPv4 Method to Manual. Enter the following information:

IP address: `192.168.131.<id>`\
Netmask: `255.255.255.255`\
Gateway: `192.168.131.1`

Turn off the Automatic toggle next to DNS and enter the following information:\
DNS: `192.168.131.1`

Once connected, you can verify the settings were successful by viewing the Details tab of the scarab network settings in the same window or manually on the command line by running `ip address show` to check the right IP address was assigned and `ping www.google.com` to verify internet access. If you are having issues connecting to the internet, executing the following might solve the connection issue:\
`sudo systemctl restart NetworkManager.service`\
If the issue persists, a restart should resolve this issue.

Ensure system packages are up to date and disable unattended upgrades:
```
sudo apt update
sudo apt upgrade
sudo apt remove unattended-upgrades
```

Copy udev rules for setting USB permissions:
```
wget https://raw.githubusercontent.com/KumarRobotics/scarab/devel_noetic/config/99-scarab.rules
sudo cp 99-scarab.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger
```

Disable automatic updates:
```
Open the Software Updater settings
Go to the Updates tab
Change the drop down menus to:
Automatically check for updates: Weekly
When there are security updates: Display immediately <-- This might be greyed out, if so, don't worry about it
When there are other updates: Display weekly
```

Install ROS Noetic (ros-noetic-desktop-full) following the instructions at: http://wiki.ros.org/noetic/Installation.

Install additional packages used by the scarab:
```
sudo apt install ros-noetic-urg-node ros-noetic-joy libarmadillo-dev libncurses5-dev libnl-3-dev \
                 libnl-route-3-dev libnl-genl-3-dev libnl-nf-3-dev libcgal-dev libcgal-qt5-dev \
                 ros-noetic-rgbd-launch ros-noetic-openni2-camera ros-noetic-openni2-launch \
                 ros-noetic-gmapping openssh-server vim emacs tmux tmuxinator python3-osrf-pycommon \
                 python3-catkin-tools git net-tools
```

Set up servo motor (TODO: change to dynamixel_sdk):
```
sudo apt install ros-noetic-dynamixel-motor
sudo usermod -a -G dialout $USER
sudo chmod 777 /dev/ttyUSB0
```
On scarab, make sure the mode switch on the servo controller is all the way towards the end the USB connects to.

Setup a catkin workspace:
```
cd ~
mkdir -p ws_scarab/src
cd ~/ws_scarab
catkin config --init -DCMAKE_BUILD_TYPE=Release --extend /opt/ros/noetic
git clone --branch devel_noetic --single-branch https://github.com/KumarRobotics/scarab.git src/scarab
catkin build
```

Configure common ROS environment variables and source calls, being sure to replace `<id>` with the appropriate scarab id number:
```
echo "export AGENT=scarab<id>" >> ~/.bashrc
echo "export ROS_IP=192.168.131.<id>" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.131.<id>:11311" >> ~/.bashrc
source ~/.bashrc
source ~/ws_scarab/devel/setup.bash
```
Turn on the H-BRDG and ROBO CLAW PWR switches, then run to test the admin user was setup successfully:
```
roslaunch scarab scarab.launch
```

Install scarab drivers to the system. catkin-tools provides profiles for devel/install space
http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_profile.html
```
cd ~/ws_scarab
catkin config --profile install --install -DCMAKE_BUILD_TYPE=Release --extend /opt/ros/noetic
catkin profile set install
catkin clean
catkin build
```
This will create an 'install' folder in the workspace. Copy it to system location:
```
sudo rsync -azP ~/ws_scarab/install/ /opt/ros/scarab
```
Remember to execute the above command every time you update ws_scarab. Note that it prints each file that gets update (i.e. copied to /opt/ros/scarab) to the console - this can be a useful sanity check to verify expected changes did actually happen.

Make a scarab user account with the password 'mrsl':
```
sudo adduser scarab
sudo usermod -a -G dialout scarab
```

Now, switch to the newly created 'scarab' user account:
```
su scarab
cd  # be sure to switch to the scarab user's home directory
```
and configure its .bashrc with common ROS environment variables and source calls:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/ros/scarab/setup.bash" >> ~/.bashrc
echo "export AGENT=scarab<id>" >> ~/.bashrc
echo "export ROS_IP=$(ifconfig | sed -En 's/.*inet (192.168.131.[0-9]*).*/\1/p')" >> ~/.bashrc
```

Add the following snippet to .bashrc, which conveniently sets ROS_MASTER_URI to the IP of the ssh_client so users don't have to explicitely do this every time they connect:

```
echo 'export MASTER_IP=${SSH_CLIENT%% *}' >> ~/.bashrc
source ~/.bashrc
echo 'if [[ ! -z $MASTER_IP ]]; then
   export ROS_MASTER_URI=http://${MASTER_IP}:11311
fi' >> ~/.bashrc
```

Finally, verify the scarab user account's ROS installation:
```
roslaunch scarab scarab.launch
```
