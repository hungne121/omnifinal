# OmniFinal Project

Đây là dự án điều khiển robot omni, với mô phỏng trong Gazebo và hỗ trợ ROS1. Dự án này sử dụng các lệnh trong ROS để điều khiển robot và theo dõi dữ liệu IMU.
### FIle PDF BAO CAO
## baocao_giuaki_ros.pdf

### 1. Clone Repository

```bash
git clone https://github.com/hungne121/omnifinal.git
cd omnifinal
```

### 2. Chay mo phong
```roslaunch omnifinal omnifinal.launch```
### 3. Chay dieu khien xe
```rosrun omnifinal omnifinal_tele```
### 4. Xem topic cua imu
```rostopic echo /imu/data```

