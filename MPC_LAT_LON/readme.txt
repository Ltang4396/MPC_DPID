参考了 B站 老王的视频 将其中的lqr替换成mpc

功能：采用了纵向双pid，横向mpc控制完成了轨迹跟踪

版本：matlab r2022a
	 carsim 2019.1
F:\MPC\my_2dof_mpc\MPC_LAT_LON\mpc2dof_lat_lon.slx
F:\MPC\my_2dof_mpc\MPC_LAT_LON

采用了preload函数在打开simulink文件后自动加载所需参数 
mpc2dof.m为mpc的sfunction

调整参考轨迹的vx速度时候还要调整carsim中的速度，以及sfunction中的对应的速度
