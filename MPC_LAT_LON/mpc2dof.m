function [sys,x0,str,ts] = mpc2dof(t,x,u,flag)
    % 状态量=[ed,ed_dot,ephi,ephi_dot]，控制量为前轮偏角delta_f
switch flag
    case 0
    [sys,x0,str,ts]=mdlInitializeSizes; % 初始化
    case 2
    sys = mdlUpdates(t,x,u);     % 离散化 
    case 3
    sys = mdlOutputs(t,x,u);     % 输出结果
    case {1,4,9}                 % Unused flags
    sys = [];  
    otherwise
    error(['unhandled flag = ',num2str(flag)]); % Error handling
end

%==============================================================
% Initialization
%==============================================================
function [sys,x0,str,ts] = mdlInitializeSizes  % 其中sys是变量的返回值，当flag=3时表示输出结果值；
                                               % x0是初始状态值，当flag=0时才有值
                                               % str是用来备用的，通常设置为空矩阵[]
                                               % ts是包含模块的采样时间和偏差值得两列矩阵。
sizes = simsizes;         %返回一个变量
sizes.NumContStates  = 0; %连续变量个数
sizes.NumDiscStates  = 4; %离散变量个数
sizes.NumOutputs     = 2; %输出个数   %前轮转角，仿真时长
sizes.NumInputs      = 4; %输入个数
sizes.DirFeedthrough = 1; %直接贯通，0或1，当输出值直接依赖于同一时刻的输入时为1
sizes.NumSampleTimes = 1; %采样时间
sys = simsizes(sizes);    %将系统的默认参数变量赋值给sys
x0 =[0.0001;0.0001;0.0001;0.0001];    
global U; 
U=[0]; %控制量初始化,这里面加了一个期望轨迹的输出，如果去掉，U为一维的
str = [];             % Set str to an empty matrix.
ts  = [0.02 0];       % 系统仿真从0.1s开始，每隔0.2s运行一次

%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)  
sys = x;

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)  %计算输出 前轮转角。
    global a b; 
    %global u_piao;
    global U;
    %global kesi;
    tic;
    Nx=4;  %状态量的个数
    Nu=1;  %控制量的个数
    Ny=4;  %输出量的个数
    %Np=14;  %预测时域
    Np=7;
    %Nc=6;    %控制时域  %自带
    Nc=3;
    Row=1000;%松弛因子权重
    fprintf('Update start, t=%6.3f\n',t)   
    %输入接口转换,x_dot后面加一个非常小的数，是防止出现分母为零的情况
    % y_dot=u(1)/3.6-0.000000001*0.4786;%CarSim输出的是km/h，转换为m/s
    ed=u(1);
    ed_dot=u(2);   
    ephi=u(3);
    ephi_dot=u(4); 
%% carsim自车参数
%syms sf sr;%分别为前后车轮的滑移率,需要提供
%     Sf=0.2; Sr=0.2;
%syms lf lr;%前后车轮距离车辆质心的距离，车辆固有参数
    lf=1.015;lr=2.91-1.015;
%syms Cf,Cr;%分别为前后车轮的横向侧偏刚度，车辆固有参数
    Cf=-148970;Cr=-82204;
%   Ccf=12548/2;Ccr=80505/2;Clf=12548/2;Clr=80505/2;
%syms m g I;%m为车辆质量，g为重力加速度，I为车辆绕Z轴的转动惯量，车辆固有参数
    m=1413;g=9.8;I=1536.7;
%% C车参数
% %syms sf sr;%分别为前后车轮的滑移率,需要提供
%     Sf=0.2; Sr=0.2;
% %syms lf lr;%前后车轮距离车辆质心的距离，车辆固有参数
%     lf=1.015;lr=2.910-1.015;
% %syms C_cf C_cr C_lf C_lr;%分别为前后车轮的纵横向侧偏刚度，车辆固有参数
%     Ccf=80505/2;Ccr=125483/2;Clf=80505/2;Clr=125483/2;
% %syms m g I;%m为车辆质量，g为重力加速度，I为车辆绕Z轴的转动惯量，车辆固有参数
%     m=1270;g=9.8;I=1536.7;
%% B车参数
% %syms sf sr;%分别为前后车轮的滑移率,需要提供
%     Sf=0.2; Sr=0.2;
% %syms lf lr;%前后车轮距离车辆质心的距离，车辆固有参数
%     lf=1.04;lr=1.56;
% %syms C_cf C_cr C_lf C_lr;%分别为前后车轮的纵横向侧偏刚度，车辆固有参数
%     Ccf=66864/2;Ccr=56864/2;Clf=66864/2;Clr=56864/2;
% %syms m g I;%m为车辆质量，g为重力加速度，I为车辆绕Z轴的转动惯量，车辆固有参数
%     m=1230;g=9.8;I=1343.1;  
%%  以下计算kesi,即状态量与控制量合在一起   
    kesi=zeros(Nx+Nu,1); %构建新的状态变量 ed ed_dot ephi ephi_dot delta_f
    kesi(1)=ed;  %u(1)==X(1)
    kesi(2)=ed_dot;  %u(2)==X(2)
    kesi(3)=ephi;    %u(3)==X(3)
    kesi(4)=ephi_dot;
    kesi(5)=U(1);    %delta_f
    %delta_f=U(1);
    vx=72/3.6;   %%根据自车需要速度实时调整
    fprintf('Update start, u(1)=%4.2f\n',U(1));
    T=0.02;  
    %权重矩阵设置 
    Q_cell=cell(Np,Np);
    for i=1:1:Np
        for j=1:1:Np
            if i==j
                %% 初始矩阵
%                 Q_cell{i,j}=[ 2000     0     0      0;
%                                0     0     0      0 ;
%                                0     0    800     0 ;
%                                0     0     0      0 ];
%                 Q_cell{i,j}=[ 2000     0     0      0;
%                                0     2000     0      0 ;
%                                0     0    800     0 ;
%                                0     0     0      2000 ];
                Q_cell{i,j}=[ 100     0     0      0;
                                0     200     0      0 ;
                                0     0    800     0 ;
                                0     0     0      200 ];
                %%%未对第np个矩阵单独设置
            else 
                Q_cell{i,j}=zeros(Ny,Ny);               
            end
        end 
    end 
    %% 初始矩阵 
    %R=1*10^2*eye(Nu*Nc);  %论文出图
    R=400*eye(Nu*Nc);
    %%
    a=[ 1,                          T,                         0,                                  0; 
        0,         1+T*(Cf+Cr)/(m*vx),              -T*(Cf+Cr)/m,             T*(lf*Cf-lr*Cr)/(m*vx);
        0,                          0,                         1,                                  T;
        0,     T*(lf*Cf-lr*Cr)/(I*vx),        -T*(lf*Cf-lr*Cr)/I,    1+T*(lf*lf*Cf+lr*lr*Cr)/(I*vx)];
    b=[        0
         -T*Cf/m    
               0
      -T*lf*Cf/I];
    %以下即为根据离散非线性模型预测下一时刻状态量    
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;
    A_cell{1,2}=b;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=b;
    B_cell{2,1}=eye(Nu);
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
    C=[ 1 0 0 0 0;
        0 1 0 0 0;
        0 0 1 0 0;
        0 0 0 1 0];
    PSI_cell=cell(Np,1);%预测模型的W
    THETA_cell=cell(Np,Nc);%预测模型的Z
%     theta_k=zeros(Nc,1);
%     for u=0:1:Nc-1
%        alfa=0.5;
%        theta_k(u+1,1)=alfa^u;
%     end
    for j=1:1:Np
        PSI_cell{j,1}=C*A^j;
        for k=1:1:Nc
            if k<=j
                THETA_cell{j,k}=C*A^(j-k)*B;% 4*5 * 5*5 * 5*1=4*1
            else 
                THETA_cell{j,k}=zeros(Ny,Nu);
            end
        end
    end
    PSI=cell2mat(PSI_cell);    %size(PSI)=[Ny*Np Nx+Nu]  %% W矩阵 
    THETA=cell2mat(THETA_cell);%size(THETA)=[Ny*Np Nu*Nc]  %%
    Q=cell2mat(Q_cell);
    H_cell=cell(2,2);
    %%H_cell{1,1}=THETA'*Q*THETA+R;
    H_cell{1,1}=2*(THETA'*Q*THETA+R);%%%%%%%%%%%%罪魁祸首    误会了
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    %%H_cell{2,2}=Row;
    H_cell{2,2}=2*Row;%%%%%%%%%%%%罪魁祸首   误会了
    H=cell2mat(H_cell);   
    error_1=-PSI*kesi; %求偏差  %没问题  后面f加了-
    %%%%%%%%%%error_1=PSI*kesi; %求偏差   对应G  %%%%%%%%%%%%罪魁祸首  
    f_cell=cell(1,2);
    f_cell{1,1}=2*error_1'*Q*THETA;
    f_cell{1,2}=0;
    f=-cell2mat(f_cell);  %%%%%注意这里有负号

    %约束部分
    A_t=zeros(Nc,Nc);%见falcone论文 P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%求克罗内克积
    Ut=kron(ones(Nc,1),U(1));
    umin=-0.1744;%维数与控制变量的个数相同
    umax=0.1744;
    delta_umin=-0.0148;
    delta_umax=0.0148;
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);    
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
    b_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值    
    %状态量约束
    M=10;   %误差约束量
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);    
    lb=[delta_Umin;0];  %（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
    ub=[delta_Umax;M];  %（求解方程）状态量上界，包含控制时域内控制增量和松弛因子    
    %% 开始求解过程
      options = optimset('Algorithm','active-set');
      x_start=zeros(Nc+1,1);%加入一个起始点
      [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,x_start,options);
      fprintf('exitflag=%d\n',exitflag);
      fprintf('H=%4.2f\n',H(1,1));
      fprintf('f=%4.2f\n',f(1,1));
    %% 计算输出
    u_piao=X(1);%得到控制增量
    U(1)=kesi(5,1)+u_piao;%当前时刻的控制量为上一刻时刻控制+控制增量
    d_t=toc;
    sys= [U d_t];

