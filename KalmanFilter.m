% This to implement the kalman filter

function [Angle,P_store,time]=KalmanFilter(acceleration_data,rate_gyro_data,tsamp,Q,R,Po,X0)

A=[1 0;0 1];
N=length(rate_gyro_data(:,1));
dt=tsamp/N;
B=[dt 0;0 dt];



p=rate_gyro_data(:,1)';% x axix.
q=rate_gyro_data(:,2)';% y axis
r=rate_gyro_data(:,3)';% z axis.
ax=acceleration_data(:,1)';%x axis.
ay=acceleration_data(:,2)'; %y axis.
az=acceleration_data(:,3)';% z axis.

Angle=X0;

P =Po;
P_store(:,:,1)=P;

for i=2:N
    a=A*Angle(:,i-1)+B*[p(i);q(i)];
    P=A*P*A'+ Q;
    z=[atan(ay(i)/az(i)) ; atan(ax(i)/sqrt(ay(i)^2 + az(i)^2))];
    K=P*(P+R);
    a=a+K*(z-a);
    Angle=[Angle a];
    P=(eye(2)-K)*P;
    P_store(:,:,i)=P;
end
time=1:N;







