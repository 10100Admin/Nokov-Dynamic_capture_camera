function ContinuousEulerDEGXYZ= Quaternion2ContinuousEuler( LastEulerDEGXYZ, quat )
%EulerAngle rotate order ZYX
%LastEulerDEGXYZ---Last Frame EulerAngle DEG,[Rx,Ry,Rz]
%quat---Current Frame Quaternion,[qx,qy,qz,qw]
%ContinuousEulerDEGXYZ---Current Frame EulerAngle DEG,[Rx,Ry,Rz]

quatZYX=[quat(4),quat(3),quat(2),quat(1)];%[qw,qz,qy,qx]
[EulerX,EulerY,EulerZ]=quat2angle(quatZYX, 'ZYX');%rotate order ZYX
EulerDEGXYZ=[EulerX,EulerY,EulerZ]*180/pi;

ContinuousEulerDEGXYZ= ModifyContinuousEuler( LastEulerDEGXYZ, EulerDEGXYZ );

end