function ContinuousEulerDEGXYZ= ModifyContinuousEuler( LastEulerDEGXYZ, EulerDEGXYZ )
%EulerAngle rotate order ZYX
%LastEulerDEGXYZ---Last Frame EulerAngle DEG,[Rx,Ry,Rz]
%EulerDEGXYZ---Current Frame Quaternion,[qx,qy,qz,qw]
%ContinuousEulerDEGXYZ---Current Frame EulerAngle DEG,[Rx,Ry,Rz]

[DegyawPitchRoll_Method3, DeltaDeg_Method3]= ModifyConEulerMethod3( LastEulerDEGXYZ, EulerDEGXYZ );
if (DeltaDeg_Method3 <= 180)
	ContinuousEulerDEGXYZ(1) = DegyawPitchRoll_Method3(1);
	ContinuousEulerDEGXYZ(2) = DegyawPitchRoll_Method3(2);
	ContinuousEulerDEGXYZ(3) = DegyawPitchRoll_Method3(3);
end

[DegyawPitchRoll_Method2, DeltaDeg_Method2] = ModifyConEulerMethod2(LastEulerDEGXYZ, EulerDEGXYZ);
if (DeltaDeg_Method2 <= 180)
	ContinuousEulerDEGXYZ(1) = DegyawPitchRoll_Method2(1);
	ContinuousEulerDEGXYZ(2) = DegyawPitchRoll_Method2(2);
	ContinuousEulerDEGXYZ(3) = DegyawPitchRoll_Method2(3);
end

[DegyawPitchRoll_Method1, DeltaDeg_Method1] = ModifyConEulerMethod1(LastEulerDEGXYZ, EulerDEGXYZ);
if (DeltaDeg_Method1 <= 180)
    ContinuousEulerDEGXYZ(1) = DegyawPitchRoll_Method1(1);
	ContinuousEulerDEGXYZ(2) = DegyawPitchRoll_Method1(2);
	ContinuousEulerDEGXYZ(3) = DegyawPitchRoll_Method1(3);
end

[DegyawPitchRoll_Method4, DeltaDeg_Method4] = ModifyConEulerMethod4(LastEulerDEGXYZ, EulerDEGXYZ);
if (DeltaDeg_Method4 <= 180)
    ContinuousEulerDEGXYZ(1) = DegyawPitchRoll_Method4(1);
	ContinuousEulerDEGXYZ(2) = DegyawPitchRoll_Method4(2);
	ContinuousEulerDEGXYZ(3) = DegyawPitchRoll_Method4(3);
end

DeltaDeg_Cnt(1) = DeltaDeg_Method1;
DeltaDeg_Cnt(2) = DeltaDeg_Method2;
DeltaDeg_Cnt(3) = DeltaDeg_Method3;
DeltaDeg_Cnt(4) = DeltaDeg_Method4;
DeltaDeg_CntT=DeltaDeg_Cnt';
DeltaDeg_Cnt=sort(DeltaDeg_CntT);

if (DeltaDeg_Cnt(1) == DeltaDeg_Method1)
	ContinuousEulerDEGXYZ(1) = DegyawPitchRoll_Method1(1);
	ContinuousEulerDEGXYZ(2) = DegyawPitchRoll_Method1(2);
	ContinuousEulerDEGXYZ(3) = DegyawPitchRoll_Method1(3);
elseif (DeltaDeg_Cnt(1) == DeltaDeg_Method2)
	ContinuousEulerDEGXYZ(1) = DegyawPitchRoll_Method2(1);
	ContinuousEulerDEGXYZ(2) = DegyawPitchRoll_Method2(2);
	ContinuousEulerDEGXYZ(3) = DegyawPitchRoll_Method2(3);
elseif (DeltaDeg_Cnt(1) == DeltaDeg_Method3)
	ContinuousEulerDEGXYZ(1) = DegyawPitchRoll_Method3(1);
	ContinuousEulerDEGXYZ(2) = DegyawPitchRoll_Method3(2);
	ContinuousEulerDEGXYZ(3) = DegyawPitchRoll_Method3(3);
elseif (DeltaDeg_Cnt(1) == DeltaDeg_Method4)
	ContinuousEulerDEGXYZ(1) = DegyawPitchRoll_Method4(1);
	ContinuousEulerDEGXYZ(2) = DegyawPitchRoll_Method4(2);
	ContinuousEulerDEGXYZ(3) = DegyawPitchRoll_Method4(3);
else
    %error
end


end