function [DegyawPitchRoll_Result,DeltaDeg_Check]= ModifyConEulerMethod4( DegyawPitchRoll_PreFrame, DegyawPitchRoll_CurFrame )
%EulerAngle rotate order ZYX
%DegyawPitchRoll_PreFrame---Last Frame EulerAngle DEG,[Rx,Ry,Rz]
%DegyawPitchRoll_CurFrame---Current Frame EulerAngle DEG,[Rx,Ry,Rz]
%DegyawPitchRoll_Result---Result EulerAngle DEG,[Rx,Ry,Rz]
%DeltaDeg_Check---Result delta flag

%static q_vec_type LastDegyawPitchRoll_Mod;
m_LastDegyawPitchRoll_Mod(1) = mod(DegyawPitchRoll_PreFrame(1), 360);
m_LastDegyawPitchRoll_Mod(2) = mod(DegyawPitchRoll_PreFrame(2), 360);
m_LastDegyawPitchRoll_Mod(3) = mod(DegyawPitchRoll_PreFrame(3), 360);

%static q_vec_type LastDegyawPitchRoll_Round;
m_LastDegyawPitchRoll_Round(1) = DegyawPitchRoll_PreFrame(1) - m_LastDegyawPitchRoll_Mod(1);
m_LastDegyawPitchRoll_Round(2) = DegyawPitchRoll_PreFrame(2) - m_LastDegyawPitchRoll_Mod(2);
m_LastDegyawPitchRoll_Round(3) = DegyawPitchRoll_PreFrame(3) - m_LastDegyawPitchRoll_Mod(3);

%X+-180度，Y1+Y2=180或者Y1+Y2=-180，Z+-180度
DegyawPitchRoll_X1 = m_LastDegyawPitchRoll_Round(1) + DegyawPitchRoll_CurFrame(1) + 180;
DegyawPitchRoll_X2 = m_LastDegyawPitchRoll_Round(1) + DegyawPitchRoll_CurFrame(1) - 180;
if (abs(DegyawPitchRoll_X1 - DegyawPitchRoll_PreFrame(1)) > abs(DegyawPitchRoll_X2 - DegyawPitchRoll_PreFrame(1)))
	DegyawPitchRoll_Result(1) = DegyawPitchRoll_X2;
else
	DegyawPitchRoll_Result(1) = DegyawPitchRoll_X1;
end

DegyawPitchRoll_Y1 = m_LastDegyawPitchRoll_Round(2) + 180 - DegyawPitchRoll_CurFrame(2);
DegyawPitchRoll_Y2 = m_LastDegyawPitchRoll_Round(2) - 180 - DegyawPitchRoll_CurFrame(2);
if (abs(DegyawPitchRoll_Y1 - DegyawPitchRoll_PreFrame(2)) > abs(DegyawPitchRoll_Y2 - DegyawPitchRoll_PreFrame(2)))
	DegyawPitchRoll_Result(2) = DegyawPitchRoll_Y2;
else
	DegyawPitchRoll_Result(2) = DegyawPitchRoll_Y1;
end

DegyawPitchRoll_Z1 = m_LastDegyawPitchRoll_Round(3) + DegyawPitchRoll_CurFrame(3) + 180;
DegyawPitchRoll_Z2 = m_LastDegyawPitchRoll_Round(3) + DegyawPitchRoll_CurFrame(3) - 180;
if (abs(DegyawPitchRoll_Z1 - DegyawPitchRoll_PreFrame(3)) > abs(DegyawPitchRoll_Z2 - DegyawPitchRoll_PreFrame(3)))
	DegyawPitchRoll_Result(3) = DegyawPitchRoll_Z2;
else
	DegyawPitchRoll_Result(3) = DegyawPitchRoll_Z1;
end

DeltaDeg_Check = abs(DegyawPitchRoll_Result(1) - DegyawPitchRoll_PreFrame(1)) + abs(DegyawPitchRoll_Result(2) - DegyawPitchRoll_PreFrame(2)) + abs(DegyawPitchRoll_Result(3) - DegyawPitchRoll_PreFrame(3));

end