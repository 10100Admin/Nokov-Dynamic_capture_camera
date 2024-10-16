function [DegyawPitchRoll_Result,DeltaDeg_Check]= ModifyConEulerMethod1( DegyawPitchRoll_PreFrame, DegyawPitchRoll_CurFrame )
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

%解法一：X-90度，Y不变，Z-90度
DegyawPitchRoll_Result(1) = m_LastDegyawPitchRoll_Round(1) + DegyawPitchRoll_CurFrame(1) - 90;
DegyawPitchRoll_Result(2) = DegyawPitchRoll_CurFrame(2);
DegyawPitchRoll_Result(3) = m_LastDegyawPitchRoll_Round(3) + DegyawPitchRoll_CurFrame(3) - 90;

DeltaDeg_Check = abs(DegyawPitchRoll_Result(1) - DegyawPitchRoll_PreFrame(1)) + abs(DegyawPitchRoll_Result(2) - DegyawPitchRoll_PreFrame(2)) + abs(DegyawPitchRoll_Result(3) - DegyawPitchRoll_PreFrame(3));

end