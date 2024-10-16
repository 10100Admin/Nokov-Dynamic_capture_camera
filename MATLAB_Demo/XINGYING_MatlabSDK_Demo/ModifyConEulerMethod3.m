function [DegyawPitchRoll_Result,DeltaDeg_Check]= ModifyConEulerMethod3( DegyawPitchRoll_PreFrame, DegyawPitchRoll_CurFrame )
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

%通用的加减360度解法，确定方向
DegyawPitchRoll_X1 = m_LastDegyawPitchRoll_Round(1) + DegyawPitchRoll_CurFrame(1);
DegyawPitchRoll_X2 = m_LastDegyawPitchRoll_Round(1) + DegyawPitchRoll_CurFrame(1) + 360;
DegyawPitchRoll_X3 = m_LastDegyawPitchRoll_Round(1) + DegyawPitchRoll_CurFrame(1) - 360;

DeltaDeg_Cnt_X(1) = abs(DegyawPitchRoll_X1 - DegyawPitchRoll_PreFrame(1));
DeltaDeg_Cnt_Xstay(1) = DeltaDeg_Cnt_X(1);
DeltaDeg_Cnt_X(2) = abs(DegyawPitchRoll_X2 - DegyawPitchRoll_PreFrame(1));
DeltaDeg_Cnt_Xstay(2) = DeltaDeg_Cnt_X(2);
DeltaDeg_Cnt_X(3) = abs(DegyawPitchRoll_X3 - DegyawPitchRoll_PreFrame(1));
DeltaDeg_Cnt_Xstay(3) = DeltaDeg_Cnt_X(3);
DeltaDeg_Cnt_XT=DeltaDeg_Cnt_X';
DeltaDeg_Cnt_X=sort(DeltaDeg_Cnt_XT);
if (DeltaDeg_Cnt_X(1) == DeltaDeg_Cnt_Xstay(1))
	DegyawPitchRoll_Result(1) = DegyawPitchRoll_X1;
elseif (DeltaDeg_Cnt_X(1) == DeltaDeg_Cnt_Xstay(2))
	DegyawPitchRoll_Result(1) = DegyawPitchRoll_X2;
elseif (DeltaDeg_Cnt_X(1) == DeltaDeg_Cnt_Xstay(3))
	DegyawPitchRoll_Result(1) = DegyawPitchRoll_X3;
else
    %error
end

DegyawPitchRoll_Y1 = m_LastDegyawPitchRoll_Round(2) + DegyawPitchRoll_CurFrame(2);
DegyawPitchRoll_Y2 = m_LastDegyawPitchRoll_Round(2) + DegyawPitchRoll_CurFrame(2) + 360;
DegyawPitchRoll_Y3 = m_LastDegyawPitchRoll_Round(2) + DegyawPitchRoll_CurFrame(2) - 360;

DeltaDeg_Cnt_Y(1) = abs(DegyawPitchRoll_Y1 - DegyawPitchRoll_PreFrame(2));
DeltaDeg_Cnt_Ystay(1) = DeltaDeg_Cnt_Y(1);
DeltaDeg_Cnt_Y(2) = abs(DegyawPitchRoll_Y2 - DegyawPitchRoll_PreFrame(2));
DeltaDeg_Cnt_Ystay(2) = DeltaDeg_Cnt_Y(2);
DeltaDeg_Cnt_Y(3) = abs(DegyawPitchRoll_Y3 - DegyawPitchRoll_PreFrame(2));
DeltaDeg_Cnt_Ystay(3) = DeltaDeg_Cnt_Y(3);

DeltaDeg_Cnt_YT=DeltaDeg_Cnt_Y';
DeltaDeg_Cnt_Y=sort(DeltaDeg_Cnt_YT);
if (DeltaDeg_Cnt_Y(1) == DeltaDeg_Cnt_Ystay(1))
	DegyawPitchRoll_Result(2) = DegyawPitchRoll_Y1;
elseif (DeltaDeg_Cnt_Y(1) == DeltaDeg_Cnt_Ystay(2))
    DegyawPitchRoll_Result(2) = DegyawPitchRoll_Y2;
elseif (DeltaDeg_Cnt_Y(1) == DeltaDeg_Cnt_Ystay(3))
	DegyawPitchRoll_Result(2) = DegyawPitchRoll_Y3;
else
    %error
end

DegyawPitchRoll_Z1 = m_LastDegyawPitchRoll_Round(3) + DegyawPitchRoll_CurFrame(3);
DegyawPitchRoll_Z2 = m_LastDegyawPitchRoll_Round(3) + DegyawPitchRoll_CurFrame(3) + 360;
DegyawPitchRoll_Z3 = m_LastDegyawPitchRoll_Round(3) + DegyawPitchRoll_CurFrame(3) - 360;

DeltaDeg_Cnt_Z(1) = abs(DegyawPitchRoll_Z1 - DegyawPitchRoll_PreFrame(3));
DeltaDeg_Cnt_Zstay(1) = DeltaDeg_Cnt_Z(1);
DeltaDeg_Cnt_Z(2) = abs(DegyawPitchRoll_Z2 - DegyawPitchRoll_PreFrame(3));
DeltaDeg_Cnt_Zstay(2) = DeltaDeg_Cnt_Z(2);
DeltaDeg_Cnt_Z(3) = abs(DegyawPitchRoll_Z3 - DegyawPitchRoll_PreFrame(3));
DeltaDeg_Cnt_Zstay(3) = DeltaDeg_Cnt_Z(3);
DeltaDeg_Cnt_ZT=DeltaDeg_Cnt_Z';
DeltaDeg_Cnt_Z=sort(DeltaDeg_Cnt_ZT);
if (DeltaDeg_Cnt_Z(1) == DeltaDeg_Cnt_Zstay(1))
	DegyawPitchRoll_Result(3) = DegyawPitchRoll_Z1;
elseif (DeltaDeg_Cnt_Z(1) == DeltaDeg_Cnt_Zstay(2))
	DegyawPitchRoll_Result(3) = DegyawPitchRoll_Z2;
elseif (DeltaDeg_Cnt_Z(1) == DeltaDeg_Cnt_Zstay(3))
	DegyawPitchRoll_Result(3) = DegyawPitchRoll_Z3;
else
	%error
end

DeltaDeg_Check = abs(DegyawPitchRoll_Result(1) - DegyawPitchRoll_PreFrame(1)) + abs(DegyawPitchRoll_Result(2) - DegyawPitchRoll_PreFrame(2)) + abs(DegyawPitchRoll_Result(3) - DegyawPitchRoll_PreFrame(3));

end