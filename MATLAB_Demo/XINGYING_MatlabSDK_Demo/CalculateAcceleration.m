function accel= CalculateAcceleration( m_FPS, m_Points, m_FrameFactor )
%m_FPS---Frame Rate
%m_Points---Point xyz
%set;m_Points[iFrame][3];m_Points[iFrame][1]--x;m_Points[iFrame][2]--y;m_Points[iFrame][3]--z
%m_FrameFactor---Frame Factor
TR=(m_FrameFactor+1)/2;
DeltaFrame=TR-1;
if(m_FrameFactor == 0)
  Ax = 0;
  Ay = 0;
  Az = 0;
  Ar = 0;
end
%Ax = m_FPS * m_FPS * ((m_Points[m_FrameFactor].x - m_Points[TR].x) / (TR)-(m_Points[TR].x - m_Points[0].x) / (TR))/(TR);
Ax = m_FPS * m_FPS * ((m_Points(m_FrameFactor,1) - 2 * m_Points(TR,1) + m_Points(1,1)) / (DeltaFrame*DeltaFrame));
Ay = m_FPS * m_FPS * ((m_Points(m_FrameFactor,2) - 2 * m_Points(TR,2) + m_Points(1,2)) / (DeltaFrame*DeltaFrame));
Az = m_FPS * m_FPS * ((m_Points(m_FrameFactor,3) - 2 * m_Points(TR,3) + m_Points(1,3))/ (DeltaFrame*DeltaFrame));
Ar = sqrt(Ax * Ax + Ay * Ay + Az * Az);

accel=[Ax Ay Az Ar];
end