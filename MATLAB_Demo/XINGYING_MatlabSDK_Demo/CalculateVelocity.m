function vel= CalculateVelocity( m_FPS, m_Points, m_FrameFactor )
%m_FPS---Frame Rate
%m_Points---Point xyz set
%m_FrameFactor---Frame Factor
%vel---velocity

if(m_FrameFactor == 0)
  Vx = 0;
  Vy = 0;
  Vz = 0;
  Vr = 0;
end
Vx = -m_FPS * (m_Points(m_FrameFactor,1) - m_Points(1,1)) / (m_FrameFactor-1);
Vy = -m_FPS * (m_Points(m_FrameFactor,2) - m_Points(1,2)) / (m_FrameFactor-1);
Vz = -m_FPS * (m_Points(m_FrameFactor,3) - m_Points(1,3)) / (m_FrameFactor-1);
Vr = sqrt(Vx * Vx + Vy * Vy + Vz * Vz);

vel=[Vx Vy Vz Vr];
end