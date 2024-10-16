clear
% fig=tiledlayout('flow')
webcamlist
cam=webcam('HP 320 FHD Webcam');
fig = figure('NumberTitle','off','MenuBar','none');
fig.Name = '点位图';
ax = axes(fig); 
frame = snapshot(cam); 
im = image(ax,zeros(size(frame),'uint8')); 
axis(ax,'image');
preview(cam,im)
% hold on 
% % preview(cam,im)
% nexttile
% p=plot([1,2],[1,2]);
% uistack(p,'top')
% % hold off