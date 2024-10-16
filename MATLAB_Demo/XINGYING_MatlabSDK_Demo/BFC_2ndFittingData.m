function [frameOfData] = BFC_2ndFittingData()
%%
szServerAddress = '192.168.31.144';%10.1.1.198
DataDescriptions = [];
data = [];

m_FPS=120;%Frame Rate,user set7
FrameFactor=3;% 3 Frame Calculate,user set
MaxMarkersetNum=50;%MaxMarkerSetNum,user set;Greater than or equal to the actual quantity
MaxMarkerNum=300;%MaxMarkerNum in Markerset,user set;Greater than or equal to the actual quantity
MaxRigidbodyNum=10;%MaxRigidbodyNum,user set;Greater than or equal to the actual quantity

% Load the SDK libraries and initialize ethernet communication
returnValue = mXINGYING_Initialize(szServerAddress);
if(returnValue==0)
    fprintf("Client initialized and ready.\n");
	
	%print DataDescriptions
	DataDescriptions = mXINGYING_GetDataDescriptions();
	if (isempty(DataDescriptions))
		fprintf("Get DataDescriptions failed.\n");
	else
		for dsIndex = 1:DataDescriptions.nDataDescriptions
			switch (DataDescriptions.arrDataDescriptions(dsIndex).type)
				case {1}%Descriptor_MarkerSetEx
					fprintf("MarkerSetExName : %s\n", DataDescriptions.arrDataDescriptions(dsIndex).MarkerSetData.szName);
                    
					for markerIndex = 1:DataDescriptions.arrDataDescriptions(dsIndex).MarkerSetData.nMarkers
						fprintf("Marker[%d] : %3.2f,%3.2f,%3.2f\n", markerIndex, DataDescriptions.arrDataDescriptions(dsIndex).MarkerSetData.Markers(markerIndex*3-2), DataDescriptions.arrDataDescriptions(dsIndex).MarkerSetData.Markers(markerIndex*3-1), DataDescriptions.arrDataDescriptions(dsIndex).MarkerSetData.Markers(markerIndex*3));
					end
					
				case {2}%Descriptor_MarkerSet
					fprintf("MarkerSetName : %s\n", DataDescriptions.arrDataDescriptions(dsIndex).MarkerSetDescription.szName);
                    
					for markerIndex = 1:DataDescriptions.arrDataDescriptions(dsIndex).MarkerSetDescription.nMarkers
						fprintf("Marker[%d] : %s\n", markerIndex, DataDescriptions.arrDataDescriptions(dsIndex).MarkerSetDescription.szMarkerNames(markerIndex).szMarkerNames);
					end
					
				case {3}%Descriptor_RigidBody
					fprintf("RigidBody:%s ID:%d\n", DataDescriptions.arrDataDescriptions(dsIndex).RigidBodyDescription.szName, DataDescriptions.arrDataDescriptions(dsIndex).RigidBodyDescription.ID);
					
				case {4}%Descriptor_Skeleton
					fprintf("Skeleton:%s\n", DataDescriptions.arrDataDescriptions(dsIndex).SkeletonDescription.szName);
					for boneIndex = 1:DataDescriptions.arrDataDescriptions(dsIndex).SkeletonDescription.nRigidBodies
						fprintf("[%d] %s\n", DataDescriptions.arrDataDescriptions(dsIndex).SkeletonDescription.RigidBodies(boneIndex).ID, DataDescriptions.arrDataDescriptions(dsIndex).SkeletonDescription.RigidBodies(boneIndex).szName);
					end
					
				case {5}%Descriptor_ForcePlate
					for channelIdx = 1:DataDescriptions.arrDataDescriptions(dsIndex).ForcePlateDescription.nChannels
						fprintf("Channel:%d %s\n", channelIdx,...
							DataDescriptions.arrDataDescriptions(dsIndex).ForcePlateDescription.szChannelNames(channelIdx).szChannelNames);
					end
				case {6}%Descriptor_Param
					fprintf("FrameRate: %d\n", DataDescriptions.arrDataDescriptions(dsIndex).DataParam.nFrameRate);
					
				otherwise
					fprintf("DataDescriptions.arrDataDescriptions(dsIndex).type is error\n");
			end
		end
    end
    %% fitting configure & int
    % figure
    num_makerset=1;
    nmaker=8;
    stack_zhou_angle=zeros(1,100);
    maxplot=100;
    tiledlayout(5,1)
    nexttile
    h1 = animatedline('MaximumNumPoints',maxplot);
    set(gca,'xtick',[],'xticklabel',[])
    set(gca,'looseInset',[0 0 0 0])
    ylim([0,180])
    set(gca,'ytick',0:30:180)
    t1=title('肘关节','FontSize',20);
    grid on
  
    nexttile([1 1])
    h2 = animatedline('MaximumNumPoints',maxplot);
    t2=title('肩关节','FontSize',20);
    set(gca,'xtick',[],'xticklabel',[])
    ylim([0,180])
    set(gca,'ytick',0:30:180)
    grid on

    nexttile([1 1])
    h3 = animatedline('MaximumNumPoints',maxplot);
    t3=title('髋关节','FontSize',20);
    set(gca,'xtick',[],'xticklabel',[])
    ylim([0,180])
    set(gca,'ytick',0:30:180)
    grid on

    nexttile([1 1])
    h4 = animatedline('MaximumNumPoints',maxplot);
    t4=title('膝关节','FontSize',20);
    set(gca,'xtick',[],'xticklabel',[])
    ylim([0,180])
    set(gca,'ytick',0:30:180)
    grid on
    
    nexttile([1 1])
    h5 = animatedline('MaximumNumPoints',maxplot);
    t5=title('踝关节','FontSize',20);
    set(gca,'xtick',[],'xticklabel',[])
    ylim([0,180])
    set(gca,'ytick',0:30:180)
    grid on

    %%
    fprintf("\n\nprint FrameOfMocapData.\n");
	preFrmNo=0;
	curFrmNo=0;
    
    m_Points_velaccCache=[];
    m_RigidBodies_velaccCache=[];
    m_Points_velaccCache=zeros(MaxMarkersetNum,MaxMarkerNum,FrameFactor,3);
    m_RigidBodies_velaccCache=zeros(MaxMarkersetNum,MaxRigidbodyNum,FrameFactor,3);
    
    i_Framecnt=0;
    LastFrameEulerDEGXYZ=[];%Last Frame EulerAngle DEG,[Rx,Ry,Rz]
    ContinuousEulerDEGXYZ=[0 0 0];
    flag_FirstFrame=0;

    
    while(1)
	% for Frame_i=1:1000
		data = mXINGYING_GetLastFrameOfMocapData();
        if(data.iFrame==0)%Get the LastFrameOfMocapData,or not
            pause(5/1000);%sleep
			continue;
		else
			curFrmNo = data.iFrame;
			if(curFrmNo==preFrmNo)
				continue;
            else
                %% 打印数据
				preFrmNo = curFrmNo;
                [hour, minute, second, frame, subframe] = mXINGYING_DecodeTimecode(data.Timecode,data.TimecodeSubframe);
				fprintf("Timecode : %ld:%ld:%ld:%ld.%ld\n", hour, minute, second, frame, subframe);%print timecode stardard
				fprintf("FrameNO:%d\tTimeStamp:%ld\n", data.iFrame, data.iTimeStamp);

				fprintf("MarkerSet [Count=%d]\n", data.nMarkerSets);%Number of Markerset
                % for i=1:data.nMarkerSets
				% 	fprintf("Markerset%d: %s [nMarkers Count=%d]\n", i, data.MocapData(i).szName, data.MocapData(i).nMarkers);%Output the id and name of the i-th Markerset and the total number of named markers in the Markerset
				% 	fprintf( "{\n");
                % 
				% 	for i_Marker=1:data.MocapData(i).nMarkers%Output the id and information (x, y, z) of the Marker point contained in the i-th Markerset
					% 	fprintf("\tMarker%d: %3.2f,%3.2f,%3.2f\t\n",...
					% 	data.MocapData(i).Markers(i_Marker*4-3),...
					% 	data.MocapData(i).Markers(i_Marker*4-2),...
					% 	data.MocapData(i).Markers(i_Marker*4-1),...
					% 	data.MocapData(i).Markers(i_Marker*4));
                % 
                %         % m_Points_velaccCache(i,i_Marker,3,:)=m_Points_velaccCache(i,i_Marker,2,:);
                %         % m_Points_velaccCache(i,i_Marker,2,:)=m_Points_velaccCache(i,i_Marker,1,:);
                %         % m_Points_velaccCache(i,i_Marker,1,:)=[data.MocapData(i).Markers(i_Marker*4-2), data.MocapData(i).Markers(i_Marker*4-1), data.MocapData(i).Markers(i_Marker*4)];
                %         % vel= CalculateVelocity( m_FPS, squeeze(m_Points_velaccCache(i,i_Marker,:,:)), FrameFactor);
                %         % acc= CalculateAcceleration( m_FPS, squeeze(m_Points_velaccCache(i,i_Marker,:,:)), FrameFactor);
                %         % fprintf("\tvel: %3.2f,%3.2f,%3.2f,%3.2f \tacc: %3.2f,%3.2f,%3.2f,%3.2f\n",...
					% 	% vel(1),vel(2),vel(3),vel(4),acc(1),acc(2),acc(3),acc(4));
				% 	end
				% 	fprintf( "}\n");%The data output of the i-th Markerset is completed
                % end
                %% plot
                fitting_data=reshape(data.MocapData(num_makerset).Markers(1:nmaker*4),[4,nmaker])';
                fitting_data(:,1)=[];

                angle_zhou=real(getangle_vec(fitting_data(2,:),fitting_data(1,:),fitting_data(2,:),fitting_data(3,:)));
                angle_jian=real(getangle_vec(fitting_data(3,:),fitting_data(1,:),fitting_data(3,:),fitting_data(4,:)));
                angle_kuan=real(getangle_vec(fitting_data(4,:),fitting_data(3,:),fitting_data(4,:),fitting_data(5,:)));
                angle_xi=real(getangle_vec(fitting_data(5,:),fitting_data(4,:),fitting_data(5,:),fitting_data(6,:)));
                angle_huai=real(getangle_vec(fitting_data(6,:),fitting_data(5,:),fitting_data(7,:),fitting_data(8,:)));
                t1.String=['肘关节角度=',num2str(angle_zhou,'%.1f'),'°'];
                t2.String=['肩关节角度=',num2str(angle_jian,'%.1f'),'°'];
                t3.String=['髋关节角度=',num2str(angle_kuan,'%.1f'),'°'];
                t4.String=['膝关节角度=',num2str(angle_xi,'%.1f'),'°'];
                t5.String=['踝关节角度=',num2str(angle_huai,'%.1f'),'°'];



                addpoints(h1,curFrmNo,angle_zhou);
                addpoints(h2,curFrmNo,angle_jian);
                addpoints(h3,curFrmNo,angle_kuan);
                addpoints(h4,curFrmNo,angle_xi);
                addpoints(h5,curFrmNo,angle_huai);
    

                
                drawnow;




    








    
            end
        end


    end


 

end
end

%% 计算向量角度
function alpha = getangle_vec(a,b,c,d)
    alpha=acos(dot(b-a,d-c)/(norm(b-a)*norm(d-c)))/pi*180;
end

