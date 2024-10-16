function [frameOfData] = BFC_2DPoint()
%%
szServerAddress = '10.1.1.198';%10.1.1.198
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
    maxplot=100;
    fig = figure('NumberTitle','off','MenuBar','none');
    h1 = animatedline('MaximumNumPoints',maxplot);
    xlim([0,1920])
    ylim([0,1080])
    hobj=gca;
    set(hobj.XAxis,'Visible','off')
    set(hobj.YAxis,'Visible','off')
    set(fig, 'Position', [100, 100, 480, 270])
    M=[1.17067263169251	3.65909965839065	0.199719682158765
-0.0325430096226696	-0.0841808531938486	0.731378412610079];
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
                pixel_data=M*fitting_data';

                clearpoints(h1)
                addpoints(h1,pixel_data(1,:),pixel_data(2,:))
    
                drawnow;




    








    
            end
        end


    end


 

end
end

