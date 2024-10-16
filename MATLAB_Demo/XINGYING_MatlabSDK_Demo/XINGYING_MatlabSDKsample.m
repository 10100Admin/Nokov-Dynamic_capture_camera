function [frameOfData] = XINGYING_MatlabSDKsample()
% Sample file to illustrate usage of the Matlab SDK for Nokov motioncapture software
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
	
	%print FrameOfMocapData
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
    
    % while(1)
	for Frame_i=1:1000
		data = mXINGYING_GetLastFrameOfMocapData();
        if(data.iFrame==0)%Get the LastFrameOfMocapData,or not
            pause(5/1000);%sleep
			continue;
		else
			curFrmNo = data.iFrame;
			if(curFrmNo==preFrmNo)
				continue;
			else
				preFrmNo = curFrmNo;
                [hour, minute, second, frame, subframe] = mXINGYING_DecodeTimecode(data.Timecode,data.TimecodeSubframe);
				fprintf("Timecode : %ld:%ld:%ld:%ld.%ld\n", hour, minute, second, frame, subframe);%print timecode stardard
				fprintf("FrameNO:%d\tTimeStamp:%ld\n", data.iFrame, data.iTimeStamp);

				fprintf("MarkerSet [Count=%d]\n", data.nMarkerSets);%Number of Markerset
                for i=1:data.nMarkerSets
					fprintf("Markerset%d: %s [nMarkers Count=%d]\n", i, data.MocapData(i).szName, data.MocapData(i).nMarkers);%Output the id and name of the i-th Markerset and the total number of named markers in the Markerset
					fprintf( "{\n");
                    
					for i_Marker=1:data.MocapData(i).nMarkers%Output the id and information (x, y, z) of the Marker point contained in the i-th Markerset
						fprintf("\tMarker%d: %3.2f,%3.2f,%3.2f\t",...
						data.MocapData(i).Markers(i_Marker*4-3),...
						data.MocapData(i).Markers(i_Marker*4-2),...
						data.MocapData(i).Markers(i_Marker*4-1),...
						data.MocapData(i).Markers(i_Marker*4));

                        m_Points_velaccCache(i,i_Marker,3,:)=m_Points_velaccCache(i,i_Marker,2,:);
                        m_Points_velaccCache(i,i_Marker,2,:)=m_Points_velaccCache(i,i_Marker,1,:);
                        m_Points_velaccCache(i,i_Marker,1,:)=[data.MocapData(i).Markers(i_Marker*4-2), data.MocapData(i).Markers(i_Marker*4-1), data.MocapData(i).Markers(i_Marker*4)];
                        vel= CalculateVelocity( m_FPS, squeeze(m_Points_velaccCache(i,i_Marker,:,:)), FrameFactor);
                        acc= CalculateAcceleration( m_FPS, squeeze(m_Points_velaccCache(i,i_Marker,:,:)), FrameFactor);
                        fprintf("\tvel: %3.2f,%3.2f,%3.2f,%3.2f \tacc: %3.2f,%3.2f,%3.2f,%3.2f\n",...
						vel(1),vel(2),vel(3),vel(4),acc(1),acc(2),acc(3),acc(4));
					end
					fprintf( "}\n");%The data output of the i-th Markerset is completed
				end
			
				%Print RigidBodies
				fprintf("Markerset.RigidBodies [Count=%d]\n", data.nRigidBodies);%Number of Markerset.Skeleton(skeleton)
				fprintf("{\n");
				for i = 1:data.nRigidBodies
                    fprintf("\t{\n");
					fprintf("\tid\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
					fprintf("\tRigidBody%d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",...
					data.RigidBodies(i).ID,...
					data.RigidBodies(i).x,...
					data.RigidBodies(i).y,...
					data.RigidBodies(i).z,...
					data.RigidBodies(i).qx,...
					data.RigidBodies(i).qy,...
					data.RigidBodies(i).qz,...
					data.RigidBodies(i).qw);
		            
                    if (flag_FirstFrame == 0)
                         for init_data = 1:data.nRigidBodies
                            LastFrameEulerDEGXYZ=[LastFrameEulerDEGXYZ;0 0 0];
                         end
                        
                        ContinuousEulerDEGXYZ= Quaternion2ContinuousEuler( [0 0 0], [data.RigidBodies(i).qx,data.RigidBodies(i).qy,data.RigidBodies(i).qz,data.RigidBodies(i).qw] );
                        LastFrameEulerDEGXYZ(i,1) = ContinuousEulerDEGXYZ(1);
                        LastFrameEulerDEGXYZ(i,2) = ContinuousEulerDEGXYZ(2);
                        LastFrameEulerDEGXYZ(i,3) = ContinuousEulerDEGXYZ(3);
                        flag_FirstFrame=1;
                       
                    else
                        ContinuousEulerDEGXYZ= Quaternion2ContinuousEuler( LastFrameEulerDEGXYZ(i,:), [data.RigidBodies(i).qx,data.RigidBodies(i).qy,data.RigidBodies(i).qz,data.RigidBodies(i).qw] );
                        LastFrameEulerDEGXYZ(i,1) = ContinuousEulerDEGXYZ(1);
                        LastFrameEulerDEGXYZ(i,2) = ContinuousEulerDEGXYZ(2);
                        LastFrameEulerDEGXYZ(i,3) = ContinuousEulerDEGXYZ(3);
                    end
                    fprintf("\tRigidBody EulerAngle:\t%3.2f\t%3.2f\t%3.2f\n",...
                        ContinuousEulerDEGXYZ(1),...
                        ContinuousEulerDEGXYZ(2),...
                        ContinuousEulerDEGXYZ(3));
                    
                    m_RigidBodies_velaccCache(1,i,3,:)=m_RigidBodies_velaccCache(1,i,2,:);
                    m_RigidBodies_velaccCache(1,i,2,:)=m_RigidBodies_velaccCache(1,i,1,:);
                    m_RigidBodies_velaccCache(1,i,1,:)=[data.RigidBodies(i).x, data.RigidBodies(i).y, data.RigidBodies(i).z];
                    vel= CalculateVelocity( m_FPS, squeeze(m_RigidBodies_velaccCache(1,i,:,:)), FrameFactor);
                    acc= CalculateAcceleration( m_FPS, squeeze(m_RigidBodies_velaccCache(1,i,:,:)), FrameFactor);
                        fprintf("\tvel: %3.2f,%3.2f,%3.2f,%3.2f \tacc: %3.2f,%3.2f,%3.2f,%3.2f\n",...
						vel(1),vel(2),vel(3),vel(4),acc(1),acc(2),acc(3),acc(4));

					fprintf("\tRigidBody markers [Count=%d]\n", data.RigidBodies(i).nMarkers);
					for iMarker = 1:data.RigidBodies(i).nMarkers %Output the id and information (x, y, z) of the marker associated with the j-th RigidBody (RigidBody)
						fprintf("\t\t");
						fprintf("Marker%d: ", data.RigidBodies(i).MarkerIDs(iMarker));
						fprintf("%3.2f,%3.2f,%3.2f\n",...
								data.RigidBodies(i).Markers(iMarker*4-2),...
								data.RigidBodies(i).Markers(iMarker*4-1),...
								data.RigidBodies(i).Markers(iMarker*4));
					end
		
					fprintf("\t}\n");%The data output of the j-th RigidBody (RigidBody) is completed
				end
				fprintf("}\n");
				
				%Print Skeletons
				fprintf("Markerset.Skeletons [Count=%d]\n", data.nSkeletons);%Number of Markerset.Skeleton(skeleton)
				for i=1:data.nSkeletons
					fprintf("Markerset%d.Skeleton [nRigidBodies Count=%d]\n", data.Skeletons(i).skeletonID, data.Skeletons(i).nRigidBodies);%Skeleton (Skeleton) of the i-th Markerset of the motion capture data, including the number of RigidBody (RigidBody)
					fprintf("{\n");
					
					for j=1:data.Skeletons(i).nRigidBodies %Output id and information(x, y, z, qx, qy, qz, qw) of the j-th RigidBody (RigidBody)
						fprintf("\tid\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
						fprintf("\tRigidBody%d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",...
						data.Skeletons(i).RigidBodyData(j).ID,...
						data.Skeletons(i).RigidBodyData(j).x,...
						data.Skeletons(i).RigidBodyData(j).y,...
						data.Skeletons(i).RigidBodyData(j).z,...
						data.Skeletons(i).RigidBodyData(j).qx,...
						data.Skeletons(i).RigidBodyData(j).qy,...
						data.Skeletons(i).RigidBodyData(j).qz,...
						data.Skeletons(i).RigidBodyData(j).qw);
						
						fprintf("\tRigidBody markers [Count=%d]\n", data.Skeletons(i).RigidBodyData(j).nMarkers);
						
						for iMarker = 1:data.Skeletons(i).RigidBodyData(j).nMarkers%Output the id and information (x, y, z) of the marker associated with the j-th RigidBody (RigidBody)
							fprintf("\t\t");
							fprintf("Marker%d: ", data.Skeletons(i).RigidBodyData(j).MarkerIDs(iMarker));
							fprintf("%3.2f,%3.2f,%3.2f\n",...
								data.Skeletons(i).RigidBodyData(j).Markers(iMarker*4-2),...
								data.Skeletons(i).RigidBodyData(j).Markers(iMarker*4-1),...
								data.Skeletons(i).RigidBodyData(j).Markers(iMarker*4));
						end
					end
					
					fprintf("}\n");%The data output of the j-th RigidBody (RigidBody) is completed
					
				end
				
				fprintf("Other Markers [Count=%d]\n", data.nOtherMarkers);%Output the total number of unnamed markers contained
				fprintf("{\n");
				for iMarker = 1:data.nOtherMarkers %Output the id and information (x, y, z) of the unnamed marker included
					fprintf("\tOther Marker%d: %3.2f,%3.2f,%3.2f\n",...
					data.OtherMarkers(iMarker*4-3),...
					data.OtherMarkers(iMarker*4-2),...
					data.OtherMarkers(iMarker*4-1),...
					data.OtherMarkers(iMarker*4));
				end
				fprintf("}\n\n");%The data output of unnamed marker is completed, and the data output of one frame is completed
				
				fprintf("Analog [Count=%d]\n", data.nAnalogdatas);%Output the total number of analog data contained
				fprintf("{\n");
				for iAnalogdatas = 1:data.nAnalogdatas
					fprintf("\tAnalogData %d: %3.2f\n", iAnalogdatas, data.Analogdata(iAnalogdatas));
				end
				fprintf("}\n\n");%The data output of analog data is completed, and the data output of one frame is completed
				
				fprintf("nLabeledMarkers [Count=%d]\n", data.nLabeledMarkers);%Output the total number of analog data contained
				fprintf("{\n");
				fprintf("\tLabeledMarkers id\t\tx\ty\tz\tsize\tparams\n");
				for iLabeledMarkers = 1:data.nLabeledMarkers
						fprintf("\tLabeledMarkers%d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",...
						data.LabeledMarkers(iLabeledMarkers).ID,...
						data.LabeledMarkers(iLabeledMarkers).x,...
						data.LabeledMarkers(iLabeledMarkers).y,...
						data.LabeledMarkers(iLabeledMarkers).z,...
						data.LabeledMarkers(iLabeledMarkers).size,...
						data.LabeledMarkers(iLabeledMarkers).params);
				end
				fprintf("}\n\n");%The data output of analog data is completed, and the data output of one frame is completed
			end
		end
		
		mXINGYING_FreeFrame(data);%Free the MocapData of the current frame
	end
	
else
    fprintf("Unable to connect to server.\n");
end

returnValue2 = mXINGYING_Uninitialize();
if(returnValue2==0)
    fprintf("Uninitialize client.  Exiting\n");
else
    fprintf("Error Uninitialize client.  Exiting\n");
end

end


