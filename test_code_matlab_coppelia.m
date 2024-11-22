clear all, close all, clc;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Program started');
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
    disp('Connected to remote API server');
    end
    
     % Now try to retrieve data in a blocking fashion (i.e. a service call):
 
     [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
     
     
      if (res==sim.simx_return_ok)
            fprintf('Number of objects in the scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
      end
        
      
 % Now retrieve streaming data (i.e. in a non-blocking fashion):
       
       sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) ; % Initialize streaming 
       sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim from Matlab!',sim.simx_opmode_oneshot);
      
       [r, h_joint(1)]=sim.simxGetObjectHandle(clientID,'J1', sim.simx_opmode_blocking);
       [r, h_joint(2)]=sim.simxGetObjectHandle(clientID,'J2', sim.simx_opmode_blocking);

        qdes_deg=[66.0930 , -111.6403];
          qdes_rad=deg2rad(qdes_deg);


     nbddl = 2 ; 

     for i=1:nbddl
        [r, q(i)]=sim.simxGetJointPosition(clientID, h_joint(i), sim.simx_opmode_blocking); %% Get robot joint position

            sim.simxSetJointTargetPosition(clientID, h_joint(i), qdes_rad(i), sim.simx_opmode_blocking) % Set Robot joint pos
  
      end

 