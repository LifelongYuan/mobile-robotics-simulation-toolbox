clear;
%% Initialization
% important hyperparas
numRobots = 8;
is_init_near=false;
max_translational_v = 1.4;  % m/s
max_rotational_w = 2.84;     % rad/s
sampleTime = 0.05;              % Sample time [s]
tVec = 0:sampleTime:10000;        % Time array
distance_control_id_pair = [1;8];
rand_id = 2;
nearest_seleted_num = 2;
delta = 0.1;
k_vector_control = 0.6;
k_gradient_control = 0.4;
k_formation_keeping = 0.5;
k_leader_tracking = 0.5;
% multi_env settings
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.15;
env.showTrajectory = false;
env.showDesired = true;
env.showConnection = true;
env.showRealTime = true;
env.showCommand = false;
env.saveData = false;

% variable initialization
vel_xy = zeros(2,numRobots);
vel_vw = zeros(2,numRobots);
d_poses = zeros(3,numRobots);
distance_u = zeros(2,1);
neighbor_list_ordered = zeros(nearest_seleted_num,1);

% calculate base graph_matrix_semi.
graph_matrix_semi = Cal_graph_matrix(numRobots);
% read q_desire from files
fid= fopen('formation_data.txt', 'r');
q_desire =fscanf(fid, '%f', [numRobots*2,1]);

% calculate initial pose
if is_init_near==true
    poses = zeros(3,numRobots);
    for i=1:numRobots
        poses(1,i) = q_desire(i*2-1) + randn;
        poses(2,i) = q_desire(i*2) + randn;
    end
else
    poses = [10*(rand(2,numRobots))-5;pi*rand(1,numRobots)];
end

%% Calculate Control Matrix
matrix_value = Cal_Control_Matrix(numRobots,graph_matrix_semi);

%% Simulation loop
for idx = 2:numel(tVec)
    % Update the environment
    real_time_now = idx * sampleTime;
    Control_Matrix = matrix_value;
    env(1:numRobots,poses,q_desire,graph_matrix_semi,real_time_now,vel_xy,vel_vw);
    xlim([-8 8]);   % Without this, axis resizing can slow things down
    ylim([-8 8]);
    for rIdx = 1:numRobots
        [vel_vw(:,rIdx),vel_xy(:,rIdx)] = swarmTeamController(poses,rIdx,Control_Matrix(2*rIdx-1:2*rIdx,:));
        [vel_vw_gradient,vel_xy_gradient] = Gradient_Controller(rIdx,poses,q_desire,delta);
        if (rIdx~=distance_control_id_pair(1))  &&  (rIdx~=distance_control_id_pair(2))
            vel_vw(:,rIdx) = vel_vw(:,rIdx) .*k_vector_control + vel_vw_gradient.* k_gradient_control;
            vel_xy(:,rIdx) = vel_xy(:,rIdx).*k_vector_control + vel_xy_gradient.* k_gradient_control;
            vel_vw(:,rIdx) = vel_vw(:,rIdx) .* [max_translational_v;max_rotational_w];
        else
            vel_vw(:,rIdx) = vel_vw_gradient;
            vel_xy(:,rIdx) = vel_xy_gradient;
        end
    


        d_poses(:,rIdx) = [cos(poses(3,rIdx)),0;sin(poses(3,rIdx)),0;0,1] * vel_vw(:,rIdx);
    end

    % leader control - distance based 
    pose1 = poses(:,distance_control_id_pair(1));
    pose2 = poses(:,distance_control_id_pair(2));
    desired_pose1 = q_desire(distance_control_id_pair(1)*2-1:distance_control_id_pair(1)*2);
    desired_pose2 = q_desire(distance_control_id_pair(2)*2-1:distance_control_id_pair(2)*2);
    [u1,u1_xy] = Calculate_Target_U(desired_pose1,pose1);
    [u2,u2_xy] = Calculate_Target_U(desired_pose2,pose2);
    u1 = u1 .* [max_translational_v;max_rotational_w];
    u2 = u2 .* [max_translational_v;max_rotational_w];


    vel_vw(:,distance_control_id_pair(1)) = vel_vw(:,distance_control_id_pair(1))*k_formation_keeping + u1*k_leader_tracking;
    vel_vw(:,distance_control_id_pair(2)) = vel_vw(:,distance_control_id_pair(2))*k_formation_keeping + u2*k_leader_tracking;
    vel_xy(:,distance_control_id_pair(1)) = vel_xy(:,distance_control_id_pair(1))*k_formation_keeping + u1_xy*k_leader_tracking;
    vel_xy(:,distance_control_id_pair(2)) = vel_xy(:,distance_control_id_pair(2))*k_formation_keeping + u2_xy*k_leader_tracking;


    d_poses(:,distance_control_id_pair(1)) = [cos(poses(3,distance_control_id_pair(1))),0;sin(poses(3,distance_control_id_pair(1))),0;0,1] * vel_vw(:,distance_control_id_pair(1));
    d_poses(:,distance_control_id_pair(2)) = [cos(poses(3,distance_control_id_pair(2))),0;sin(poses(3,distance_control_id_pair(2))),0;0,1] * vel_vw(:,distance_control_id_pair(2));

    poses = poses + d_poses*sampleTime;
end

%% Main Controller
function [vel_vw,vel_xy] = swarmTeamController(poses,rIdx,control_m)
    vel_xy = [0;0];
    [~,col]=size(poses);
    for neighbor_index = 1:col
        if neighbor_index ~= rIdx
            pose_diff =(poses(1:2,neighbor_index) - poses(1:2,rIdx));
            vel_xy = vel_xy + control_m(1:2,2*neighbor_index-1:2*neighbor_index)*-pose_diff;
        end
    end
    norm_vel = norm(vel_xy);

    % only normalize when the norm of control law is bigger than 1.
    if norm_vel ~=0 && norm_vel >1
        vel_xy = vel_xy ./ norm_vel;
    end

    h_matrix= zeros(2,2);
    h_matrix(:,:) = [   cos(poses(3,rIdx)),...
                        sin(poses(3,rIdx));
                        -sin(poses(3,rIdx)),...
                        cos(poses(3,rIdx)) 
                    ];
    vel_vw = h_matrix  * vel_xy;
end

%% sub-controllers and utility functions
function [target_u,target_xy] = Calculate_Target_U(target_pose,pose_now)  % linear control law ;return [v,w]
    pose_xy_diff = target_pose(1:2) - pose_now(1:2);
    if norm(pose_xy_diff) ~=0 && norm(pose_xy_diff)>1
        pose_xy_diff = pose_xy_diff./norm(pose_xy_diff);
    end
    co_matrix = [cos(pose_now(3)),sin(pose_now(3));-sin(pose_now(3)),cos(pose_now(3))];
    target_u =   co_matrix * pose_xy_diff;
    target_xy = pose_xy_diff;
end


function q_distance = Get_Desired_Distance_from_ID(q_desire,id1,id2)
    q_1 = q_desire(id1*2-1:id1*2);
    q_2 = q_desire(id2*2-1:id2*2);
    q_distance = norm(q_1-q_2);
end

function cost = Distance_Cost_Function(real_distance,desired_distance)
    if real_distance<desired_distance
        cost = (real_distance-desired_distance)^2/real_distance^2;
    else
        cost=0;  
    end
end

function [vel_vw,vel_xy] = Gradient_Controller(robot_idx,poses,q_desire,delta)
    J = 0;
    Jdx=0;
    Jdy=0;
    vel_xy = zeros(2,1);
    [~,col] = size(poses);
    for i=1:col
        if i ~=robot_idx
            pose_diff = poses(1:2,i) - poses(1:2,robot_idx);
            pose_diff_x = poses(1:2,i) - poses(1:2,robot_idx) + [delta; 0]; 
            pose_diff_y = poses(1:2,i) - poses(1:2,robot_idx) + [0; delta];  
            real_d = norm(pose_diff);
            real_d_x = norm(pose_diff_x);
            real_d_y = norm(pose_diff_y);
            desired_d = 0.7;
            J = J + Distance_Cost_Function(real_d,desired_d);
            Jdx = Jdx + Distance_Cost_Function(real_d_x,desired_d);
            Jdy = Jdy + Distance_Cost_Function(real_d_y,desired_d);
        end
    end
    vel_xy(1,1) = (Jdx - J)*1000;
    vel_xy(2,1) = (Jdy - J)*1000;
    if norm(vel_xy) > 1
        vel_xy = vel_xy/norm(vel_xy);
    end
    h_matrix(:,:) = [   cos(poses(3,robot_idx)),...
                        sin(poses(3,robot_idx));
                        -sin(poses(3,robot_idx)),...
                        cos(poses(3,robot_idx))
                       ];
    vel_vw = h_matrix  * vel_xy;                    
end

function graph_matrix_semi = Cal_graph_matrix(numRobots)
    graph_matrix_semi=zeros(numRobots,numRobots);
    for row = 1:numRobots-2
        graph_matrix_semi(row,row) = 1;
        graph_matrix_semi(row,row+1) = 1;                                                                                                                                                                                            
        graph_matrix_semi(row,row+2) = 1;
    end
    graph_matrix_semi(numRobots-1,numRobots-1) = 1;
    graph_matrix_semi(numRobots-1,numRobots) = 1;
    graph_matrix_semi(numRobots,numRobots) = 1;
    graph_matrix_semi(1,numRobots) = 1;
end 