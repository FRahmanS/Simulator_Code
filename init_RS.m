 %INIT_ROBOT_SIM  Robot simulator module for simple multi-agent robots
%  
%  rs = INIT_ROBOT_SIM(init_env,init_robot,init_battery,init_vis)
%    init_env        initialization values for env
%     .seed           random seed (-1: random initialization)
%     .dim            [dim_x,dim_y]: environment dimensions
%     .min_pos_dist   minimal initial distance between objects
%     .rand_var       amplitude for random variation in initial placements
%     .type           0: walls, 1: wraparound
%     .                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
%    init_robot     initialization values for robots
%     .r             robot radius 
%     .pop           population
%    init_battery   initialization values for batteries
%     .num           number of batteries
%     .r             vector of battery radius
%     .col           vector of battery colors
%    init_vis       visualization struct
%     .show          true/false

%  Public functions
%
%  Author: Farzana Rahman


function rs = init_RS(init_env,init_robot,init_battery,init_vis) 


if init_env.seed == -1 %random seed (-1: random initialization)
  rng('shuffle')
  rand_init = rng;
  rs.env.seed = rand_init.Seed;
else
  rng(init_env.seed);
  rs.env.seed = init_env.seed;
end
rs.env.dim          = init_env.dim; % environment dimention
rs.env.min_pos_dist = init_env.min_pos_dist; % initial position distance between objects in environmrnt 
rs.env.rand_var     = init_env.rand_var;
rs.env.type         = init_env.type; %environment type wrapped or unwrapped
rs.env.time_step    = init_env.time_step; % time step of the environment (not clear)

%initializing robot parameters from init_robot in rs simulator
rs.robot.RealId      =init_robot.RealId;
rs.robot.r          = init_robot.r; %robot radius
rs.robot.m          =init_robot.m;  %robot mass
rs.robot.d          = init_robot.r*2; %robot diameter
rs.robot.energy     =init_robot.energy; %set initial energy
rs.robot.mate_state =init_robot.mate_state; %initialize mate_state
rs.robot.mate_off   =init_robot.mate_off; % initialize postmating state
rs.robot.module     =init_robot.module;  %initialize robot module
rs.robot.gr         =init_robot.gr;
rs.robot.as         =init_robot.as;
rs.robot.action     =init_robot.action;
rs.robot.pop = init_robot.pop; % robot population
rs.robot.vision_max_a = init_robot.vision_max_a; %robot vision maximum angle
rs.robot.vision_max_d = init_robot.vision_max_d; %robot vision maximum distance
rs.robot.vision_face_a = init_robot.vision_face_a; % face detection angle
rs.robot.mate_touch_a = init_robot.mate_touch_a; % mate detection angle for touch mating
rs.robot.mate_ir_a    = init_robot.mate_ir_a; %mate detection angle
rs.robot.mate_ir_d    = init_robot.mate_ir_d; %mate detection distance
rs.robot.mate_ir_d2   = rs.robot.mate_ir_d*rs.robot.mate_ir_d;%mate detection distance^2
rs.robot.Max_eng      = init_robot.Max_eng;
rs.robot.Max_life     = init_robot.Max_life;
rs.robot.growth_cost  = init_robot.growth_cost;
rs.robot.Eo= init_robot.Eo;
rs.robot.Mi =init_robot.Mi;
rs.robot.Md =init_robot.Md;
rs.robot.Cr =init_robot.Cr;
rs.robot.w1 =init_robot.w1;
rs.robot.w2 =init_robot.w2;
rs.robot.w3 =init_robot.w3;
rs.robot.w4 =init_robot.w4;
rs.robot.s0 =init_robot.s0;
rs.robot.cross_over_rate=init_robot.cross_over_rate;
rs.robot.mutation_prob=init_robot.mutation_prob;
rs.robot.gdm=init_robot.gdm;
rs.robot.gds=init_robot.gds;
rs.robot.drag=init_robot.drag;
rs.robot.gene=init_robot.Robot_gene;
rs.robot.clife=init_robot.clife;

rs.robot.col = zeros(rs.robot.pop,3); %robot color
rs.robot.battery_cap_idx = 1:rs.robot.pop; %battery capture (not sure)
rs.robot.mate_type       = zeros(1,rs.robot.pop); %0: none, 1: sender, 2:receiver

%initializing battery parameters from init_battery
rs.battery.num    = init_battery.num; %battery number
rs.battery.r      = init_battery.r; % battery radius
rs.battery.col    = init_battery.col; %battery color
rs.battery.reward = init_battery.reward; %battery reward

rs.vis.show    = init_vis.show; % the state of visualization

% Initial random Possible positions of robots and batteries
posx = rs.env.min_pos_dist:rs.env.min_pos_dist:(rs.env.dim(1)-rs.env.min_pos_dist); % Vector for possible X axis positions with minimunm distance
posy = rs.env.min_pos_dist:rs.env.min_pos_dist:(rs.env.dim(2)-rs.env.min_pos_dist); % Vector for possible Y axis positions
lenx  = length(posx); %number of X positions
leny  = length(posy);%number of Y positions
posX = posx'*ones(1,leny); %X position Matrix 
posY = ones(lenx,1)*posy; %Y position Matrix

%randomly select positions for robot and battery from enviromnet positions
rs.env.poslen = lenx*leny;
rs.env.posX   = posX(:)' + rs.env.rand_var*(2*rand(1,rs.env.poslen) - 1);
rs.env.posY   = posY(:)' + rs.env.rand_var*(2*rand(1,rs.env.poslen) - 1);

rand_pos_idx    = randperm(rs.env.poslen);
robot_pos_idx   = rand_pos_idx(1:rs.robot.pop);
battery_pos_idx = rand_pos_idx((rs.robot.pop+1):(rs.robot.pop+rs.battery.num)); % battery position

rs.robot.X   = rs.env.posX(robot_pos_idx); % robot  center X axis position
rs.robot.Y   = rs.env.posY(robot_pos_idx); %robot center Y axis position
%rs.robot.A    = (pi/2)*ones(1,rs.robot.pop);
rs.robot.A   = pi*(2*rand(1,rs.robot.pop)-1);
rs.robot.vel_L = zeros(1,rs.robot.pop); %initialize left wheel velocity to Zero
rs.robot.vel_R = zeros(1,rs.robot.pop);%initialize right wheel velocity to Zero

rs.battery.X = rs.env.posX(battery_pos_idx); %battery X position
rs.battery.Y = rs.env.posY(battery_pos_idx); %battery Y position
%rectangle environmnet draw
if rs.vis.show
  close all
  figure(666)
  set(gcf, 'Name', 'Robot simulator')
  rectangle('Curvature', [0 0], 'Position', [0, 0,rs.env.dim(1),rs.env.dim(2)]);
  set(gcf, 'Color', 'w')
  hold on
  axis([-100 rs.env.dim(1)+100 -100 rs.env.dim(2)+100]);
  axis equal
  axis off  

%draw robot and battery in the rectangular environment 
  a = [0:20:360]'*pi/180;
  if length(rs.robot.r) == 1 
    rs.vis.robot_x0 = cos(a)*(rs.robot.r*ones(1,rs.robot.pop));
    rs.vis.robot_y0 = sin(a)*(rs.robot.r*ones(1,rs.robot.pop));
  else
    rs.vis.robot_x0 = cos(a)*rs.robot.r;
    rs.vis.robot_y0 = sin(a)*rs.robot.r;
  end

  X = bsxfun(@plus, rs.vis.robot_x0, rs.robot.X); %robot X, Y and Center for visualization
  Y = bsxfun(@plus, rs.vis.robot_y0, rs.robot.Y);
  C = zeros(1,rs.robot.pop,3);

  rs.vis.robot_hl  = patch(X,Y,C,'LineWidth', 1);
  rs.vis.robot_col = get(rs.vis.robot_hl, 'CData'); % draw robot in environment

  rs.vis.robot_ang_hl   = plot(rs.robot.X + 200*cos(rs.robot.A), ...
  			    rs.robot.Y + 200*sin(rs.robot.A), 'k.', 'MarkerSize',12); % robots face angle patch draw
  
% drawing battery
  if length(rs.battery.r) == 1 
    rs.vis.battery_x0 = cos(a)*(rs.battery.r*ones(1,rs.battery.num));
    rs.vis.battery_y0 = sin(a)*(rs.battery.r*ones(1,rs.battery.num));
  else
    rs.vis.battery_x0 = cos(a)*rs.battery.r;
    rs.vis.battery_y0 = sin(a)*rs.battery.r;
  end
  
  X = bsxfun(@plus, rs.vis.battery_x0, rs.battery.X);
  Y = bsxfun(@plus, rs.vis.battery_y0, rs.battery.Y); % circular battery positions
  C = zeros(1,rs.battery.num,3); %color of patch
  
  rs.vis.battery_hl  = patch(X,Y,C);
  rs.vis.battery_col = get(rs.vis.battery_hl, 'CData'); % battery drawing in environment

  rs.vis.battery_col(:,1,:) = rs.battery.col; 
  set(rs.vis.battery_hl, 'CData', rs.vis.battery_col);

  
  % rs.vis.battery_hl = plot(rs.battery.X,rs.battery.Y,'go', 'LineWidth',2);
  
end


% Public functions
rs.remove_dead_robot             = @remove_dead_robot; %remove dead robot
rs.draw                          = @draw;
rs.set_velocity                  = @set_velocity;% for setting velocity of wheels
rs.step                          = @step; % calculationg position on each step
rs.mate_step                     = @mate_step; % calculationg position on each step on mating
rs.set_robot_col                 = @set_robot_col; % robot color
rs.set_robot_batt_cap            = @set_robot_batt_cap; % probably captured battery index. not clear
rs.set_robot_mate_type           = @set_robot_mate_type; % set mating type
rs.get_robot_mate_type           = @get_robot_mate_type; % get mating type
rs.set_robot_mate_state          = @set_robot_mate_state; % set mating state
rs.get_robot_mate_state          = @get_robot_mate_state; % set mating state
rs.set_robot_mate_off            = @set_robot_mate_off; % setting postmating state
rs.get_robot_mate_off            = @get_robot_mate_off; %getting postmating state 
rs.set_robot_module              = @set_robot_module; % setting module
rs.get_robot_module              = @get_robot_module; %getting module 
rs.set_robot_energy              = @set_robot_energy; % setting energy level
rs.get_robot_energy              = @get_robot_energy; %getting energy level 
rs.set_robot_size                = @set_robot_size; % setting size
rs.get_robot_size                = @get_robot_size; %getting size 
rs.set_robot_gene                = @set_robot_gene; % setting gene
rs.get_robot_gene                = @get_robot_gene; %getting gene 
rs.get_battery_ang_dist          = @get_battery_ang_dist; % batteries visual in the field 
rs.get_battery_min_ang_dist      = @get_battery_min_ang_dist; %the closest battery
rs.get_robot_ang_dist            = @get_robot_ang_dist; % robots visual in the field
rs.get_robot_min_ang_dist        = @get_robot_min_ang_dist; % closest robot
rs.get_robot_face_ang_dist       = @get_robot_face_ang_dist; % faces visible
rs.get_robot_min_face_ang_dist   = @get_robot_min_face_ang_dist; % closest face
rs.get_robot_min_back_ang_dist   = @get_robot_min_back_ang_dist; % closest back
rs.get_robot_min_ang_dist_size   = @get_robot_min_ang_dist_size; % closest robots angle,dist,size 
rs.mate_touch                    = @mate_touch;%mating with touch
rs.mate_ir                       = @mate_ir; %mating with specific angle and distance

%%
% Public function implementations
%

function [rs] = set_velocity(rs, idx, vel_L, vel_R)
rs.robot.vel_L(idx) = vel_L;
rs.robot.vel_R(idx) = vel_R;

function [rs,battery_r,robot_idx] = step(rs) % step functions

battery_r = zeros(1,rs.robot.pop);
air_mass = 0.5;
rs.robot.m=(4/3)*pi*((rs.robot.r).^3);
dragcomp=(rs.robot.m)./(rs.robot.m+air_mass);
rs.robot.drag=dragcomp.^rs.robot.m;


 %old robot position
old_a = rs.robot.A;
old_x = rs.robot.X;
old_y = rs.robot.Y;

%% prevent v_diff == 0
vr = (rs.robot.vel_R + randn(1,rs.robot.pop)).*rs.robot.drag; 
vl = (rs.robot.vel_L + randn(1,rs.robot.pop)).*rs.robot.drag; 

  
v_diff = (vr-vl);
da      = rs.env.time_step*(v_diff)./rs.robot.d;
new_a   = adjust_ang_vec(old_a + da);
Vel= (vr+vl)/2;
face_a=new_a;




turn_r = rs.robot.d.*Vel./(v_diff);

dx                      = turn_r.*(sin(new_a)-sin(old_a));
new_x                   = old_x + dx;

 dy                      = -turn_r.*(cos(new_a)-cos(old_a));
new_y                   = old_y + dy;                                                               


for idr1=1:rs.robot.pop

    for idr2=1:rs.robot.pop
       
    check_col = hypot((rs.robot.X(idr1)-rs.robot.X(idr2)),(rs.robot.Y(idr1)-rs.robot.Y(idr2)));        
    if (check_col<=(rs.robot.r(idr1)+rs.robot.r(idr2)))&&(idr1~=idr2)
    Tx= rs.robot.X(idr1)-rs.robot.X(idr2);
    Ty=rs.robot.Y(idr1)-rs.robot.Y(idr2);
    angle= atan2(Ty,Tx)+0.5*pi;
    total_m=rs.robot.m(idr1)+rs.robot.m(idr2);
    a1=old_a(idr1);
    v1=(vr(idr1)+vl(idr1))/2;
    a2=old_a(idr2);
    v2=(vr(idr2)+vl(idr2))/2;
    k1=v1*((rs.robot.m(idr1)-rs.robot.m(idr2))/total_m);
    q1=2*v2*rs.robot.m(idr2)/total_m;
    
    [new_a1,new_v1]=addvector (a1,k1,angle,q1);
    new_a(idr1)= adjust_ang_vec(new_a1);   
    Vel(idr1)=new_v1*0.2;
    
    k2=v2*((rs.robot.m(idr2)-rs.robot.m(idr1))/total_m);
    an2=angle+pi;
    q2=2*v1*rs.robot.m(idr1)/total_m;
    [new_a2,new_v2]=addvector (a2,k2,an2,q2);
    new_a(idr2)= adjust_ang_vec(new_a2);   
    Vel(idr2)=new_v2*0.2;
    new_x(idr1)=old_x(idr1)+sin( new_a(idr1))*Vel(idr1);
    new_y(idr1)=old_y(idr1)-cos( new_a(idr1))*Vel(idr1);
    new_x(idr2)=old_x(idr2)+sin( new_a(idr2))*Vel(idr2);
    new_y(idr2)=old_y(idr2)-cos( new_a(idr2))*Vel(idr2);
   
    end
               
    end
   
end   


switch rs.env.type
 case 0
  new_x(new_x < 0)          = 0;
  new_x(new_x > rs.env.dim(1)) = rs.env.dim(1);
  new_y(new_y < 0)          = 0;
  new_y(new_y > rs.env.dim(2)) = rs.env.dim(2);
 case 1
  i0        = find(new_x < 0);
  new_x(i0) = rs.env.dim(1) + new_x(i0);
  i1        = find(new_x > rs.env.dim(1));
  new_x(i1) = new_x(i1) - rs.env.dim(1);  

  i0 = find(new_y < 0);
  new_y(i0) = rs.env.dim(2) + new_y(i0); 
  i1 = find(new_y > rs.env.dim(2));
  new_y(i1) = new_y(i1) - rs.env.dim(2);  
end

rs.robot.A = face_a;
rs.robot.X=new_x;
rs.robot.Y=new_y;

[battery_r, battery_cap_idx, robot_idx] = battery_capture(rs); % batt_capture_idx is the index of the batteries which are captured
% testb=battery_cap_idx
%% move batteries
if ~isempty(battery_cap_idx)
   rs = move_battery(rs, battery_cap_idx); 
   
end

% mmr=find(rs.robot.module~=1);  
% if(~isempty(mmr))
growth_cost=((rs.robot.r*2)+(rs.robot.gr))*rs.robot.growth_cost;
% else
% growth_cost=0;   
% end
rs.robot.energy =rs.robot.energy +battery_r;
maxen= find(rs.robot.energy>rs.robot.Max_eng);
rs.robot.energy(maxen)=rs.robot.Max_eng;
rs.robot.energy=rs.robot.energy-growth_cost;
Growing_Robot=find((rs.robot.energy-rs.robot.Eo)>0);
if ~isempty(Growing_Robot)
s=rs.robot.r(Growing_Robot);
L=rs.robot.as(Growing_Robot);
ediff=(rs.robot.energy(Growing_Robot)-rs.robot.Eo)/(rs.robot.Max_eng-rs.robot.Eo);
alpha=rs.robot.gr(Growing_Robot);
L_minus_s=L-s;
grate=alpha.*ediff;
newr=s+(grate.*L_minus_s);
% % L=rs.robot.as;
% % alpha=rs.robot.gr;
% % ex=exp(-(alpha*(rs.robot.clife)));
% % calc=(1-ex);
% % s=L.*calc;
% % rs.robot.r=s;
rs.robot.r(Growing_Robot)=newr;
GRPS=rs.robot.r(Growing_Robot);
GRAS=rs.robot.as(Growing_Robot);
AST= find((GRAS-GRPS)<0);


if(~isempty(AST))
     rs.robot.r(Growing_Robot(AST))= rs.robot.as(Growing_Robot(AST));
%    disp('size bigger than Asymptotic size')
%    pause
end

end

% rs.robot.m=(4/3)*pi*((rs.robot.r).^3);
if rs.vis.show
   draw(rs);
end









function [rs] = set_robot_col(rs, idx, col_mat)

rs.robot.col(idx,:) =  col_mat;

if rs.vis.show
  rs.vis.robot_col(idx,1,:) = col_mat; 
  set(rs.vis.robot_hl, 'CData', rs.vis.robot_col);
end

function [rs] = set_robot_batt_cap(rs,idx)

rs.robot.batt_cap_idx = idx;

function [rs] = set_robot_mate_type(rs,idx, mate_type)

rs.robot.mate_type(idx) = mate_type;

function [mate_type, rs] = get_robot_mate_type(rs,idx)

mate_type = rs.robot.mate_type(idx);


 
 function[rs,idr,idx1,idx2,s_d_m,prob]=remove_dead_robot(rs)
 idx1=find(rs.robot.energy<=0);
 
 k=exp(-(rs.robot.r/rs.robot.s0));
 s_d_m=rs.robot.Mi+rs.robot.Md*k;
 prob=rand(1,rs.robot.pop);
 idx2=find(s_d_m>=prob);
 idx=union(idx1,idx2);
 len=length(idx);
 idr=rs.robot.RealId(idx);
 
 if len>0
 rs.robot.r(idx)=[];
 rs.robot.m(idx)=[];
 rs.robot.d(idx)=[];
 rs.robot.energy(idx)=[];
 rs.robot.mate_state(idx)=[];
 rs.robot.mate_off(idx)=[];
 rs.robot.module(idx)=[];
 rs.robot.gr(idx)=[];
 rs.robot.as(idx)=[];
 rs.robot.col(idx,:)=[];
 rs.robot.mate_type(idx)=[];
 rs.robot.X(idx)=[];
 rs.robot.Y(idx)=[];
 rs.robot.A(idx)=[];
 rs.robot.vel_L(idx)=[];
 rs.robot.vel_R(idx)=[];
 rs.robot.gene(idx,:)=[];
 rs.robot.action(idx)=[]; 
 rs.robot.RealId(idx)=[];
 rs.robot.drag(idx)=[];
 rs.robot.pop=rs.robot.pop-len;
 bc=find(rs.robot.battery_cap_idx>rs.robot.pop);
 rs.robot.battery_cap_idx(bc)=[];
 end
 
 
 
 
     
     
     
     

function [num_hit, ang, dist] = get_battery_ang_dist(rs,idx)

ang  = [];
dist = [];

len = length(idx);
if len == 0
  return;
end

tx = rs.battery.X;
ty = rs.battery.Y;
rx = rs.robot.X(idx);
ry = rs.robot.Y(idx);
ra = rs.robot.A(idx);

[num_hit, ang, dist] = get_targets_ang_dist(tx,ty,rx,ry,ra,rs.robot.vision_max_a, ...
					       0.1, rs.robot.vision_max_d);

function [hit_vec, ang, dist] = get_battery_min_ang_dist(rs,idx)

ang  = [];
dist = [];

len = length(idx);
if len == 0
  return;
end

tx = rs.battery.X;
ty = rs.battery.Y;
rx = rs.robot.X(idx);
ry = rs.robot.Y(idx);
ra = rs.robot.A(idx);

[hit_vec, ang, dist] = get_targets_min_ang_dist(tx,ty,rx,ry,ra,rs.robot.vision_max_a, ...
						   0.1, rs.robot.vision_max_d);



function [num_hit, ang, dist] = get_robot_ang_dist(rs,idx, target_idx)

ang  = [];
dist = [];

len = length(idx);

if (len == 0)
  return;
end



tx = rs.robot.X(target_idx);
ty = rs.robot.Y(target_idx);
rx = rs.robot.X(idx);
ry = rs.robot.Y(idx);
ra = rs.robot.A(idx);

[num_hit, ang, dist] = get_targets_ang_dist(tx,ty,rx,ry,ra,rs.robot.vision_max_a, ...
					       0.1, rs.robot.vision_max_d);

function [hit_vec, ang, dist] = get_robot_min_ang_dist(rs,idx, target_idx)

ang  = [];
dist = [];

len = length(idx);
if (len == 0)
  return;
end



tx = rs.robot.X(target_idx);
ty = rs.robot.Y(target_idx);
rx = rs.robot.X(idx);
ry = rs.robot.Y(idx);
ra = rs.robot.A(idx);

[hit_vec, ang, dist] = get_targets_min_ang_dist(tx,ty,rx,ry,ra,rs.robot.vision_max_a, ...
						   0.1, rs.robot.vision_max_d);

function [num_hit, ang, dist] = get_robot_face_ang_dist(rs,idx, target_idx)

ang  = [];
dist = [];

len = length(idx);

if (len == 0)
  return;
end


tx = rs.robot.X(target_idx);
ty = rs.robot.Y(target_idx);
ta = rs.robot.A(target_idx);
rx = rs.robot.X(idx);
ry = rs.robot.Y(idx);
ra = rs.robot.A(idx);

[num_hit, ang, dist] = get_targets_ang_dist_allign(tx,ty,ta,rx,ry,ra,...
						      rs.robot.vision_max_a, ...
						      0.1, rs.robot.vision_max_d,...
						      rs.robot.vision_face_a);

function [hit_vec, ang, dist] = get_robot_min_face_ang_dist(rs,idx, target_idx)

ang  = [];
dist = [];

len = length(idx);

if (len == 0)
  return;
end

tx = rs.robot.X(target_idx);
ty = rs.robot.Y(target_idx);
ta = adjust_ang_vec(rs.robot.A(target_idx)+pi);
rx = rs.robot.X(idx);
ry = rs.robot.Y(idx);
ra = rs.robot.A(idx);

[hit_vec, ang, dist] = get_targets_min_ang_dist_allign(tx,ty,ta,rx,ry,ra,...
						      rs.robot.vision_max_a, ...
						      0.1, rs.robot.vision_max_d,...
						      rs.robot.vision_face_a);

                          
function [hit_vec, ang, dist] = get_robot_min_back_ang_dist(rs,idx, target_idx)

ang  = [];
dist = [];

len = length(idx);

if (len == 0)
  return;
end



tx = rs.robot.X(target_idx);
ty = rs.robot.Y(target_idx);
ta = adjust_ang_vec(rs.robot.A(target_idx)+pi);
rx = rs.robot.X(idx);
ry = rs.robot.Y(idx);
ra = rs.robot.A(idx);

[hit_vec, ang, dist] = get_targets_min_ang_dist_allignb(tx,ty,ta,rx,ry,ra,...
						      rs.robot.vision_max_a, ...
						      0.1, rs.robot.vision_max_d,...
						      rs.robot.vision_face_a);

                          

function [hit_vec, ang, dist,size] = get_robot_min_ang_dist_size(rs,idx, target_idx)

ang  = [];
dist = [];

len = length(idx);

if (len == 0)
  return;
end

tx = rs.robot.X(target_idx);
ty = rs.robot.Y(target_idx);
ts = rs.robot.r(target_idx);
rx = rs.robot.X(idx);
ry = rs.robot.Y(idx);
ra = rs.robot.A(idx);

[hit_vec, ang, dist, size] = get_targets_min_ang_dist_size(tx,ty,ts,rx,ry,ra,rs.robot.vision_max_a, ...
						   0.1, rs.robot.vision_max_d);
                          
                          
                         


function [rs, mate] = mate_ir(rs,send_R_idx,rcv_R_idx)

mate.len = 0;
mate.send_idx = [];
mate.receive_idx = [];

send_idx    = send_R_idx;
receive_idx = rcv_R_idx;

if( length(send_idx) == 0 | length(receive_idx) == 0) 
  return
end

% if( length(send_idx) ~= length(receive_idx)) 
%     m=1
% end

rx = rs.robot.X(receive_idx);
ry = rs.robot.Y(receive_idx);
ra = rs.robot.A(receive_idx);
tx = rs.robot.X(send_idx);
ty = rs.robot.Y(send_idx);
ta = rs.robot.A(send_idx);


xd = bsxfun(@minus, tx',rx);
yd = bsxfun(@minus, ty',ry);
d2 = xd.^2 + yd.^2;
robot_d2 = bsxfun(@plus, rs.robot.r(send_idx)', rs.robot.r(receive_idx)).^2;
d2=d2-robot_d2;
% max_v = max(max_d2(:)) + 1;

a_r = atan2(yd,xd);
a_t = adjust_ang_vec(a_r + pi);
da_r = abs(adjust_ang_vec(bsxfun(@minus, a_r, rs.robot.A(receive_idx))));
da_t = abs(adjust_ang_vec(bsxfun(@minus, a_t, rs.robot.A(send_idx)')));

i_miss =  (d2 >= rs.robot.mate_ir_d2) ...
	 | (da_r >= rs.robot.mate_ir_a) | (da_t >= rs.robot.mate_ir_a);

 d2(i_miss) = rs.robot.mate_ir_d2;
[rd,cd]=size(d2);
if(rd==1)
  d2 =[d2;ones(rd,cd)*rs.robot.mate_ir_d2]; 
end    

[min_d, min_d_idx] = min(d2);

i_hit = find(min_d < rs.robot.mate_ir_d2);

len = length(i_hit);
if len == 0
  return;
end



snd_vector_index=min_d_idx(i_hit);
snd_vector_index=unique(snd_vector_index);
rcv_vector_index=i_hit;
if(length(snd_vector_index)~=length(rcv_vector_index))

  d3=d2';
  [smin_d, smin_d_idx] = min(d3(:,snd_vector_index));
   mrcv=receive_idx(smin_d_idx);
   msend=send_idx(snd_vector_index);
else   
   mrcv=receive_idx(i_hit);
   msend=send_idx(min_d_idx(i_hit));
end


%robots not have same mating state (sneaking or force interuption)
rms=rs.robot.mate_state(mrcv);
sms=rs.robot.mate_state(msend);

NEms=find(sms~=rms);
if(length(NEms)>0)
    rs.robot.mate_state(msend(NEms))=rs.robot.mate_state(mrcv(NEms)); %making unequal state of robots equal
end
%------------------------------------------------------------------

mate.len = length(msend);
mate.receive_idx = mrcv;
mate.send_idx    = msend;





%%
% Private function implementations
%

function [new_a,new_v]= addvector (angle1,length1,angle2,length2)
x=sin(angle1)*length1+sin(angle2)*length2;
y=cos(angle1)*length1+cos(angle2)*length2;
new_a=0.5*pi-atan2(y,x);
new_v=hypot(x,y);

function adj_ang = adjust_ang_vec(ang)

%adjust if abs(ang) > pi

small_ang = ang <= -pi;
big_ang   = ang > pi;

adj_ang = ang + small_ang*(2*pi) - big_ang*(2*pi);

function draw(rs)

    
   a = [0:20:360]'*pi/180;
  if length(rs.robot.r) == 1 
    rs.vis.robot_x0 = cos(a)*(rs.robot.r*ones(1,rs.robot.pop));
    rs.vis.robot_y0 = sin(a)*(rs.robot.r*ones(1,rs.robot.pop));
  else
    rs.vis.robot_x0 = cos(a)*rs.robot.r;
    rs.vis.robot_y0 = sin(a)*rs.robot.r;
  end
  
  
  
X = bsxfun(@plus, rs.vis.robot_x0, rs.robot.X);
Y = bsxfun(@plus, rs.vis.robot_y0, rs.robot.Y);
rs.vis=rmfield(rs.vis,'robot_col');
rs.vis.robot_col(:,1,:) = rs.robot.col; 
C=rs.vis.robot_col;

rs.vis.robot_hl.XData=X;
rs.vis.robot_hl.YData =Y;
rs.vis.robot_hl.CData =C;

% set(rs.vis.robot_hl, 'XData',X);
% set(rs.vis.robot_hl, 'YData',Y);
% set(rs.vis.robot_hl, 'CData', C);   

XA=rs.robot.X + rs.robot.r.*cos(rs.robot.A);
YA=rs.robot.Y + rs.robot.r.*sin(rs.robot.A);

rs.vis.robot_ang_hl.XData=XA;
rs.vis.robot_ang_hl.YData=YA;

% set(rs.vis.robot_ang_hl, 'XData',XA);
% set(rs.vis.robot_ang_hl, 'YData',YA);

X = bsxfun(@plus, rs.vis.battery_x0, rs.battery.X);
Y = bsxfun(@plus, rs.vis.battery_y0, rs.battery.Y);

set(rs.vis.battery_hl, 'XData',X);
set(rs.vis.battery_hl, 'YData',Y);
    
    
       
drawnow;



function [battery_r, battery_cap_idx, robot_idx] = battery_capture(rs)

battery_r       = zeros(1, rs.robot.pop);
battery_cap_idx = [];
robot_idx=[];
len_robot = length(rs.robot.battery_cap_idx);

if len_robot == 0
  return;
end
  
xd = bsxfun(@minus, rs.robot.X(rs.robot.battery_cap_idx)',rs.battery.X);
yd = bsxfun(@minus, rs.robot.Y(rs.robot.battery_cap_idx)',rs.battery.Y);

dist2 = xd.^2 + yd.^2;
rad2  = bsxfun(@plus, rs.robot.r(rs.robot.battery_cap_idx)',rs.battery.r).^2;

dist_rad = dist2 - rad2;

if len_robot == 1
  min_dist     = dist_rad;
  min_dist_idx = ones(1,rs.battery.num);
else
  [min_dist min_dist_idx] = min(dist_rad);
end

battery_cap_idx = find(min_dist < 0);

len_cap = length(battery_cap_idx);

if len_cap > 0
  robot_idx = rs.robot.battery_cap_idx(min_dist_idx(battery_cap_idx));
  battery_r(robot_idx) = rs.battery.reward(battery_cap_idx);

end


function rs = move_battery(rs, battery_cap_idx) 


len = length(battery_cap_idx);

xd = bsxfun(@minus, [rs.robot.X, rs.battery.X]', rs.env.posX);
yd = bsxfun(@minus, [rs.robot.Y, rs.battery.Y]', rs.env.posY);

dist2     = xd.^2 + yd.^2;
min_dist2 = min(dist2);
i_empty = find(min_dist2 > (2*max([rs.robot.r, rs.battery.r])));

rp_empty = randperm(length(i_empty));

new_idx = i_empty(rp_empty(1:len));

rs.battery.X(battery_cap_idx) = rs.env.posX(new_idx);
rs.battery.Y(battery_cap_idx) = rs.env.posY(new_idx);

function [num_hit, ang, dist] = get_targets_ang_dist(tx,ty,rx,ry,ra,lim_a, min_d, max_d)

len_t = length(tx);
len_r = length(rx);


if len_r == 0
  ang  = [];
  dist = [];
  num_hit = [];
  return
elseif len_t == 0
  ang  = -ones(1, len_r);
  dist = -ones(1, len_r);
  num_hit = zeros(1, len_r);
  return
end

ang  = -ones(len_t, len_r);
dist = -ones(len_t, len_r);

xd = bsxfun(@minus,tx',rx);
yd = bsxfun(@minus,ty',ry);

d     = sqrt(xd.^2 + yd.^2);

ang_r = ones(len_t,1)*ra;
ang_t = atan2(yd,xd);
a     = adjust_ang_vec(ang_t - ang_r);

i_hit = (d > min_d) & (d < max_d) & (abs(a) < lim_a);

num_hit = sum(i_hit);
dist = ones(len_t, len_r)*max_d;
dist(i_hit) = d(i_hit);
[dist sort_idx] = sort(dist);
sort_idx = bsxfun(@plus,sort_idx, [0:len_t:(len_t*(len_r-1))]);
ang = a(sort_idx);


function [hit_vec, ang, dist] = get_targets_min_ang_dist(tx,ty,rx,ry,ra,lim_a, min_d, max_d)

len_t = length(tx);
len_r = length(rx);


if len_r == 0
  ang  = [];
  dist = [];
  hit_vec = [];
  return
elseif len_t == 0
  ang  = -ones(1, len_r);
  dist = -ones(1, len_r);
  hit_vec = zeros(1, len_r);
  return
end

hit_vec = zeros(1, len_r);

ang  = -ones(len_t, len_r);
dist = -ones(len_t, len_r);

xd = bsxfun(@minus,tx',rx);
yd = bsxfun(@minus,ty',ry);

d     = sqrt(xd.^2 + yd.^2);

ang_r = ones(len_t,1)*ra;
ang_t = atan2(yd,xd);
a     = adjust_ang_vec(ang_t - ang_r);

i_hit = (d > min_d) & (d < max_d) & (abs(a) < lim_a);

dist = ones(len_t, len_r)*max_d;
 
dist(i_hit) = d(i_hit);

row_dist=size(dist,1);
if(row_dist<2)
    add_row=ones(len_t, len_r)*max_d;
    dist=[dist;add_row];
end  
[dist, d_idx] = min(dist);
i_hit = find(dist < max_d);
hit_vec(i_hit) = 1;
a_idx = d_idx + [0:len_t:(len_t*(len_r-1))];
ang = a(a_idx);




function [num_hit, ang, dist] = get_targets_ang_dist_allign(tx,ty,ta,rx,ry,ra, ...
							       lim_a, min_d, max_d, allign_a)

len_t = length(tx);
len_r = length(rx);


if len_r == 0
  ang  = [];
  dist = [];
  num_hit = [];
  return
elseif len_t == 0
  ang  = -ones(1, len_r);
  dist = -ones(1, len_r);
  num_hit = zeros(1, len_r);
  return
end

ang  = -ones(len_t, len_r);
dist = -ones(len_t, len_r);

xd = bsxfun(@minus,tx',rx);
yd = bsxfun(@minus,ty',ry);

d     = sqrt(xd.^2 + yd.^2);

ang_r = ones(len_t,1)*ra;
ang_t = atan2(yd,xd);
a     = adjust_ang_vec(ang_t - ang_r);

da_rt = adjust_ang_vec(ang_t - ta'*(ones(1,len_r))); 

i_hit = (d > min_d) & (d < max_d) & (abs(a) < lim_a) & (abs(da_rt) > (pi-allign_a));

num_hit = sum(i_hit);
dist = ones(len_t, len_r)*max_d;
dist(i_hit) = d(i_hit);
[dist sort_idx] = sort(dist);
sort_idx = bsxfun(@plus,sort_idx, [0:len_t:(len_t*(len_r-1))]);
ang = a(sort_idx);




function [hit_vec, ang, dist] = get_targets_min_ang_dist_allign(tx,ty,ta,rx,ry,ra, ...
							       lim_a, min_d, max_d, allign_a)

len_t = length(tx);
len_r = length(rx);


if len_r == 0
  ang  = [];
  dist = [];
  hit_vec = [];
  return
elseif len_t == 0
  ang  = -ones(1, len_r);
  dist = -ones(1, len_r);
  hit_vec = zeros(1, len_r);
  return
end

ang  = -ones(len_t, len_r);
dist = -ones(len_t, len_r);

xd = bsxfun(@minus,tx',rx);
yd = bsxfun(@minus,ty',ry);

d     = sqrt(xd.^2 + yd.^2);

ang_r = ones(len_t,1)*ra;
ang_t = atan2(yd,xd);
a     = adjust_ang_vec(ang_t - ang_r);

da_rt = adjust_ang_vec(ang_t - ta'*(ones(1,len_r))); 

i_hit = (d > min_d) & (d < max_d) & (abs(a) < lim_a) & (abs(da_rt) > (pi-allign_a));

dist = ones(len_t, len_r)*max_d;
dist(i_hit) = d(i_hit);
row_dist=size(dist,1);
if(row_dist<2)
    add_row=ones(len_t, len_r)*max_d;
    dist=[dist;add_row];
end  


[dist, d_idx] = min(dist);
i_hit = find(dist < max_d);
hit_vec(i_hit) = 1;
a_idx = d_idx + [0:len_t:(len_t*(len_r-1))];
ang = a(a_idx);


function [hit_vec, ang, dist] = get_targets_min_ang_dist_allignb(tx,ty,ta,rx,ry,ra, ...
							       lim_a, min_d, max_d, allign_a)

len_t = length(tx);
len_r = length(rx);


if len_r == 0
  ang  = [];
  dist = [];
  hit_vec = [];
  return
elseif len_t == 0
  ang  = -ones(1, len_r);
  dist = -ones(1, len_r);
  hit_vec = zeros(1, len_r);
  return
end

ang  = -ones(len_t, len_r);
dist = -ones(len_t, len_r);

xd = bsxfun(@minus,tx',rx);
yd = bsxfun(@minus,ty',ry);

d     = sqrt(xd.^2 + yd.^2);

ang_r = ones(len_t,1)*ra;
ang_t = atan2(yd,xd);
a     = adjust_ang_vec(ang_t - ang_r);

da_rt = adjust_ang_vec(ang_t - ta'*(ones(1,len_r))); 

i_hit = (d > min_d) & (d < max_d) & (abs(a) < lim_a) & (abs(da_rt) <= (pi-allign_a));

dist = ones(len_t, len_r)*max_d;
dist(i_hit) = d(i_hit);

row_dist=size(dist,1);
if(row_dist<2)
    add_row=ones(len_t, len_r)*max_d;
    dist=[dist;add_row];
end  

[dist, d_idx] = min(dist);
i_hit = find(dist < max_d);
hit_vec(i_hit) = 1;
a_idx = d_idx + [0:len_t:(len_t*(len_r-1))];
ang = a(a_idx);


function [hit_vec, ang, dist, Size] = get_targets_min_ang_dist_size(tx,ty,ts,rx,ry,ra,lim_a, min_d, max_d)

len_t = length(tx);
len_r = length(rx);


if len_r == 0
  ang  = [];
  dist = [];
  hit_vec = [];
  Size=[];
  return
elseif len_t == 0
  ang  = -ones(1, len_r);
  dist = -ones(1, len_r);
  Size = -ones(1,len_r);
  hit_vec = zeros(1, len_r);
  return
end

hit_vec = zeros(1, len_r);

ang  = -ones(len_t, len_r);
dist = -ones(len_t, len_r);

xd = bsxfun(@minus,tx',rx);
yd = bsxfun(@minus,ty',ry);

d     = sqrt(xd.^2 + yd.^2);

ang_r = ones(len_t,1)*ra;
ang_t = atan2(yd,xd);
a     = adjust_ang_vec(ang_t - ang_r);

i_hit = (d > min_d) & (d < max_d) & (abs(a) < lim_a);

dist = ones(len_t, len_r)*max_d;
dist(i_hit) = d(i_hit);

row_dist=size(dist,1);
if(row_dist<2)
    add_row=ones(len_t, len_r)*max_d;
    dist=[dist;add_row];
end  

[dist, d_idx] = min(dist);

i_hit = find(dist < max_d);
i_hit_not= find (dist==max_d);
Size(i_hit)=ts(d_idx(i_hit));

hit_vec(i_hit) = 1;
a_idx = d_idx + [0:len_t:(len_t*(len_r-1))];
ang = a(a_idx);
% Those are functions for computing angles and distances to targets. (tx,ty) is positions of targets and 
% (rx, ry) is positions of robots (i.e., targets can be other robots). 
% 
% the "*_min_ang_dist_*" functions gives only angles and distances to closest targets.
% The two other functions give angles and distances to all targets.
% the "*_allign" functions are used for (e.g. mating) when they can only see other within 
% an angle, "allign_a", (i.e., the angle they can see the face of another robot in my old study)
% 
% 




