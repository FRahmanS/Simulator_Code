

'Beginning' 

x
parm=[10 15 20 25 30 35 ];
nloop=25;
npar=length(parm);
loop=[];
parameters=[];
for ij=1:npar
   loop=[loop 1:nloop]; 
   parameters=[parameters parm(ij)*ones(1,nloop)];    
end    
%loop= [1:nloop 1:nloop 1:nloop 1:nloop];
%parameters=[10*ones(1,nloop) 20*ones(1,nloop) 30*ones(1,nloop) 40*ones(1,nloop) 50*ones(1,nloop) 60*ones(1,nloop) 70*ones(1,nloop) 80*ones(1,nloop) 90*ones(1,nloop) 100*ones(1,nloop) 110*ones(1,nloop)  120*ones(1,nloop) 130*ones(1,nloop) 140*ones(1,nloop) 150*ones(1,nloop) 160*ones(1,nloop) 170*ones(1,nloop) 180*ones(1,nloop) 190*ones(1,nloop) 200*ones(1,nloop) 210*ones(1,nloop) 220*ones(1,nloop) 230*ones(1,nloop) 240*ones(1,nloop) 250*ones(1,nloop) 260*ones(1,nloop) 270*ones(1,nloop) 280*ones(1,nloop) 290*ones(1,nloop) 300*ones(1,nloop)];
loopx=loop(x)
RC=parameters(x)



    
 
'BeginningNFM'
%%% initialize environment parameters -------------------------------------
init_env.seed         = -1;             %% define random seed, -1: rng('shuffle').
init_env.dim          = [13000 13000];  %% 12.5x12.5m based on. 
init_env.time_step    = 0.2;            %% time step of simulator in s.
init_env.min_pos_dist = 500;            %% distance between possible placement positions. 
init_env.rand_var     = 100; 
init_env.type         = 0;              %% 0: walls, 1: wraparound. 

testy=0; %T3 only with 0.15 offspring size
%%% set parameter----------------------------------------------------------
NNP=100;
NP=100;                                  %number of population.
MP=round(NP/2);                         %number of male.
FP=NP-MP;                               %number of female.
NB=100;                                  %number of battery.
NG=700;                                 %number of generation.
Max_size= 300;                          %Maximum asymtotic size.
Max_life=5000;                          %set maximum life time.
Max_eng=300;                            %set maximum energy
growth_cost= 0.001;                      %set growth cost.
Eo=40;                                %energy threshold for growth.
Mi=0.00001;                                 %size independent mortality.
Md=0.0000001;                                 %size dependent Mortality.
Cr=RC;                                   %reproductive cost.
w1=0.5;w2=0.5;w3=0.5;w4=0.5;            %reproduction parameter.
s0=10;                                  %smallest size an agent can have.
cross_over_rate=0.0001;                    %Cross over rate.
mutation_prob=0.0001;                      %mutation probability.
gdm=0;                                  %mean of gaussian distribution.
gds=0.1;                                %standard deviation of gaussian distribution.
air_mass = 0.5; 
mate_end_time=5;
post_mate_time=60;

%%initialize robot parameter-----------------------------------------------

                    
init_robot.pop = NP;                                 % structure init_robot defining robot parameters, robot population.
init_robot.vision_max_a  = pi/2;                     % robot vision maximum angle. it is [-45,45] here. 
init_robot.vision_max_d  = 2500;                     % maximum  distance for vision it is 2.5 m.
init_robot.vision_face_a = 15*pi/36;                 % robots capability to detect face in angle range.
init_robot.mate_touch_a  = pi/4;                     % infrared touch communication angle range.
init_robot.mate_ir_a = pi/2;                         % infrared communication angle range.
init_robot.mate_ir_d = 200;                         % infrared communication range 1m.
init_robot.Max_life = Max_life;                      % initialized maximum life.
init_robot.Max_eng = Max_eng;                        % initialize maximum energy.
init_robot.growth_cost = growth_cost;                % initialize growth cost.
init_robot.Eo = Eo;                                  % initialize energy thershold.
init_robot.Mi = Mi;                                  % initialize size independent mortality.
init_robot.Md = Md;                                  % initialize size dependent mortality.
init_robot.Cr = Cr;                                  % initialize reproductive cost.
init_robot.w1 = w1;
init_robot.w2 = w2;                                  % initialize reproductive parameter w1,w2,w3,w4.
init_robot.w3 = w3;
init_robot.w4 = w4;
init_robot.s0 = s0;                                  % initialize smallest size of robot.
init_robot.cross_over_rate = cross_over_rate;        % initialize cross over rate.
init_robot.mutation_prob = mutation_prob;            % initialize mutation probability.
init_robot.gdm = gdm;                                % initialize gaussian distribution mean.
init_robot.gds = gds;                                % initialize gaussian distribution standard deviation.
init_robot.clife=0;

%------structure to get analysis information------------------
analyze.gen=[];
analyze.avgLife=[];
analyze.TotMate=[];
analyze.avgBatt=[];
analyze.avgML=[];
analyze.avgFL=[];
analyze.HMM=[];
analyze.HFM=[];
analyze.avgMB=[];
analyze.avgFB=[];

%-------analysis table structure--------------------------------

AnTable.Genetation=[];
AnTable.MRealId=[];
AnTable.FRealId=[];
AnTable.age=[];
AnTable.Male_size=[];
AnTable.Male_energy=[];
AnTable.Female_size=[];
AnTable.Female_energy=[];
AnTable.Male_parent=[];
AnTable.Female_parent=[];
AnTable.Male_Offspring=[];
AnTable.Female_Offspring=[];



%init_battery structure with the properties of battery------------------------

init_battery.num    = NB;                                               %Initialize number of battery.
init_battery.r      = 100*ones(1,NB);                                   %Initialize radius of battery.
init_battery.col    = 0.5*ones(init_battery.num,1)*[1 1 1];             %Initialize colour of battery.
init_battery.reward = 50*ones(1,NB);                                   %Initialize reward of battery. 



%-----visualization------------
init_vis.show = false;  %Initialize visualization true.


%-----velocity-----------------

% v  = [-500, -250   250, 500, 500, 350;
%       -500,  250, -250, 500, 350, 500];

% v  = [-100, -50   50, 100, 100, 70, 0, 200, 200, 140;                                  %7 action set with velocity of left and right wheel.
%       -100,  50, -50, 100, 70, 100, 0, 200, 140, 200];



v  = [-300, -150   150, 300, 300, 210,0, 600 600 420;
      -300,  150, -150, 300, 210, 300,0, 600 420 600];

% v  = [-100, -50   50, 100, 100, 70, 0;                                  %7 action set with velocity of left and right wheel.
%       -100,  50, -50, 100, 70, 100, 0];

%---------------- Initialize structure to get new generation------
NewG.MaleOf=[];
NewG.FemaleOf=[];
NewG.OEng=[];
NewG.OSize=[];

LeftG.MaleOf=[];
LeftG.FemaleOf=[];
LeftG.MOEng=[];
LeftG.FOEng=[];
LeftG.MOSize=[];
LeftG.FOSize=[];

%----------generation start-----------

TG=table;
AG=[];
NG
for gen=1:NG
gen   


NP=NNP;   
init_robot.action=zeros(1,NP);                       % set robot initial action.
init_robot.mate_state=zeros(1,NP);                   %initialize mate_state.
init_robot.mate_off=zeros(1,NP);                     %initializing postmating state.
init_robot.module=zeros(1,NP);                        %initializing robot module.
init_robot.RealId=(1:NP)';



init_robot.r   = 10+(30-10).*rand(1,NP);             %radius, formula r=a+(b-a).*rand(N,1).
init_robot.energy=100+(200-100).*rand(1,NP);                   % set robot initial energy.                                                                                                                                                                   
initial_NN_W=normrnd(0,0.1,[NP,48]);                                %initial weight of neural network.                                                 
growthRate= 0.0001+(0.0009-0.0001).*rand(NP,1);                                  %growth rate initialization from range 0.01 to 0.9 %formula r=a+(b-a).*rand(N,1).
asymptoticSize= 50+(300-50).*rand(NP,1);                          %asymptotic size initialization.                                                                                                                        
asymptoticSizeG=asymptoticSize/Max_size;
init_robot.Robot_gene=[initial_NN_W growthRate asymptoticSizeG];     %Initialize robot gene.
init_robot.gr=growthRate';                                           %initialize growrth rate.
init_robot.as=(asymptoticSizeG*Max_size)';                                      %Initialize asymptotic size.

LengthNewG1=size(NewG.MaleOf,1);
LengthNewG2=size(NewG.FemaleOf,1);
LengthNewG=LengthNewG1+LengthNewG2;

if(LengthNewG>0 && LengthNewG<=NP)
    
init_robot.r(1:LengthNewG1)=NewG.OSize;         %initialize maleOf size
init_robot.r(MP+1:MP+LengthNewG2)=NewG.OSize;

init_robot.energy(1:LengthNewG1)=NewG.OEng;        
init_robot.energy(MP+1:MP+LengthNewG2)=NewG.OEng;

init_robot.Robot_gene((1:LengthNewG1),:)=NewG.MaleOf;        
init_robot.Robot_gene((MP+1:MP+LengthNewG2),:)=NewG.FemaleOf;

sizeLeft=size(LeftG.FOSize,1);
if((NP-LengthNewG)>0 && sizeLeft>0)
MP1=MP-LengthNewG1;
if(sizeLeft<=MP1)

init_robot.r(LengthNewG1+1:LengthNewG1+sizeLeft)=LeftG.MOSize;
init_robot.r(LengthNewG1+1+MP:LengthNewG1+sizeLeft+MP)=LeftG.FOSize;

init_robot.energy(LengthNewG1+1:LengthNewG1+sizeLeft)=LeftG.MOEng;        
init_robot.energy(LengthNewG1+1+MP:LengthNewG1+sizeLeft+MP)=LeftG.FOEng;

init_robot.Robot_gene((LengthNewG1+1:LengthNewG1+sizeLeft),:)=LeftG.MaleOf;        
init_robot.Robot_gene((LengthNewG1+1+MP:LengthNewG1+sizeLeft+MP),:)=LeftG.FemaleOf;
LeftG.MaleOf=[];
LeftG.FemaleOf=[];
LeftG.MOEng=[];
LeftG.FOEng=[];
LeftG.MOSize=[];
LeftG.FOSize=[];
else
randM=randperm(sizeLeft,MP1);
randF=randperm(sizeLeft,MP1);   
init_robot.r(LengthNewG1+1:MP)=LeftG.MOSize(randM);
init_robot.r(LengthNewG1+1+MP:NP)=LeftG.FOSize(randF);

init_robot.energy(LengthNewG1+1:MP)=LeftG.MOEng(randM);        
init_robot.energy(LengthNewG1+1+MP:NP)=LeftG.FOEng(randF);

init_robot.Robot_gene((LengthNewG1+1:MP),:)=LeftG.MaleOf(randM,:);        
init_robot.Robot_gene((LengthNewG1+1+MP:NP),:)=LeftG.FemaleOf(randF,:);

LeftG.MaleOf(randM,:)=[];
LeftG.FemaleOf(randF,:)=[];
LeftG.MOEng(randM)=[];
LeftG.FOEng(randF)=[];
LeftG.MOSize(randM)=[];
LeftG.FOSize(randF)=[];

end
end

init_robot.gr=(init_robot.Robot_gene(:,49))';
init_robot.as=((init_robot.Robot_gene(:,50))*Max_size)';


elseif(LengthNewG>0 && LengthNewG>NP)
    

randMOf = randperm(LengthNewG1,MP); 
randFOf = randperm(LengthNewG2,FP);

init_robot.r(1:MP)=NewG.OSize(randMOf); 
init_robot.r(MP+1:NP)=NewG.OSize(randFOf); 

init_robot.energy(1:MP)=NewG.OEng(randMOf); 
init_robot.energy(MP+1:NP)=NewG.OEng(randFOf);

init_robot.Robot_gene((1:MP),:)=NewG.MaleOf(randMOf,:);         
init_robot.Robot_gene((MP+1:NP),:)=NewG.FemaleOf(randFOf,:);

init_robot.gr=(init_robot.Robot_gene(:,49))';
init_robot.as=((init_robot.Robot_gene(:,50))*Max_size)';

left_OfM_idx=setdiff((1:size(NewG.OSize,1)),randMOf);
left_OfF_idx=setdiff((1:size(NewG.OSize,1)),randFOf);

 LeftG.MaleOf=[LeftG.MaleOf; NewG.MaleOf(left_OfM_idx,:)];
 LeftG.FemaleOf=[LeftG.FemaleOf; NewG.FemaleOf(left_OfF_idx,:)];
 LeftG.MOEng=[LeftG.MOEng;NewG.OEng(left_OfM_idx)];
 LeftG.FOEng=[LeftG.FOEng;NewG.OEng(left_OfF_idx)];
 LeftG.MOSize=[LeftG.MOSize;NewG.OSize(left_OfM_idx)];
 LeftG.FOSize=[LeftG.FOSize;NewG.OSize(left_OfF_idx)];
          

end

init_robot.m=(4/3)*pi*((init_robot.r).^3);                              %mass,formula=(4/3)*pi*r^3.
dragcomp=(init_robot.m)./(init_robot.m+air_mass);
init_robot.drag=dragcomp.^init_robot.m;



rs = init_RS(init_env,init_robot,init_battery,init_vis);                %initialize simulator parameters.

rs = rs.set_robot_mate_type(rs,1:MP, ones(1,MP));                     %set mating type of male robots.
rs = rs.set_robot_mate_type(rs,(MP+1):NP, zeros(1,FP));               %set mating type of female robots. 


rs = rs.set_robot_col(rs, 1:MP, ones(MP,1)*[0 1 0]);                  % robot color for male
rs = rs.set_robot_col(rs, (MP+1):NP, ones(FP,1)*[1 0 1]);             % robot color for female
                                                            
NewG.MaleOf=[];
NewG.FemaleOf=[];
NewG.OEng=[];
NewG.OSize=[];

analyzeGen=zeros(NP,12);
analyzeGen(:,1)=rs.robot.RealId;
analyzeGen(:,12)=gen;
analyzeGen(:,9)=rs.robot.r;
analyzeGen(:,10)=rs.robot.gr;
analyzeGen(:,11)=rs.robot.as;


% if gen==NG
%     finalgen=rs.robot;
% end
% rschk=rs.robot;

for i=1:100
    agent(i).gen=gen;
    agent(i).id=i;
    agent(i).timestep=1:5000;
    agent(i).Size=zeros(5000,1);
    agent(i).OSz=zeros(5000,1);
    agent(i).OEng=zeros(5000,1);
end




for j=1:Max_life  %for each generation
rs.robot.clife=j;   
AllMaleid=find(rs.robot.mate_type==1);
AllFemaleid=find(rs.robot.mate_type==0); 

if (rs.robot.pop<2||length(AllMaleid)<1||length(AllFemaleid)<1)

break;

end  





NP=rs.robot.pop;
action=rs.robot.action;

Mate_off_robot_idx=find(rs.robot.mate_off>0); %find robots in postmating state
   if(~isempty(Mate_off_robot_idx))
     rs.robot.mate_off(Mate_off_robot_idx)=rs.robot.mate_off(Mate_off_robot_idx)-1;   
   end

robot_module_idx=find (rs.robot.module==0); %robots that has finished one module and can choose other module
   if(~isempty(robot_module_idx))
    
        len_mod_idx=length(robot_module_idx);
        
       
        
        x1=rs.robot.mate_type(robot_module_idx); %sex
        
        x2=ones(1,len_mod_idx); %bias
        
        x3=ones(1,len_mod_idx);  %mate state
        mof=rs.robot.mate_off(robot_module_idx);
        mofid=find(mof>0);
        x3(mofid)=0;
        
        energy=rs.robot.energy(robot_module_idx);
        x4=energy/Max_eng;
    
        Size=rs.robot.r(robot_module_idx); 
        x5=Size/Max_size;
        
        life=j*ones(1, len_mod_idx);
        x6=life/Max_life; %might need to change
        
    
        
        if(length(AllMaleid)<1)
        x7=-1*ones(1,len_mod_idx);    
        x8=-1*ones(1,len_mod_idx); 
        x9=-1*ones(1,len_mod_idx);
       
        else
        [hit_vec, ang, dist,Size] = rs.get_robot_min_ang_dist_size(rs,robot_module_idx,AllMaleid);
        Size=Size/Max_size;
        t=find(hit_vec==0);
        Size(t)=-1;
        x7=Size;
        
        [hit_vec, ang, dist] = rs.get_robot_min_face_ang_dist(rs,robot_module_idx,AllMaleid);
        dist=dist/2500;
        t=find(hit_vec==0);
        dist(t)=-1;
        x8=dist;
        
        [hit_vec, ang, dist] = rs.get_robot_min_back_ang_dist(rs,robot_module_idx,AllMaleid);
        dist=dist/2500;
        t=find(hit_vec==0);
        dist(t)=-1;
        x9=dist;
        end
        
        if(length(AllFemaleid)<1)
        x10=-1*ones(1,len_mod_idx);    
        x11=-1*ones(1,len_mod_idx); 
      
       
        else
        [hit_vec, ang, dist] = rs.get_robot_min_face_ang_dist(rs,robot_module_idx,AllFemaleid);
        dist=dist/2500;
        t=find(hit_vec==0);
        dist(t)=-1;
        x10=dist;
        
        [hit_vec, ang, dist] = rs.get_robot_min_back_ang_dist(rs,robot_module_idx,AllFemaleid);
        dist=dist/2500;
        t=find(hit_vec==0);
        dist(t)=-1;
        x11=dist;
        end
        
        [hit_vec, ang, dist] = rs.get_battery_min_ang_dist(rs,robot_module_idx);
        dist=dist/2500;
        t=find(hit_vec==0);
        dist(t)=-1;
        x12=dist;
        
        Get_Robot_gene=rs.robot.gene(robot_module_idx,:);
%         
%         X=[x1' x2' x3' x4' x5' x6' x7' x8' x9' x10' x11' x12'];
        W1=Get_Robot_gene(:,1:4);
        W2=Get_Robot_gene(:,5:8);
        W3=Get_Robot_gene(:,9:12);
        W4=Get_Robot_gene(:,13:16);
        W5=Get_Robot_gene(:,17:20);
        W6=Get_Robot_gene(:,21:24);
        W7=Get_Robot_gene(:,25:28);
        W8=Get_Robot_gene(:,29:32);
        W9=Get_Robot_gene(:,33:36);
        W10=Get_Robot_gene(:,37:40);
        W11=Get_Robot_gene(:,41:44);
        W12=Get_Robot_gene(:,45:48);
        
        
       X1=[x1' x1' x1' x1'];
       X2=[x2' x2' x2' x2'];
       X3=[x3' x3' x3' x3'];
       X4=[x4' x4' x4' x4'];
       X5=[x5' x5' x5' x5']; 
       X6=[x6' x6' x6' x6']; 
       X7=[x7' x7' x7' x7'];
       X8=[x8' x8' x8' x8']; 
       X9=[x9' x9' x9' x9'];
       X10=[x10' x10' x10' x10'];
       X11=[x11' x11' x11' x11']; 
       X12=[x12' x12' x12' x12']; 
       
       P1=X1.*W1;
       P2=X2.*W2;
       P3=X3.*W3;
       P4=X4.*W4;
       P5=X5.*W5;
       P6=X6.*W6;
       P7=X7.*W7;
       P8=X8.*W8;
       P9=X9.*W9;
       P10=X10.*W10;
       P11=X11.*W11;
       P12=X12.*W12;
       
       M=P1+P2+P3+P4+P5+P6+P7+P8+P9+P10+P11+P12;

       [maxv,ch_module]=max(M,[],2);

        
        
        
     
        rs.robot.module(robot_module_idx)=ch_module;
        

   end
   
 
   
 mate_module_idx=find(rs.robot.module==1);
    if(~isempty(mate_module_idx))
      ank=rs.robot.RealId( mate_module_idx);
      analyzeGen( ank,5)= analyzeGen( ank,5)+1; 
        
         mate_robot=find(rs.robot.mate_off(mate_module_idx)==0);
         Mating_robot_idx= mate_module_idx(mate_robot);
         
         send_R_idx=Mating_robot_idx(rs.robot.mate_type(Mating_robot_idx)==1);
         rcv_R_idx= Mating_robot_idx(rs.robot.mate_type(Mating_robot_idx)==0);
         
%-------------------------Mating------------------------------------------
        [rs, mate] = rs.mate_ir(rs,send_R_idx,rcv_R_idx);
        m_rcv_idx=mate.receive_idx;
        m_snd_idx=mate.send_idx;
        m_len=mate.len;

        if(m_len~=0)
            
              
            ms_rcv=rs.robot.mate_state(m_rcv_idx);
            ms_snd=rs.robot.mate_state(m_snd_idx);

            mate_going_idx=find(ms_snd<mate_end_time & ms_rcv<mate_end_time);
            action(m_rcv_idx(mate_going_idx))=7;
            action(m_snd_idx(mate_going_idx))=7;
            ms_rcv(mate_going_idx)=ms_rcv(mate_going_idx)+1; % update state of mating robot
            ms_snd(mate_going_idx)=ms_snd(mate_going_idx)+1; %update state of mating robots 

            mate_end_idx=find(ms_snd==mate_end_time & ms_rcv==mate_end_time);
            if length(mate_end_idx)>0
            ms_rcv(mate_end_idx)=0; % mate finished. So set robot to premating state
            ms_snd(mate_end_idx)=0;
            action(m_rcv_idx(mate_end_idx))=2;  %have to check cross over and mutation part
            action(m_snd_idx(mate_end_idx))=3;
            
            %transfer and created kid
          
            m_idx=m_snd_idx(mate_end_idx);
            f_idx=m_rcv_idx(mate_end_idx);
            
            %reproductive condition.---------------------------------------
            sm=rs.robot.r(m_idx);
            em=rs.robot.energy(m_idx);
            sf=rs.robot.r(f_idx);
            ef=rs.robot.energy(f_idx);
            Rm=(rs.robot.w1*(sm/Max_size))+(rs.robot.w2*(em/Max_eng));
            Rf=(rs.robot.w3*(sf/Max_size))+(rs.robot.w4*(ef/Max_eng));
            Rp=(Rm+Rf)/2;
            prob=rand(1,length(Rp));
            rpt=find(Rp>=prob);
            excep=find(Rp>1);
            if~isempty(excep)
                rpt
                pause
            end   
            
            if ~isempty(rpt)
%                 rpt
              for ofp=1:1
%                     ofp
             m_idxk=m_idx(rpt);
             f_idxk=f_idx(rpt);
            %cross over and muattion---------------------------------------
             male_parent=rs.robot.gene(m_idxk,:);
             female_parent=rs.robot.gene(f_idxk,:);
             if cross_over_rate>=rand
             cross_over_point=randi([1,50],1,1);
             male_cut=male_parent(:,cross_over_point:50);
             female_cut=female_parent(:,cross_over_point:50);
             male_offspring=[male_parent(:,1:cross_over_point-1) female_cut];
             female_offspring=[female_parent(:,1:cross_over_point-1) male_cut];
             else
             male_offspring=male_parent;
             female_offspring=female_parent;
             end
            
             if mutation_prob>=rand
             mutation_value=normrnd(gdm,gds);
             male_offspring=male_offspring+mutation_value;
             female_offspring=female_offspring+mutation_value;    
             end
            
            gentn=gen*ones(length(m_idxk),1);
            age=j*ones(length(m_idxk),1);
            


             grsm=find(male_offspring(:,49)<0);
            if (~isempty(grsm))
            male_offspring(grsm,49)=0.000001;    
            end
            grsf=find(female_offspring(:,49)<0);
            if (~isempty(grsf))
            female_offspring(grsf,49)=0.000001;
            end

            
            grbmas=find(male_offspring(:,50)>1);
            if (~isempty(grbmas))
            male_offspring(grbmas,50)=1;    
            end
            grbfas=find(female_offspring(:,50)>1);
            if (~isempty(grbfas))
            female_offspring(grbfas,50)=1;
            end

            grsmas=find(male_offspring(:,50)<0.15);
            if (~isempty(grsmas))
            male_offspring(grsmas,50)=0.15;    
            end
            grsfas=find(female_offspring(:,50)<0.15);
            if (~isempty(grsfas))
            female_offspring(grsfas,50)=0.15;
            end
              
            
            NewG.MaleOf=[NewG.MaleOf; male_offspring];
            NewG.FemaleOf=[NewG.FemaleOf;female_offspring];
            calengOf=(ef(rpt)'*0.2);

            NewG.OEng=[ NewG.OEng;calengOf]; 
            NewG.OSize=[NewG.OSize;(sf(rpt)')*0.15];
            NGS=find(NewG.OSize<10);
            if(~isempty(NGS))    
            NewG.OSize(NGS)=10;    
            end
            
            
            
            AnTable.Genetation=[AnTable.Genetation;gentn];
            AnTable.MRealId=[AnTable.MRealId;rs.robot.RealId(m_idxk)];
            AnTable.FRealId=[AnTable.FRealId;rs.robot.RealId(f_idxk)];
            AnTable.age=[AnTable.age;age];
            AnTable.Male_size=[AnTable.Male_size;sm(rpt)'];
            AnTable.Male_energy=[AnTable.Male_energy;em(rpt)'];
            AnTable.Female_size=[ AnTable.Female_size;sf(rpt)'];
            AnTable.Female_energy=[AnTable.Female_energy;ef(rpt)'];
            AnTable.Male_parent=[AnTable.Male_parent;male_parent];
            AnTable.Female_parent=[AnTable.Female_parent;female_parent];
            AnTable.Male_Offspring=[AnTable.Male_Offspring;male_offspring];
            AnTable.Female_Offspring=[ AnTable.Female_Offspring;female_offspring];
            
            
            analyzeGen(rs.robot.RealId(m_idxk),2)=analyzeGen(rs.robot.RealId(m_idxk),2)+1;
            analyzeGen(rs.robot.RealId(f_idxk),2)=analyzeGen(rs.robot.RealId(f_idxk),2)+1;
            
             end
            rs.robot.energy(f_idxk)=rs.robot.energy(f_idxk)-Cr;
            agm=rs.robot.RealId(m_idxk);
            ags=rs.robot.RealId(f_idxk);
            
            agmS=rs.robot.r(m_idxk);
            agmE=rs.robot.energy(m_idxk);
           
            agsS=rs.robot.r(f_idxk);
            agsE=rs.robot.energy(f_idxk);
            
            
            for ts=1:length(m_idxk)
            agent(agm(ts)).OSz(j)=agmS(ts);   
            agent(ags(ts)).OSz(j)=agsS(ts);
             
            agent(agm(ts)).OEng(j)=agmE(ts);   
            agent(ags(ts)).OEng(j)=agsE(ts);
            end          
                        
            
                
            end
            
            end
            %--------------------------------------------------------------
            % set robot in postmating state--------------------------------
            
            rs.robot.mate_off(m_rcv_idx(mate_end_idx))=post_mate_time;
            rs.robot.mate_off(m_snd_idx(mate_end_idx))=post_mate_time;
            
            rs.robot.module(m_rcv_idx(mate_end_idx))=0;
            rs.robot.module(m_snd_idx(mate_end_idx))=0;
            
            rs.robot.mate_state(m_snd_idx)=ms_snd;   
            rs.robot.mate_state(m_rcv_idx)=ms_rcv;
    
        end
%-------------------------------------------------------------------------
        
        get_left_idx=setdiff(mate_module_idx,m_rcv_idx);
      
        robot_left_idx=setdiff(get_left_idx,m_snd_idx);
        rs.robot.mate_state(robot_left_idx)=0;
       
         if(~isempty(robot_left_idx))
% 
            arand=randi([2 3],1,length(robot_left_idx));
            action(robot_left_idx)=arand;

            rs.robot.module(robot_left_idx)=0;
         
         
         end
 
 
    end



%-------------------------------battery module----------------------------------

  batt_module_idx=find(rs.robot.module==2); 
  if(~isempty(batt_module_idx))
   bnk=rs.robot.RealId( batt_module_idx);
   analyzeGen( bnk,6)= analyzeGen( bnk,6)+1; 
    [hit_vec, ang, dist] = rs.get_battery_min_ang_dist(rs,batt_module_idx);
     
            hit_bat_index = find(hit_vec == 1);
            Len_bat_index=length(hit_bat_index); 
    
            if(Len_bat_index==0)
                arand=randi([1 6],1,length(batt_module_idx));       %set action
                action(batt_module_idx)=arand;
                
            else

                arand=randi([1 6],1,length(batt_module_idx)); 
                action(batt_module_idx)=arand;
                ang1_r_idx=find(ang>0.01);
                action(batt_module_idx(ang1_r_idx))=6;
                ang2_r_idx=find(ang<-0.01);
                action(batt_module_idx(ang2_r_idx))=5;
                ang3_r_idx=find(ang<0.01 & ang>-0.01);
                action(batt_module_idx(ang3_r_idx))=4;
                
            end
              rs.robot.module(batt_module_idx)=0;
   
  end

  
%-------------------------push module--------------------
  
push_module_idx=find(rs.robot.module==3); 
if(~isempty(push_module_idx))
    cnk=rs.robot.RealId( push_module_idx);
    analyzeGen( cnk,7)= analyzeGen( cnk,7)+1; 
  [hit_vec, ang, dist] = rs.get_robot_min_ang_dist(rs,push_module_idx,1:NP);
   hit_push_index = find(hit_vec == 1);
            Len_push_index=length(hit_push_index); 
    
            if(Len_push_index==0)
                arand=randi([1 6],1,length(push_module_idx));       %set action
                action(push_module_idx)=arand;
            else
                
                arand=randi([1 6],1,length(push_module_idx)); 
                action(push_module_idx)=arand;
                
                ang11_r_idx=find(ang>=0.01);
                action(push_module_idx(ang11_r_idx))=6;
                ang12_r_idx=find(ang>=0.01 & dist<=rs.robot.mate_ir_d);
                action(push_module_idx(ang12_r_idx))=10;
               
                ang21_r_idx=find(ang<=-0.01);
                action(push_module_idx(ang21_r_idx))=5;
                ang22_r_idx=find(ang<=-0.01& dist<=rs.robot.mate_ir_d);
                action(push_module_idx(ang22_r_idx))=9;
                
                ang31_r_idx=find(ang<0.01 & ang>-0.01);
                action(push_module_idx(ang31_r_idx))=4;
                ang32_r_idx=find(ang<0.01 & ang>-0.01& dist<=rs.robot.mate_ir_d);
                action(push_module_idx(ang32_r_idx))=8;
            end
                rs.robot.module(push_module_idx)=0; %set module to zero module finished
            
            
end




%---------------------------wait module-------------------------------

 wait_module_idx=find(rs.robot.module==4); 
if(~isempty(wait_module_idx))
   dnk=rs.robot.RealId( wait_module_idx);
   analyzeGen( dnk,8)= analyzeGen( dnk,8)+1;   
  action( wait_module_idx)=7;  
  rs.robot.module(wait_module_idx)=0; %set module to zero module finished
end
   


  %managing corner
  cnr= (rs.robot.X<0)|(rs.robot.X> rs.env.dim(1))|(rs.robot.Y<0)|(rs.robot.Y>rs.env.dim(2));
  cnra=randi([2 3],1,NP);
  action(cnr)=cnra(cnr);
  action=action';
  
  rs = rs.set_velocity(rs, 1:NP, v(1,action), v(2,action));
  rs.robot.action(1:NP)=action;
  [rs, batt_r,robot_idx] = rs.step(rs); 
  
  if(~isempty(robot_idx))
      analyzeGen(rs.robot.RealId(robot_idx),3)=analyzeGen(rs.robot.RealId(robot_idx),3)+1;
  end    
  %delete robots-------------------------------------------------
  %set the energy of energy and if energy is less than or equal zero delete
  %agents
  %calculate size dependent mortality
 [rs,idr,idx1,idx2,s_d_m,prob]=rs.remove_dead_robot(rs);
  
  if(~isempty(idr))
      analyzeGen(idr,4)=analyzeGen(idr,4)+j;
  end
  
  realid=rs.robot.RealId;    
  realR=rs.robot.r;  
  for  t=1:length(realid)
     agent(realid(t)).Size(j)=realR(t);   
  end
  

end 

AG=[AG;agent];
analyze.gen(gen,1)=gen;
lf=find(analyzeGen(:,4)==0);
  if(~isempty(lf))
      analyzeGen(lf,4)=j;
  end
analyze.avgLife(gen,1)=sum(analyzeGen(:,4))/NNP;
analyze.TotMate(gen,1)=sum(analyzeGen(:,2));

analyze.avgBatt(gen,1)=sum(analyzeGen(:,3))/NNP;
analyze.avgML(gen,1)=sum(analyzeGen(1:MP,4))/MP;
analyze.avgFL(gen,1)=sum(analyzeGen(MP+1:NNP,4))/FP;
analyze.HMM(gen,1)=max(analyzeGen(1:MP,2));
analyze.HFM(gen,1)=max(analyzeGen(MP+1:NNP,2));
analyze.avgMB(gen,1)=sum(analyzeGen(1:MP,3))/MP;
analyze.avgFB(gen,1)=sum(analyzeGen(MP+1:NNP,3))/FP;

GenT=array2table(analyzeGen);
TG=[TG;GenT];




end
Ag1=AG(1:100,1:100);
Ag2=AG(101:200,1:100);
Ag3=AG(201:300,1:100);
clear AG;
lpn=int2str(loopx);
nb=int2str(Cr);
name=strcat('MC7_F100/',lpn,'NMCTest',nb);

save (name);





