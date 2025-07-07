### This is used to modify the MPC module. ###
### focus on line147 modified the cost     ###

import numpy as np
import sys
import os
import do_mpc
import time
from casadi import norm_2

import matplotlib.pyplot as plt
# plt.ion()
from matplotlib import rcParams
import contextlib
import io
from matplotlib.patches import Ellipse
from occupied_map import get_boundary


# Breaks simulation. Has to be casadi. TODO
# # 2-norm of a 2D tuple.
# def norm_2(x):
#     return (x[0] * x[0] + x[1] * x[1])**0.5

class DummyFile(object):
    def write(self, x): pass

@contextlib.contextmanager
def nostdout():
    save_stdout = sys.stdout
    sys.stdout = DummyFile()
    yield
    sys.stdout = save_stdout

class MPC_controller():
    def __init__(self,
                 obstacle,
                 formation_pattern,
                 formation_center,
                 ID,
                dim = 2,
                n_agent = 4,
                max_u = 100,
                min_u = -100,
                n_neighbor = 0,
                horizon = 10,
                t_step = 0.3,
                n_obstacle = 50) -> None:
        # n_agent here is not how many agent in formation team
        self.max_u = max_u
        self.min_u = min_u
        self.horizon = horizon
        self.t_step = t_step
        self.n_agent = n_agent
        self.dim = dim
        self.formation_pattern = formation_pattern
        self.ID = ID
        self.obstacle = obstacle
        self.n_obstacle = n_obstacle
        self.neighbors = np.zeros((dim,n_neighbor,self.horizon+1))
        self.n_neighbor = n_neighbor
        self.formation_center = formation_center
        
        self.target = [0]*(n_agent*dim)
        self.setup_model()

    def get_mpc_param(self):
        return {
            'n_horizon': self.horizon,
            'open_loop': 0,
            't_step': self.t_step,
            'state_discretization': 'collocation',
            'collocation_type': 'radau',
            'collocation_deg': 3,
            'collocation_ni': 1,
            'store_full_solution': True,
            # Use MA27 linear solver in ipopt for faster calculations:
            'nlpsol_opts': {'ipopt.linear_solver': 'mumps'}
        }

    def get_simulator_param(self):
        return {
            # Note: cvode doesn't support DAE systems.
            'integration_tool': 'idas',
            'abstol': 1e-8,
            'reltol': 1e-8,
            't_step': self.t_step,
        }

    def setup_model(self):
        model_type = 'continuous' # either 'discrete' or 'continuous'
        self.model = do_mpc.model.Model(model_type)
        self.setup_model_dynamics()
        self.model.set_variable(var_type='_tvp', var_name='target',shape=(self.dim,1)) # should set in mpc and simulator
        self.model.set_variable(var_type='_tvp', var_name='fc',shape=(self.dim,1)) # should set in mpc and simulator
        self.model.set_variable(var_type='_tvp', var_name='neighbors',shape=(self.dim,self.n_neighbor))
        self.model.set_variable(var_type='_tvp', var_name='obstacle',shape=(self.n_obstacle,self.dim*2))
        self.model.setup()
        

    def setup_model_dynamics(self):
        _    = self.model.set_variable('_x',  'pos', (self.n_agent*self.dim,1))
        dpos = self.model.set_variable('_x',  'dpos', (self.n_agent*self.dim,1))
        u    = self.model.set_variable('_u',  'force', (self.n_agent*self.dim,1))
        _    = self.model.set_variable('_u',  'real_target', (self.n_agent*self.dim,1))
        self.model.set_rhs('pos', dpos)
        self.model.set_rhs('dpos', u)

    def setup_stage_cost(self):
        '''
        lterm (Union[SX, MX, None])  Stage cost - scalar symbolic expression with respect to _x, _u, _z, _tvp, _p
        '''
        pos    = self.model['x']['pos']
        dpos   = self.model['x']['dpos']
        target = self.model['tvp']['target']
        # real_target = self.model['u']['real_target']
        # current_fc = self.model['tvp']['fc']
        neighbor_pos = self.model['tvp']['neighbors']

        fc = pos
        for i in range(self.n_neighbor):
            fc += neighbor_pos[:,i]
        fc = fc / (self.n_neighbor+1)

        formation_cost = norm_2(pos - (fc+self.formation_pattern[self.ID]))**2

        cost = norm_2(pos-target)**2 + 3*((norm_2(dpos))**2)
        # v = formation_deformation_value_casadi(pos - self.formation_pattern[self.ID], self.formation_pattern, self.obstacle)
        # case 1 fail:0.3,  0, 1
        # case 1 succ:0.3, 3, 1
        # case 2 fail:0.3,  0, 1
        # case 2 succ:0.3, 3, 1
        # case 3 fail:0.3,  0, 1
        # case 3 succ:0.3, 3, 1
        return 1*cost  + 3*formation_cost #+ 3*v     5,7   #1,5 1,20
        
    def setup_terminal_cost(self):
        '''
        mterm (Union[SX, MX, None])  Terminal cost - scalar symbolic expression with respect to _x and _p
        '''
        pos    = self.model['x']['pos']
        dpos   = self.model['x']['dpos']
        target = self.model['tvp']['target']

        cost = norm_2(pos-target)**2 + 5*((norm_2(dpos))**2)
        # fc = pos - self.formation_pattern[self.ID]
        # case 1 fail:0,0
        # case 1 succ:0,0
        # case 2 fail:0.1,0
        # case 2 succ:0.1,2
        return 0.1*cost
    
    def get_constraints(self):
        constraints = []
        pos          = self.model['x']['pos']
        dpos         = self.model['x']['dpos']
        # fc = self.model['tvp']['fc']
        neighbor_pos = self.model['tvp']['neighbors']
        # real_target = self.model['u']['real_target']
        obstacles    = self.model['tvp']['obstacle']
        
        # obstacle collision avoidance
        for K in range(obstacles.shape[0]):
            ob = obstacles[K,:].T
            # print(ob.shape)
            # pos_i = pos[:self.dim]
            cons1 = norm_2((pos - ob[:self.dim])/(ob[-self.dim:]))**2

            # print(cons1)
            constraints.append([-cons1,-1])
            # for i in range(self.n_agent):
            #     pos_i = pos[i*self.dim:(i+1)*self.dim]
            #     cons1 = norm_2((pos_i - ob[:self.dim])/(ob[-self.dim:]))
            #     constraints.append([-cons1,-1])

        # speed limit
        for i in range(self.n_agent):
            dpos_i = dpos[i*self.dim:(i+1)*self.dim]
            cons1 = norm_2(dpos_i)**2
            constraints.append([cons1,0.2**2])

        # formation deform upper bound
        # fc = pos + self.formation_pattern[self.ID]
        # cons = formation_deformation_value_casadi(fc,formation_pattern,self.obstacle)
        # constraints.append([cons,2])
        # for i in range(self.formation_pattern.shape[0]):
        #     # cons1 = get_adjusted_value_casadi(fc+self.formation_pattern[i],self.obstacle)
        #     fc = real_target - self.formation_pattern[self.ID]
        #     cons1 = get_adjusted_value_casadi(fc+self.formation_pattern[i],self.obstacle)
        #     constraints.append([cons1,0.01])

        # inter-agent collision avoidance
        for i in range(self.n_neighbor):
            cons1 = norm_2(pos - neighbor_pos[:,i])**2
            constraints.append([-cons1,-(0.2**2)])
        
        return constraints
        
    def get_mpc(self):
        mpc = do_mpc.controller.MPC(self.model)
        mpc.set_param(**self.get_mpc_param())

        tvp_template = mpc.get_tvp_template()
        def tvp_fun(t_now):
            for k in range(self.horizon+1):
                tvp_template['_tvp',k,'target'] = self.target
                tvp_template['_tvp',k,'fc'] = self.formation_center
                tvp_template['_tvp',k,'neighbors'] = self.neighbors[:,:,k]
                tvp_template['_tvp',k,'obstacle'] = self.obstacle
            return tvp_template
        mpc.set_tvp_fun(tvp_fun)
        
        mpc.set_objective(mterm=self.setup_terminal_cost(), lterm=self.setup_stage_cost())
        mpc.set_rterm(force=0.1)

        mpc.bounds['lower','_u','force'] = self.min_u*10
        mpc.bounds['upper','_u','force'] = self.max_u*10

        constraints = self.get_constraints()
        for i in range(len(constraints)):
            mpc.set_nl_cons(f'constraint{i}', constraints[i][0], constraints[i][1])

        mpc.setup()
        return mpc

    def get_simulator(self):
        simulator = do_mpc.simulator.Simulator(self.model)
        simulator.set_param(**self.get_simulator_param())
        tvp_template2 = simulator.get_tvp_template()
        def tvp_fun2(t_now):
            tvp_template2['target'] = self.target
            tvp_template2['fc']= self.formation_center
            tvp_template2['neighbors'] = self.neighbors[:,:,0]
            tvp_template2['obstacle'] = self.obstacle
            return tvp_template2
        simulator.set_tvp_fun(tvp_fun2)
        simulator.setup()
        return simulator

    def setup_do_mpc(self):
        self.mpc = self.get_mpc()
        self.simulator = self.get_simulator()
        self.estimator = do_mpc.estimator.StateFeedback(self.model)
    
    def reset_do_mpc(self, init_x):
        self.simulator.reset_history()
        self.mpc.reset_history()
    
        self.simulator.x0['pos'] = init_x[:self.n_agent*self.dim]
        self.simulator.x0['dpos'] = init_x[-(self.n_agent*self.dim):]
        x0 = self.simulator.x0.cat.full()
        self.mpc.x0 = x0

        self.estimator.x0 = x0

        # before initial guess: all future state = 0
        self.mpc.set_initial_guess()
        # after initial guess: all future state = initial(current) state
        self.mpc.make_step(x0)
        # after make_step: all future state = predicted future trajactory
        self.simulator.reset_history()
        self.mpc.reset_history()
        self.mpc.settings.supress_ipopt_output()

    def run_do_mpc(self, state):
        with nostdout():
            u0 = self.mpc.make_step(state)
        return u0

def get_adjusted_waypoint(waypoint,all_obs):
    new_waypoint = waypoint
    N = waypoint.shape[0] #dimension
    for obs in all_obs:
        p_oc = waypoint - obs[:N]
        norm_dist = np.linalg.norm(p_oc/obs[-N:])
        if norm_dist > 1:
            continue
        elif norm_dist < 1e-3:
            p_oc = waypoint - obs[:N] + obs[-N:]*0.1
            norm_dist = np.linalg.norm(p_oc/(obs[-N:]**2))
        # else:
        #     print('norm_dist',norm_dist)
        adjustment = p_oc*((1-norm_dist)/(norm_dist))
        new_waypoint = new_waypoint + adjustment
        # print('adjustment',adjustment)
    return new_waypoint

def formation_deformation_value_casadi(fc,formation_pattern,all_obs):
    value = 0
    for i in range(formation_pattern.shape[0]):
        position = formation_pattern[i] + fc
        value += get_adjusted_value_casadi(position,all_obs)
    return value

def get_adjusted_value_casadi(waypoint,all_obs):
    new_waypoint = waypoint
    N = waypoint.shape[0] #dimension
    adjustment_value = 0
    for obs in all_obs:
        p_oc = waypoint - obs[:N]
        norm_dist = norm_2(p_oc/obs[-N:])
        out = if_else(norm_dist > 1, 0, norm_2(p_oc*((1-norm_dist)/(norm_dist + 0.1))))
        adjustment_value += out**2 
        # new_waypoint = new_waypoint + adjustment
    return adjustment_value


def fill_up_obstacle(obstacle, obstacle_size):
    if obstacle.shape[0] > obstacle_size: # too many obstacle
        obstacle = obstacle[:obstacle_size,:]
    num, dim = obstacle.shape
    obstacle_fill_up = np.zeros(shape = (obstacle_size,dim))
    obstacle_fill_up[:,:dim//2] = -10000
    obstacle_fill_up[:,-(dim//2):] = 1
    obstacle_fill_up[:num,:] = obstacle
    return obstacle_fill_up
    

def ellipsoid_mesh(ellipsoid_param,shrink = 0):
    rx, ry, rz = ellipsoid_param[-3:] - shrink
    u, v = np.mgrid[0:2*np.pi:20j,-np.pi/2:np.pi/2:10j]    

    x = rx*np.cos(u)*np.cos(v) + ellipsoid_param[0]
    y = ry*np.sin(u)*np.cos(v) + ellipsoid_param[1]
    z = rz*np.sin(v) + ellipsoid_param[2]
    return x,y,z


class PltVisualizer():
    def __init__(self,dim) -> None:
        self.dim = dim
        self.count = 0
    def setup_canvas(self):
        if self.dim == '3D':
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, projection='3d')
        elif self.dim == '2D':
            pass
            # self.fig = plt.figure()
            # self.ax = self.fig.add_subplot(111, projection='3d')
        
    def setup_fix_data(self,data):
        if self.dim == '3D':
            obs = data
            self.ellipsoid = []
            for ob in obs:
                self.ellipsoid.append(ellipsoid_mesh(ob,0.2))
        elif self.dim == '2D':
            self.obs = data

        
    def set_data(self,data):        
        if self.dim == '3D':
            self.pos,self.trajectory,self.other_points = data
            
        elif self.dim == '2D':
            self.pos,self.trajectory,self.other_points = data

    def update(self):
        self.count += 1
        if self.dim == '3D':
            plt.clf()
            self.ax = self.fig.add_subplot(111, projection='3d')
            for ellipsoid in self.ellipsoid:
                self.ax.plot_surface(ellipsoid[0],
                                ellipsoid[1],
                                ellipsoid[2],
                                cstride = 1, rstride = 1, alpha = 0.6)
            for ID in range(len(self.trajectory)):
                self.ax.plot3D(self.trajectory[ID][:,0],self.trajectory[ID][:,1],self.trajectory[ID][:,2])
                self.ax.scatter3D(self.pos[ID][0],self.pos[ID][1],self.pos[ID][2])
            for point in self.other_points:
                self.ax.scatter3D(point[0],point[1],point[2])
        
            self.ax.set_zlim(0,10)
            self.ax.set_xlim(0,10)
            self.ax.set_ylim(0,10)
            
            self.fig.canvas.flush_events()    
            if self.count < 80:
                # print('2',self.count)
                # plt.show()
                # self.fig = plt.figure()
                plt.draw()
                plt.pause(0.01)
            else:
                plt.draw()
                plt.pause(0.1)
                print('plt count',self.count)
                plt.show()
                self.fig = plt.figure()
                # self.ax = self.fig.add_subplot(111, projection='3d')
        elif self.dim == '2D':
            plt.clf()
            for ob in self.obs:
                ellipse = Ellipse(xy=(ob[0], ob[1]), width=2*(ob[2]-0.2), height=2*(ob[3]-0.2), edgecolor='r', fc='None', lw=2)
                plt.gca().add_patch(ellipse)
            for ID in range(len(self.trajectory)):
                # plt.scatter(self.pos[ID,0],self.pos[ID,1])
                plt.scatter(self.trajectory[ID][-1,0],self.trajectory[ID][-1,1])
                plt.plot(self.trajectory[ID][:,0],self.trajectory[ID][:,1])
            for point in self.other_points:
                plt.scatter(point[0],point[1])
            plt.xlim([-5,25])
            plt.ylim([-5,25])
            plt.pause(0.01)

def Rz(theta,dim):
    if dim == 2:
        return np.array([[ np.cos(theta), -np.sin(theta)],
                        [ np.sin(theta), np.cos(theta)]])
    else:
        return np.array([[ np.cos(theta), -np.sin(theta), 0 ],
                        [ np.sin(theta), np.cos(theta) , 0 ],
                        [ 0            , 0             , 1 ]])

def local_goal_selection(fc,target,formation_pattern,obstacle,distance,last_index = None):
    weight = 1
    angle = np.arange(-90,91,10)*(np.pi/180)#np.zeros(1)#
    dim = fc.shape[0]
    if last_index != None:
        candidate_idx_range = [max(0,last_index-2),min(last_index+2,angle.shape[0]-1)]
    else:
        candidate_idx_range = [0,angle.shape[0]-1]

    local_goal_candidate = []
    cost = []
    for i in range(candidate_idx_range[0],candidate_idx_range[1]+1):
        local_goal_candidate.append(fc + Rz(angle[i],dim).dot(((target-fc)/np.linalg.norm(target-fc))*min(distance,np.linalg.norm(target-fc))))
        cost.append(formation_deformation_value_casadi(local_goal_candidate[-1],formation_pattern,obstacle) + weight*abs(angle[i]))
    
    idx = np.argmin(cost)
    angle_idx = idx+candidate_idx_range[0]
    return local_goal_candidate[idx],angle_idx


class Full_controller():
    def __init__(self,
                 formation_pattern,
                 formation_center,
                 formation_target,
                 obs,
                 n_obstacle,
                 max_u,
                 min_u,
                 horizon = 5,
                 t_step = 0.3,
                 ) -> None:
        
        self.formation_pattern = formation_pattern
        self.formation_center = formation_center
        self.formation_target = formation_target
        self.obs = obs
        self.n_obstacle = n_obstacle
        self.max_u = max_u
        self.min_u = min_u
        self.horizon = horizon
        self.t_step = t_step
        self.dim = self.formation_center.shape[0]
        self.n_agent = self.formation_pattern.shape[0]



    def setup_controller(self):
        ### initialize controller ##############################################
        self.mpc_objs = []
        self.trajectory = []
        self.init_all_pos = self.formation_center + self.formation_pattern
        self.position_record = self.init_all_pos
        # future_state_record: shape = (dim,n_agent,horizon+1)
        self.future_state_record = np.repeat(np.expand_dims((self.formation_center + self.formation_pattern).T,axis=2),self.horizon+1,axis=2)    
        for ID in range(self.n_agent):
            self.start_pos = self.formation_center + self.formation_pattern[ID]
            self.end_pos = self.formation_target + self.formation_pattern[ID] # formation_target + formation_pattern[ID]
            mpc_obj = MPC_controller(self.obs,
                                     self.formation_pattern,
                                     self.formation_center,
                                     ID=ID,
                                     n_agent=1,
                                     max_u= self.max_u,
                                     min_u= self.min_u,
                                     dim=self.dim,
                                     n_neighbor=self.n_agent-1,
                                     horizon = self.horizon,
                                     t_step = self.t_step,
                                     n_obstacle=self.n_obstacle)
            mpc_obj.target = self.end_pos
            # print('123123',np.delete(future_state_record,ID,1).shape)
            mpc_obj.neighbors = np.delete(self.future_state_record,ID,1)
            mpc_obj.setup_do_mpc()
            init_x = np.hstack([self.start_pos ,np.zeros(mpc_obj.n_agent*mpc_obj.dim)])
            mpc_obj.reset_do_mpc(init_x)
            mpc_obj.simulator.x0['pos'] = init_x[:mpc_obj.n_agent*mpc_obj.dim]
            mpc_obj.simulator.x0['dpos'] = init_x[-(mpc_obj.n_agent*mpc_obj.dim):]
            self.mpc_objs.append(mpc_obj)
            self.trajectory.append(self.start_pos)

    def update_data(self,current_formation_target,current_formation_center):
        # self.current_formation_target = current_formation_target
        for ID in range(self.n_agent):
            self.mpc_objs[ID].formation_center = current_formation_center
            self.mpc_objs[ID].obstacle = self.obs
            self.mpc_objs[ID].target = current_formation_target[ID]

            # end_pos = current_formation_target + self.formation_pattern[ID]
            # end_pos = find_clostest_point_full(self.obs,end_pos)#  get_adjusted_waypoint(end_pos,self.obs)
            # self.mpc_objs[ID].target = end_pos

            # print("end_pos",current_formation_target[ID])
            self.mpc_objs[ID].neighbors = np.delete(self.future_state_record,ID,1)
            

    def post_update_data(self):
        for ID in range(self.n_agent):
            mpc_obj = self.mpc_objs[ID]
            x_arr = np.array(mpc_obj.mpc.opt_x_num['_x', :, 0, -1])
            u_arr = np.array(mpc_obj.mpc.opt_x_num['_x', :, 0, -1])
            local_target = u_arr[:,:2,0]
            if ID == 1:
                xxx = [local_target[i] for i in range(local_target.shape[0])]
            
            # future_state_record: shape = (dim,n_agent,horizon+1) = (2,4,11)
            for i in range(mpc_obj.n_agent):
                current_position = x_arr[0,i*self.dim:i*self.dim+self.dim].T[0]
                self.position_record[ID,:] = current_position
                self.future_state_record[:,ID,:] = x_arr[:,i*self.dim:i*self.dim+self.dim,0].T
                self.trajectory[ID] = np.vstack([self.trajectory[ID],current_position])

        self.draw_data = []
        self.draw_data.append(self.position_record)
        self.draw_data.append(self.trajectory)
        # [self.current_formation_target]+
        self.draw_data.append([self.formation_target+self.formation_pattern[i] for i in range(self.n_agent)])
        
        
    def run(self,state):
        all_u0 = np.zeros(shape=(self.dim,self.n_agent))
        for ID in range(self.n_agent):
            mpc_obj = self.mpc_objs[ID]
            u0 = mpc_obj.run_do_mpc(state[:,ID]) # u0.shape = (self.dim*2,1)
            all_u0[:,ID] = u0[:self.dim,0]
        return all_u0
    

def run_controller_once(controller,state,idx):
    current_formation_center = np.mean(state[:controller.dim,:],axis = 1)
    current_formation_target,idx = local_goal_selection(current_formation_center,controller.formation_target,
                                                        controller.formation_pattern,controller.obs,distance=2,last_index=idx)
    
    controller.update_data(current_formation_target)
    all_u0 = controller.run(state)
    controller.post_update_data()
    return all_u0, idx

def calculate_waypoint(all_state, all_u0, dt):
    dim,n_agent = all_u0.shape
    all_waypoint = np.zeros(shape=(dim*n_agent,))
    for i in range(n_agent):
        all_waypoint[i*dim:(i+1)*dim] = all_state[:dim,i] + dt*all_state[-dim:,i] + 1*0.5*dt*dt*all_u0[:,i]
    return all_waypoint

def get_current_state(data_record):
    n_agent = len(data_record)
    dim = 3 #len(data_record[j][-1])
    state = np.zeros(shape=(dim,n_agent))
    for i in range(n_agent):
        state[:,i] = data_record[i][-1][:dim]
    return state


if __name__ == '__main__':
    #
    formation_pattern = np.array([[2,3],
                                [2,-3],
                                [-2,-3],
                                [-2,3],
                                # [0,0,1],
                         ])
    formation_center = np.array([2,10])
    formation_target = np.array([18,10])
    
    obstacle = np.array([[10,5,4,4],
                        [10,15,4,4]])
    # obstacle.shape = (num,dim*2)

    # formation_pattern = np.array([[0.5,0.5,0],
    #                             #   [0.5,-0.5,0],
    #                             # [-0.5,-0.5,0],
    #                             # [-0.5,0.5,0],
    #                             # [0,0,1],
    #                      ])
    # formation_center = np.array([0,0,2])
    # formation_target = np.array([0,10,2])
    
    # obstacle = np.array([   [1,5,4,1.5,1.5,4],
    #                         [-2,6,4,1.5,1.5,4],
    #                         [0.5,3,4,1.5,1.5,4],
    #                 ])


    
    obstacle_size = 10
    min_u = -4
    max_u = 4
    

    empty_obstacle = np.array([[-50,-50,1,1]])
    #
    controller = Full_controller(formation_pattern=formation_pattern,
                                 formation_center=formation_center,
                                 formation_target=formation_target,
                                 obs=fill_up_obstacle(empty_obstacle,obstacle_size),
                                 n_obstacle=obstacle_size,
                                 max_u=max_u,
                                 min_u = min_u,
                                 horizon=10,
                                 t_step=0.3)
    
    

    # setup visualizer
    Vis_dim = '2D' if controller.dim == 2 else '3D'
    Visualizer = PltVisualizer(Vis_dim)
    Visualizer.setup_canvas()
    Visualizer.setup_fix_data(controller.obs)
    
    # parameter setting and initialize
    controller.setup_controller() # change parameter before setup controller #
    idx = None
    n_steps = 200
    # planning ###
    slice_size = 30
    boundary = get_boundary(obstacle,formation_center,formation_target,buffer = 1)
    # path = get_path(obstacle,formation_center,formation_target,boundary,slice_size,show_image = False)
    xxx = np.linspace(start=formation_center[0], stop=formation_target[0], num=30)
    yyy = np.linspace(start=formation_center[1], stop=formation_target[1], num=30)
    # zzz = np.linspace(start=formation_center[2], stop=formation_target[2], num=30)
    # path = np.vstack((xxx,yyy,zzz)).T  # initial straight line path
    path = np.vstack((xxx,yyy)).T
    
    
    ### start loop ##############################################    
    for k in range(n_steps): 
        # 1. get state and observation ##
        state = np.ones(shape=(controller.dim*2,controller.n_agent)) # shape = (dim*2,num_agent) = (pos+vel,num_agent)
        for ID in range(controller.n_agent):
            mpc_obj = controller.mpc_objs[ID]
            state[:,ID] = mpc_obj.simulator.x0.cat.full()[:,0]
        
        # 2. run controller and planner ########################
        current_formation_center = np.mean(state[:controller.dim,:],axis = 1)
        # if k == 0:
        #     boundary = get_boundary(obstacle,current_formation_center,formation_target,buffer = 1)
        #     path = get_path(obstacle,current_formation_center,formation_target,boundary,slice_size,show_image = False)
        current_formation_target = path[min(path.shape[0]-1,k//3),:]
        
        
        controller.obs = fill_up_obstacle(obstacle,obstacle_size)
        controller.update_data(current_formation_target,current_formation_center)
        # for i in range(controller.n_agent):
        #     controller.mpc_objs[i].obstacle = fill_up_obstacle(obstacle,obstacle_size)
        all_u0 = controller.run(state)
        

        # 3. send control command and run simulation ##
        for ID in range(controller.n_agent):
            mpc_obj = controller.mpc_objs[ID]
            u0 = np.hstack([all_u0[:,ID],np.zeros(shape=(controller.dim,))])
            mpc_obj.simulator.make_step(np.expand_dims(u0,axis=1))
        controller.post_update_data()

        # 4. visualize ##############
        Visualizer.setup_fix_data(controller.obs)
        Visualizer.set_data(controller.draw_data)
        Visualizer.update()


        
        

