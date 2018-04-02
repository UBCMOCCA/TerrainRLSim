

import numpy as np
from dill.source import indent

class ActionSpace(object):
    """
        Wrapper for the action space of an env
    """
    
    def __init__(self, action_space):
        self._minimum = np.array(action_space[0])
        self._maximum = np.array(action_space[1])
        
    def getMinimum(self):
        return self._minimum

    def getMaximum(self):
        return self._maximum
    
    @property
    def low(self):
        return self._minimum
    
    @property
    def high(self):
        return self._maximum

class TerrainRLSimWrapper(object):
    """
        Wrapper for the TerrainRLSim env to make function calls more simple
    """
    def __init__(self, sim, render=False, config=None):
        
        self._sim = sim
        self._render = render
        self._done = None
        
        act_low = [-1] * self.getEnv().getActionSpaceSize()
        act_high = [1] * self.getEnv().getActionSpaceSize() 
        action_space = [act_low, act_high]
        self._action_space = ActionSpace(action_space)
        ob_low = [-1] * self.getEnv().getObservationSpaceSize()
        ob_high = [1] * self.getEnv().getObservationSpaceSize() 
        observation_space = [ob_low, ob_high]
        self._observation_space = ActionSpace(observation_space)
        self._config = config
        print ("TerrainRLSim Config: ", self._config)
        
    def render(self):
        if (self._render):
            self._sim.display()
        
    def updateAction(self, action):
        # print ("step action: ", action)
        self._sim.updateAction(action[0])
            
        self._sim.handleUpdatedAction()
        
    def getLLCState(self):

        ob = self._sim.getLLCState()
        ob = np.reshape(np.array(ob), (-1, len(ob)))
        return ob
    
    def updateLLCAction(self, action):
        self._sim.updateLLCAction(action[0])
            
    def update(self):
        self._sim.update()
        self._done = self._done or self._sim.agentHasFallen()
        # self.render()
        # print("Trying to render...")
        
    def getObservation(self):
        
        ob = self._sim.getState()
        ob = np.reshape(np.array(ob), (-1, len(ob)))
        # ob = np.array(ob)
        return ob
    
    def step(self, action):
        
        action = np.array(action, dtype="float64")
        # print ("step action: ", action)
        self.updateAction(action)
        
        # for i in range(15):
        if ( "control_return" in self._config and (self._config["control_return"] == True) ):
            i=0
            while ( (not self._sim.needUpdatedAction()) and (i < 50 )):
                # print ("Controlling return")
                self._sim.update()
                self.render()
                i=i+1
        else:
            self._sim.update()
            self.render()
        # if ( self._render == True ):
        #    self._sim.display()
        # print("Num Agents: ", self._sim.getNumAgents())
        
        ob = self.getObservation()    
        reward = self.calcRewards()
            
        self._done = self._sim.agentHasFallen() or self._done
        # observation, reward, done, info
        # ob = np.array(ob)
        # print ("ob shape: ", ob.shape)
        return ob, reward, self._done, None
        
    def calcRewards(self):
        
        reward = self._sim.calcReward()
            
        return reward
        
    def reset(self):
        self._sim.initEpoch()
        self._done = False
        ob = self.getObservation()
        return ob
    
    def initEpoch(self):
        self.reset()
        
    def finish(self):
        """
            Unload simulation, free memory.
        """
        self._sim.finish()
        
    def getActionSpace(self):
        return self._action_space
    
    @property
    def action_space(self):
        return self._action_space
    
    @property
    def observation_space(self):
        return self._observation_space
    
    def endOfEpoch(self):
        return self._done
        
    def init(self):
        self._sim.init()
        
    def getEnv(self):
        return self._sim
    
    def onKeyEvent(self, c, x, y):
        self.getEnv().onKeyEvent(c, x, y)
        
    def setRandomSeed(self, seed):
        """
            Set the random seed for the simulator
            This is helpful if you are running many simulations in parallel you don't
            want them to be producing the same results if they all init their random number 
            generator the same.
        """
        # print ( "Setting random seed: ", seed )
        self.getEnv().setRandomSeed(seed)
    
def getEnvsList():
    import os, sys, json
    
    terrainRL_PATH = os.environ['TERRAINRL_PATH']
    print ("terrainRL_PATH: ", terrainRL_PATH)
    sys.path.append(terrainRL_PATH+'/lib')
    from simAdapter import terrainRLAdapter
    
    file = open(terrainRL_PATH+"/args/envs.json")
    env_data = json.load(file)
    file.close()
    
    return env_data

def getEnv(env_name, render=False):
    import os, sys, json
    
    terrainRL_PATH = os.environ['TERRAINRL_PATH']
    print ("terrainRL_PATH: ", terrainRL_PATH)
    sys.path.append(terrainRL_PATH+'/lib')
    from simAdapter import terrainRLAdapter
    
    env_data = getEnvsList()
    # print("Envs: ", json.dumps(env_data, indent=2))

    if (env_name in env_data):
        config_file = env_data[env_name]['config_file']
    else:
        print("Env: ", env_name, " not found. Check that you have the correct env name.")
        return None
    
    ## place holder  
    sim = terrainRLAdapter.cSimAdapter(['train', '-arg_file=', terrainRL_PATH+'/'+config_file, '-relative_file_path=', terrainRL_PATH+'/'])
    sim.setRender(render)
    sim.init()
    
    sim_ = TerrainRLSimWrapper(sim, render=render, config=env_data[env_name])
    
    return sim_

if __name__ == '__main__':

    env = getEnv(env_name="PD_Biped2D_Gaps_Terrain-v0", render=True)
    
    env.reset()
    actionSpace = env.getActionSpace()
    
    for i in range(100):
        action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform()) + actionSpace.getMinimum()  
        observation, reward,  done, info = env.step(action)
        env.render()
        print ("Done: ", done)
        # if (done):
        #     env.reset()
        
    env.finish()
    print (env)
    