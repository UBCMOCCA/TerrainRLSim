
import terrainRLSim
import numpy as np
import matplotlib.pyplot as plt
import json

if __name__ == '__main__':

    # env = getEnv(env_name="FSM_Biped2D_Terrain_Walls-v0", render=True)
    # env = getEnv(env_name="PD_Biped3D_MutliChar_LargeBlocks-v0", render=True)
    envs_list = terrainRLSim.getEnvsList()
    print ("# of envs: ", len(envs_list))
    print ("Envs:\n", json.dumps(envs_list, sort_keys=True, indent=4).replace('}','').replace(',','').replace('"','').replace('{',''))

    env = terrainRLSim.getEnv(env_name="PD_Biped3D_FULL_Imitate-Steps-v0", render=True)
    
    env.reset()
    actionSpace = env.getActionSpace()
    env.setRandomSeed(1234)
    
    actions = []
    action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
    actions.append(action)            
    print("Actions: ", actions)
    
    print("observation_space: ", env.observation_space.getMaximum())
    
    print("Actions space max: ", len(env.action_space.getMaximum()))
    print("Actions space min: ", env.action_space.getMinimum())
    print("Actions space max: ", env.action_space.getMaximum())
    
    for e in range(10):
        env.reset()
        
        for t in range(100):
            observation, reward,  done, info = env.step(actions)
            print ("Done: ", done)
            if ( done ):
                break
            """
            states = []
            for i in range(sim.getNumAgents()):
                ### get all states and check that they are different
                state = np.array(sim.getStateForAgent(i))
                print ("Agent: ", i, " state: ", state.shape)
                states.append(state)
                
                sim.updateActionForAgent(i, actions[i])
                """
            # print("Observation: ", observation)
            print("Reward: ", reward)
            # print("action: ", actions)
                
            states = np.array(observation)
            print("states shape ", states[0].shape)
            print ("std length: ", len(np.std(states, axis=0)) )
            # print ("std for states: ", np.std(states, axis=0))
            #### LLC states. If there is an LLC
            # llc_state = env.getLLCState()
            # print ("LLC state:", llc_state.shape)
            
            ## Get and vis terrain data
            """
                img_ = np.reshape(states[0][:1024], (32,32))
                print("img_ shape", img_.shape)
                plt.imshow(img_)
                plt.show()
            """
            
            # print ("Agent state: ", state)
            # if (done):
            #     env.reset()
        
    env.finish()
    print (env)
