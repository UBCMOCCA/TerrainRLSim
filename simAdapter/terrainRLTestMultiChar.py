
import terrainRLSim
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':

    # env = getEnv(env_name="FSM_Biped2D_Terrain_Walls-v0", render=True)
    # env = getEnv(env_name="PD_Biped3D_MutliChar_LargeBlocks-v0", render=True)
    envs_list = terrainRLSim.getEnvsList()
    print ("# of envs: ", len(envs_list))
    print ("Envs: ", str(envs_list))
    env = terrainRLSim.getEnv(env_name="PD_Biped3D_HLC_LargeBlocks-v0", render=True)
    
    env.reset()
    actionSpace = env.getActionSpace()
    
    actions = []
    action = ((actionSpace.getMaximum() - actionSpace.getMinimum()) * np.random.uniform(size=actionSpace.getMinimum().shape[0])  ) + actionSpace.getMinimum()
    actions.append(action)            
    print("Actions: ", actions)
    
    print("observation_space: ", env.observation_space.getMaximum())
    
    for e in range(10):
        env.reset()
        
        for t in range(100):
            observation, reward,  done, info = env.step(actions)
            print ("Done: ", done)
            if ( done ):
                break
            
            # print("Observation: ", observation)
            print("Reward: ", reward)
            print("action: ", actions)
                
            states = np.array(observation)
            print("states shape ", states[0].shape)
            print ("std length: ", len(np.std(states, axis=0)) )
            
            ## Get and vis terrain data
            """
                img_ = np.reshape(states[0][:1024], (32,32))
                print("img_ shape", img_.shape)
                plt.imshow(img_)
                plt.show()
            """
            
            
    env.finish()
    print (env)
