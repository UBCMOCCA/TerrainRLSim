
import sys
sys.path.append("./lib")
sys.path.append("./simAdapter")
import terrainRLAdapter


if __name__ == '__main__':
    
    sim = terrainRLAdapter.cSimAdapter(sys.argv)
    sim.setRender(True)
    
    print ("ArgV: ", sys.argv)

    # biped2d walk params
    walk = [ 0.082500, 0.251474, 2.099796, 0.000000, 0.000000, -0.097252, -0.993935, 0.273527, 0.221481, 1.100288, -3.076833, 0.180141, -0.176967, 0.310372, -1.642646, -0.406771, 1.240827, -1.773369, -0.508333, -0.170533, -0.063421, -2.091676, -1.418455, -1.242994, -0.262842, 0.453321, -0.170533, -0.366870, -1.494344, 0.794701, -1.408623, 0.655703, 0.634434]
    jump = [ 0.082500, 0.316504, 1.963316, 0.000000, 0.000000, -0.325309, -1.802222, 1.668542, 2.453011, 2.139391, -3.636978, -0.855670, -0.350402, -1.342939, 0.337384, -3.272340, 2.048047, -0.938193, -1.799840 -0.170533, 0.384958, -2.357088, 0.076791, -1.513792, -1.855033, 0.340609, -0.170533, -0.400898, -0.903512, 0.144128, 0.524855, -1.534278, 1.856180 ]
    land = [ 0.082500, 0.289427, 1.994835, 0.000000, 0.000000 -0.254906, -1.167773, 0.275834, -0.197735, 2.298369, -3.272985, -0.179217 -0.530879, -0.362771, -0.690035, -0.558768, 1.806804, -1.341557, -0.688935 -0.170533, 0.337809, -1.773089, -1.025832, -0.472297, -0.766728, 0.358616 -0.170533, -0.481754, -1.328176, 0.498767, -0.236538, -0.471444, 1.154247]
    
    stand = [ 2.0, -2.0, 1.0, -0.0, -0.0, -0.0]
    
    sim.init()
    while (True):
        sim.initEpoch()
        sim.act(walk)
        while (not sim.endOfEpoch()):
            state_ = sim.getState()
            sim.act(stand)
            updates_=0
            while (not sim.needUpdatedAction() and (updates_ < 20)):
                sim.update()
                sim.display()
                updates_+=1
                # print("Root Velocity: ", sim.calcVelocity())
            reward_ = sim.calcReward()    
            print ("Reward: ", reward_)
            print ("End of Action? ", sim.needUpdatedAction())
        print ("End of Episode? ", sim.endOfEpoch())
            # print ("State: ", state_)

