import rospy
from transfer.msg import PPoint
# Normalization
HALF_MAX_DIS = 1500 #cm
FULL_MAX_DIS = 2163 #cm

state = []

def input2state(data):
    input_list = [data.rival_An, data.rival_Dis, data.right_angle, data.right_radius, data.border_An, data.border_Dis]
    # print(input_list)
    global state
    state = [data.rival_An/180, data.rival_Dis/HALF_MAX_DIS, data.right_angle/180, data.right_radius/HALF_MAX_DIS, data.border_An/180, data.border_Dis/HALF_MAX_DIS]

rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo/GoalInfo", PPoint, input2state)

class sac_calculate():
    def __init__(self):
        pass
    def input(self):
        global state
        # print(state)
        return state
    def output(self, action):
        action2robot = action[0] * 180
        # print(action2robot) 
        return action2robot
    def reward(self,t):   
        a = 0.3
        b = 0.3
        c = 0.4
        
        # print(reward)
        # reward = a *(t[0])+ b *(t[1]/900)+ c *((900-t[2])/900) # + ball_in
        # reward = 10 * t[0] + 0.5 * t[1] + 10000 * (1/t[2])
        reward = t[0]
        
        return reward



# def action():
# def action2output(action):
    # pass
    # kickpwr 



# if __name__ == '__main__':
#     get_input()


# '''calculate'''

    


# ''' [] about input ''' 



# ''' [] about output '''



# ''' [] about reward '''



# done
