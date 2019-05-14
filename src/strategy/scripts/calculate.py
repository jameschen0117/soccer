import rospy
from transfer.msg import PPoint
# Normalization
HALF_MAX_DIS = 1500 #cm
FULL_MAX_DIS = 2163 #cm

state = []

def input2state(data):
    input_list = [data.rival_An, data.rival_Dis, data.right_angle, data.right_radius, data.border_An, data.border_Dis]
    print(all_input_list)
    global state
    state =  [data.rival_An/180, data.rival_Dis/HALF_MAX_DIS, data.right_angle/180, data.right_radius/HALF_MAX_DIS, data.border_An/180, data.border_Dis/HALF_MAX_DIS]

def get_input():
    rospy.init_node('input', anonymous=True)
    rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo/GoalInfo", PPoint, input2state)
    rospy.spin()

# def action():
def action2output(action):
    kickpwr 


if __name__ == '__main__':
    get_input()


'''calculate'''
class sac_cal():
    def __init__():

    def input(self):
        
        
    def output(self):

    def reward(self):
        
ask coach do reset


''' [] about input ''' 



''' [] about output '''



''' [] about reward '''



# done


