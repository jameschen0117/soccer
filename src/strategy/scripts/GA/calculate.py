import rospy
from transfer.msg import PPoint
# Normalization
HALF_MAX_DIS = 1500 #cm
FULL_MAX_DIS = 2163 #cm

input_std_list = []

def input2std_list(data):
    input_list = [data.rival_An, data.rival_Dis, data.right_angle, data.right_radius, data.border_An, data.border_Dis]
    print(all_input_list)
    global input_std_list
    input_std_list =  [data.rival_An/180, data.rival_Dis/HALF_MAX_DIS, data.right_angle/180, data.right_radius/HALF_MAX_DIS, data.border_An/180, data.border_Dis/HALF_MAX_DIS]


def get_input():

    rospy.init_node('input', anonymous=True)
    rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo/GoalInfo", PPoint, input2std_list)
    rospy.spin()

if __name__ == '__main__':
    get_input()


'''calculate'''
class Calculate():
    def __init__():

    def input(self):
        
    def output(self):

    def reward(self):
        
ask coach do reset


''' [] about input ''' 



''' [] about output '''



''' [] about reward '''



# done
