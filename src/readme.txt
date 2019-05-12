原本的gazebo的包只有機器人翰球門的座標
設立transfer去轉換現有的資訊去達成我要的數據
transfer
資料夾nodes：負責接收和輸出數據
    model.py:接收gazebo的數據，進而演算、輸出

msg:
 left_angle : 離左球門的角度
 left_radius： 離左球門的角度
 right_angle ： 離右球門的角度
 right_radius： 離右球門的角度
 rival_Dis：離最近敵人的距離
 rival_An：離最近敵人的角度
 border_Dis：離最近邊線的距離
 border_An：離最近邊線的角度


 cross 的功能：
    def rotate(self,obj):
  alpha = 0.5 
  v_x =  math.cos(alpha) - math.sin(alpha)
  v_y =  math.sin(alpha)+ math.cos(alpha)
  v_yaw = self.ball_ang
  strategy_type = 'cross'
  
  return __pub_info
