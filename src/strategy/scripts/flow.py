















A
if not holding ball 
    chace()
if holding ball
    kick_ang = Fuzzy(env_info input[x1~X5])
    turn(kick_ang) 
    if ang == kick_ang  
        kick(pwr)   

B
if holding ball 5 sec 
    send "B got ball" to C 

C
while game_end
    stop game
    get_data
    send to GA
    wait evo (new generation)
    
def game_end
    1.ball out
    2.B got ball
    3.GOAL