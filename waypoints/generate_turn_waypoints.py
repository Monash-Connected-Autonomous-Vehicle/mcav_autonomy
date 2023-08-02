turn = -1 #{1: right, -1: left}
N_straight_before = 10
N_straight_after = 50
velocity = 10.0
yaw = 3.14

prefix = 'waypoints_'
if turn == 1:
    prefix += 'right'
else:
    prefix += 'left'

prefix += '_' +  str(N_straight_before) + '_' + str(N_straight_after)
new_file = open(prefix, 'w')

#start at (0,0,0)
waypoints = ''

for i in range(N_straight_before + 1):
    waypoints += '{},{},{},{},{},{}\n'.format(float(i), 0.0, 0.0, yaw, velocity, 0)

for i in range(1, N_straight_after):
    waypoints += '{},{},{},{},{},{}\n'.format(float(N_straight_before), float(i * turn), 0.0, yaw, velocity, 0)

new_file.write(waypoints)



