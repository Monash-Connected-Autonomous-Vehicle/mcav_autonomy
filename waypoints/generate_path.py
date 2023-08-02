coords = []


#straight forward 5m
for i in range(0,5):
    coords.append((i,0.0,10))

#cubic function right turn, first point @ (5.0,0.0)

i = 0
while i <= 3.5:
    y = 1/9 * (i-3.5)**3 + 10
    coords.append((y,-i,3))
    i += 0.5

#striaght to right for 13m, first point @ (10.0, 4.0)
for i in range(0,15):
    coords.append((10.0, -(4.0 + i), 10))

#cubic function left turn, first point @ (10.0, 16.5)
i = 16.5
while i <= 20:
    y = 1/9 * (i-16.5)**3 + 10
    coords.append((y,-i,3))
    i += 0.5

new_file = open('points.csv', 'w')
    
new_file.write('x,y,z,yaw,velocity,lookahead\n')
for coord in coords:
    new_file.write(f'{coord[0]},{coord[1]},0.0,3.14,8.0,{coord[2]}\n')
