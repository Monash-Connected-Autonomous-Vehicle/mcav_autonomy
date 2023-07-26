coords = []


#straight forward 5m
for i in range(0,5):
    coords.append((i,0.0,10))

i = 0
while i <= 5:
    x = 5 + (10*i - i**2)**0.5
    coords.append((x,-i,3))
    i += 0.5

i = 5
#striaght to right for 13m, first point @ (10.0, 4.0)
while i <= 17.5:
    coords.append((10.0,-i,10))
    i += 0.5

i = 17.5
while i <= 22.5:
    x = 0.5*(30 - (-1125 + 140*i - 4*i**2)**0.5)
    coords.append((x,-i,3))
    i += 0.5


i = 15
while i <= 20:
    coords.append((i,-22.5,10))
    i += 1

new_file = open('points.csv', 'w')
    
new_file.write('x,y,z,yaw,velocity,lookahead\n')
for coord in coords:
    new_file.write(f'{coord[0]},{coord[1]},0.0,3.14,8.0,{coord[2]}\n')
