file = open('90_deg_turn_right.csv', 'r')
new_file = open('90_deg_turn_right_new.csv', 'w')


y = -24
i = 0
for i in range(50):
    if i == 0:
        i += 1
        continue

    new_file.write("{},{},{},{},{},{}\n".format(10.0, float(y - i), 0.0, 3.14, 10.0, 0))
    i += 1



