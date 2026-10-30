with open('app/src/console.c', 'r', encoding='utf-8') as f:
    c_lines = f.readlines()

for i in range(1540, min(1570, len(c_lines))):
    if c_lines[i].strip() == 'return;':
        c_lines[i] = c_lines[i].replace('return;', 'return 0;')

with open('app/src/console.c', 'w', encoding='utf-8', newline='\r\n') as f:
    f.writelines(c_lines)

print('Updated console.c')

with open('app/src/ble_comm.c', 'r', encoding='utf-8') as f:
    ble_lines = f.readlines()

new_ble_lines = []
for line in ble_lines:
    if '#include <string.h>' in line:
        new_ble_lines.append(line)
        new_ble_lines.append('#include <strings.h>\n')
    elif 'now_cal.minute' in line or 'now_cal.second' in line:
        line = line.replace('now_cal.minute', 'now_cal.min').replace('now_cal.second', 'now_cal.sec')
        new_ble_lines.append(line)
    else:
        new_ble_lines.append(line)

with open('app/src/ble_comm.c', 'w', encoding='utf-8', newline='\r\n') as f:
    f.writelines(new_ble_lines)

print('Updated ble_comm.c')

