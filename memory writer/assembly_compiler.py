file = open("assembly.in", "r")
lines = file.readlines()
for i in range(len(lines)):
    lines[i] = lines[i].strip()
    lines[i] = lines[i].split(';')[0]

Reg_Code = dict()
lables_adr = dict()
hex_commands = list()

R_Type = ['ADD', 'SUB', 'AND', 'OR', 'SLT', 'ADDI', 'ANDI']
Mem_Ref = ['SW', 'LW']
Control_Flow = ['BEQ', 'BNE', 'J', 'JAL', 'JR']

opc = {'ADD':'000000' , 'SUB':'000000' , 'AND':'000000' , 'OR':'000000' , 'SLT':'000000' , 'SW':'101011' , 
       'LW':'100011'  , 'BEQ':'000100' , 'BNE':'000101' , 'J':'000010'  , 'JAL':'000011' , 'JR':'100000' ,
       'ADDI':'001000' , 'ANDI':'001100	'}

func = {'ADD':'100000' , 'SUB':'100010' , 'AND':'100100', 'OR':'100101' , 'SLT':'101010'}

for i in range(32):
    Reg_Code['R'+str(i)] = '{0:05b}'.format(i)

for i in range(len(lines)):
    command = lines[i].split()[0]
    if (command not in R_Type + Mem_Ref + Control_Flow):
        lable = command.split(':')[0]
        lables_adr[lable] = i

for i in range(len(lines)):
    binary = ""
    command = lines[i].split()[0]
    if (command in R_Type):
        info = lines[i].split()[1].split(',')
        if(command in ['ADD', 'SUB', 'AND', 'OR', 'SLT']):
            binary += opc[command] + Reg_Code[info[1]] + Reg_Code[info[2]] + Reg_Code[info[0]] + '00000' + func[command]
        if(command in ['ADDI', 'ANDI']):
            binary += opc[command] + Reg_Code[info[1]] + Reg_Code[info[0]] + '{0:016b}'.format(int(info[2]))
    if (command in Mem_Ref):
        info = lines[i].split()[1].split(',')
        rt = info[0]
        adr = int(info[1].split('(')[0])
        rs = info[1].split('(')[1].split(')')[0]
        binary += opc[command] + Reg_Code[rs] + Reg_Code[rt] + '{0:016b}'.format(adr)
    if (command in Control_Flow):
        if (command in ['BEQ', 'BNE']):
            info = lines[i].split()[1].split(',')
            rs = info[0]
            rt = info[1]
            L = int(info[2])
            binary += opc[command] + Reg_Code[rs] + Reg_Code[rt] + '{0:016b}'.format(L)
        if (command in ['J', 'JAL']):
            info = lines[i].split()[1]
            target = lables_adr[info]
            binary += opc[command] + '{0:026b}'.format(target)
        if (command in ['JR']):
            info = lines[i].split()[1]
            rs = info
            binary += opc[command] + Reg_Code[rs] + '000000000000000001000'

    if (command in R_Type + Mem_Ref + Control_Flow):
        h = hex(int(binary, 2))[2:]
        while(len(h) < 8) : h = '0' + h
        hex_commands.append(h)
file.close()

file = open("data.in", "r")
lines = file.readlines()
file.close()
adrs = list()
datas = list()
for i in range(len(lines)):
    adr = lines[i].split()[0]
    data = lines[i].split()[1]
    data = hex(int(data))[2:]
    while(len(data) < 8) : data = '0' + data
    adrs.append(int(adr))
    datas.append(data)

file = open('..\modelsim\Mem.data' , 'w')
i = 0
while(i < len(hex_commands)):
    file.write(hex_commands[i][:2] + '\n' + hex_commands[i][2:4] + '\n' + hex_commands[i][4:6] + '\n' + hex_commands[i][6:8] + '\n')
    i = i + 1
i = i * 4
while i < 128000:
    if (i in adrs):
        indx = adrs.index(i)
        file.write(datas[indx][:2] + '\n' + datas[indx][2:4] + '\n' + datas[indx][4:6] + '\n' +datas[indx][6:8] + '\n')
    else:
        file.write('00\n00\n00\n00\n')
    i = i + 4
file.close()