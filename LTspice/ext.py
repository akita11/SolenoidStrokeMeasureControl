import sys

f = open('SolDrive6.txt', "r")
line = f.readlines()
N = len(line)
f.close()
T1s = (10.001) * 10**(-3)
T1e = (10.003) * 10**(-3)
T2  = (10.500) * 10**(-3)
T3  = (10.990) * 10**(-3)
st = 0
t0 = 0.0
v0 = 0.0
Ton0 = 0
for i in range(1,N):
    field = line[i].split()
    if field[0] == 'Step':
        L = int(field[2][2:4])
        Ton = float(field[3][5:len(field[3])-1])
        st = 0
    else:
        t = float(field[0])
        v = float(field[1])
        if (st == 0 and t > T1s):
            st = 1
            vs1 = 0.0
            t1s = t0
        elif (st == 1 and t > T1e):
            st = 2
            t1e = t0
        elif (st == 2 and t > T2):
            st = 3
            # interpolating (t0, v0) - (T1, ?) - (t, v)
            k = (T2 - t0) / (t - t0)
            v2 = v0 + k * (v- v0)
        elif (st == 3 and t > T3):
            st = 4
            k = (T3 - t0) / (t - t0)
            v3 = v0 + k * (v- v0)
            v1 = vs1 / (t1e - t1s)
            if (Ton0 != Ton):
                print(" ")
            Ton0 = Ton
            print(L, Ton, v1, v2, v3)
        if (st == 1):
            vs1 = vs1 + (t - t0) * v
        t0 = t
        v0 = v
# V@{10ms, 10.5ms, 11ms} vs L & Ton(Duty)


