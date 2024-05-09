import math
import sys

def Ie(vs, rs, ls, T1, T):
    return( (vs / rs) * (1 - math.exp(-(rs / ls) * T1)) / (1 - math.exp(-(rs / ls) * T)))

def Is(vs, rs, ls, T1, T):
    return(Ie(vs, rs, ls, T1, T) * math.exp(-(rs / ls) * (T - T1)))

def I1(vs, rs, ls, T1, T, t):
    return(vs / rs - (vs / rs - Is(vs, rs, ls, T1, T) ) * math.exp(-(rs / ls) * t))

vs = 12 # [V]
rs = 51.9 # [ohm]
T = 10e-3 # 10ms
#L = float(sys.argv[1]) * 1e-3
mag = 1
#for T1i in range(1, 10):
#    T1 = T1i * 1e-3
#    I005 = I1(vs, rs, L, T1, T, 50e-6) * mag
#    I010 = I1(vs, rs, L, T1, T, 100e-6) * mag
#    I05 = I1(vs, rs, L, T1, T, 0.5e-3) * mag
#    print(T1*1e3, I005, I010, I05, I05-I005, I05-I010)

for T1i in range(1, 10):
    for lm in [65.3, 86.33, 94.43, 111.5, 123.3]:
        T1 = T1i * 1e-3
        l = lm * 1e-3
        I005 = I1(vs, rs, l, T1, T, 50e-6) * mag
        I010 = I1(vs, rs, l, T1, T, 100e-6) * mag
        I05 = I1(vs, rs, l, T1, T, 0.5e-3) * mag
        print(T1*1e3, lm, I005, I010, I05, I05-I005, I05-I010)
    print("")
#T1 = 1e-3
#for ti in range(0, 50):
#    t = ti * 1e-4
#    i = I1(vs, rs, L, T1, T, t) * mag
#    print(t, Is(vs, rs, L, T1, T), i)
#    print(t, i)


#T1 = 1e-3
#print(T1, Is(vs, rs, L, T1, T), Ie(vs, rs, L, T1, T), I1(vs, rs, L, T1, T, 50e-6))
#T1 = 5e-3
#print(T1, Is(vs, rs, L, T1, T), Ie(vs, rs, L, T1, T), I1(vs, rs, L, T1, T, 50e-6))
#T1 = 9e-3
#print(T1, Is(vs, rs, L, T1, T), Ie(vs, rs, L, T1, T), I1(vs, rs, L, T1, T, 50e-6))
