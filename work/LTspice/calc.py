import math
import sys

def Ie(vs, rs, ls, T1, T):
    return( (vs / rs) * (1 - math.exp(-(rs / ls) * T1)) / (1 - math.exp(-(rs / ls) * T)))

def Is(vs, rs, ls, T1, T):
    return(Ie(vs, rs, ls, T1, T) * math.exp(-(rs / ls) * (T - T1)))

def I1(vs, rs, ls, T1, T, t):
    return(vs / rs - (vs / rs - Is(vs, rs, ls, T1, T) ) * math.exp(-(rs / ls) * t))

vs = 12
rs = 11
T = 5e-3
#T1 = 4.5e-3
T1 = float(sys.argv[1]) * 1e-3
for ls in (25e-3, 27e-3, 29e-3, 31e-3):
#    for T1 in (0.5e-3, 1.0e-3, 1.5e-3, 2.0e-3, 
    I0 = Is(vs, rs, ls, T1, T)
    I05 = I1(vs, rs, ls, T1, T, 0.5e-3)
    I10 = I1(vs, rs, ls, T1, T, 1.0e-3)
    print(ls*1e3, I0, I05, I10, I05-I0, I10-I0)
    
