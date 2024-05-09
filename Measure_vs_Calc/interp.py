# 双線形補間
# https://risalc.info/src/linear-bilinear-interporation.html#bil
# 双線形補間（長方形領域出ない場合）
# https://hexadrive.jp/hexablog/program/26905/

def bilinear_interpolation(x, y, points):
    """
    双線形補間を行う関数
    (x, y): 補間を求めたい点の座標
    points: 辺の点とその点での値を含むリスト。各要素は ((x1, y1), value1) の形式。
            [(x0, y0, Q00), (x1, y1, Q10), (x2, y2, Q01), (x3, y3, Q11)]
    """
    (x0, y0, Q0), (x1, y1, Q1), (x2, y2, Q2), (x3, y3, Q3) = points
    
    w_x01 = (x - x0) / (x1 - x0)
    w_x23 = (x - x2) / (x3 - x2)
    x01, y01 = (1 - w_x01) * x0 + w_x01 * x1, (1 - w_x01) * y0 + w_x01 * y1
    x23, y23 = (1 - w_x23) * x2 + w_x23 * x3, (1 - w_x23) * y2 + w_x01 * y3
    Q01 = (1 - w_x01) * Q0 + w_x01 * Q1
    Q23 = (1 - w_x01) * Q2 + w_x01 * Q3
    w_y = (y - y01) / (y23 - y01)
    x0123, y0123 = (1 - w_y) * x01 + w_y * x23, (1 - w_y) * y01 + w_y * y23
    Q0123 = (1 - w_y) * Q01 + w_y * Q23
    return Q0123

#points = [(0, 0, 1), (1, 0, 2), (0, 1, 3), (1, 1, 4)]
#print(bilinear_interpolation(0, 0, points))


filename = 'exp_table.dat'
with open(filename, 'r') as file:
    array = []  # 結果を格納する2次元配列
    with open(filename, 'r') as file:
        for line in file:
            # タブで分割してから、それぞれの要素をfloatに変換
            numbers = [float(value) for value in line.strip().split('\t')]
            array.append(numbers)

Nx = 5
Ny = 9
Ton = [[0 for _ in range(Nx)] for _ in range(Ny)]
ADC = [[0 for _ in range(Nx)] for _ in range(Ny)]
L = [[0 for _ in range(Nx)] for _ in range(Ny)]
for i in range(Nx*Ny):
    x = int(i / Nx)
    y = i % Nx
    Ton[x][y] = array[i][0]
    ADC[x][y] = array[i][1]
    L[x][y] = array[i][2]

#print(Ton)
#print(ADC)
#print(L)

def calcL(Ton, ADC, L, Ton0, ADC0):
    L0 = -1
    for y in range(Nx - 1):
        for x in range(Ny - 1):
            Ton_min, Ton_max = 1000, -1000
            ADC_min, ADC_max = 1000, -1000
            for x0 in range(2):
                for y0 in range(2):
                    if  Ton[x+x0][y+y0] < Ton_min:
                        Ton_min = Ton[x+x0][y+y0]
                    if  Ton[x+x0][y+y0] > Ton_max:
                        Ton_max = Ton[x+x0][y+y0]
                    if  ADC[x+x0][y+y0] < ADC_min:
                        ADC_min = ADC[x+x0][y+y0]
                    if  ADC[x+x0][y+y0] > ADC_max:
                        ADC_max = ADC[x+x0][y+y0]
            Ton00, Ton10, Ton01, Ton11 = Ton[x][y], Ton[x+1][y], Ton[x][y+1], Ton[x+1][y+1]
            ADC00, ADC10, ADC01, ADC11 = ADC[x][y], ADC[x+1][y], ADC[x][y+1], ADC[x+1][y+1]
            L00, L10, L01, L11 = L[x][y], L[x+1][y], L[x][y+1], L[x+1][y+1]
            if (Ton_min <= Ton0 and Ton0 <= Ton_max and ADC_min <= ADC0 and ADC0 <= ADC_max):
#                print(x, y, Ton0, Ton00, Ton01, Ton10, Ton11, ADC00, ADC01, ADC10, ADC11, L00, L10, L01, L11)
                points = [(Ton00, ADC00, L00), (Ton10, ADC10, L10), (Ton01, ADC01, L01), (Ton11, ADC11, L11)]
                L0 = bilinear_interpolation(Ton0, ADC0, points)
    return(L0)

for Ton0i in range(100):
    for ADC0 in range(80,140):
        Ton0 = Ton0i / 10
        Lp = calcL(Ton, ADC, L, Ton0, ADC0)
        if (Lp > 0):
            print(Ton0, ADC0, Lp)
    print("")
             
# note:
# measured "I10-I01" -> *(101*0.2)[V] -> /5*1024[ADC(10bit)]


