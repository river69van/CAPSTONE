V_base = float(input("Vb: "))
Vcc = float(input("Vcc: "))
beta = float(input("beta: "))
R1 = float(input("R1: "))
R2 = float(input("R2: "))
R3 = float(input("R3: "))
R4 = float(input("R4: "))
Rc = float(input("Rc: "))
calc_Vx = Vcc*(R2*(R3+R4)) + V_base*R1*R2
calc_Vx = calc_Vx/(R2*(R1+R3+R4)+R1*(R3+R4))
print(f"Vx out = {calc_Vx}")

calc_Ib = (calc_Vx - V_base)/(R3+R4) 
calc_Ice = calc_Ib*beta
calc_Vc = Vcc - calc_Ice*Rc

print(f"base current out = {calc_Ib}")
print(f"colector current out = {calc_Ice}")
print(f"colector voltage out = {calc_Vc}")