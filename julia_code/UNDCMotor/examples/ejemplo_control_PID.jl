import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using UNDCMotor
include("ControlUN.jl")



# definicion del sistema
sys = MotorSystem(port="/dev/ttyUSB0");

s=tf("s")

# identificacion del sistema
G_ang = transfer_function(sys) 
#funcion del ángulo


# parametros del sistema
b = numvec(G_ang)[1][1]
a = denvec(G_ang)[1][2]




# especificaciones transitorias
tee = 0.5
tr = 0.2
SP = 0.1

# Valores iniciales
n = 3
ζ0 = abs(log(SP))/sqrt(pi^2+(log(SP))^2)
ωn1 = (1.197 - 0.43*ζ0 + 2.553*ζ0^2)/tr
ωn2 = 5/tee
ωn3 = a/(2*ζ0+n)

ωn0 = maximum([ωn1, ωn2, ωn3])


# Iteraciones

ωn = 9
ζ = 0.7

# Calculo de las constantes del PID Segundo orden

T  = n*ωn^3/((s+n*ωn)*(s^2 + 2*ζ0*ωn*s +ωn^2))
Kd = ((2ζ + n) * ωn - a) / b
Kp = (1 + 2n * ζ) * ωn^2 / b
Ki = n * ωn^3 / b

# calculo de las constantes del PID con ITAE
T = (ωn^3)/(s^3 + 1.75*s^2*ωn + 2.15*s*ωn^2 + ωn^3)
Kd = (1.78 * ωn - a) / b
Kp = 2.15 * ωn^2 / b
Ki = ωn^3/b
 

set_pid(sys;  kp=Kp, ki=Ki, kd=Kd, beta=0, output=:angle, deadzone=0.2)

result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 1, t1 =3);stepinfo_exp(result;T)




