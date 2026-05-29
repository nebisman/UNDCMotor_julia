import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using UNDCMotor
include("ControlUN.jl")



# definicion del sistema
sys = MotorSystem(port="/dev/ttyUSB0");

s=tf("s")

# identificacion del sistema
G = get_model_prbs(sys; yop=150)
#funcion del ángulo


# parametros del sistema
b = numvec(G)[1][1]
a = denvec(G)[1][2]

G_ang = G/s


n = 3
Kd = ((2ζ + n) * ωn - a) / b
Kp = (1 + 2n * ζ) * ωn^2 / b
Ki = n * ωn^3 / b


# especificaciones temporales
SP = 0.05
tee = 0.5
tr = 0.2


# Valores iniciales
ζ0 = abs(log(SP))/sqrt(pi^2+(log(SP))^2)
wn0 = (1.37 - 0.952*ζ0 + 2.922*ζ0^2)/tr


# Iteraciones
n = 3
ωn = 12
ζ = 0.7

# Calculo de las constantes del PID SSO

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

result = step_closed(sys; r0 = 0, r1 = 100,  t0 = 2, t1 =2);stepinfo_exp(result;T)




