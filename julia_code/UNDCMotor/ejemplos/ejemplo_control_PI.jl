import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using UNDCMotor
include("ControlUN.jl")



# definicion del sistema
sys = MotorSystem(port="/dev/ttyUSB0");

s=tf("s")

# identificacion del sistema
G = get_model_prbs(sys; yop=360)

# parametros del sistema
b = numvec(G)[1][1]
a = denvec(G)[1][2]


# especificaciones temporales
SP = 0.05
tee = 0.5
tr = 0.2


# Valores iniciales
ζ0 = abs(log(SP))/sqrt(pi^2+(log(SP))^2)
ωn0 = (1.37 - 0.952*ζ0 + 2.922*ζ0^2)/tr


# iteraciones
ωn=12
ζ = 0.7
T  = ωn^2/(s^2 + 2*ζ0*ωn*s +ωn^2)

# calculo de las constantes
Kp = (2*ζ*ωn-a)/b
Ki = ωn^2/b


# exoerimentos
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=0, output=:speed, deadzone=0)

result = step_closed(sys; r0 = 0, r1 = 360,  t0 = 2, t1 =2); stepinfo_exp(result;T)




