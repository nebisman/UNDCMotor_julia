import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using UNDCMotor
include("ControlUN.jl")







# definicion del sistema
sist = MotorSystem(port="/dev/ttyUSB0", bauds=460800);

s=tf("s")

# identificacion del sistema
G = get_model_prbs(sys; yop=360)

# parametros del sistema
b = numvec(G)[1][1]
a = denvec(G)[1][2]


# especificaciones transitorias
SP = 0.05
tee = 0.5
tr = 0.2



# Valores iniciales
ζ0 = abs(log(SP))/sqrt(pi^2+(log(SP))^2)
ωn1 = (1.197 - 0.43*ζ0 + 2.553*ζ0^2)/tr
ωn2 = 5/tee
ωn3 = a/(2*ζ0)

ωn0 = maximum([ωn1, ωn2, ωn3])

# iteraciones
ωn=12
ζ = 0.7
T  = ωn^2/(s^2 + 2*ζ*ωn*s +ωn^2)

# calculo de las constantes
Kp = (2*ζ*ωn-a)/b
Ki = ωn^2/b


# exoerimentos
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=1, output=:speed, deadzone=0)

result = step_closed(sys; r0 = 0, r1 = 360,  t0 = 1, t1 =3); stepinfo_exp(result;T)
#stairs_closed(sys, stairs=[180, 270, 360, 450,540,620] )



