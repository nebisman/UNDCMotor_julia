
# inclusion del paquete
import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using UNDCMotor


include("ControlUN.jl")



# definicin del sistema
sys = MotorSystem(port="/dev/ttyUSB0");

#funcion de transferencia
G_ang = transfer_function(sys)


# funcion de angulo

s=tf("s")

# funcion itae
w0 = 12
T = (w0^3)/(s^3 + 1.75*s^2*w0 + 2.15*s*w0^2 + w0^3)

#Diseño y carga del controlador 
C2 = dise2p(G_ang,T, 2 ,[-100 ])
set_controller(sys, C2; output=:angle, deadzone=0.4);


# respuesta del controlador
result = step_closed(sys; r0 = 0, r1 = 80,  t0 = 1, t1 =2); stepinfo_exp(result;T)
profile_closed(sys; timevalues =[0,2,4,6,8], refvalues=[0,90,180,90, 0])
