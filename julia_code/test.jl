#include("/home/leonardo/datos/share_desktop/proyecto_julia/motor/julia_code/UNDCMotor/setup.jl")

# # Paso 2: usar el paquete
import Pkg
Pkg.activate("/home/leonardo/datos/share_desktop/proyecto_julia/motor/julia_code/UNDCMotor")
Pkg.instantiate()

using Pkg
Pkg.add("Revise")
using Revise
using UNDCMotor

sys = MotorSystem();
# Configurar PID para control de posición
#= set_reference!(sys, 0.0);
set_pid!(sys; kp=0.1, ki=0.04, kd=0.0, N=5.0, beta=1.0,
             output=:angle, deadzone=0.125); =#

includet("ControlUN.jl")

includet("stepinfo_real_dev.jl");


#get_static_model(sys)
G = get_model_prbs(sys; yop=150)
#G, L = get_fomodel_step(sys; yop=150)

s = tf("s")
G1=G/s
wn=10
z=0.7
T1 = tf(wn^2, [1, 2*z*wn, wn^2]) 

w0 = 9
z= 0.7
k=5
#T = w0**4 / (s**4 + 2.1*w0*s**3 + 3.4*w0**2*s**2 + 2.7*w0**3*s + w0**4)
#T = (k*w0^3)/((s^2 + 2*z*w0*s + w0^2)*(s+k*w0))
#T =  (3.25*s*w0**2 + w0**3)/(s**3 + 1.75*s**2*w0 + 3.25*s*w0**2 + w0**3)
# print(T)
T = w0^3/(s^3 + 1.75*s^2*w0 + 2.15*s*w0^2 + w0^3)
T1 = w0^2/(s^2 + 2*z*w0*s + w0^2)


C2=dise2p(G1,T, 2 ,[-100])
#dise2p1(G1, T, 2,[-10, -10 ])


set_controller(sys, C2; output=:angle, deadzone=0.4);

res = step_closed(sys; r0 = 0, r1 = 100,  t0 = 1, t1 =4);
stepinfo_exp(res; T)


