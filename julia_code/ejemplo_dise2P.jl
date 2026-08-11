

using Pkg
Pkg.activate("/home/leonardo/datos/share_desktop/proyecto_julia/motor/julia_code/UNDCMotor")
Pkg.add("ControlSystemIdentification")

using ControlSystemIdentification
using UNDCMotor



include("ControlUN.jl")

sys = MotorSystem();

   


G = get_model_prbs(sys; yop=150)
#G, L = get_fomodel_step(sys; yop=150) # ussando PRBS

s = tf("s")
G = 1249.4892/(0.3713*s + 1.0000)
G_ang = G/s


# funcion itae
w0 = 11
T = (2.15*w0^2*s+w0^3)/(s^3 + 1.75*s^2*w0 + 2.15*s*w0^2 + w0^3)
T = (w0^3)/(s^3 + 1.75*s^2*w0 + 2.15*s*w0^2 + w0^3)

#Diseño y carga del controlador 
C2 = dise2p(G_ang,T, 2 ,[-100 ])
set_controller(sys, C2; output=:angle, deadzone=0.4);


using Revise
#implementacion y comparacion
res=step_closed(sys; r0 = 0, r1 = 80,  t0 = 1, t1 =3);
includet("stepinfo_real_dev.jl");stepinfo_exp(res;T)
plot!(res[1],100.5res[4])
Gru=minreal(T/G_ang)




