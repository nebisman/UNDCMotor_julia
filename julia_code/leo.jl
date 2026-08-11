### A Pluto.jl notebook ###
# v1.0.3

using Markdown
using InteractiveUtils

# ╔═╡ 7559c14c-91da-11f1-a24f-874e038aac79
begin
import Pkg
Pkg.activate("/home/leonardo/datos/share_desktop/proyecto_julia/UNDCMotor_julia/julia_code/UNDCMotor")
using ControlSystems, Plots, PlutoUI
using UNDCMotor    
    md"Paquetes cargados desde el entorno local."
end

# ╔═╡ c5482d94-e7be-4855-9518-72ec0164ab28
begin       
include("ControlUN.jl")


# definicion del sistema
sys = MotorSystem(port="/dev/ttyUSB0", bauds=460800);

s=tf("s")
G =  get_model_prbs(sys; yop=360,sigma=100);
md""
end

# ╔═╡ 56625d26-3256-48be-8d82-ea1207eac58d
begin
ωn=20
ζ = 0.7
# calculo de las constantes



a = denvec(G)[1][2]
b = numvec(G)[1][1]
Kp = (2*ζ*ωn-a)/b
Ki = ωn^2/b
Gd=G*delay(0.01)
G1  = feedback(Gd,Kp)
T= feedback(G1*Ki/s,1)
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=0, output=:speed, deadzone=0)
result = step_closed(sys; r0 = 0, r1 =360,  t0 = 0.5, t1 =1.5); 
stepinfo_exp(result;T=T)
md""
end

# ╔═╡ 77665c93-60e9-4a55-a47c-00ad919b60de
screen_pluto()

# ╔═╡ Cell order:
# ╠═7559c14c-91da-11f1-a24f-874e038aac79
# ╠═c5482d94-e7be-4855-9518-72ec0164ab28
# ╠═56625d26-3256-48be-8d82-ea1207eac58d
# ╠═77665c93-60e9-4a55-a47c-00ad919b60de
