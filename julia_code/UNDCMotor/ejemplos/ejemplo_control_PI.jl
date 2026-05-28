#mport Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using UNDCMotor
include("ControlUN.jl")
using UNDCMotor


# definicion del sistema

sys = MotorSystem(port="/dev/ttyUSB0");

# identificacion del sistema
G = get_fomodel_step(sys; yop=400)

SP = 0.05
tee = 0.5
tr = 0.1

wn = 



