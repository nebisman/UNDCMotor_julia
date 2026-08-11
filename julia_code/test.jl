import Pkg
include("/home/leonardo/datos/share_desktop/proyecto_julia/UNDCMotor_julia/julia_code/UNDCMotor/setup.jl")
Pkg.develop(path="/home/leonardo/datos/share_desktop/proyecto_julia/UNDCMotor_julia/julia_code/UNDCMotor")
using UNDCMotor
# include("ControlUN.jl")  
Pkg.precompile()