%%v.Fsmopos = exp((-v.Rs/v.Ls)*(v.Ts))
%%v.Gsmopos = (v.Vb/v.Ib)*(1/v.Rs)*(1-v.Fsmopos)

Rs = 0.175
Ls = 12.7/1000000
Ts = 1/(48000000 / 4095)
Ib = 19.5
Vb = 12


Fsmopos = exp( (Rs/Ls) * (Ts) )
Gsmopos = (Vb/Ib) * (1/Rs) * ( 1 - Fsmopos )
