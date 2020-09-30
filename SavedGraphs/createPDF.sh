#fdp $1 -Tpdf > $1.pdf
dot -Tps $1 -o $1.ps
