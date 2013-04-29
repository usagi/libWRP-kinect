#!/usr/bin/Rscript
library(rgl)
a = data.frame(scan("model.vertices.tsv", list(x=0,y=0,z=0,r=0,g=0,b=0)));
plot3d(a$x,a$y,a$z,col=rgb(a$r,a$g,a$b))
