set key off
 set xrange [-5:5]
 set zrange [-5:5]
 set yrange [-5:5]
set view equal xyz
splot 'map.tsv' using 1:2:3 with points pointsize 0.25 pointtype 7, \
      'pose.tsv' using 2:3:4 with points pointsize 0.35 linecolor rgb "green" pointtype 1
pause -1
