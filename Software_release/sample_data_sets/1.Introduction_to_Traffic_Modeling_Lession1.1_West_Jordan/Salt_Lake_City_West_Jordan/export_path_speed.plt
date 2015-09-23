set title "Dynamic Speed Contour (Path 1  ) Unit: mph" 
set xlabel "Time Horizon"
set ylabel "Space (Node Sequence)"  offset -1
set xtics ("15:30" 0 ,"16:30" 60 ,"17:30" 120 ,"18:30" 180 ,"19:30" 240 ,"20:30" 300 ,"21:30" 360 ,"22:30" 420 ) 
set ytics ("7800 Sout" 0, "7800 Sout" 4, "7800 Sout" 6, "7800 Sout" 8, "7800 Sout" 13, "Center St" 15, " " 17)
set xrange [0:421] 
set yrange [0:17] 
set palette defined (0 "white", 0.1 "red", 20 "yellow", 40 "green")
set pm3d map
splot 'C:\Software_release\sample_data_sets\1.Introduction_to_Traffic_Modeling_Lession1.1_West_Jordan\Salt_Lake_City_West_Jordan\export_path_speed.txt' matrix notitle
