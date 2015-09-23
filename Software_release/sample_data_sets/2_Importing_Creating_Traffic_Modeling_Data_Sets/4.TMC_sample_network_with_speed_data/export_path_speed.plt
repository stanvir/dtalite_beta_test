set title "Dynamic Speed Contour (Path 1 )" 
set xlabel "Time"
set ylabel "Space" offset -3
set xtics (" 0:00" 0 ," 2:00" 120 ," 4:00" 240 ," 6:00" 360 ," 8:00" 480 ,"10:00" 600 ,"12:00" 720 ,"14:00" 840 ,"16:00" 960 ,"18:00" 1080 ,"20:00" 1200 ,"22:00" 1320 ,"24:00" 1440 ) 
set ytics ("114+04395" 0, "114P04395" 13, "114+04396" 20, " " 28)
set xrange [0:1441] 
set yrange [0:28] 
set palette defined (0 "white", 0.1 "red", 40 "yellow", 50 "green")
set pm3d map
splot 'C:\NEXTA_OpenSource\Software_release\sample_data_sets\2.Importing_sample_data_sets\4.TMC_sample_network_with_speed_data\export_path_speed.txt' matrix notitle
