set title "Dynamic Volume Over Capcity Contour (Path 1 New Path)" 
set xlabel "Time"
set ylabel "Space" offset -3
set xtics (" 0:00" 0 ," 2:00" 120 ," 4:00" 240 ," 6:00" 360 ," 8:00" 480 ,"10:00" 600 ,"12:00" 720 ,"14:00" 840 ,"16:00" 960 ,"18:00" 1080 ,"20:00" 1200 ,"22:00" 1320 ,"24:00" 1440 ) 
set ytics ("Redwood R" 0, "Redwood R" 18, "Redwood R" 32, "Redwood R" 45, "" 61)
set xrange [0:1441] 
set yrange [0:61] 
set palette defined (0 "white", 0.4 "green", 0.6 "yellow", 1 "red")
set pm3d map
splot 'C:\NEXTA_OpenSource\Software_release\sample_data_sets\2. Importing_Creating_Traffic_Modeling_Data_Sets\1.Cube_sample_GIS_files\export_path_v_over_c.txt' matrix notitle
