set title "Dynamic Speed Contour (Path 1 New Path)" 
set xlabel "Time Horizon"
set ylabel "Space (Node Sequence)"  offset -1
set xtics (" 0:00" 0 ," 2:00" 120 ," 4:00" 240 ," 6:00" 360 ," 8:00" 480 ,"10:00" 600 ,"12:00" 720 ,"14:00" 840 ,"16:00" 960 ,"18:00" 1080 ,"20:00" 1200 ,"22:00" 1320 ,"24:00" 1440 ) 
set ytics ("4" 0, "3" 1223, "2" 2170, "1" 3337)
set xrange [0:1441] 
set yrange [0:3337] 
set palette defined (0 "white", 0.1 "red", 20 "yellow", 40 "green")
set pm3d map
splot 'C:\Software_release\sample_data_sets\2_Importing_Creating_Traffic_Modeling_Data_Sets\7.Create A Network Based on Background_Image\export_path_speed.txt' matrix notitle
