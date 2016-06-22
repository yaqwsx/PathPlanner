set key left top

set terminal pdfcairo size 20cm,20cm font "sans,20"

set output dir."velocity.pdf"

plot dir."velocity.txt" with linespoints pointinterval 10 title "Center velocity", \
     dir."left_velocity.txt" with lines title "Left velocity", \
     dir."right_velocity.txt" with lines title "Right velocity"

set output dir."shape.pdf"
set size ratio -1

plot dir."control_points.txt" with linespoints pointinterval 10 title "Control points", \
     dir."trajectory_points.txt" with linespoints pointinterval 10 title "Trajectory", \
     dir."environment_points.txt" with lines lw 6 title "Environment", \
     dir."left_points.txt" with linespoints pointinterval 10 title "Left wheel", \
     dir."right_points.txt" with linespoints pointinterval 10 title "Right wheel", \
     dir."reconstructed_points.txt" with linespoints pointinterval 10 title "Reconstructed"