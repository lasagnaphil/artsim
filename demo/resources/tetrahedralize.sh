=$1
shift
for f
~/dev/TetWild/build/TetWild -l 0.05 $file
~/dev/OctoCon/mesh/build/mesh "${file%.obj}_.msh"