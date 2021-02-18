file=$1
~/dev/TetWild/build/TetWild -l 0.05 --is-laplacian $file
~/dev/OctoCon/mesh/build/mesh "${file%.obj}_.msh"