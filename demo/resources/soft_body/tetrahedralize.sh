filename=$(basename -- "$1")
filename="${filename%.*}"
~/dev/TetWild/build/TetWild $1 -l $2 && ~/dev/OctoCon/mesh/build/mesh "${filename}_.msh"