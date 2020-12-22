cmake-build-release/demo/artsim_demo_contact_bench pgs > perf_$1_pgs.txt
cmake-build-release/demo/artsim_demo_contact_bench bisection > perf_$1_bisection.txt
cmake-build-release/demo/artsim_demo_contact_bench ncp > perf_$1_ncp.txt

for file in perf_$1_*.txt
do
  echo $file
  awk '/Contact solver:/{ total += $3; count++ } END { print total/count }' $file
done