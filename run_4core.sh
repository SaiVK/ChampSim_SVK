TRACE_DIR=/home/sai/ChampSim-master/
binary=${1}
n_warm=${2}
n_sim=${3}
#num=${4}
#option=${5}

trace1=zeusmp_600B	
trace2=sphinx3_2520B
trace3=soplex_66B 	
trace4=bzip2_183B 

mkdir -p results_4core
(./bin/${binary} -warmup_instructions ${n_warm}000000 -simulation_instructions ${n_sim}000000 ${option} -traces ${TRACE_DIR}/${trace1}.trace.gz ${TRACE_DIR}/${trace2}.trace.gz ${TRACE_DIR}/${trace3}.trace.gz ${TRACE_DIR}/${trace4}.trace.gz) &> results_4core/mix-${binary}.txt
