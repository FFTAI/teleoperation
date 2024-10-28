task_name=${1}

export HYDRA_FULL_ERROR=1
python -m teleoperation.main --config-name daq_gr1 recording.task_name=${task_name}