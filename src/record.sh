task_name=${1}

export HYDRA_FULL_ERROR=1
python -m teleoperation.main --config-name teleop_gr1_12dof recording.task_name=${task_name} use_waist=false use_head=false camera=zed 
